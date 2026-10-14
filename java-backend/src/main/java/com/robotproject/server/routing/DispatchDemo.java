package com.robotproject.server.routing;

import com.robotproject.robotagent.Command;
import com.robotproject.robotagent.CommandType;
import com.robotproject.robotagent.Telemetry;
import com.robotproject.server.RobotChannelService;

import io.grpc.stub.StreamObserver;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

/**
 * Obstruction-to-command dispatch through the REAL RobotChannelService, without a network: service.connect(fake)
 * returns the telemetry observer, whose onNext simulates the robot; the fake command observer records every command.
 * Covers the immediate (urgent) retreat, the safety gate (STOP / SLOW / emergency / stale telemetry), duplicates,
 * the stale-forward-target race, a blocked write, reconnect mid-retreat, proximity wiring and concurrency.
 * The whole demo runs twice in a row.
 */
public class DispatchDemo {

    private static final int RUNS = 2;
    private static final String ROBOT = "robot-1";
    /** D11 seeds: the reporter's obstruction sequence and each robot's obstacle-distance sequence. Printed for reproduction. */
    private static final long D11_REPORTER_SEED = 5;
    private static final long D11_ROBOT_SEED = 11;

    // INFO only (not a pass/fail condition): how often the randomized D11 hit a "boundary case", and the measured window
    // in which one is possible. D11 does not reliably reach it (sub-microsecond window per ~3 ms tick); the directed D13
    // is the Java-side proof, and that all of its checks ran and passed IS checked (d13PassedRuns, D13_CHECKS).
    private static int boundaryReached = 0;
    private static int d11Runs = 0;
    private static long d11GapNanos = 0;
    private static long d11GapSamples = 0;
    private static long d11RunNanos = 0;
    private static long d11Ticks = 0;
    private static int d13PassedRuns = 0;
    /** The number of checks D13 executes; a run counts as passed only if exactly these all ran and passed. */
    private static final int D13_CHECKS = 11;

    private static int passCount = 0;
    private static int failCount = 0;
    private static List<Boolean> outcomes = new ArrayList<>();

    private static ByteArrayOutputStream fullLog;
    private static PrintStream originalOut;
    private static PrintStream teeOut;

    public static void main(String[] args) throws Exception {
        originalOut = System.out;
        fullLog = new ByteArrayOutputStream();
        teeOut = new PrintStream(new TeeOutputStream(originalOut, fullLog), true);
        System.setOut(teeOut);

        List<List<Boolean>> perRun = new ArrayList<>();
        int[] runFailures = new int[RUNS];
        try {
            for (int run = 1; run <= RUNS; run++) {
                header("RUN " + run + " of " + RUNS);
                outcomes = new ArrayList<>();
                int failuresBefore = failCount;
                d1_immediateRetreat();
                d2_heldByProximityStop();
                d3_slowBand();
                d4_emergency();
                d5_staleTelemetry();
                d6_duplicatesAndOldReports();
                d7_staleForwardTargetRace();
                d8_blockedWriteDoesNotStall();
                d9_reconnectMidRetreat();
                d10_proximityWiring();
                d11_concurrency();
                d12_retreatDueUnderStop();
                d13_boundaryCaseDirected();
                perRun.add(outcomes);
                runFailures[run - 1] = failCount - failuresBefore;
            }
            outcomes = new ArrayList<>();                   // these final lines are not part of either run's comparison
            header("BOUNDARY-CASE COVERAGE");
            System.out.println(String.format("INFO: randomized D11 hit the 'boundary case' %d times over %d seeded runs "
                    + "(seeds reporter=%d, robots=%d); measured window: mean measure-to-send gap %.2f us vs mean tick "
                    + "%.2f ms. Not a pass/fail condition: D11 does not reliably sample this window; the directed D13 "
                    + "is the Java-side proof.",
                    boundaryReached, d11Runs, D11_REPORTER_SEED, D11_ROBOT_SEED,
                    d11GapSamples == 0 ? 0.0 : d11GapNanos / 1000.0 / d11GapSamples,
                    d11Ticks == 0 ? 0.0 : d11RunNanos / 1_000_000.0 / d11Ticks));
            check("the directed boundary-case test D13 ran all " + D13_CHECKS + " of its checks and passed them in every run ("
                    + d13PassedRuns + " of " + RUNS + ")", d13PassedRuns == RUNS);
        } finally {
            System.setOut(originalOut);
        }

        System.out.println();
        for (int run = 1; run <= RUNS; run++) {
            System.out.println("run " + run + ": " + (runFailures[run - 1] == 0 ? "all passed" : runFailures[run - 1] + " failed"));
        }
        System.out.println("second run identical to the first (same checks, same pass/fail): "
                + (perRun.size() == 2 && perRun.get(0).equals(perRun.get(1)) ? "yes" : "NO"));
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
    }

    // ------------------------------------------------------------------
    // Cases
    // ------------------------------------------------------------------
    private static void d1_immediateRetreat() {
        header("D1: a retreat reaches the robot immediately, through the proactive path");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);

        int n = r.obs.size();
        long lastId = r.obs.last().id;
        int since = mark();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        check("the REROUTE arrived BEFORE the report call returned (1 new command, got " + (r.obs.size() - n) + ")",
                r.obs.size() == n + 1);
        Cmd urgent = r.obs.last();
        checkCmd("that command", urgent, CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0);
        check("its id is the previous id + 1 (" + lastId + " + 1, got " + urgent.id + ")", urgent.id == lastId + 1);
        check("it was written by the reporting thread itself (got " + urgent.thread + ")",
                Thread.currentThread().getName().equals(urgent.thread));
        check("log has \"Dispatch (urgent):\"", segment(since).contains("Dispatch (urgent):"));

        r.tick(10, 2);
        checkCmd("next tick (10, 2)", r.obs.last(), CommandType.COMMAND_TYPE_CONTINUE, null, null);
        checkCount("retreat REROUTEs to (10.0, 0.0) after the prelude so far", r.obs.countReroutesTo(10.0, 0.0, 3), 1);
        r.tick(10, 0);
        checkCmd("tick (10, 0) (retreat completed)", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 20.0, 0.0);
        checkIdsConsecutive("D1", r.obs);
    }

    private static void d2_heldByProximityStop() {
        header("D2: a retreat decided under a proximity STOP is held, then released exactly once");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        r.tick(10, 2, 0.5, false, 1.0);
        checkCmd("tick with obstacle 0.5", r.obs.last(), CommandType.COMMAND_TYPE_STOP, null, null);

        int n = r.obs.size();
        int since = mark();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        check("no routing command is sent (got " + (r.obs.size() - n) + " new)", r.obs.size() == n);
        for (int i = 0; i < 3; i++) {
            r.tick(10, 2, 0.5, false, 1.0);
            checkCmd("hold tick " + (i + 1), r.obs.last(), CommandType.COMMAND_TYPE_STOP, null, null);
        }
        checkCount("\"Dispatch held (safety STOP_PROXIMITY)\" lines", count(segment(since), "Dispatch held (safety STOP_PROXIMITY)"), 1);

        n = r.obs.size();
        since = mark();
        r.tick(10, 2, 5.0, false, 1.0);
        List<Cmd> atRelease = r.obs.since(n);
        check("the releasing tick (obstacle 5.0) sends exactly ONE command, the REROUTE (10.0, 0.0) (got " + atRelease + ")",
                atRelease.size() == 1 && atRelease.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        check("log has \"Dispatch (released)\"", segment(since).contains("Dispatch (released)"));
        r.tick(10, 2, 5.0, false, 1.0);
        r.tick(10, 2, 5.0, false, 1.0);
        checkCount("retreat REROUTEs to (10.0, 0.0) after the prelude after two more ticks", r.obs.countReroutesTo(10.0, 0.0, 3), 1);
        r.tick(10, 0, 5.0, false, 1.0);
        checkCmd("the retreat completes: tick (10, 0)", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 20.0, 0.0);
        checkIdsConsecutive("D2", r.obs);
    }

    private static void d3_slowBand() {
        header("D3: in the SLOW band the urgent REROUTE carries the SLOW speed, never faster");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, 2.0, 1.5, CommandType.COMMAND_TYPE_SLOW_DOWN);
        int n = r.obs.size();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        List<Cmd> sent = r.obs.since(n);
        check("one urgent REROUTE (10.0, 0.0) (got " + sent + ")",
                sent.size() == 1 && sent.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        check("its target_speed is 1.0 although the robot reports 1.5 (got "
                + (sent.isEmpty() ? "none" : sent.get(0).speed) + ")", !sent.isEmpty() && sent.get(0).speed == 1.0);
    }

    private static void d4_emergency() {
        header("D4: under an emergency stop nothing but STOP is sent; the retreat follows once it lifts");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        r.tick(10, 2, null, true, 0.0);
        checkCmd("emergency tick", r.obs.last(), CommandType.COMMAND_TYPE_STOP, null, null);
        int n = r.obs.size();
        int since = mark();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        check("nothing is sent by the report (got " + (r.obs.size() - n) + " new)", r.obs.size() == n);
        check("log has \"Dispatch held (safety STOP_EMERGENCY)\"", segment(since).contains("Dispatch held (safety STOP_EMERGENCY)"));
        r.tick(10, 2, null, true, 0.0);
        checkCmd("still emergency", r.obs.last(), CommandType.COMMAND_TYPE_STOP, null, null);
        check("nothing but STOP since the report", r.obs.since(n).stream().allMatch(c -> c.type == CommandType.COMMAND_TYPE_STOP));
        n = r.obs.size();
        r.tick(10, 2, 8.0, false, 1.0);
        List<Cmd> atRelease = r.obs.since(n);
        check("emergency lifted, obstacle 8.0: exactly one REROUTE (10.0, 0.0) (got " + atRelease + ")",
                atRelease.size() == 1 && atRelease.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        r.tick(10, 2, 8.0, false, 1.0);
        checkCount("retreat REROUTEs to (10.0, 0.0) after the prelude after one more tick", r.obs.countReroutesTo(10.0, 0.0, 3), 1);
    }

    private static void d5_staleTelemetry() throws InterruptedException {
        header("D5: with stale telemetry nothing is sent; the next tick yields the retreat exactly once");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        mm.setPositionMaxAge(Duration.ofMillis(300));
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        Thread.sleep(400);
        int n = r.obs.size();
        int since = mark();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        check("no command is sent (got " + (r.obs.size() - n) + " new)", r.obs.size() == n);
        String seg = segment(since);
        check("the log says deferred or held for stale telemetry",
                seg.contains("is deferred") || seg.contains("Dispatch held (stale telemetry)"));
        n = r.obs.size();
        r.tick(10, 2);
        List<Cmd> next = r.obs.since(n);
        check("the next tick yields the retreat REROUTE (10.0, 0.0) (got " + next + ")",
                next.size() == 1 && next.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        r.tick(10, 2);
        checkCount("retreat REROUTEs to (10.0, 0.0) after the prelude after one more tick", r.obs.countReroutesTo(10.0, 0.0, 3), 1);
    }

    private static void d6_duplicatesAndOldReports() {
        header("D6: a duplicate and an older world report send no extra command");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        WorldObstructionReport rep = new WorldObstructionReport("w", 10.0, 8.1, 1.3, null,
                ObstructionKind.SEMI_PERMANENT, null);
        int n = r.obs.size();
        checkOutcome("first report (blocking, ahead)", mm.reportWorldObstruction(rep), MissionManager.ReportOutcome.APPLIED);
        checkCount("commands sent by it (the retreat)", r.obs.size() - n, 1);
        n = r.obs.size();
        checkOutcome("the same report again", mm.reportWorldObstruction(rep), MissionManager.ReportOutcome.REFRESHED);
        checkOutcome("an older one (observedAt now-30s)", mm.reportWorldObstruction(new WorldObstructionReport("w", 10.0, 8.1,
                1.3, null, ObstructionKind.SEMI_PERMANENT, Instant.now().minusSeconds(30))), MissionManager.ReportOutcome.IGNORED_OLDER);
        checkCount("extra commands for the duplicate and the older report", r.obs.size() - n, 0);
        r.tick(10, 2);
        checkCmd("the next tick", r.obs.last(), CommandType.COMMAND_TYPE_CONTINUE, null, null);
        checkCount("retreat REROUTEs to (10.0, 0.0) after the prelude", r.obs.countReroutesTo(10.0, 0.0, 3), 1);
    }

    private static void d7_staleForwardTargetRace() {
        header("D7: a forward target decided before a retreat is dropped as superseded");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        r.tick(0, 0);
        checkCmd("tick (0, 0)", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0);
        AtomicBoolean fired = new AtomicBoolean(false);
        svc.setBeforeRoutingSendHookForTests(() -> {
            if (fired.compareAndSet(false, true)) {
                mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);     // same thread, between decision and send
            }
        });
        int since = mark();
        r.tick(10, 0);
        svc.setBeforeRoutingSendHookForTests(null);
        String seg = segment(since);
        check("the hook fired", fired.get());
        check("log has \"Dispatch (urgent): robot-1 REROUTE -> (10.0, 0.0)\"",
                seg.contains("Dispatch (urgent): robot-1 REROUTE -> (10.0, 0.0)"));
        check("log has \"Dispatch dropped (superseded): robot-1 REROUTE -> (10.0, 9.0)\"",
                seg.contains("Dispatch dropped (superseded): robot-1 REROUTE -> (10.0, 9.0)"));
        checkCmd("the last command the robot received", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0);
        checkCount("REROUTEs to (10.0, 9.0) ever sent", r.obs.countReroutesTo(10.0, 9.0), 0);
        checkIdsConsecutive("D7", r.obs);
    }

    private static void d8_blockedWriteDoesNotStall() throws InterruptedException {
        header("D8: a blocked gRPC write does not stall the mission layer (L1/L2)");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot a = new Robot(svc, "d8a");
        a.obs.blockNextRerouteFor(150);
        AtomicReference<Throwable> err = new AtomicReference<>();
        Thread writer = new Thread(() -> {
            try { a.tick(0, 0); } catch (Throwable t) { err.set(t); }
        }, "d8-telemetry");
        long started = System.nanoTime();
        writer.start();
        check("the write is blocked inside onNext", a.obs.awaitBlocked(2000));

        long t0 = System.nanoTime();
        mm.missionStatus("d8a");
        long statusMs = (System.nanoTime() - t0) / 1_000_000;
        t0 = System.nanoTime();
        mm.onTelemetry("d8b", 0, 0, "E");
        long otherTickMs = (System.nanoTime() - t0) / 1_000_000;
        t0 = System.nanoTime();
        mm.reportEdgeObstruction("C", "E", "ce", 0.3, 0.5);
        long reportMs = (System.nanoTime() - t0) / 1_000_000;
        boolean stillBlocked = writer.isAlive();
        writer.join(5000);
        long writerMs = (System.nanoTime() - started) / 1_000_000;

        check("the three calls ran while the write was still blocked", stillBlocked);
        check("missionStatus(d8a) returns within 60 ms (took " + statusMs + " ms)", statusMs < 60);
        check("onTelemetry of another robot returns within 60 ms (took " + otherTickMs + " ms)", otherTickMs < 60);
        check("a report on a corridor neither robot is on returns within 60 ms (took " + reportMs + " ms)", reportMs < 60);
        check("the blocked tick itself took >= 150 ms (took " + writerMs + " ms) and threw nothing (" + err.get() + ")",
                writerMs >= 150 && err.get() == null);
    }

    private static void d9_reconnectMidRetreat() {
        header("D9: reconnecting mid-retreat re-sends the retreat, never the forward target");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        checkCmd("the retreat was sent on the first stream", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0);
        r.stream.onCompleted();
        check("the first observer was completed", r.obs.closed);

        FakeObserver second = new FakeObserver();
        StreamObserver<Telemetry> stream2 = svc.connect(second);
        stream2.onNext(tel(ROBOT, 10, 1.5, null, false, 1.0, 1));
        check("the new stream's first command is REROUTE (10.0, 0.0) (got " + second.snapshot() + ")",
                second.size() >= 1 && second.snapshot().get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        checkCount("REROUTEs to (10.0, 9.0) on the new stream", second.countReroutesTo(10.0, 9.0), 0);
    }

    private static void d10_proximityWiring() {
        header("D10: a matching proximity reading corroborates a stale obstruction and the retreat is dispatched");
        Robot r = proximitySetup();
        MissionManager mm = r.svc.getMissionManager();
        int n = r.obs.size();
        int since = mark();
        r.tick(10, 2.7, 2.0, false, 1.0);
        String seg = segment(since);
        check("log has \"corroborated\"", seg.contains("corroborated"));
        check("the obstruction is FRESH again (got " + stateOf(mm, "stale") + ")", stateOf(mm, "stale") == ExpiryPolicy.State.FRESH);
        check("a retreat is decided", seg.contains("retreating to B"));
        check("the REROUTE to (10.0, 0.0) arrives (got " + r.obs.since(n) + ")",
                r.obs.since(n).stream().anyMatch(c -> c.is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0)));

        Robot q = proximitySetup();
        MissionManager mq = q.svc.getMissionManager();
        n = q.obs.size();
        since = mark();
        q.tick(10, 2.7, 8.0, false, 1.0);
        seg = segment(since);
        check("with distance 8.0 nothing happens: no \"corroborated\", no REROUTE, still STALE",
                !seg.contains("corroborated")
                        && q.obs.since(n).stream().noneMatch(c -> c.type == CommandType.COMMAND_TYPE_REROUTE)
                        && stateOf(mq, "stale") == ExpiryPolicy.State.STALE);
    }

    private static Robot proximitySetup() {
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        MutableClock clock = new MutableClock(Instant.parse("2026-01-01T00:00:00Z"));
        mm.graph().setClock(clock);
        mm.graph().setStaleProbingEnabled(true);
        mm.graph().setExpiryPolicy(policy(Duration.ofSeconds(5), Duration.ofSeconds(25)));
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        // Reported only now that the robot is on B->D: with probing ON a stale obstruction known at planning time
        // would add its expected-encounter cost and route the robot via C instead.
        mm.reportEdgeObstruction("B", "D", "stale", 0.3, 0.5, ObstructionKind.TRANSIENT, clock.instant().minusSeconds(15));
        return r;
    }

    private static ExpiryPolicy.State stateOf(MissionManager mm, String eventId) {
        for (ObstructionView v : mm.graph().getEdge("B", "D").getObstructionViews()) {
            if (v.eventId.equals(eventId)) { return v.state; }
        }
        return null;
    }

    // ------------------------------------------------------------------
    // D11 - concurrency
    // ------------------------------------------------------------------
    /**
     * D12: a retreat that becomes due while the verdict is STOP is held - only STOP goes out - and once the verdict
     * clears it is the FIRST non-STOP command, sent exactly once. (a) the retreat is decided by a report, i.e. through
     * the dispatch listener; (b) it is decided on the telemetry path, by a deferred decision drained on a STOP tick.
     */
    private static void d12_retreatDueUnderStop() {
        header("D12: a retreat due under a STOP verdict is held, then sent exactly once as the first non-STOP command");

        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        Robot r = new Robot(svc, ROBOT);
        prelude(r, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        r.tick(10, 2, 0.5, false, 1.0);
        check("(a) the verdict before the report is STOP_PROXIMITY (got " + svc.debugVerdict(ROBOT) + ")",
                "STOP_PROXIMITY".equals(svc.debugVerdict(ROBOT)));
        int n = r.obs.size();
        int since = mark();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        check("(a) the retreat became due (\"retreating to B\")", segment(since).contains("retreating to B"));
        check("(a) it is held: hasHeldDispatch is true", mm.hasHeldDispatch(ROBOT));
        check("(a) the report itself sent no command (got " + (r.obs.size() - n) + ")", r.obs.size() == n);
        for (int i = 0; i < 3; i++) { r.tick(10, 2, 0.5, false, 1.0); }
        List<Cmd> during = r.obs.since(n);
        check("(a) during the STOP only STOP is sent (got " + during + ")",
                during.size() == 3 && during.stream().allMatch(c -> c.type == CommandType.COMMAND_TYPE_STOP));
        check("(a) still held after the STOP ticks", mm.hasHeldDispatch(ROBOT));
        check("(a) no \"Dispatch (urgent)\" line while the verdict is STOP", !segment(since).contains("Dispatch (urgent)"));
        int beforeClear = r.obs.size();
        r.tick(10, 2, 8.0, false, 1.0);
        r.tick(10, 2, 8.0, false, 1.0);
        List<Cmd> after = r.obs.since(beforeClear);
        check("(a) the first non-STOP command after the clear is REROUTE (10.0, 0.0) (got " + after + ")",
                !after.isEmpty() && after.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        checkCount("(a) retreat REROUTEs to (10.0, 0.0) after the prelude", r.obs.countReroutesTo(10.0, 0.0, 3), 1);
        check("(a) the held slot is empty after the release", !mm.hasHeldDispatch(ROBOT));

        RobotChannelService svc2 = new RobotChannelService();
        MissionManager m2 = svc2.getMissionManager();
        MutableClock clock = new MutableClock(Instant.parse("2026-01-01T00:00:00Z"));
        m2.graph().setClock(clock);
        Robot q = new Robot(svc2, ROBOT);
        prelude(q, null, 1.0, CommandType.COMMAND_TYPE_CONTINUE);
        clock.advance(Duration.ofSeconds(5));                          // the recorded position is now older than 3 s
        since = mark();
        m2.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        check("(b) with a stale position the decision is deferred (\"is deferred\")", segment(since).contains("is deferred"));
        n = q.obs.size();
        since = mark();
        q.tick(10, 2, 0.5, false, 1.0);                                // a STOP tick drains the deferred decision
        String seg = segment(since);
        List<Cmd> drained = q.obs.since(n);
        check("(b) the retreat became due on the STOP tick (\"retreating to B\")", seg.contains("retreating to B"));
        check("(b) that tick sent only STOP (got " + drained + ")",
                drained.size() == 1 && drained.get(0).type == CommandType.COMMAND_TYPE_STOP);
        check("(b) the retreat is held (\"Dispatch held (safety STOP_PROXIMITY)\" and hasHeldDispatch)",
                seg.contains("Dispatch held (safety STOP_PROXIMITY)") && m2.hasHeldDispatch(ROBOT));
        q.tick(10, 2, 0.5, false, 1.0);
        q.tick(10, 2, 0.5, false, 1.0);
        List<Cmd> held = q.obs.since(n);
        check("(b) during the STOP only STOP is sent (got " + held + ")",
                held.size() == 3 && held.stream().allMatch(c -> c.type == CommandType.COMMAND_TYPE_STOP));
        check("(b) no \"Dispatch (urgent)\" line while the verdict is STOP", !segment(since).contains("Dispatch (urgent)"));
        beforeClear = q.obs.size();
        q.tick(10, 2, 8.0, false, 1.0);
        q.tick(10, 2, 8.0, false, 1.0);
        after = q.obs.since(beforeClear);
        check("(b) the first non-STOP command after the clear is REROUTE (10.0, 0.0) (got " + after + ")",
                !after.isEmpty() && after.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        checkCount("(b) retreat REROUTEs to (10.0, 0.0) after the prelude", q.obs.countReroutesTo(10.0, 0.0, 3), 1);
        check("(b) the held slot is empty after the release", !m2.hasHeldDispatch(ROBOT));
    }

    /**
     * D13 (directed): constructs the D11 "boundary case" on purpose, single-threaded. The robot has measured an
     * obstacle at 0.5 m (its own verdict is STOP from this tick on) but that telemetry has not reached Java yet, so Java
     * still holds CLEAR and a retreat decided in this window goes out at once. The detector must count it as a boundary
     * case, not as a late REROUTE; and it must flag a REROUTE that arrives after STOP has lasted more than one tick.
     */
    private static void d13_boundaryCaseDirected() {
        header("D13 (directed): a REROUTE racing the robot's own switch to STOP, constructed on purpose");
        int failuresBefore = failCount;
        int checksBefore = passCount + failCount;
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        SimRobot b = new SimRobot(svc, "b1", 0, 0, new AtomicBoolean(true));
        b.connect();
        b.beginTick(8.0);
        b.sendTelemetry();
        checkCmd("tick (0, 0)", b.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0);
        b.x = 10;
        b.y = 0;
        b.beginTick(8.0);
        b.sendTelemetry();
        checkCmd("tick (10, 0)", b.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 9.0);
        b.y = 2;
        b.beginTick(8.0);
        b.sendTelemetry();
        checkCmd("tick (10, 2)", b.obs.last(), CommandType.COMMAND_TYPE_CONTINUE, null, null);

        b.beginTick(0.5);                                   // measured, not yet sent: Java still has CLEAR
        check("Java's verdict is still CLEAR (got " + svc.debugVerdict("b1") + ")", "CLEAR".equals(svc.debugVerdict("b1")));
        int n = b.obs.size();
        mm.reportEdgeObstruction("B", "D", "blk", 0.3, 0.9);
        List<Cmd> sent = b.obs.since(n);
        check("the retreat REROUTE (10.0, 0.0) went out immediately (got " + sent + ")",
                sent.size() == 1 && sent.get(0).is(CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0));
        checkCount("the detector counts it as a boundary case", b.boundaryCases.get(), 1);
        check("and not as a late REROUTE (got " + b.lateReroute.get() + ")", b.lateReroute.get() == null);

        b.sendTelemetry();                                  // the STOP telemetry reaches Java
        checkCmd("Java answers that telemetry", b.obs.last(), CommandType.COMMAND_TYPE_STOP, null, null);
        n = b.obs.size();
        b.beginTick(0.5);
        b.sendTelemetry();
        List<Cmd> next = b.obs.since(n);
        check("the next STOP tick sends only STOP (got " + next + ")",
                next.size() == 1 && next.get(0).type == CommandType.COMMAND_TYPE_STOP);
        check("still one boundary case and no late REROUTE", b.boundaryCases.get() == 1 && b.lateReroute.get() == null);

        b.beginTick(0.5);                                   // STOP has now lasted more than one tick
        Command late = Command.newBuilder().setRobotId("b1").setCommandId(999)
                .setType(CommandType.COMMAND_TYPE_REROUTE).setTarget(mm.graph().getNode("B").toWaypoint()).build();
        b.obs.onRecord.accept(new Cmd(late));
        check("detector sanity: a REROUTE after STOP lasted more than one tick is flagged as late (got "
                + b.lateReroute.get() + ")", b.lateReroute.get() != null);
        if (failCount == failuresBefore && passCount + failCount - checksBefore == D13_CHECKS) {
            d13PassedRuns++;
        }
    }

    private static void d11_concurrency() throws InterruptedException {
        header("D11: two robots on their own telemetry threads plus a reporter (real clock) - ids, mutual exclusion, "
                + "no late REROUTE, invariants, arrival. D11 does NOT reliably cover the boundary case (a REROUTE racing "
                + "the robot's own switch to STOP): the directed D13 is the Java-side proof; the robot-side in-flight-STOP "
                + "concern is left to the future C++ local-safety precedence (Part B)");
        RobotChannelService svc = new RobotChannelService();
        MissionManager mm = svc.getMissionManager();
        AtomicBoolean reporterDone = new AtomicBoolean(false);
        SimRobot d1 = new SimRobot(svc, "d1", 0, 0, reporterDone);
        SimRobot d2 = new SimRobot(svc, "d2", 20, 0, reporterDone);
        AtomicInteger reports = new AtomicInteger();
        AtomicReference<Throwable> reporterError = new AtomicReference<>();
        Thread reporter = new Thread(() -> {
            try {
                Random random = new Random(D11_REPORTER_SEED);
                String[][] ids = {{"A", "B"}, {"B", "C"}, {"B", "D"}, {"D", "E"}, {"C", "E"}};
                double[] widths = {0.4, 0.9, 1.3};
                long end = System.nanoTime() + Duration.ofSeconds(3).toNanos();
                while (System.nanoTime() < end) {
                    String id = "ev" + random.nextInt(8);
                    if (random.nextDouble() < 0.7) {
                        String[] c = ids[random.nextInt(ids.length)];
                        GraphNode a = mm.graph().getNode(c[0]);
                        GraphNode b = mm.graph().getNode(c[1]);
                        double f = 0.1 + 0.8 * random.nextDouble();
                        double ex = b.getX() - a.getX();
                        double ey = b.getY() - a.getY();
                        double len = Math.hypot(ex, ey);
                        double offset = (random.nextDouble() * 2 - 1) * 0.2;
                        ObstructionKind kind = random.nextBoolean() ? ObstructionKind.TRANSIENT : ObstructionKind.SEMI_PERMANENT;
                        mm.reportWorldObstruction(new WorldObstructionReport(id,
                                a.getX() + f * ex - ey / len * offset, a.getY() + f * ey + ex / len * offset,
                                widths[random.nextInt(widths.length)], null, kind, null));
                    } else {
                        mm.clearWorldObstruction(id);
                    }
                    reports.incrementAndGet();
                    Thread.sleep(5);
                }
                for (int i = 0; i < 8; i++) { mm.clearWorldObstruction("ev" + i); }
            } catch (Throwable t) {
                reporterError.set(t);
            } finally {
                reporterDone.set(true);
            }
        }, "d11-reporter");

        Thread[] threads = {new Thread(d1, "d11-d1"), new Thread(d2, "d11-d2"), reporter};
        System.setOut(new PrintStream(OutputStream.nullOutputStream()));
        boolean timedOut = false;
        try {
            for (Thread t : threads) { t.start(); }
            long deadline = System.nanoTime() + Duration.ofSeconds(100).toNanos();
            for (Thread t : threads) {
                t.join(Math.max(1, (deadline - System.nanoTime()) / 1_000_000));
                if (t.isAlive()) { timedOut = true; }
            }
        } finally {
            System.setOut(teeOut);
        }
        if (timedOut) {
            for (Map.Entry<Thread, StackTraceElement[]> e : Thread.getAllStackTraces().entrySet()) {
                System.out.println("THREAD " + e.getKey().getName() + " (" + e.getKey().getState() + ")");
                for (StackTraceElement s : e.getValue()) { System.out.println("    at " + s); }
            }
            for (Thread t : threads) { t.interrupt(); }
        }

        System.out.println("seeds reporter=" + D11_REPORTER_SEED + " robots=" + D11_ROBOT_SEED + "; reports: " + reports.get()
                + "; d1 ticks " + d1.ticks + ", commands " + d1.obs.size() + ", STOP entries " + d1.stopEntries.get()
                + ", urgent REROUTEs " + d1.reroutesFromOtherThreads.get() + ", boundary cases " + d1.boundaryCases.get()
                + "; d2 ticks " + d2.ticks + ", commands " + d2.obs.size() + ", STOP entries " + d2.stopEntries.get()
                + ", urgent REROUTEs " + d2.reroutesFromOtherThreads.get() + ", boundary cases " + d2.boundaryCases.get());
        System.out.println("d1 " + d1.timing() + "; d2 " + d2.timing());
        boundaryReached += d1.boundaryCases.get() + d2.boundaryCases.get();
        d11Runs++;
        for (SimRobot d : new SimRobot[] {d1, d2}) {
            d11GapNanos += d.windowNanosTotal;
            d11GapSamples += d.windowSamples;
            d11RunNanos += d.runNanos;
            d11Ticks += d.ticks;
        }
        check("watchdog: every thread finished", !timedOut);
        check("no exception in the reporter (got " + reporterError.get() + ")", reporterError.get() == null);
        for (SimRobot d : new SimRobot[] {d1, d2}) {
            check(d.id + ": no exception (got " + d.error.get() + ")", d.error.get() == null);
            check(d.id + ": command ids strictly increasing, no duplicates", d.obs.idsStrictlyIncreasing());
            check(d.id + ": mutual exclusion inside onNext never violated", !d.obs.overlap.get());
            check(d.id + ": no REROUTE followed a STOP verdict by more than one tick (first: " + d.lateReroute.get() + ")",
                    d.lateReroute.get() == null);
            check(d.id + ": no invariant violation (first: " + d.violation.get() + ")", d.violation.get() == null);
            check(d.id + ": reaches its destination", d.reached);
        }
    }

    /** A simulated robot on its own stream and thread; follows the last REROUTE target it received. */
    private static final class SimRobot implements Runnable {
        final RobotChannelService svc;
        final String id;
        final FakeObserver obs = new FakeObserver();
        final AtomicBoolean reporterDone;
        final AtomicReference<Throwable> error = new AtomicReference<>();
        final AtomicReference<String> violation = new AtomicReference<>();
        final AtomicReference<String> lateReroute = new AtomicReference<>();
        final AtomicInteger boundaryCases = new AtomicInteger();
        final AtomicInteger reroutesFromOtherThreads = new AtomicInteger();   // proactive (urgent) deliveries
        final AtomicInteger stopEntries = new AtomicInteger();                // verdict switches into STOP
        volatile double x;
        volatile double y;
        volatile Double targetX;
        volatile Double targetY;
        volatile boolean stopped;
        volatile boolean reached;
        volatile int ticks;
        volatile boolean lastWasStop;
        volatile int lastChangeTick;
        private volatile String ownThread;
        private StreamObserver<Telemetry> stream;
        private double pendingDistance;
        private long timestamp;
        // Measurement: the gap between measuring a tick's verdict and Java receiving it - the only window in which a
        // proactive REROUTE can still pass Java's gate after the robot itself has already switched to STOP.
        private long beginNanos;
        volatile long windowNanosTotal;
        volatile long windowSamples;
        volatile long runNanos;

        SimRobot(RobotChannelService svc, String id, double x, double y, AtomicBoolean reporterDone) {
            this.svc = svc; this.id = id; this.x = x; this.y = y; this.reporterDone = reporterDone;
            obs.onRecord = c -> {
                if (c.type == CommandType.COMMAND_TYPE_REROUTE) {
                    if (!c.thread.equals(ownThread)) { reroutesFromOtherThreads.incrementAndGet(); }
                    if (lastWasStop) {
                        if (ticks - lastChangeTick <= 1) { boundaryCases.incrementAndGet(); }
                        else { lateReroute.compareAndSet(null, "tick " + ticks + ", STOP since tick " + lastChangeTick); }
                    }
                    targetX = c.x;
                    targetY = c.y;
                }
                stopped = c.type == CommandType.COMMAND_TYPE_STOP;
            };
        }

        /** Opens this robot's stream on the calling thread (which becomes the robot's own thread). */
        void connect() {
            ownThread = Thread.currentThread().getName();
            stream = svc.connect(obs);
        }

        /** The robot measures its obstacle distance for the next tick: from here on its own verdict is STOP or not. */
        void beginTick(double distance) {
            boolean stop = distance < 1.0;
            int tick = ticks + 1;
            if (stop != lastWasStop) {
                lastChangeTick = tick;
                if (stop) { stopEntries.incrementAndGet(); }
            }
            lastWasStop = stop;
            ticks = tick;
            pendingDistance = distance;
            beginNanos = System.nanoTime();
        }

        /** Sends that tick's telemetry - the moment Java learns the new verdict. */
        void sendTelemetry() {
            windowNanosTotal += System.nanoTime() - beginNanos;
            windowSamples++;
            stream.onNext(tel(id, x, y, pendingDistance, false, 1.0, ++timestamp));
        }

        String timing() {
            return String.format("mean measure-to-send gap %.2f us, mean tick %.2f ms",
                    windowSamples == 0 ? 0.0 : windowNanosTotal / 1000.0 / windowSamples,
                    ticks == 0 ? 0.0 : runNanos / 1_000_000.0 / ticks);
        }

        @Override
        public void run() {
            long started = System.nanoTime();
            try {
                connect();
                MissionManager mm = svc.getMissionManager();
                Random random = new Random(D11_ROBOT_SEED);
                double[] distances = {8.0, 2.0, 0.5};
                long deadline = System.nanoTime() + Duration.ofSeconds(90).toNanos();
                while (System.nanoTime() < deadline) {
                    double distance = reporterDone.get() ? 8.0 : distances[random.nextInt(distances.length)];
                    beginTick(distance);
                    int tick = ticks;
                    sendTelemetry();
                    if (mm.missionStatus(id) == RobotMission.Status.AT_DESTINATION) { reached = true; break; }
                    Double tx = targetX;
                    Double ty = targetY;
                    if (tx != null && !stopped) {
                        double dx = tx - x;
                        double dy = ty - y;
                        double d = Math.hypot(dx, dy);
                        if (d <= 0.02) { x = tx; y = ty; } else { x += dx / d * 0.02; y += dy / d * 0.02; }
                    }
                    Thread.sleep(2);
                    if (tick % 50 == 0) {
                        List<String> v = mm.checkInvariants(id);
                        if (!v.isEmpty()) { violation.compareAndSet(null, id + ": " + v); }
                    }
                }
            } catch (Throwable t) {
                error.set(t);
            } finally {
                runNanos = System.nanoTime() - started;
            }
        }
    }

    // ------------------------------------------------------------------
    // Robot, fake observer, telemetry
    // ------------------------------------------------------------------
    private static final class Robot {
        final RobotChannelService svc;
        final String id;
        final FakeObserver obs = new FakeObserver();
        final StreamObserver<Telemetry> stream;
        long timestamp;

        Robot(RobotChannelService svc, String id) {
            this.svc = svc;
            this.id = id;
            this.stream = svc.connect(obs);
        }

        void tick(double x, double y) {
            tick(x, y, null, false, 1.0);
        }

        void tick(double x, double y, Double obstacleDistance, boolean emergency, double speed) {
            stream.onNext(tel(id, x, y, obstacleDistance, emergency, speed, ++timestamp));
        }
    }

    /** Prelude P: (0,0) -> REROUTE (10.0, 0.0); (10,0) -> REROUTE (10.0, 9.0); (10,2) -> the expected third command. */
    private static void prelude(Robot r, Double distance, double speed, CommandType third) {
        r.tick(0, 0, distance, false, speed);
        checkCmd("P: tick (0, 0)", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 0.0);
        r.tick(10, 0, distance, false, speed);
        checkCmd("P: tick (10, 0)", r.obs.last(), CommandType.COMMAND_TYPE_REROUTE, 10.0, 9.0);
        r.tick(10, 2, distance, false, speed);
        checkCmd("P: tick (10, 2)", r.obs.last(), third, null, null);
    }

    static Telemetry tel(String robot, double x, double y, Double obstacleDistance, boolean emergency, double speed,
            long timestamp) {
        Telemetry.Builder b = Telemetry.newBuilder()
                .setRobotId(robot)
                .setPositionX(x)
                .setPositionY(y)
                .setSpeed(speed)
                .setBattery(90.0)
                .setTimestamp(timestamp)
                .setEmergencyStop(emergency);
        if (obstacleDistance != null) { b.setNearestObstacleDistance(obstacleDistance); }
        return b.build();
    }

    private static final class Cmd {
        final CommandType type;
        final Double x;
        final Double y;
        final double speed;
        final long id;
        final String thread;

        Cmd(Command c) {
            this.type = c.getType();
            this.x = c.hasTarget() ? c.getTarget().getX() : null;
            this.y = c.hasTarget() ? c.getTarget().getY() : null;
            this.speed = c.getTargetSpeed();
            this.id = c.getCommandId();
            this.thread = Thread.currentThread().getName();
        }

        boolean is(CommandType t, Double ex, Double ey) {
            if (type != t) { return false; }
            if (ex == null) { return x == null; }
            return x != null && x.equals(ex) && y.equals(ey);
        }

        @Override
        public String toString() {
            return type.name().replace("COMMAND_TYPE_", "") + (x == null ? "" : " (" + x + ", " + y + ")") + " id=" + id;
        }
    }

    /** Records every command; detects concurrent callers inside onNext; can block once on the next REROUTE. */
    private static final class FakeObserver implements StreamObserver<Command> {
        private final List<Cmd> commands = Collections.synchronizedList(new ArrayList<>());
        final AtomicInteger inside = new AtomicInteger();
        final AtomicBoolean overlap = new AtomicBoolean();
        volatile boolean closed;
        volatile Consumer<Cmd> onRecord;
        private volatile long blockMillis;
        private final CountDownLatch blocked = new CountDownLatch(1);

        @Override
        public void onNext(Command command) {
            if (inside.incrementAndGet() > 1) { overlap.set(true); }
            try {
                Cmd c = new Cmd(command);
                commands.add(c);
                Consumer<Cmd> hook = onRecord;
                if (hook != null) { hook.accept(c); }
                long b = blockMillis;
                if (b > 0 && command.getType() == CommandType.COMMAND_TYPE_REROUTE) {
                    blockMillis = 0;
                    blocked.countDown();
                    Thread.sleep(b);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                inside.decrementAndGet();
            }
        }

        @Override public void onError(Throwable t) { closed = true; }
        @Override public void onCompleted() { closed = true; }

        void blockNextRerouteFor(long millis) { blockMillis = millis; }
        boolean awaitBlocked(long millis) throws InterruptedException { return blocked.await(millis, TimeUnit.MILLISECONDS); }

        int size() { return commands.size(); }
        List<Cmd> snapshot() { synchronized (commands) { return new ArrayList<>(commands); } }
        Cmd last() { List<Cmd> s = snapshot(); return s.isEmpty() ? null : s.get(s.size() - 1); }
        List<Cmd> since(int n) { List<Cmd> s = snapshot(); return new ArrayList<>(s.subList(Math.min(n, s.size()), s.size())); }

        int countReroutesTo(double x, double y) {
            int n = 0;
            for (Cmd c : snapshot()) { if (c.is(CommandType.COMMAND_TYPE_REROUTE, x, y)) { n++; } }
            return n;
        }

        int countReroutesTo(double x, double y, int fromIndex) {
            int n = 0;
            for (Cmd c : since(fromIndex)) { if (c.is(CommandType.COMMAND_TYPE_REROUTE, x, y)) { n++; } }
            return n;
        }

        boolean idsStrictlyIncreasing() {
            long previous = Long.MIN_VALUE;
            for (Cmd c : snapshot()) {
                if (c.id <= previous) { return false; }
                previous = c.id;
            }
            return true;
        }
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    private static ExpiryPolicy policy(Duration transientSoft, Duration transientHard) {
        Map<ObstructionKind, ExpiryPolicy.Lifetime> m = new EnumMap<>(ObstructionKind.class);
        m.put(ObstructionKind.TRANSIENT, ExpiryPolicy.Lifetime.of(transientSoft, transientHard));
        m.put(ObstructionKind.SEMI_PERMANENT, ExpiryPolicy.Lifetime.of(Duration.ofSeconds(60), Duration.ofSeconds(600)));
        m.put(ObstructionKind.PERMANENT, ExpiryPolicy.Lifetime.permanent());
        m.put(ObstructionKind.UNKNOWN, ExpiryPolicy.Lifetime.of(Duration.ofSeconds(10), Duration.ofSeconds(100)));
        return new ExpiryPolicy(m, 20.0);
    }

    private static void checkCmd(String description, Cmd actual, CommandType type, Double x, Double y) {
        String expected = type.name().replace("COMMAND_TYPE_", "") + (x == null ? "" : " (" + x + ", " + y + ")");
        check(description + " -> " + expected + " (got " + actual + ")", actual != null && actual.is(type, x, y));
    }

    private static void checkIdsConsecutive(String label, FakeObserver obs) {
        List<Cmd> all = obs.snapshot();
        boolean ok = true;
        for (int i = 0; i < all.size(); i++) {
            if (all.get(i).id != i + 1) { ok = false; break; }
        }
        check(label + ": command ids strictly increase from 1 with no gaps and no duplicates (got "
                + all.stream().map(c -> String.valueOf(c.id)).reduce((a, b) -> a + "," + b).orElse("") + ")", ok);
    }

    private static void checkOutcome(String description, MissionManager.ReportOutcome actual, MissionManager.ReportOutcome expected) {
        check(description + " -> " + expected + " (got " + actual + ")", actual == expected);
    }

    private static void checkCount(String description, int actual, int expected) {
        check(description + " == " + expected + " (got " + actual + ")", actual == expected);
    }

    private static int mark() {
        return fullLog.size();
    }

    private static String segment(int since) {
        byte[] all = fullLog.toByteArray();
        return new String(all, since, all.length - since);
    }

    private static int count(String text, String needle) {
        int n = 0;
        for (int i = text.indexOf(needle); i >= 0; i = text.indexOf(needle, i + needle.length())) { n++; }
        return n;
    }

    private static void check(String description, boolean condition) {
        outcomes.add(condition);
        if (condition) {
            passCount++;
            System.out.println("  PASS: " + description);
        } else {
            failCount++;
            System.out.println("  FAIL: " + description);
        }
    }

    private static void header(String title) {
        System.out.println();
        System.out.println("=== " + title + " ===");
    }

    private static class TeeOutputStream extends OutputStream {
        private final OutputStream a;
        private final OutputStream b;

        TeeOutputStream(OutputStream a, OutputStream b) {
            this.a = a;
            this.b = b;
        }

        @Override
        public synchronized void write(int b1) throws IOException {
            a.write(b1);
            b.write(b1);
        }

        @Override
        public synchronized void flush() throws IOException {
            a.flush();
            b.flush();
        }
    }
}
