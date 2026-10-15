package com.robotproject.server.routing;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Mission-state robustness: "no route" never means "complete", per-robot positions with receipt time
 * (stale -> deferred), report outcomes and quiet refreshes, and concurrent telemetry vs. reporter threads.
 * Pure JVM, no network. The whole demo runs three times in a row - concurrency tests must not be flaky.
 */
public class MissionRobustnessDemo {

    private static final int RUNS = 3;
    /** Seed of the randomized reporter in R9 and R10. Printed so any failure can be reproduced. */
    private static final long REPORTER_SEED = 7;

    private static int passCount = 0;
    private static int failCount = 0;

    // INFO only (not a pass/fail condition): how often the randomized R9/R10 reached the "stranded" (NO_ROUTE) condition.
    // Whether they reach it depends on how the host timer paces the reporter against the robots; the directed R11 is the
    // proof, and that all of its checks ran and passed IS checked (r11PassedRuns, R11_CHECKS).
    private static int strandedReached = 0;
    private static final List<Long> seedsUsed = new ArrayList<>();
    private static int r11PassedRuns = 0;
    /** The number of checks R11 executes; a run counts as passed only if exactly these all ran and passed. */
    private static final int R11_CHECKS = 9;

    private static ByteArrayOutputStream fullLog;
    private static PrintStream originalOut;
    private static PrintStream teeOut;

    public static void main(String[] args) throws Exception {
        originalOut = System.out;
        fullLog = new ByteArrayOutputStream();
        teeOut = new PrintStream(new TeeOutputStream(originalOut, fullLog), true);
        System.setOut(teeOut);

        int[] runFailures = new int[RUNS];
        try {
            for (int run = 1; run <= RUNS; run++) {
                header("RUN " + run + " of " + RUNS);
                int failuresBefore = failCount;
                r1_noRouteAtStart();
                r2_strandedAfterRetreat();
                r3_finishInFlightLegThenHold();
                r4_normalArrivalUnchanged();
                r5_twoRobotsOneCorridor();
                r6_latestRecordedPositionWins();
                r7_reportOutcomesAndQuietRefresh();
                r8_stalePositionDefers();
                r9_telemetryVsReporter();
                r10_twoRobotsPlusReporter();
                r11_strandedDirected();
                runFailures[run - 1] = failCount - failuresBefore;
            }
            header("STRANDED-CONDITION COVERAGE");
            System.out.println("INFO: condition 'stranded' (R9/R10) reached " + strandedReached + " times over "
                    + seedsUsed.size() + " seeded reporter runs (seeds " + seedsUsed + "). Not a pass/fail condition: "
                    + "the randomized runs do not reliably reach it; the directed R11 is the proof.");
            check("the directed stranded test R11 ran all " + R11_CHECKS + " of its checks and passed them in every run ("
                    + r11PassedRuns + " of " + RUNS + ")", r11PassedRuns == RUNS);
        } finally {
            System.setOut(originalOut);
        }

        System.out.println();
        for (int run = 1; run <= RUNS; run++) {
            System.out.println("run " + run + ": " + (runFailures[run - 1] == 0 ? "all passed" : runFailures[run - 1] + " failed"));
        }
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
    }

    // ------------------------------------------------------------------
    // R1 - no route at the start
    // ------------------------------------------------------------------
    private static void r1_noRouteAtStart() {
        header("R1: no route at the start - NO_ROUTE, hold, resume when a route appears");
        RouteGraph g = new RouteGraph();
        g.addNode(new GraphNode("P", 0.0, 0.0));
        g.addNode(new GraphNode("Q", 10.0, 0.0));
        g.addNode(new GraphNode("R", 20.0, 0.0));
        g.addEdge("P", "Q", true, 2.0);
        MissionManager mm = new MissionManager(g, new AStarRouter());

        int since = mark();
        check("onTelemetry(n1, 0, 0, R) returns null", mm.onTelemetry("n1", 0, 0, "R") == null);
        check("initialTargetFor(n1) is null", mm.initialTargetFor("n1") == null);
        checkStatus("status", mm, "n1", RobotMission.Status.NO_ROUTE);
        check("log contains \"No route from P to R\"", segment(since).contains("No route from P to R"));

        since = mark();
        mm.onTelemetry("n1", 0, 0, "R");
        check("a second tick logs nothing new (got \"" + segment(since).trim() + "\")", segment(since).isEmpty());

        g.addEdge("Q", "R", true, 2.0);
        since = mark();
        GraphNode next = mm.onTelemetry("n1", 0, 0, "R");
        checkTarget("after addEdge(Q,R): the next onTelemetry returns Q", next, "Q");
        checkStatus("status", mm, "n1", RobotMission.Status.EN_ROUTE);
        check("log contains \"exists again from P - resuming\"", segment(since).contains("exists again from P - resuming"));
        checkInvariantsEmpty(mm, "n1");
    }

    // ------------------------------------------------------------------
    // R2 - stranded after a retreat
    // ------------------------------------------------------------------
    private static void r2_strandedAfterRetreat() {
        header("R2: stranded after a retreat - NO_ROUTE at the retreat node, resume after a clear");
        RouteGraph g = tri();
        MissionManager mm = new MissionManager(g, new AStarRouter());

        mm.onTelemetry("r", 0, 0, "Y");
        checkTarget("initialTargetFor(r)", mm.initialTargetFor("r"), "Y");
        check("onTelemetry(r, 3, 0) returns null", mm.onTelemetry("r", 3, 0, "Y") == null);

        int since = mark();
        mm.reportEdgeObstruction("Z", "Y", "zy", 0.3, 0.5);
        String seg = segment(since);
        check("blocking Z-Y (not on the route X->Y): neither \"replanning ahead\" nor \"retreating\"",
                !seg.contains("replanning ahead") && !seg.contains("retreating"));

        since = mark();
        mm.reportEdgeObstruction("X", "Y", "xy", 0.3, 0.9);
        check("blocking X-Y at 0.9 (ahead): log contains \"retreating to X\"", segment(since).contains("retreating to X"));
        checkTarget("onTelemetry(r, 3, 0) returns X", mm.onTelemetry("r", 3, 0, "Y"), "X");
        check("the next call returns null", mm.onTelemetry("r", 3, 0, "Y") == null);

        since = mark();
        check("onTelemetry(r, 0, 0) at X returns null", mm.onTelemetry("r", 0, 0, "Y") == null);
        checkStatus("status", mm, "r", RobotMission.Status.NO_ROUTE);
        check("log contains \"no route to Y exists; holding position and retrying\"",
                segment(since).contains("no route to Y exists; holding position and retrying"));
        checkInvariantsEmpty(mm, "r");

        mm.clearEdgeObstruction("Z", "Y", "zy");
        since = mark();
        checkTarget("after clearing Z-Y: onTelemetry(r, 0, 0) returns Z", mm.onTelemetry("r", 0, 0, "Y"), "Z");
        checkStatus("status", mm, "r", RobotMission.Status.EN_ROUTE);
        check("log contains \"exists again from X - resuming\"", segment(since).contains("exists again from X - resuming"));
        checkTarget("onTelemetry(r, 10, 10) returns Y", mm.onTelemetry("r", 10, 10, "Y"), "Y");
        check("onTelemetry(r, 10, 0) returns null", mm.onTelemetry("r", 10, 0, "Y") == null);
        checkStatus("status", mm, "r", RobotMission.Status.AT_DESTINATION);
    }

    // ------------------------------------------------------------------
    // R3 - finish the in-flight leg, then hold
    // ------------------------------------------------------------------
    private static void r3_finishInFlightLegThenHold() {
        header("R3: no route onward - finish the in-flight leg, hold at its end, resume after a clear");
        RouteGraph g = tri();
        MissionManager mm = new MissionManager(g, new AStarRouter());

        mm.reportEdgeObstruction("X", "Y", "xy", 0.3, 0.5);
        mm.onTelemetry("r", 0, 0, "Y");
        checkTarget("initialTargetFor(r) (X-Y blocked, route via Z)", mm.initialTargetFor("r"), "Z");
        check("onTelemetry(r, 5, 5) returns null", mm.onTelemetry("r", 5, 5, "Y") == null);

        int since = mark();
        mm.reportEdgeObstruction("Z", "Y", "zy", 0.3, 0.5);
        String seg = segment(since);
        check("log contains \"replanning ahead\"", seg.contains("replanning ahead"));
        check("log contains \"No route to Y from Z right now\"", seg.contains("No route to Y from Z right now"));
        checkStatus("still", mm, "r", RobotMission.Status.EN_ROUTE);

        since = mark();
        check("onTelemetry(r, 10, 10) at Z returns null", mm.onTelemetry("r", 10, 10, "Y") == null);
        seg = segment(since);
        check("log contains \"confirmed at Z\" and \"holding position and retrying\"",
                seg.contains("confirmed at Z") && seg.contains("holding position and retrying"));
        checkStatus("status", mm, "r", RobotMission.Status.NO_ROUTE);

        mm.clearEdgeObstruction("Z", "Y", "zy");
        checkTarget("after clearing Z-Y: onTelemetry(r, 10, 10) returns Y", mm.onTelemetry("r", 10, 10, "Y"), "Y");
        mm.onTelemetry("r", 10, 0, "Y");
        checkStatus("onTelemetry(r, 10, 0): status", mm, "r", RobotMission.Status.AT_DESTINATION);
    }

    // ------------------------------------------------------------------
    // R4 - normal arrival unchanged
    // ------------------------------------------------------------------
    private static void r4_normalArrivalUnchanged() {
        header("R4: normal arrival is unchanged");
        RouteGraph g = tri();
        MissionManager mm = new MissionManager(g, new AStarRouter());
        int since = mark();
        mm.onTelemetry("r", 0, 0, "Y");
        checkTarget("initialTargetFor(r)", mm.initialTargetFor("r"), "Y");
        check("arrival tick returns null", mm.onTelemetry("r", 10, 0, "Y") == null);
        checkStatus("status", mm, "r", RobotMission.Status.AT_DESTINATION);
        check("a later tick returns null", mm.onTelemetry("r", 10, 0, "Y") == null);
        check("another later tick returns null", mm.onTelemetry("r", 10, 0, "Y") == null);
        check("the log never contains \"holding position\"", !segment(since).contains("holding position"));
    }

    // ------------------------------------------------------------------
    // R5 - two robots, one corridor, two decisions
    // ------------------------------------------------------------------
    private static void r5_twoRobotsOneCorridor() throws IOException {
        header("R5: two robots on one corridor - each decided from its OWN latest position");
        MissionManager mm = new MissionManager(map(), new AStarRouter());

        mm.onTelemetry("r1", 0, 0, "E");
        checkTarget("r1 initialTargetFor", mm.initialTargetFor("r1"), "B");
        checkTarget("r1 at B gets D", mm.onTelemetry("r1", 10, 0, "E"), "D");
        check("r1 at (10, 6.3) (progress 0.7) returns null", mm.onTelemetry("r1", 10, 6.3, "E") == null);

        mm.onTelemetry("r2", 10, 1.8, "E");
        checkTarget("r2 (created at (10, 1.8): a B->D->E mission) initialTargetFor", mm.initialTargetFor("r2"), "D");

        int since = mark();
        mm.reportEdgeObstruction("B", "D", "shared", 0.3, 0.5);
        String seg = segment(since);
        checkCount("\"continuing forward\" in the new log", count(seg, "continuing forward"), 1);
        checkCount("\"retreating to B\" in the new log", count(seg, "retreating to B"), 1);
        check("r1 at (10, 6.3) returns null", mm.onTelemetry("r1", 10, 6.3, "E") == null);
        checkStatus("r1", mm, "r1", RobotMission.Status.EN_ROUTE);
        checkTarget("r2 at (10, 1.8) returns B (its retreat)", mm.onTelemetry("r2", 10, 1.8, "E"), "B");
    }

    // ------------------------------------------------------------------
    // R6 - the latest recorded position wins
    // ------------------------------------------------------------------
    private static void r6_latestRecordedPositionWins() throws IOException {
        header("R6: the latest recorded position wins");
        MissionManager a = new MissionManager(map(), new AStarRouter());
        a.onTelemetry("r1", 0, 0, "E");
        a.onTelemetry("r1", 10, 0, "E");
        a.onTelemetry("r1", 10, 1.8, "E");
        int since = mark();
        a.reportEdgeObstruction("B", "D", "blk", 0.3, 0.5);
        check("(a) last telemetry at (10, 1.8): \"retreating to B\"", segment(since).contains("retreating to B"));

        MissionManager b = new MissionManager(map(), new AStarRouter());
        b.onTelemetry("r1", 0, 0, "E");
        b.onTelemetry("r1", 10, 0, "E");
        b.onTelemetry("r1", 10, 1.8, "E");
        b.updateRobotPosition("r1", 10, 6.3);
        since = mark();
        b.reportEdgeObstruction("B", "D", "blk", 0.3, 0.5);
        check("(b) updateRobotPosition(r1, 10, 6.3) after it: \"continuing forward\"",
                segment(since).contains("continuing forward"));
    }

    // ------------------------------------------------------------------
    // R7 - report outcomes and quiet refresh
    // ------------------------------------------------------------------
    private static void r7_reportOutcomesAndQuietRefresh() throws IOException {
        header("R7: report outcomes and the quiet refresh");
        MutableClock clock = new MutableClock(Instant.parse("2026-01-01T00:00:00Z"));
        RouteGraph g = map();
        g.setClock(clock);
        MissionManager mm = new MissionManager(g, new AStarRouter());
        ObstructionKind semi = ObstructionKind.SEMI_PERMANENT;

        WorldObstructionReport o = new WorldObstructionReport("o", 10.0, 4.5, 1.0, null, semi, null);
        checkOutcome("\"o\" SEMI_PERMANENT at (10.0, 4.5) width 1.0", mm.reportWorldObstruction(o),
                MissionManager.ReportOutcome.APPLIED);
        int since = mark();
        checkOutcome("identical again", mm.reportWorldObstruction(o), MissionManager.ReportOutcome.REFRESHED);
        String seg = segment(since);
        check("the refresh logs no \"mapped to edge\" and no \"WARNING\"",
                !seg.contains("mapped to edge") && !seg.contains("WARNING"));
        checkOutcome("same with observedAt now-30s",
                mm.reportWorldObstruction(new WorldObstructionReport("o", 10.0, 4.5, 1.0, null, semi,
                        clock.instant().minusSeconds(30))), MissionManager.ReportOutcome.IGNORED_OLDER);
        checkOutcome("width 1.5",
                mm.reportWorldObstruction(new WorldObstructionReport("o", 10.0, 4.5, 1.5, null, semi, null)),
                MissionManager.ReportOutcome.APPLIED);
        checkClear("clearWorldObstruction(o)", mm.clearWorldObstruction("o"), MissionManager.ClearOutcome.CLEARED);
        checkOutcome("report observed now-5s after the clear",
                mm.reportWorldObstruction(new WorldObstructionReport("o", 10.0, 4.5, 1.0, null, semi,
                        clock.instant().minusSeconds(5))), MissionManager.ReportOutcome.IGNORED_AFTER_CLEAR);
        checkOutcome("(500, 500)", mm.reportWorldObstruction("far", 500.0, 500.0, 1.0),
                MissionManager.ReportOutcome.IGNORED_TOO_FAR);
        checkOutcome("TRANSIENT observed now-60s",
                mm.reportWorldObstruction(new WorldObstructionReport("t", 10.0, 4.5, 1.0, null,
                        ObstructionKind.TRANSIENT, clock.instant().minusSeconds(60))),
                MissionManager.ReportOutcome.IGNORED_EXPIRED);
        checkClear("clearWorldObstruction(never)", mm.clearWorldObstruction("never"),
                MissionManager.ClearOutcome.RECORDED_ONLY);
        checkOutcome("\"o2\" reported", mm.reportWorldObstruction(
                new WorldObstructionReport("o2", 10.0, 4.5, 1.0, null, semi, null)), MissionManager.ReportOutcome.APPLIED);
        checkClear("clearWorldObstruction(o2, now-100s)",
                mm.clearWorldObstruction("o2", clock.instant().minusSeconds(100)), MissionManager.ClearOutcome.IGNORED_OLDER);
        checkClear("clearWorldObstruction(o2)", mm.clearWorldObstruction("o2"), MissionManager.ClearOutcome.CLEARED);

        // A robot on that corridor, a blocking obstruction already reported: a refresh is quiet.
        RouteGraph g2 = map();
        g2.setClock(clock);
        MissionManager m2 = new MissionManager(g2, new AStarRouter());
        m2.onTelemetry("r", 0, 0, "E");
        m2.onTelemetry("r", 10, 0, "E");
        m2.onTelemetry("r", 10, 1.8, "E");
        WorldObstructionReport blk = new WorldObstructionReport("blk", 10.0, 8.1, 1.3, null, semi, null);
        since = mark();
        checkOutcome("robot on B->D: blocking report ahead", m2.reportWorldObstruction(blk),
                MissionManager.ReportOutcome.APPLIED);
        check("it starts a retreat", segment(since).contains("retreating to B"));
        since = mark();
        checkOutcome("the same report again", m2.reportWorldObstruction(blk), MissionManager.ReportOutcome.REFRESHED);
        seg = segment(since);
        check("the refresh produces no new \"WARNING\" or \"retreat\" text",
                !seg.contains("WARNING") && !seg.contains("retreat"));
    }

    // ------------------------------------------------------------------
    // R8 - stale position defers, the next telemetry decides
    // ------------------------------------------------------------------
    private static void r8_stalePositionDefers() throws IOException {
        header("R8: a stale position defers the decision; the next telemetry decides");
        MutableClock clock = new MutableClock(Instant.parse("2026-01-01T00:00:00Z"));

        MissionManager a = robotOnBD(clock, 1.8);
        clock.advance(Duration.ofSeconds(5));
        int since = mark();
        a.reportEdgeObstruction("B", "D", "ahead", 0.3, 0.9);
        String seg = segment(since);
        check("blocking report ahead while stale: \"has no fresh position\" and \"is deferred\"",
                seg.contains("has no fresh position") && seg.contains("is deferred"));
        check("no retreat yet", !seg.contains("retreating"));
        since = mark();
        checkTarget("onTelemetry at a fresh position returns B in the SAME call", a.onTelemetry("r", 10, 1.8, "E"), "B");
        check("the log then has \"retreating to B\"", segment(since).contains("retreating to B"));

        clock.set(Instant.parse("2026-01-01T00:00:00Z"));
        MissionManager b = robotOnBD(clock, 1.8);
        clock.advance(Duration.ofSeconds(5));
        b.reportEdgeObstruction("B", "D", "behind", 0.3, 0.1);
        since = mark();
        check("obstruction BEHIND, next telemetry at progress 0.7 returns null",
                b.onTelemetry("r", 10, 6.3, "E") == null);
        check("then \"continuing forward\"", segment(since).contains("continuing forward"));

        clock.set(Instant.parse("2026-01-01T00:00:00Z"));
        MissionManager c = robotOnBD(clock, 1.8);
        clock.advance(Duration.ofSeconds(5));
        since = mark();
        c.reportEdgeObstruction("D", "E", "f1", 0.3, 0.5);
        c.reportEdgeObstruction("B", "D", "f2", 0.3, 0.9);
        checkCount("two reports on two corridors while stale: \"is deferred\" lines", count(segment(since), "is deferred"), 2);
        since = mark();
        c.onTelemetry("r", 10, 1.8, "E");
        seg = segment(since);
        check("both are decided at the next telemetry (\"replanning ahead\" and \"retreating to B\")",
                seg.contains("replanning ahead") && seg.contains("retreating to B"));
    }

    private static MissionManager robotOnBD(MutableClock clock, double y) throws IOException {
        RouteGraph g = map();
        g.setClock(clock);
        MissionManager mm = new MissionManager(g, new AStarRouter());
        mm.onTelemetry("r", 0, 0, "E");
        mm.onTelemetry("r", 10, 0, "E");
        mm.onTelemetry("r", 10, y, "E");
        return mm;
    }

    // ------------------------------------------------------------------
    // R9 / R10 - concurrent telemetry vs. reporter (real clock)
    // ------------------------------------------------------------------
    private static void r9_telemetryVsReporter() throws Exception {
        header("R9: one robot's telemetry thread vs. a reporter thread (real clock)");
        RouteGraph g = map();
        g.setExpiryPolicy(fastPolicy());
        MissionManager mm = new MissionManager(g, new AStarRouter());
        RobotRunner c1 = new RobotRunner(mm, "c1", 0, 0, "E");
        Reporter reporter = new Reporter(mm, g, new Random(REPORTER_SEED));
        runConcurrently(new Thread[] {new Thread(c1, "r9-robot"), new Thread(reporter, "r9-reporter")});
        System.out.println("seed " + REPORTER_SEED + "; reports: " + reporter.calls.get() + ", ticks: " + c1.ticks
                + ", stranded: " + c1.strandedEpisodes + ", robot ran " + c1.elapsedMillis + " ms ("
                + String.format("%.2f", c1.elapsedMillis / (double) Math.max(1, c1.ticks)) + " ms per tick)");
        strandedReached += c1.strandedEpisodes;
        seedsUsed.add(REPORTER_SEED);
        assertRunner(c1);
        check("no exception in the reporter (got " + reporter.error.get() + ")", reporter.error.get() == null);
        check("the reporter made more than 100 calls (got " + reporter.calls.get() + ")", reporter.calls.get() > 100);
    }

    private static void r10_twoRobotsPlusReporter() throws Exception {
        header("R10: two robots on two telemetry threads plus a reporter (real clock)");
        RouteGraph g = map();
        g.setExpiryPolicy(fastPolicy());
        MissionManager mm = new MissionManager(g, new AStarRouter());
        RobotRunner r1 = new RobotRunner(mm, "r1", 0, 0, "E");
        RobotRunner r2 = new RobotRunner(mm, "r2", 20, 0, "A");
        Reporter reporter = new Reporter(mm, g, new Random(REPORTER_SEED));
        runConcurrently(new Thread[] {new Thread(r1, "r10-r1"), new Thread(r2, "r10-r2"), new Thread(reporter, "r10-reporter")});
        System.out.println("seed " + REPORTER_SEED + "; reports: " + reporter.calls.get() + ", r1 ticks: " + r1.ticks
                + " (stranded " + r1.strandedEpisodes + ", " + r1.elapsedMillis + " ms), r2 ticks: " + r2.ticks
                + " (stranded " + r2.strandedEpisodes + ", " + r2.elapsedMillis + " ms)");
        strandedReached += r1.strandedEpisodes + r2.strandedEpisodes;
        seedsUsed.add(REPORTER_SEED);
        assertRunner(r1);
        assertRunner(r2);
        check("no exception in the reporter (got " + reporter.error.get() + ")", reporter.error.get() == null);
        check("the reporter made more than 100 calls (got " + reporter.calls.get() + ")", reporter.calls.get() > 100);
    }

    /**
     * R11 (directed): constructs the R9/R10 "stranded" condition on purpose, single-threaded, with the same RobotRunner
     * step() and episode counter. B-D and B-C are blocked while the robot is on A->B, so it finishes the leg and stands
     * at B with NO_ROUTE; clearing B-C must resume it to E. No threads, no sleeps.
     */
    private static void r11_strandedDirected() throws IOException {
        header("R11 (directed): a robot is stranded on purpose, counted once, and resumes after a clear");
        int failuresBefore = failCount;
        int checksBefore = passCount + failCount;
        MissionManager mm = new MissionManager(map(), new AStarRouter());
        RobotRunner s1 = new RobotRunner(mm, "s1", 0, 0, "E");
        s1.start();
        boolean healthy = true;
        int guard = 0;
        while (s1.x < 5.0 && guard++ < 1000) {
            s1.step();
            healthy &= mm.checkInvariants("s1").isEmpty();
        }
        check("s1 is on A->B (at " + String.format("(%.2f, %.2f)", s1.x, s1.y) + ")", s1.x >= 5.0 && s1.y == 0.0);

        int since = mark();
        mm.reportEdgeObstruction("B", "D", "s-bd", 0.3, 0.5);
        mm.reportEdgeObstruction("B", "C", "s-bc", 0.3, 0.5);
        check("blocking B-D and B-C logs \"No route to E from B right now\"",
                segment(since).contains("No route to E from B right now"));

        guard = 0;
        while (mm.missionStatus("s1") != RobotMission.Status.NO_ROUTE && guard++ < 2000) {
            s1.step();
            healthy &= mm.checkInvariants("s1").isEmpty();
        }
        checkStatus("after finishing the leg", mm, "s1", RobotMission.Status.NO_ROUTE);
        check("s1 stands at B (at " + String.format("(%.2f, %.2f)", s1.x, s1.y) + ")",
                Math.hypot(s1.x - 10.0, s1.y) < 0.5);
        checkCount("stranded episodes counted", s1.strandedEpisodes, 1);
        for (int i = 0; i < 20; i++) {
            s1.step();
            healthy &= mm.checkInvariants("s1").isEmpty();
        }
        check("20 more ticks at B: still NO_ROUTE and still ONE episode (got " + s1.strandedEpisodes + ")",
                mm.missionStatus("s1") == RobotMission.Status.NO_ROUTE && s1.strandedEpisodes == 1);

        mm.clearEdgeObstruction("B", "C", "s-bc");
        boolean reached = false;
        guard = 0;
        while (!reached && guard++ < 5000) {
            reached = s1.step();
            healthy &= mm.checkInvariants("s1").isEmpty();
        }
        check("after clearing B-C, s1 resumes and reaches AT_DESTINATION", reached
                && mm.missionStatus("s1") == RobotMission.Status.AT_DESTINATION);
        checkCount("stranded episodes in total", s1.strandedEpisodes, 1);
        check("checkInvariants(s1) was empty at every tick", healthy);
        if (failCount == failuresBefore && passCount + failCount - checksBefore == R11_CHECKS) {
            r11PassedRuns++;
        }
    }

    private static void assertRunner(RobotRunner r) {
        check(r.id + ": no exception (got " + r.error.get() + ")", r.error.get() == null);
        check(r.id + ": no invariant violation (first: " + r.violation.get() + ")", r.violation.get() == null);
        check(r.id + ": reaches AT_DESTINATION", r.reached);
    }

    /** Starts the threads with System.out silenced, joins with a 90 s watchdog, restores System.out. */
    private static void runConcurrently(Thread[] threads) throws InterruptedException {
        System.setOut(new PrintStream(OutputStream.nullOutputStream()));
        boolean timedOut = false;
        try {
            for (Thread t : threads) { t.start(); }
            long deadline = System.nanoTime() + Duration.ofSeconds(90).toNanos();
            for (Thread t : threads) {
                long left = Math.max(1, (deadline - System.nanoTime()) / 1_000_000);
                t.join(left);
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
        check("watchdog: every thread finished within 90 s", !timedOut);
    }

    /** A simulated robot: ticks, follows the last returned target at <= 0.02 m per step, checks invariants every 50 steps. */
    private static final class RobotRunner implements Runnable {
        final MissionManager mm;
        final String id;
        final String destination;
        double x;
        double y;
        volatile boolean reached;
        volatile int ticks;
        volatile int strandedEpisodes;
        volatile long elapsedMillis;
        final AtomicReference<Throwable> error = new AtomicReference<>();
        final AtomicReference<String> violation = new AtomicReference<>();
        private GraphNode target;
        private RobotMission.Status previous;

        RobotRunner(MissionManager mm, String id, double x, double y, String destination) {
            this.mm = mm; this.id = id; this.x = x; this.y = y; this.destination = destination;
        }

        @Override
        public void run() {
            long started = System.nanoTime();
            try {
                start();
                long deadline = System.nanoTime() + Duration.ofSeconds(60).toNanos();
                int iteration = 0;
                while (System.nanoTime() < deadline) {
                    if (step()) { break; }
                    Thread.sleep(2);
                    if (++iteration % 50 == 0) {
                        List<String> v = mm.checkInvariants(id);
                        if (!v.isEmpty()) { violation.compareAndSet(null, id + ": " + v); }
                    }
                }
            } catch (Throwable t) {
                error.set(t);
            } finally {
                elapsedMillis = (System.nanoTime() - started) / 1_000_000;
            }
        }

        /** First tick, then the initial target - exactly what the threaded loop does before it starts stepping. */
        void start() {
            target = mm.onTelemetry(id, x, y, destination);
            GraphNode initial = mm.initialTargetFor(id);
            if (initial != null) { target = initial; }
        }

        /**
         * One iteration of the robot loop: tick, adopt a returned target, count a new NO_ROUTE ("stranded") episode,
         * move at most 0.02 m. Returns true once the mission is AT_DESTINATION. Used by the threaded loop and by R11.
         */
        boolean step() {
            GraphNode next = mm.onTelemetry(id, x, y, destination);
            ticks++;
            if (next != null) { target = next; }
            RobotMission.Status status = mm.missionStatus(id);
            if (status == RobotMission.Status.AT_DESTINATION) { reached = true; return true; }
            if (status == RobotMission.Status.NO_ROUTE && previous != RobotMission.Status.NO_ROUTE) {
                strandedEpisodes++;
            }
            previous = status;
            if (target != null) { stepToward(target, 0.02); }
            return false;
        }

        private void stepToward(GraphNode target, double step) {
            double dx = target.getX() - x;
            double dy = target.getY() - y;
            double d = Math.hypot(dx, dy);
            if (d <= step) { x = target.getX(); y = target.getY(); }
            else { x += dx / d * step; y += dy / d * step; }
        }
    }

    /** For 3 s, about every 5 ms: 70% report one of 8 ids near a random corridor, 30% clear it. Then clear all 8. */
    private static final class Reporter implements Runnable {
        final MissionManager mm;
        final Random random;
        final GraphNode[][] corridors;
        final AtomicInteger calls = new AtomicInteger();
        final AtomicReference<Throwable> error = new AtomicReference<>();

        Reporter(MissionManager mm, RouteGraph g, Random random) {
            this.mm = mm;
            this.random = random;
            String[][] ids = {{"A", "B"}, {"B", "C"}, {"B", "D"}, {"D", "E"}, {"C", "E"}};
            corridors = new GraphNode[ids.length][];
            for (int i = 0; i < ids.length; i++) {
                corridors[i] = new GraphNode[] {g.getNode(ids[i][0]), g.getNode(ids[i][1])};
            }
        }

        @Override
        public void run() {
            double[] widths = {0.4, 0.9, 1.3};
            try {
                long end = System.nanoTime() + Duration.ofSeconds(3).toNanos();
                while (System.nanoTime() < end) {
                    String id = "ev" + random.nextInt(8);
                    if (random.nextDouble() < 0.7) {
                        GraphNode[] c = corridors[random.nextInt(corridors.length)];
                        double f = 0.1 + 0.8 * random.nextDouble();
                        double ex = c[1].getX() - c[0].getX();
                        double ey = c[1].getY() - c[0].getY();
                        double len = Math.hypot(ex, ey);
                        double offset = (random.nextDouble() * 2 - 1) * 0.2;
                        double wx = c[0].getX() + f * ex - ey / len * offset;
                        double wy = c[0].getY() + f * ey + ex / len * offset;
                        ObstructionKind kind = random.nextBoolean() ? ObstructionKind.TRANSIENT : ObstructionKind.SEMI_PERMANENT;
                        mm.reportWorldObstruction(new WorldObstructionReport(id, wx, wy,
                                widths[random.nextInt(widths.length)], null, kind, null));
                    } else {
                        mm.clearWorldObstruction(id);
                    }
                    calls.incrementAndGet();
                    Thread.sleep(5);
                }
                for (int i = 0; i < 8; i++) {
                    mm.clearWorldObstruction("ev" + i);
                    calls.incrementAndGet();
                }
            } catch (Throwable t) {
                error.set(t);
            }
        }
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    private static RouteGraph tri() {
        RouteGraph g = new RouteGraph();
        g.addNode(new GraphNode("X", 0.0, 0.0));
        g.addNode(new GraphNode("Y", 10.0, 0.0));
        g.addNode(new GraphNode("Z", 10.0, 10.0));
        g.addEdge("X", "Y", true, 2.0);
        g.addEdge("X", "Z", true, 2.0);
        g.addEdge("Z", "Y", true, 2.0);
        return g;
    }

    private static RouteGraph map() throws IOException {
        return MapLoader.loadFromClasspath("maps/mission_map.json");
    }

    private static ExpiryPolicy fastPolicy() {
        Map<ObstructionKind, ExpiryPolicy.Lifetime> m = new EnumMap<>(ObstructionKind.class);
        m.put(ObstructionKind.TRANSIENT, ExpiryPolicy.Lifetime.of(Duration.ofMillis(300), Duration.ofMillis(1500)));
        m.put(ObstructionKind.SEMI_PERMANENT, ExpiryPolicy.Lifetime.of(Duration.ofSeconds(2), Duration.ofSeconds(6)));
        m.put(ObstructionKind.PERMANENT, ExpiryPolicy.Lifetime.permanent());
        m.put(ObstructionKind.UNKNOWN, ExpiryPolicy.Lifetime.of(Duration.ofSeconds(1), Duration.ofSeconds(3)));
        return new ExpiryPolicy(m, 25.0);
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

    private static void checkTarget(String description, GraphNode actual, String expectedId) {
        check(description + " is " + expectedId + " (got " + (actual == null ? "null" : actual.getId()) + ")",
                actual != null && expectedId.equals(actual.getId()));
    }

    private static void checkStatus(String description, MissionManager mm, String robotId, RobotMission.Status expected) {
        RobotMission.Status s = mm.missionStatus(robotId);
        check(description + ": " + robotId + " is " + expected + " (got " + s + ")", s == expected);
    }

    private static void checkOutcome(String description, MissionManager.ReportOutcome actual,
            MissionManager.ReportOutcome expected) {
        check(description + " -> " + expected + " (got " + actual + ")", actual == expected);
    }

    private static void checkClear(String description, MissionManager.ClearOutcome actual,
            MissionManager.ClearOutcome expected) {
        check(description + " -> " + expected + " (got " + actual + ")", actual == expected);
    }

    private static void checkCount(String description, int actual, int expected) {
        check(description + " == " + expected + " (got " + actual + ")", actual == expected);
    }

    private static void checkInvariantsEmpty(MissionManager mm, String robotId) {
        List<String> v = mm.checkInvariants(robotId);
        check("checkInvariants(" + robotId + ") is empty (got " + v + ")", v.isEmpty());
    }

    private static void check(String description, boolean condition) {
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
        public void write(int b1) throws IOException {
            a.write(b1);
            b.write(b1);
        }

        @Override
        public void flush() throws IOException {
            a.flush();
            b.flush();
        }
    }
}
