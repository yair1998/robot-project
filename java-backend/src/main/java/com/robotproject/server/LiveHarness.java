package com.robotproject.server;

import io.grpc.Server;
import io.grpc.ServerBuilder;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.robotproject.server.routing.MissionManager;
import com.robotproject.server.routing.ObstructionKind;
import com.robotproject.server.routing.RobotMission;
import com.robotproject.server.routing.WorldObstructionReport;

/**
 * A Java-only stand-in for the future perception process. It starts the SAME real gRPC server as RobotServer
 * (port 50051, one RobotChannelService) and a scripted injector that reads the live robot state through
 * MissionManager and RobotChannelService.debugVerdict, and injects obstructions at robot-driven moments.
 * A tool, not a test: no assertions of its own. Exit 0 once the mission is at its destination, 2 on timeout.
 */
public class LiveHarness {

    private static final String ROBOT = "robot-1";
    private static final Duration OVERALL_LIMIT = Duration.ofSeconds(150);
    private static final Duration MISSION_WAIT = Duration.ofSeconds(60);

    public static void main(String[] args) throws Exception {
        String scenario = args.length > 0 ? args[0] : "reroute_before_arrival";

        RobotChannelService service = new RobotChannelService();
        MissionManager mm = service.getMissionManager();
        Server server = ServerBuilder.forPort(50051).addService(service).build().start();
        System.out.println("Robot gRPC server listening on port 50051");
        System.out.println("[harness] scenario=" + scenario);

        Thread watchdog = new Thread(() -> {
            try {
                Thread.sleep(OVERALL_LIMIT.toMillis());
                System.out.println("[harness] no result after " + OVERALL_LIMIT.getSeconds() + " s - exiting with 2");
                System.exit(2);
            } catch (InterruptedException ignored) {
                // normal shutdown
            }
        }, "harness-watchdog");
        watchdog.setDaemon(true);
        watchdog.start();

        int code = runScenario(scenario, service, mm);
        server.shutdownNow();
        System.exit(code);
    }

    private static int runScenario(String scenario, RobotChannelService service, MissionManager mm)
            throws InterruptedException {
        if (!waitFor(() -> mm.missionStatus(ROBOT) != null, MISSION_WAIT)) {
            System.out.println("[harness] no mission for " + ROBOT + " within " + MISSION_WAIT.getSeconds() + " s");
            return 2;
        }
        long missionAt = System.nanoTime();
        System.out.println("[harness] mission for " + ROBOT + " exists");

        switch (scenario) {
            case "reroute_before_arrival":
                sleepUntil(missionAt + Duration.ofMillis(1500).toNanos());
                report(service, mm, new WorldObstructionReport("h1", 10.0, 4.5, 1.3, null,
                        ObstructionKind.SEMI_PERMANENT, null));
                break;
            case "retreat_now":
                waitFor(() -> onBD(mm, 2.0, 5.0), OVERALL_LIMIT);
                report(service, mm, WorldObstructionReport.basic("h2", 10.0, 8.1, 1.3));
                break;
            case "retreat_held":
                waitFor(() -> onBD(mm, 4.0, Double.MAX_VALUE)
                        && "STOP_PROXIMITY".equals(service.debugVerdict(ROBOT)), OVERALL_LIMIT);
                report(service, mm, WorldObstructionReport.basic("h3", 10.0, 8.1, 1.3));
                break;
            case "quiet_refresh":
                quietRefresh(service, mm);
                break;
            default:
                System.out.println("[harness] unknown scenario '" + scenario + "'");
                return 2;
        }

        waitFor(() -> mm.missionStatus(ROBOT) == RobotMission.Status.AT_DESTINATION, OVERALL_LIMIT);
        System.out.println("[harness] mission for " + ROBOT + " is AT_DESTINATION");
        Thread.sleep(2000);
        return 0;
    }

    /** n1 on C-E (off the planned route), 24 identical refreshes, one older report, a clear of an unknown id, then the real clear. */
    private static void quietRefresh(RobotChannelService service, MissionManager mm) throws InterruptedException {
        WorldObstructionReport n1 = new WorldObstructionReport("n1", 20.0, 5.0, 1.0, null,
                ObstructionKind.SEMI_PERMANENT, null);
        long start = System.nanoTime();
        report(service, mm, n1);

        List<long[]> schedule = new ArrayList<>();          // {millis after start, action}
        for (int i = 1; i <= 24; i++) { schedule.add(new long[] {i * 1000L, 0}); }
        schedule.add(new long[] {3000L, 1});
        schedule.add(new long[] {4000L, 2});
        schedule.add(new long[] {30000L, 3});
        schedule.sort(Comparator.comparingLong(e -> e[0]));

        for (long[] event : schedule) {
            sleepUntil(start + Duration.ofMillis(event[0]).toNanos());
            switch ((int) event[1]) {
                case 0:
                    report(service, mm, n1);
                    break;
                case 1:
                    report(service, mm, new WorldObstructionReport("n1", 20.0, 5.0, 1.0, null,
                            ObstructionKind.SEMI_PERMANENT, mm.now().minusSeconds(30)));
                    break;
                case 2:
                    clear(service, mm, "ghost");
                    break;
                default:
                    clear(service, mm, "n1");
                    break;
            }
        }
    }

    private static boolean onBD(MissionManager mm, double minY, double maxY) {
        double[] p = mm.lastKnownPosition(ROBOT);
        return p != null && Math.abs(p[0] - 10.0) < 0.5 && p[1] >= minY && p[1] <= maxY;
    }

    private static void report(RobotChannelService service, MissionManager mm, WorldObstructionReport r) {
        String state = state(service, mm);
        MissionManager.ReportOutcome outcome = mm.reportWorldObstruction(r);
        System.out.println("[harness] " + state + ": reporting " + r.eventId + " -> " + outcome);
    }

    private static void clear(RobotChannelService service, MissionManager mm, String eventId) {
        String state = state(service, mm);
        MissionManager.ClearOutcome outcome = mm.clearWorldObstruction(eventId);
        System.out.println("[harness] " + state + ": clearing " + eventId + " -> " + outcome);
    }

    private static String state(RobotChannelService service, MissionManager mm) {
        double[] p = mm.lastKnownPosition(ROBOT);
        String at = p == null ? "(unknown)" : String.format("(%.2f, %.2f)", p[0], p[1]);
        return ROBOT + " at " + at + " status=" + mm.missionStatus(ROBOT) + " verdict=" + service.debugVerdict(ROBOT);
    }

    private static boolean waitFor(BooleanSupplier condition, Duration timeout) throws InterruptedException {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            if (condition.getAsBoolean()) { return true; }
            Thread.sleep(10);
        }
        return condition.getAsBoolean();
    }

    private static void sleepUntil(long nanoTime) throws InterruptedException {
        long left = nanoTime - System.nanoTime();
        if (left > 0) { Thread.sleep(left / 1_000_000, (int) (left % 1_000_000)); }
    }
}
