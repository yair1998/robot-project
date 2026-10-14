package com.robotproject.server.routing;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.List;

public class MissionDemo {

    private static final double STEP_SIZE_METERS = 1.0;
    private static final double ARRIVAL_THRESHOLD_METERS = 0.5;

    private static int passCount = 0;
    private static int failCount = 0;

    private static ByteArrayOutputStream fullLog;
    private static PrintStream originalOut;

    public static void main(String[] args) throws IOException {

        originalOut = System.out;
        fullLog = new ByteArrayOutputStream();
        System.setOut(new PrintStream(new TeeOutputStream(originalOut, fullLog), true));

        try {
            scenarioA_fullyBlockedCurrentEdgeRetreats();
            scenarioB_futureEdgeBlockDefersThenBacktracks();
            scenarioC_degradedButPassableFinishesLeg();
            scenarioD_locationAwareBlockageChoosesDirection();
            scenarioE_signalOnRetreatEdgeIsRecognized();
            scenarioF_bothDirectionsBlockedIsFlaggedNotHandled();
            scenarioH_crossDirectionReportAffectsLiveRobot();
            scenarioI_bothDirectionsBlockedAtInitialDecision();
            scenarioJ_widthAffectsRouteCostDirectly();
            scenarioK_realisticWorldCoordinateReportDrivesCorrectReaction();
        } finally {
            System.setOut(originalOut);
        }

        System.out.println();
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");

        if (failCount > 0) {
            System.exit(1);
        }
    }

    // SCENARIO A: the robot's CURRENT edge (B->D) becomes fully blocked
    // while it is mid-way along it, well before arriving at D. The blockage
    // is reported at 0.9 along B->D, ahead of the robot, so forward
    // progress is not safe - the robot must retreat to the last
    // confirmed node (B), and only after that retreat is physically
    // confirmed does MissionManager replan from B (which, with B->D now
    // excluded, must go via C).
    private static void scenarioA_fullyBlockedCurrentEdgeRetreats() throws IOException {
        header("SCENARIO A: current edge (B->D) fully blocked mid-flight -> retreat to B, then reroute via C");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        AStarRouter router = new AStarRouter();
        MissionManager mm = new MissionManager(graph, router);

        String robotId = "robot-a";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        check("initial dispatched target is B", initial != null && "B".equals(initial.getId()));

        GraphNode afterB = driveUntilNextTarget(mm, robotId, pos, initial, 50);
        check("after arriving at B, mission dispatches D next", afterB != null && "D".equals(afterB.getId()));

        // Move a little way into the B->D edge - well before arrival - then
        // fully block that edge while it is the robot's CURRENT edge.
        moveToward(pos, graph.getNode("D"), 2.0);
        System.out.println("Robot now at (" + fmt(pos[0]) + ", " + fmt(pos[1])
                + "), partway along B->D (edge length 9m, ~2m traveled). Fully blocking B->D...");

        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("B", "D", "a-block", 0.3, 0.9);
        String segment = newSegment(before);
        check("fully blocking the CURRENT edge (B->D) logs a retreat to B, not a mid-edge continue",
                segment.contains("retreating to B"));

        // The retreat target must be dispatched immediately (one-shot),
        // even though the robot is still physically far from B - this is
        // the exact bug the correction requires fixing: onTelemetry()
        // must proactively hand back B, not silently wait for an arrival
        // check against a target the robot was never told to head for.
        GraphNode retreatTarget = mm.onTelemetry(robotId, pos[0], pos[1], "E");
        System.out.println("New target: " + describe(retreatTarget));
        check("next dispatched target is B itself (retreat), not D",
                retreatTarget != null && "B".equals(retreatTarget.getId()));

        GraphNode repeated = mm.onTelemetry(robotId, pos[0], pos[1], "E");
        check("retreat target is dispatched only once (one-shot), not re-issued every tick",
                repeated == null);

        // Drive the robot physically back to B's real coordinates.
        GraphNode afterRetreat = driveUntilNextTarget(mm, robotId, pos, graph.getNode("B"), 50);
        check("physical arrival back at B is detected and mission replans "
                        + "(next target is C - the only remaining route to E since B->D is excluded)",
                afterRetreat != null && "C".equals(afterRetreat.getId()));

        GraphNode afterC = driveUntilNextTarget(mm, robotId, pos, afterRetreat, 50);
        check("after C, next target is E", afterC != null && "E".equals(afterC.getId()));

        before = fullLog.size();
        boolean reachedE = driveToCompletion(mm, robotId, pos, afterC, 50);
        String finalSegment = newSegment(before);
        check("mission completes at E after the retreat-and-reroute",
                reachedE && finalSegment.contains("confirmed at E"));
    }

    // SCENARIO B: a FUTURE edge (D->E) is blocked while the robot is still
    // mid-flight on its CURRENT edge (B->D). Since the blocked edge isn't
    // the one the robot is on, there is nothing unsafe about continuing -
    // the robot finishes B->D uninterrupted, and only once it arrives at D
    // does the mission discover D->E is gone and reroute, which (D only
    // connects to B and E) means backtracking through B and C.
    private static void scenarioB_futureEdgeBlockDefersThenBacktracks() {
        header("SCENARIO B: future edge (D->E) blocked mid-flight on B->D -> finish to D, then backtrack via B, C");

        RouteGraph graph;
        try {
            graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        MissionManager mm = new MissionManager(graph, new AStarRouter());

        String robotId = "robot-b";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        check("initial dispatched target is B", initial != null && "B".equals(initial.getId()));

        GraphNode afterB = driveUntilNextTarget(mm, robotId, pos, initial, 50);
        check("after arriving at B, mission dispatches D next", afterB != null && "D".equals(afterB.getId()));

        moveToward(pos, graph.getNode("D"), 2.0);
        System.out.println("Robot now at (" + fmt(pos[0]) + ", " + fmt(pos[1])
                + "), partway along B->D. Blocking the FUTURE edge D->E (robot is not on it yet)...");

        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("D", "E", "b-block", 0.3, 0.5);
        String segment = newSegment(before);
        check("blocking a FUTURE edge logs a forward replan, not a retreat",
                segment.contains("A future edge on the planned route changed"));
        check("blocking a FUTURE edge does not trigger retreat",
                !segment.contains("retreating"));

        // The robot must NOT be interrupted mid-edge - it keeps heading to D.
        GraphNode afterD = driveUntilNextTarget(mm, robotId, pos, graph.getNode("D"), 50);
        check("robot finishes B->D uninterrupted, and the backtrack begins: next target is B",
                afterD != null && "B".equals(afterD.getId()));

        GraphNode afterBacktrackToB = driveUntilNextTarget(mm, robotId, pos, afterD, 50);
        check("after backtracking to B, next target is C",
                afterBacktrackToB != null && "C".equals(afterBacktrackToB.getId()));

        GraphNode afterC = driveUntilNextTarget(mm, robotId, pos, afterBacktrackToB, 50);
        check("after C, next target is E", afterC != null && "E".equals(afterC.getId()));

        before = fullLog.size();
        boolean reachedE = driveToCompletion(mm, robotId, pos, afterC, 50);
        String finalSegment = newSegment(before);
        check("mission completes at E after the deferred backtrack",
                reachedE && finalSegment.contains("confirmed at E"));
    }

    // SCENARIO C: the robot's CURRENT edge (A->B) is degraded but not fully
    // blocked. The policy always finishes a leg the robot is already
    // committed to when it's still physically passable - there is no
    // distance-based grace window anymore, only a pass/fail capacity check.
    private static void scenarioC_degradedButPassableFinishesLeg() {
        header("SCENARIO C: current edge (A->B) degraded but passable -> finish leg, replan continuation only");

        RouteGraph graph;
        try {
            graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        MissionManager mm = new MissionManager(graph, new AStarRouter());

        String robotId = "robot-c";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        check("initial dispatched target is B", initial != null && "B".equals(initial.getId()));

        moveToward(pos, initial, 3.0);
        System.out.println("Robot now at (" + fmt(pos[0]) + ", " + fmt(pos[1]) + "), mid-way on A->B.");

        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("A", "B", "c-degrade", 0.75, 0.5);
        String segment = newSegment(before);
        check("degrading the CURRENT edge (not fully blocked) logs 'degraded but still passable'",
                segment.contains("degraded but still passable"));
        check("degrading (not fully blocked) does not trigger a retreat",
                !segment.contains("retreating"));

        GraphNode afterB = driveUntilNextTarget(mm, robotId, pos, initial, 50);
        check("edge is finished normally and the route continuation is still D (unaffected elsewhere in the graph)",
                afterB != null && "D".equals(afterB.getId()));
    }

    private static void scenarioD_locationAwareBlockageChoosesDirection() throws IOException {
        header("SCENARIO D: location-aware full block - behind continues, ahead still retreats");

        // Each half of this scenario needs its own graph instance - an
        // obstruction report is stored on the shared B-D corridor and
        // applies to the whole corridor, whichever direction's naming it
        // was reported through, so sharing one instance would let mm1's
        // full block on B-D leak into mm2's supposedly-independent
        // mission (mm2 would then never route through D at all,
        // invalidating the "ahead of the robot" half of this test).
        RouteGraph graph1 = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm1 = new MissionManager(graph1, new AStarRouter());
        String robotId1 = "robot-d1";
        double[] pos1 = {0.0, 0.0};
        mm1.onTelemetry(robotId1, pos1[0], pos1[1], "E");
        GraphNode initial1 = mm1.initialTargetFor(robotId1);
        driveUntilNextTarget(mm1, robotId1, pos1, initial1, 50);
        moveToward(pos1, graph1.getNode("D"), 6.0);
        System.out.println("Robot at (" + fmt(pos1[0]) + ", " + fmt(pos1[1])
                + "), ~67% along B->D. Reporting a full block at 30% (behind the robot)...");

        int before1 = fullLog.size();
        mm1.updateRobotPosition(robotId1, pos1[0], pos1[1]);
        mm1.reportEdgeObstruction("B", "D", "d1-block", 0.3, 0.3);
        String segment1 = newSegment(before1);
        check("blockage behind the robot logs 'continuing forward', not a retreat",
                segment1.contains("continuing forward") && !segment1.contains("retreating to"));

        GraphNode afterD1 = driveUntilNextTarget(mm1, robotId1, pos1, graph1.getNode("D"), 50);
        check("robot proceeds uninterrupted to D, next target is E",
                afterD1 != null && "E".equals(afterD1.getId()));

        before1 = fullLog.size();
        boolean reachedE1 = driveToCompletion(mm1, robotId1, pos1, afterD1, 50);
        check("mission completes at E", reachedE1 && newSegment(before1).contains("confirmed at E"));

        RouteGraph graph2 = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm2 = new MissionManager(graph2, new AStarRouter());
        String robotId2 = "robot-d2";
        double[] pos2 = {0.0, 0.0};
        mm2.onTelemetry(robotId2, pos2[0], pos2[1], "E");
        GraphNode initial2 = mm2.initialTargetFor(robotId2);
        driveUntilNextTarget(mm2, robotId2, pos2, initial2, 50);
        moveToward(pos2, graph2.getNode("D"), 6.0);
        System.out.println("Robot at (" + fmt(pos2[0]) + ", " + fmt(pos2[1])
                + "), ~67% along B->D. Reporting a full block at 90% (ahead of the robot)...");

        int before2 = fullLog.size();
        mm2.updateRobotPosition(robotId2, pos2[0], pos2[1]);
        mm2.reportEdgeObstruction("B", "D", "d2-block", 0.3, 0.9);
        String segment2 = newSegment(before2);
        check("blockage ahead of the robot still logs a retreat to B",
                segment2.contains("retreating to B"));

        GraphNode retreatTarget2 = mm2.onTelemetry(robotId2, pos2[0], pos2[1], "E");
        check("retreat target dispatched is B", retreatTarget2 != null && "B".equals(retreatTarget2.getId()));
    }

    private static void scenarioE_signalOnRetreatEdgeIsRecognized() throws IOException {
        header("SCENARIO E: a capacity signal on the REVERSE (retreat) edge is now recognized as current");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm = new MissionManager(graph, new AStarRouter());
        String robotId = "robot-e";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        driveUntilNextTarget(mm, robotId, pos, initial, 50); // to B, next target D

        moveToward(pos, graph.getNode("D"), 2.0);
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("B", "D", "e-block", 0.3, 0.9); // full block at 0.9, ahead of the robot -> retreat
        GraphNode retreatTarget = mm.onTelemetry(robotId, pos[0], pos[1], "E");
        check("retreat to B begins as expected", retreatTarget != null && "B".equals(retreatTarget.getId()));

        // While still retreating (not yet at B), report a signal on the
        // REVERSE directed edge D->B - the exact edge the robot is now
        // physically on. Before this fix, this input matched neither
        // the (frozen, forward) mission edge nor the remaining path, and
        // was silently ignored.
        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("D", "B", "e-block", 0.75, 0.1);
        String segment = newSegment(before);
        check("signal on the reverse retreat edge (D->B) is now recognized as the current edge",
                segment.contains("Current edge (D->B) degraded but still passable"));

        // Confirm recognition didn't disturb the retreat itself - it
        // still completes normally.
        GraphNode afterRetreat = driveUntilNextTarget(mm, robotId, pos, graph.getNode("B"), 50);
        check("retreat still completes normally after the recognized signal (next target C)",
                afterRetreat != null && "C".equals(afterRetreat.getId()));
    }

    private static void scenarioF_bothDirectionsBlockedIsFlaggedNotHandled() throws IOException {
        header("SCENARIO F: retreat edge itself fully blocked mid-retreat - flagged, not auto-handled, no crash");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm = new MissionManager(graph, new AStarRouter());
        String robotId = "robot-f";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        driveUntilNextTarget(mm, robotId, pos, initial, 50);

        moveToward(pos, graph.getNode("D"), 2.0);
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("B", "D", "f-block-1", 0.3, 0.9);
        mm.onTelemetry(robotId, pos[0], pos[1], "E"); // dispatch retreat target

        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("D", "B", "f-block-2", 0.3, 0.1); // full block on retreat edge itself
        String segment = newSegment(before);
        check("both-directions-blocked case logs the explicit warning",
                segment.contains("WARNING: retreat edge"));

        // The already-in-progress retreat should still complete normally -
        // the flagged case takes no action, it doesn't corrupt state.
        GraphNode afterRetreat = driveUntilNextTarget(mm, robotId, pos, graph.getNode("B"), 50);
        check("retreat still completes despite the flagged warning (next target C)",
                afterRetreat != null && "C".equals(afterRetreat.getId()));
    }

    private static void scenarioH_crossDirectionReportAffectsLiveRobot() throws IOException {
        header("SCENARIO H: obstruction reported via the REVERSE direction's naming still triggers the correct reaction for a robot traveling forward");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm = new MissionManager(graph, new AStarRouter());
        String robotId = "robot-h";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        driveUntilNextTarget(mm, robotId, pos, initial, 50);

        moveToward(pos, graph.getNode("D"), 2.0);
        System.out.println("Robot at (" + fmt(pos[0]) + ", " + fmt(pos[1])
                + "), traveling B->D. Reporting a severe obstruction using the REVERSE "
                + "edge's own naming (D->B, local position 0.1 - close to D)...");

        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("D", "B", "python-cross-event", 0.3, 0.1);
        check("a report using the REVERSE direction's naming still triggers a retreat "
                        + "for a robot tracked on the FORWARD direction",
                newSegment(before).contains("retreating to B"));

        GraphNode retreatTarget = mm.onTelemetry(robotId, pos[0], pos[1], "E");
        check("retreat target dispatched is B", retreatTarget != null && "B".equals(retreatTarget.getId()));
    }

    private static void scenarioI_bothDirectionsBlockedAtInitialDecision() throws IOException {
        header("SCENARIO I: both directions blocked at the INITIAL decision - flagged, no action");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm = new MissionManager(graph, new AStarRouter());
        String robotId = "robot-i";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        driveUntilNextTarget(mm, robotId, pos, initial, 50);

        moveToward(pos, graph.getNode("D"), 4.0);
        System.out.println("Robot at (" + fmt(pos[0]) + ", " + fmt(pos[1])
                + "). Reporting obstructions on BOTH sides of the robot's position, same edge...");

        // Order matters here: reporting BEHIND first causes a single-sided
        // "continuing forward" reaction (matches Scenario D's proven
        // behavior) that does NOT enter recovery state. Reporting AHEAD
        // second then finds both sides already blocked in the SAME
        // reaction pass and takes the flagged no-action branch, from a
        // clean (non-recovering) base. Reporting AHEAD first would
        // instead trigger a real retreat after just that one event, which
        // the second event's "both blocked, no action" branch does not
        // roll back - leaving the robot stuck mid-retreat rather than
        // genuinely untouched, which would defeat the point of this case.
        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("B", "D", "behind-event", 0.3, 0.1);
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportEdgeObstruction("B", "D", "ahead-event", 0.3, 0.9);
        check("both-directions-blocked at the INITIAL decision logs the explicit warning",
                newSegment(before).contains("WARNING: both directions"));

        GraphNode result = mm.onTelemetry(robotId, pos[0], pos[1], "E");
        check("no target is dispatched while both directions are flagged as blocked",
                result == null);
    }

    private static void scenarioJ_widthAffectsRouteCostDirectly() throws IOException {
        header("SCENARIO J: a width-based obstruction changes A*'s chosen route, proving effectiveCost() actually reflects it");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        AStarRouter router = new AStarRouter();

        List<GraphNode> beforePath = router.findPath(graph, "A", "E");
        check("before any obstruction, A* picks the D-route",
                beforePath.size() == 4 && "D".equals(beforePath.get(2).getId()));

        GraphEdge edge = graph.getEdge("B", "D");
        edge.reportObstructionEvent("scenario-j-event", 0.3);

        List<GraphNode> afterPath = router.findPath(graph, "A", "E");
        check("after a severe width-based obstruction, A* now avoids B->D and routes via C",
                afterPath.stream().noneMatch(n -> "D".equals(n.getId()))
                        && afterPath.stream().anyMatch(n -> "C".equals(n.getId())));

        edge.clearObstructionEvent("scenario-j-event");
        List<GraphNode> clearedPath = router.findPath(graph, "A", "E");
        check("clearing the obstruction restores the original D-route",
                clearedPath.size() == 4 && "D".equals(clearedPath.get(2).getId()));
    }

    private static void scenarioK_realisticWorldCoordinateReportDrivesCorrectReaction() throws IOException {
        header("SCENARIO K: a raw world-coordinate report (no edge/direction knowledge assumed) drives the correct reaction via the new translation layer");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        MissionManager mm = new MissionManager(graph, new AStarRouter());
        String robotId = "robot-k";
        double[] pos = {0.0, 0.0};

        mm.onTelemetry(robotId, pos[0], pos[1], "E");
        GraphNode initial = mm.initialTargetFor(robotId);
        driveUntilNextTarget(mm, robotId, pos, initial, 50);

        moveToward(pos, graph.getNode("D"), 2.0);
        System.out.println("Robot at (" + fmt(pos[0]) + ", " + fmt(pos[1])
                + "). Narrowing B->D's structural width, then reporting a raw WORLD-COORDINATE "
                + "obstruction near (10, 6) - ahead of the robot, no edge/direction specified...");

        graph.getEdge("B", "D").setPassableWidthMeters(0.8);

        int before = fullLog.size();
        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.reportWorldObstruction("python-world-event", 10.0, 6.0, 0.5);
        String segment = newSegment(before);
        check("the world-coordinate report is logged as mapped onto the B-D corridor",
                segment.contains("mapped to edge B->D") || segment.contains("mapped to edge D->B"));
        check("the resulting severe obstruction (ahead of the robot) triggers a retreat",
                segment.contains("retreating to B"));

        GraphNode retreatTarget = mm.onTelemetry(robotId, pos[0], pos[1], "E");
        check("retreat target dispatched is B", retreatTarget != null && "B".equals(retreatTarget.getId()));

        mm.updateRobotPosition(robotId, pos[0], pos[1]);
        mm.clearWorldObstruction("python-world-event");
    }

    // Steps the fake robot toward `aimAt`, calling onTelemetry() each tick,
    // until MissionManager returns a non-null (new) target or maxSteps is
    // exhausted (returns null on timeout, which assertions treat as failure).
    private static GraphNode driveUntilNextTarget(
            MissionManager mm, String robotId, double[] pos, GraphNode aimAt, int maxSteps) {

        for (int i = 0; i < maxSteps; i++) {
            moveToward(pos, aimAt, STEP_SIZE_METERS);
            GraphNode result = mm.onTelemetry(robotId, pos[0], pos[1], "E");
            if (result != null) {
                System.out.println("New target: " + describe(result));
                return result;
            }
        }
        return null;
    }

    // Like driveUntilNextTarget, but for the final leg: arrival at the
    // destination causes the mission to complete, which onTelemetry signals
    // by returning null (same as "not arrived yet"), so completion is instead
    // detected by physical proximity to the destination.
    private static boolean driveToCompletion(
            MissionManager mm, String robotId, double[] pos, GraphNode destination, int maxSteps) {

        for (int i = 0; i < maxSteps; i++) {
            moveToward(pos, destination, STEP_SIZE_METERS);
            mm.onTelemetry(robotId, pos[0], pos[1], "E");
            if (closeTo(pos, destination)) {
                return true;
            }
        }
        return false;
    }

    private static void moveToward(double[] pos, GraphNode target, double stepSize) {
        double dx = target.getX() - pos[0];
        double dy = target.getY() - pos[1];
        double dist = Math.hypot(dx, dy);
        if (dist <= stepSize) {
            pos[0] = target.getX();
            pos[1] = target.getY();
        } else {
            pos[0] += dx / dist * stepSize;
            pos[1] += dy / dist * stepSize;
        }
    }

    private static boolean closeTo(double[] pos, GraphNode node) {
        return distance(pos, node.getX(), node.getY()) < ARRIVAL_THRESHOLD_METERS;
    }

    private static double distance(double[] pos, double x, double y) {
        double dx = pos[0] - x;
        double dy = pos[1] - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static String newSegment(int before) {
        byte[] all = fullLog.toByteArray();
        return new String(all, before, all.length - before);
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

    private static String describe(GraphNode node) {
        return node.getId() + "(" + fmt(node.getX()) + ", " + fmt(node.getY()) + ")";
    }

    private static String fmt(double v) {
        return String.format("%.2f", v);
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
