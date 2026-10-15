package com.robotproject.server.routing;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class WidthCapacityDemo {

    private static int passCount = 0;
    private static int failCount = 0;

    public static void main(String[] args) throws IOException {

        System.out.println("=== single-report regression cases (through reportObstructionEvent) ===");

        GraphEdge edgeA = new GraphEdge("X", "Y", 10.0);
        edgeA.reportObstructionEvent("evt", 2.5);
        checkExact(
                "(a) fresh edge, one event leaves 2.5m, robot=0.6 -> 1.0 (still easily passable)",
                edgeA.getCapacityFactor(),
                1.0);

        GraphEdge edgeB = new GraphEdge("X", "Y", 10.0);
        edgeB.setPassableWidthMeters(0.5);
        checkExact(
                "(b) fresh edge, structural width 0.5m, no events -> 0.0",
                edgeB.getCapacityFactor(),
                0.0);

        GraphEdge edgeC = new GraphEdge("X", "Y", 10.0);
        edgeC.reportObstructionEvent("evt", 0.5);
        checkExact(
                "(c) fresh edge, one event leaves 0.5m, robot=0.6 -> 0.0 (same outcome as (b), different cause)",
                edgeC.getCapacityFactor(),
                0.0);

        GraphEdge edgeD = new GraphEdge("X", "Y", 10.0);
        edgeD.reportObstructionEvent("evt", 0.75);
        System.out.println("(d) fresh edge, one event leaves 0.75m, robot=0.6 -> " + edgeD.getCapacityFactor());
        checkApprox(
                "(d) result is ~0.5 (strictly between 0.0 and 1.0)",
                edgeD.getCapacityFactor(),
                0.5);

        System.out.println();
        System.out.println("=== multi-event correctness (the bug this redesign fixes) ===");

        GraphEdge multi = new GraphEdge("X", "Y", 10.0);

        multi.reportObstructionEvent("obstacle-A", 0.8);
        double afterAOnly = multi.getCapacityFactor();
        System.out.println("after obstacle-A only (0.8m remaining): capacityFactor=" + afterAOnly);
        check("obstacle-A alone leaves a partial capacity (strictly between 0.0 and 1.0)",
                afterAOnly > 0.0 && afterAOnly < 1.0);

        multi.reportObstructionEvent("obstacle-B", 0.5);
        double afterBAlsoActive = multi.getCapacityFactor();
        System.out.println("after obstacle-B also active (0.5m remaining, more severe): capacityFactor="
                + afterBAlsoActive);
        check("obstacle-B (more severe) now governs -> capacityFactor == 0.0",
                afterBAlsoActive == 0.0);

        multi.clearObstructionEvent("obstacle-B");
        double afterClearingB = multi.getCapacityFactor();
        System.out.println("after clearing obstacle-B (obstacle-A still active): capacityFactor="
                + afterClearingB);
        check("clearing B reverts to the SAME partial value as after A alone - "
                        + "proves clearing B did not erase A's still-active contribution",
                afterClearingB == afterAOnly);

        multi.clearObstructionEvent("obstacle-A");
        double afterClearingLastEvent = multi.getCapacityFactor();
        System.out.println("after clearing obstacle-A (last remaining event): capacityFactor="
                + afterClearingLastEvent);
        check("clearing the last active event reopens the edge fully -> capacityFactor == 1.0, "
                        + "no special-casing needed",
                afterClearingLastEvent == 1.0);

        System.out.println();
        System.out.println("=== structural floor dominates a less-restrictive dynamic event ===");

        GraphEdge doorway = new GraphEdge("X", "Y", 10.0);
        doorway.setPassableWidthMeters(0.5);

        doorway.reportObstructionEvent("large-but-irrelevant-object", 2.0);
        double withWideEventOnNarrowDoorway = doorway.getCapacityFactor();
        System.out.println("narrow doorway (0.5m) + a 2.0m-remaining event: capacityFactor="
                + withWideEventOnNarrowDoorway);
        check("the doorway's structural 0.5m stays the real bottleneck, not the wide event "
                        + "-> capacityFactor == 0.0",
                withWideEventOnNarrowDoorway == 0.0);

        doorway.clearObstructionEvent("large-but-irrelevant-object");
        double afterClearingIrrelevantEvent = doorway.getCapacityFactor();
        System.out.println("after clearing that event: capacityFactor=" + afterClearingIrrelevantEvent);
        check("structural floor persists with no events at all, exactly as before -> capacityFactor == 0.0",
                afterClearingIrrelevantEvent == 0.0);

        System.out.println();
        System.out.println("=== cross-direction: a report on one direction is visible to the other ===");

        RouteGraph bdGraph = new RouteGraph();
        bdGraph.addNode(new GraphNode("B", 0.0, 0.0));
        bdGraph.addNode(new GraphNode("D", 10.0, 0.0));
        bdGraph.addEdge("B", "D", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);

        GraphEdge forwardBD = bdGraph.getEdge("B", "D");
        GraphEdge backwardDB = bdGraph.getEdge("D", "B");

        forwardBD.reportObstructionEventAt("cross-dir-event", 0.4, 0.3);

        System.out.println("after reporting on FORWARD (B->D) at local 0.3, remaining 0.4m: "
                + "backward (D->B) getCurrentEffectiveWidthMeters()=" + backwardDB.getCurrentEffectiveWidthMeters()
                + ", capacityFactor=" + backwardDB.getCapacityFactor());
        checkApprox(
                "the BACKWARD edge's effective width also reflects the ~0.4m obstruction",
                backwardDB.getCurrentEffectiveWidthMeters(),
                0.4);
        checkExact(
                "the BACKWARD edge's capacityFactor is 0.0 (0.4m <= 0.6m robot width)",
                backwardDB.getCapacityFactor(),
                0.0);

        double backwardAheadAt09 = backwardDB.getEffectiveWidthAheadOfPosition(0.9);
        System.out.println("BACKWARD edge, getEffectiveWidthAheadOfPosition(0.9) [obstruction is BEHIND "
                + "this direction] = " + backwardAheadAt09);
        checkExact(
                "ahead of the BACKWARD direction at its own local 0.9, the obstruction (behind it) "
                        + "is excluded -> unconstrained width",
                backwardAheadAt09,
                GraphEdge.UNCONSTRAINED_WIDTH_METERS);

        double backwardAheadAt01 = backwardDB.getEffectiveWidthAheadOfPosition(0.1);
        System.out.println("BACKWARD edge, getEffectiveWidthAheadOfPosition(0.1) [obstruction is still "
                + "AHEAD from this direction] = " + backwardAheadAt01);
        checkApprox(
                "ahead of the BACKWARD direction at its own local 0.1, the obstruction (still ahead) "
                        + "is included -> ~0.4m",
                backwardAheadAt01,
                0.4);

        System.out.println();
        System.out.println("=== structural floor is included in EVERY directional query, both directions ===");

        RouteGraph floorGraph = new RouteGraph();
        floorGraph.addNode(new GraphNode("B", 0.0, 0.0));
        floorGraph.addNode(new GraphNode("D", 10.0, 0.0));
        floorGraph.addEdge("B", "D", true, 0.5);

        GraphEdge floorForward = floorGraph.getEdge("B", "D");
        GraphEdge floorBackward = floorGraph.getEdge("D", "B");

        double[] positions = {0.1, 0.5, 0.9};
        for (double p : positions) {
            checkApprox(
                    "forward edge, getEffectiveWidthAheadOfPosition(" + p + ") includes structural floor -> 0.5",
                    floorForward.getEffectiveWidthAheadOfPosition(p),
                    0.5);
            checkApprox(
                    "forward edge, getEffectiveWidthBehindPosition(" + p + ") includes structural floor -> 0.5",
                    floorForward.getEffectiveWidthBehindPosition(p),
                    0.5);
            checkApprox(
                    "backward edge, getEffectiveWidthAheadOfPosition(" + p + ") includes structural floor -> 0.5",
                    floorBackward.getEffectiveWidthAheadOfPosition(p),
                    0.5);
            checkApprox(
                    "backward edge, getEffectiveWidthBehindPosition(" + p + ") includes structural floor -> 0.5",
                    floorBackward.getEffectiveWidthBehindPosition(p),
                    0.5);
        }

        System.out.println();
        System.out.println("=== mission_map.json regression guard ===");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        boolean allDeclared = true;
        int edgeCount = 0;
        for (GraphNode node : graph.allNodes()) {
            for (GraphEdge edge : graph.edgesFrom(node.getId())) {
                edgeCount++;
                double declared = declaredMissionMapWidth(edge.getFromId(), edge.getToId());
                if (Math.abs(edge.getCurrentEffectiveWidthMeters() - declared) > 1e-9) {
                    allDeclared = false;
                    System.out.println("  UNEXPECTED WIDTH: " + edge.getFromId() + " -> " + edge.getToId()
                            + " = " + edge.getCurrentEffectiveWidthMeters() + " (declared " + declared + ")");
                }
            }
        }
        check("every one of " + edgeCount + " directed edges in mission_map.json has "
                        + "getCurrentEffectiveWidthMeters() == its declared width "
                        + "(A-B 2.4, B-C 2.4, B-D 1.8, D-E 1.8, C-E 1.2)",
                allDeclared && edgeCount == 10);

        System.out.println();
        System.out.println("=== metric-only invariants ===");

        // (a) A report through the backward instance blocks the forward direction too.
        RouteGraph tri = new RouteGraph();
        tri.addNode(new GraphNode("X", 0.0, 0.0));
        tri.addNode(new GraphNode("Y", 10.0, 0.0));
        tri.addNode(new GraphNode("Z", 10.0, 10.0));
        tri.addEdge("X", "Y", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        tri.addEdge("X", "Z", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        tri.addEdge("Z", "Y", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        AStarRouter router = new AStarRouter();

        tri.getEdge("Y", "X").reportObstructionEvent("rev", 0.3);
        checkExact("(a) report via BACKWARD Y->X: FORWARD X->Y getCapacityFactor() is 0.0",
                tri.getEdge("X", "Y").getCapacityFactor(), 0.0);
        String blockedRoute = pathIds(router.findPath(tri, "X", "Y"));
        check("(a) A* routes X->Y via Z (got " + blockedRoute + ")", blockedRoute.equals("X,Z,Y"));
        tri.getEdge("Y", "X").clearObstructionEvent("rev");
        String clearedRoute = pathIds(router.findPath(tri, "X", "Y"));
        check("(a) cleared through the backward instance: the direct route returns (got "
                + clearedRoute + ")", clearedRoute.equals("X,Y"));

        // (b) No staleness: both directions agree after every single step.
        RouteGraph pair = new RouteGraph();
        pair.addNode(new GraphNode("B", 0.0, 0.0));
        pair.addNode(new GraphNode("D", 10.0, 0.0));
        pair.addEdge("B", "D", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        GraphEdge fwd = pair.getEdge("B", "D");
        GraphEdge bwd = pair.getEdge("D", "B");

        fwd.reportObstructionEvent("e1", 0.75);
        checkDirectionsAgree("(b) after reporting e1 0.75 via forward", fwd, bwd);
        bwd.reportObstructionEventAt("e2", 0.3, 0.5);
        checkDirectionsAgree("(b) after reporting e2 0.3 at 0.5 via backward", fwd, bwd);
        bwd.clearObstructionEvent("e2");
        checkDirectionsAgree("(b) after clearing e2 via backward", fwd, bwd);
        bwd.setPassableWidthMeters(0.8);
        checkDirectionsAgree("(b) after setPassableWidthMeters(0.8) via backward", fwd, bwd);
        fwd.clearObstructionEvent("e1");
        checkDirectionsAgree("(b) after clearing e1 via forward", fwd, bwd);
        checkApprox("(b) final value equals computeCapacityFromWidth(0.8, 0.6)",
                fwd.getCapacityFactor(), GraphEdge.computeCapacityFromWidth(0.8, 0.6));

        // (c) A one-way edge has its own corridor: a report changes only it.
        RouteGraph oneWay = new RouteGraph();
        oneWay.addNode(new GraphNode("P", 0.0, 0.0));
        oneWay.addNode(new GraphNode("Q", 10.0, 0.0));
        oneWay.addNode(new GraphNode("R", 0.0, 10.0));
        oneWay.addEdge("P", "Q", false, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        oneWay.addEdge("P", "R", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        GraphEdge pq = oneWay.getEdge("P", "Q");
        boolean oneWayThrew = false;
        try {
            pq.reportObstructionEvent("ow", 0.3);
        } catch (RuntimeException e) {
            oneWayThrew = true;
        }
        check("(c) reporting on a one-way edge throws nothing", !oneWayThrew);
        checkExact("(c) the one-way edge P->Q itself is blocked", pq.getCapacityFactor(), 0.0);
        check("(c) there is no reverse Q->P edge to be affected", oneWay.getEdge("Q", "P") == null);
        checkExact("(c) unrelated edge P->R is unchanged", oneWay.getEdge("P", "R").getCapacityFactor(), 1.0);
        checkExact("(c) unrelated edge R->P is unchanged", oneWay.getEdge("R", "P").getCapacityFactor(), 1.0);

        // (d) Band pins.
        checkExact("(d) computeCapacityFromWidth(0.6, 0.6)", GraphEdge.computeCapacityFromWidth(0.6, 0.6), 0.0);
        checkApprox("(d) computeCapacityFromWidth(0.69, 0.6) ~ 0.3", GraphEdge.computeCapacityFromWidth(0.69, 0.6), 0.3);
        checkApprox("(d) computeCapacityFromWidth(0.75, 0.6) ~ 0.5", GraphEdge.computeCapacityFromWidth(0.75, 0.6), 0.5);
        checkExact("(d) computeCapacityFromWidth(0.9, 0.6)", GraphEdge.computeCapacityFromWidth(0.9, 0.6), 1.0);

        // (e) Structure guard: the deleted percentage mechanism cannot come back unnoticed.
        for (String name : new String[] {"setCapacityFactor", "applyCapacityFactor", "linkSibling", "recomputeCapacity"}) {
            check("(e) GraphEdge declares no method " + name, !declaresMethod(GraphEdge.class, name));
        }
        for (String name : new String[] {"capacityFactor", "sibling"}) {
            check("(e) GraphEdge declares no field " + name, !declaresField(GraphEdge.class, name));
        }
        check("(e) RouteGraph declares no method setEdgeCapacity", !declaresMethod(RouteGraph.class, "setEdgeCapacity"));
        for (String name : new String[] {"onGraphCapacityChanged", "clearStaleOnCompletedEdge"}) {
            check("(e) MissionManager declares no method " + name, !declaresMethod(MissionManager.class, name));
        }
        check("(e) MissionManager declares no field obstructionEventEdges",
                !declaresField(MissionManager.class, "obstructionEventEdges"));

        // (f) Stress: a writer on the forward instance vs. a reader on the backward instance.
        runDirectionStress();

        // (g) isPassableWidthKnown().
        GraphEdge widthProbe = new GraphEdge("X", "Y", 10.0);
        check("(g) a fresh unconstrained edge: isPassableWidthKnown() is false", !widthProbe.isPassableWidthKnown());
        widthProbe.setPassableWidthMeters(0.8);
        check("(g) after setPassableWidthMeters(0.8): isPassableWidthKnown() is true",
                widthProbe.isPassableWidthKnown());
        RouteGraph missionGraph = MapLoader.loadFromClasspath("maps/mission_map.json");
        int directedEdges = 0;
        int knownWidths = 0;
        int matchingDeclared = 0;
        for (GraphEdge edge : missionGraph.allEdges()) {
            directedEdges++;
            if (edge.isPassableWidthKnown()) {
                knownWidths++;
            }
            if (Math.abs(edge.getPassableWidthMeters()
                    - declaredMissionMapWidth(edge.getFromId(), edge.getToId())) <= 1e-9) {
                matchingDeclared++;
            }
        }
        check("(g) all 10 directed edges of mission_map.json report isPassableWidthKnown() == true, each equal to "
                + "its declared width (got " + directedEdges + " edges, " + knownWidths + " known, "
                + matchingDeclared + " matching)",
                directedEdges == 10 && knownWidths == 10 && matchingDeclared == 10);

        List<String> missionUnknown = missionGraph.corridorsWithUnknownWidth();
        check("(g) corridorsWithUnknownWidth() is empty for mission_map.json (got " + missionUnknown + ")",
                missionUnknown.isEmpty());
        List<String> exampleUnknown = MapLoader.loadFromClasspath("maps/example_map.json").corridorsWithUnknownWidth();
        check("(g) corridorsWithUnknownWidth() lists A-B, B-C, B-D, D-E for example_map.json (got "
                + exampleUnknown + ")", exampleUnknown.equals(List.of("A-B", "B-C", "B-D", "D-E")));
        RouteGraph oneWayOnly = new RouteGraph();
        oneWayOnly.addNode(new GraphNode("P", 0.0, 0.0));
        oneWayOnly.addNode(new GraphNode("Q", 10.0, 0.0));
        oneWayOnly.addEdge("P", "Q", false, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        List<String> oneWayUnknown = oneWayOnly.corridorsWithUnknownWidth();
        check("(g) corridorsWithUnknownWidth() returns [P->Q] for a graph with one one-way edge (got "
                + oneWayUnknown + ")", oneWayUnknown.equals(List.of("P->Q")));

        System.out.println();
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
    }

    /** Both directions of one corridor must report identical capacity and cost - no stored copy to go stale. */
    private static void checkDirectionsAgree(String step, GraphEdge forward, GraphEdge backward) {
        double fCap = forward.getCapacityFactor();
        double bCap = backward.getCapacityFactor();
        double fCost = forward.effectiveCost();
        double bCost = backward.effectiveCost();
        check(step + ": forward and backward capacity equal (" + fCap + " / " + bCap + ")", fCap == bCap);
        check(step + ": forward and backward cost equal (" + fCost + " / " + bCost + ")", fCost == bCost);
    }

    /** The widths declared in mission_map.json, either direction. */
    private static double declaredMissionMapWidth(String from, String to) {
        String key = from.compareTo(to) < 0 ? from + "-" + to : to + "-" + from;
        switch (key) {
            case "A-B": case "B-C": return 2.4;
            case "B-D": case "D-E": return 1.8;
            case "C-E": return 1.2;
            default: return Double.NaN;
        }
    }

    private static boolean declaresMethod(Class<?> type, String name) {
        for (Method m : type.getDeclaredMethods()) {
            if (m.getName().equals(name)) {
                return true;
            }
        }
        return false;
    }

    private static boolean declaresField(Class<?> type, String name) {
        for (Field f : type.getDeclaredFields()) {
            if (f.getName().equals(name)) {
                return true;
            }
        }
        return false;
    }

    private static String pathIds(List<GraphNode> path) {
        List<String> ids = new ArrayList<>();
        for (GraphNode n : path) {
            ids.add(n.getId());
        }
        return String.join(",", ids);
    }

    /** (f) ~1s: a writer alternates report/clear on the forward instance, a reader polls the backward one. */
    private static void runDirectionStress() {
        RouteGraph stressPair = new RouteGraph();
        stressPair.addNode(new GraphNode("B", 0.0, 0.0));
        stressPair.addNode(new GraphNode("D", 10.0, 0.0));
        stressPair.addEdge("B", "D", true, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
        GraphEdge forward = stressPair.getEdge("B", "D");
        GraphEdge backward = stressPair.getEdge("D", "B");

        AtomicBoolean writerDone = new AtomicBoolean(false);
        AtomicReference<Throwable> writerError = new AtomicReference<>();
        AtomicReference<Throwable> readerError = new AtomicReference<>();
        AtomicReference<Double> outOfRange = new AtomicReference<>();
        AtomicInteger writes = new AtomicInteger();
        AtomicInteger reads = new AtomicInteger();

        Thread writer = new Thread(() -> {
            try {
                long end = System.nanoTime() + 1_000_000_000L;
                while (System.nanoTime() < end) {
                    forward.reportObstructionEvent("s", 0.3);
                    forward.clearObstructionEvent("s");
                    writes.incrementAndGet();
                }
            } catch (Throwable t) {
                writerError.set(t);
            } finally {
                writerDone.set(true);
            }
        }, "stress-writer");

        Thread reader = new Thread(() -> {
            try {
                while (!writerDone.get()) {
                    double c = backward.getCapacityFactor();
                    if (c < 0.0 || c > 1.0) {
                        outOfRange.compareAndSet(null, c);
                    }
                    backward.effectiveCost();
                    reads.incrementAndGet();
                }
            } catch (Throwable t) {
                readerError.set(t);
            }
        }, "stress-reader");

        reader.start();
        writer.start();
        try {
            writer.join();
            reader.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            readerError.compareAndSet(null, e);
        }

        System.out.println("(f) stress: " + writes.get() + " report/clear pairs, " + reads.get() + " reads");
        check("(f) the writer thread threw nothing (got " + writerError.get() + ")", writerError.get() == null);
        check("(f) the reader thread threw nothing (got " + readerError.get() + ")", readerError.get() == null);
        check("(f) every observed capacity is within [0,1] (first out of range: " + outOfRange.get() + ")",
                outOfRange.get() == null);
    }

    private static void checkExact(String description, double actual, double expected) {
        check(description + " (got " + actual + ")", actual == expected);
    }

    private static void checkApprox(String description, double actual, double expected) {
        check(description + " (got " + actual + ")", Math.abs(actual - expected) < 1e-9);
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
}
