package com.robotproject.server.routing;

import java.io.IOException;

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
        System.out.println("=== mission_map.json regression guard ===");

        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        boolean allUnconstrained = true;
        int edgeCount = 0;
        for (GraphNode node : graph.allNodes()) {
            for (GraphEdge edge : graph.edgesFrom(node.getId())) {
                edgeCount++;
                if (edge.getCurrentEffectiveWidthMeters() != GraphEdge.UNCONSTRAINED_WIDTH_METERS) {
                    allUnconstrained = false;
                    System.out.println("  UNEXPECTED WIDTH: " + edge.getFromId() + " -> " + edge.getToId()
                            + " = " + edge.getCurrentEffectiveWidthMeters());
                }
            }
        }
        check("every one of " + edgeCount + " directed edges in mission_map.json still has "
                        + "getCurrentEffectiveWidthMeters() == UNCONSTRAINED_WIDTH_METERS ("
                        + GraphEdge.UNCONSTRAINED_WIDTH_METERS + ")",
                allUnconstrained && edgeCount > 0);

        System.out.println();
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
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
