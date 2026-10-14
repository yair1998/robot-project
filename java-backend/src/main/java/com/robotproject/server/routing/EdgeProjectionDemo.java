package com.robotproject.server.routing;

import java.io.IOException;

/**
 * Standalone proof of RouteGraph.findNearestEdge's pure projection
 * geometry - no width, no capacity, no MissionManager. This is the one
 * new piece of math this round introduces.
 */
public class EdgeProjectionDemo {

    private static int passCount = 0;
    private static int failCount = 0;

    public static void main(String[] args) throws IOException {
        RouteGraph graph = MapLoader.loadFromClasspath("maps/mission_map.json");
        // mission_map.json nodes: A(0,0) B(10,0) C(20,0) D(10,9) E(20,10)

        checkProjection("point on A-B at x=3", graph, 3.0, 0.0, "A", "B", 0.3, 0.01);
        checkProjection("point near B-D, offset 0.2m", graph, 10.2, 4.5, "B", "D", 0.5, 0.25);
        // NOTE: the original version of this case used a point just past D
        // along B-D (10.0, 9.5). That point sits almost exactly at the
        // shared vertex where B-D and D-E meet; D-E's shallow slope
        // ((10,1) direction) means it does NOT clamp there and ends up
        // ~2.5mm closer than B-D's clamped distance to D (0.4975m vs
        // 0.5000m) - a genuine geometric near-tie in the map's own
        // coordinates, not a bug in findNearestEdge. Any point whose
        // projection clamps to D on BOTH edges is an exact tie by
        // construction (both reduce to "distance to D"), so no nearby
        // offset reliably favors B-D. Replaced with an unambiguous
        // clamping test instead: a point behind node A, which has only
        // one edge (A-B) and so no competing geometry nearby.
        checkProjection("point just past A, beyond the segment", graph, -1.0, 0.0, "A", "B", 0.0, 1.1);

        RouteGraph.EdgeProjection farProjection = graph.findNearestEdge(500.0, 500.0);
        check("a point far from everything reports a large perpendicular distance",
                farProjection != null && farProjection.perpendicularDistanceMeters > 100.0);

        System.out.println();
        System.out.println("=== SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
    }

    private static void checkProjection(
            String label, RouteGraph graph, double worldX, double worldY,
            String end1, String end2, double expectedFraction, double maxDistance) {

        RouteGraph.EdgeProjection projection = graph.findNearestEdge(worldX, worldY);
        boolean edgeMatches = projection != null
                && ((projection.fromId.equals(end1) && projection.toId.equals(end2))
                    || (projection.fromId.equals(end2) && projection.toId.equals(end1)));

        System.out.println(label + ": " + (projection == null ? "no projection found"
                : projection.fromId + "->" + projection.toId + " at fraction "
                        + String.format("%.3f", projection.positionFraction)
                        + ", perpendicular " + String.format("%.3f", projection.perpendicularDistanceMeters) + "m"));

        check(label + " - matched edge is " + end1 + "/" + end2, edgeMatches);

        if (edgeMatches) {
            double f = projection.positionFraction;
            boolean fractionMatches =
                    Math.abs(f - expectedFraction) < 0.05 || Math.abs((1.0 - f) - expectedFraction) < 0.05;
            check(label + " - fraction is approximately " + expectedFraction, fractionMatches);
            check(label + " - perpendicular distance under " + maxDistance + "m",
                    projection.perpendicularDistanceMeters < maxDistance);
        }
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
