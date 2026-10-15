package com.robotproject.server.routing;

import java.time.Clock;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RouteGraph {

    private final Map<String, GraphNode> nodes = new HashMap<>();
    private final Map<String, List<GraphEdge>> adjacency = new HashMap<>();
    private final ObstructionContext obstructionContext = new ObstructionContext();

    public void addNode(GraphNode node) {
        nodes.put(node.getId(), node);
        adjacency.putIfAbsent(node.getId(), new ArrayList<>());
    }

    public void addEdge(String fromId, String toId, boolean bidirectional) {
        addEdge(fromId, toId, bidirectional, GraphEdge.UNCONSTRAINED_WIDTH_METERS);
    }

    public void addEdge(String fromId, String toId, boolean bidirectional, double widthMeters) {
        GraphNode from = nodes.get(fromId);
        GraphNode to = nodes.get(toId);
        if (from == null || to == null) {
            throw new IllegalArgumentException(
                    "Unknown node id in edge: " + fromId + " -> " + toId);
        }
        double cost = from.distanceTo(to);
        Corridor corridor = new Corridor(obstructionContext);
        corridor.setPassableWidthMeters(widthMeters);
        adjacency.get(fromId).add(new GraphEdge(fromId, toId, cost, corridor, false));
        if (bidirectional) {
            adjacency.get(toId).add(new GraphEdge(toId, fromId, cost, corridor, true));
        }
    }

    public GraphNode getNode(String id) {
        return nodes.get(id);
    }

    public List<GraphEdge> edgesFrom(String nodeId) {
        return adjacency.getOrDefault(nodeId, Collections.emptyList());
    }

    public GraphEdge getEdge(String fromId, String toId) {
        for (GraphEdge edge : adjacency.getOrDefault(fromId, Collections.emptyList())) {
            if (edge.getToId().equals(toId)) {
                return edge;
            }
        }
        return null;
    }

    public java.util.Collection<GraphNode> allNodes() {
        return nodes.values();
    }

    /** Test hook: a graph-wide clock for obstruction ageing. Default is the system clock. */
    public void setClock(Clock clock) { obstructionContext.setClock(clock); }
    public void setExpiryPolicy(ExpiryPolicy policy) { obstructionContext.setPolicy(policy); }
    public java.util.List<GraphEdge> allEdges() {
        java.util.List<GraphEdge> all = new java.util.ArrayList<>();
        for (java.util.List<GraphEdge> list : adjacency.values()) {
            all.addAll(list);
        }
        return all;
    }

    /** The graph-wide obstruction clock's time, so every component ages and stamps events consistently. */
    public Instant now() { return obstructionContext.now(); }
    public ExpiryPolicy getExpiryPolicy() { return obstructionContext.policy(); }

    /** OFF by default. On: STALE obstructions stop blocking and add an expected-encounter cost instead. */
    public void setStaleProbingEnabled(boolean enabled) { obstructionContext.setStaleProbingEnabled(enabled); }
    public boolean isStaleProbingEnabled() { return obstructionContext.staleProbingEnabled(); }

    /** Corridor width assumed when neither a structural nor a measured free width is known. Default 1.5m. */
    public void setAssumedUnknownCorridorWidthMeters(double widthMeters) {
        obstructionContext.setAssumedUnknownCorridorWidthMeters(widthMeters);
    }
    public double getAssumedUnknownCorridorWidthMeters() {
        return obstructionContext.assumedUnknownCorridorWidthMeters();
    }

    public List<ObstructionHistoryEntry> getObstructionHistory() { return obstructionContext.historySnapshot(); }

    /**
     * One entry per physical corridor whose structural width is unknown: "from-to" for a bidirectional
     * pair (listed once, from < to alphabetically), "from->to" for a one-way edge. Sorted.
     */
    public List<String> corridorsWithUnknownWidth() {
        java.util.TreeSet<String> result = new java.util.TreeSet<>();
        for (GraphEdge edge : allEdges()) {
            if (edge.isPassableWidthKnown()) {
                continue;
            }
            GraphEdge reverse = getEdge(edge.getToId(), edge.getFromId());
            if (reverse != null && reverse.sharesCorridorWith(edge)) {
                String a = edge.getFromId();
                String b = edge.getToId();
                result.add(a.compareTo(b) < 0 ? a + "-" + b : b + "-" + a);
            } else {
                result.add(edge.getFromId() + "->" + edge.getToId());
            }
        }
        return new ArrayList<>(result);
    }

    public static class EdgeProjection {
        public final String fromId;
        public final String toId;
        public final double positionFraction;
        public final double perpendicularDistanceMeters;

        EdgeProjection(String fromId, String toId, double positionFraction,
                double perpendicularDistanceMeters) {
            this.fromId = fromId;
            this.toId = toId;
            this.positionFraction = positionFraction;
            this.perpendicularDistanceMeters = perpendicularDistanceMeters;
        }
    }

    /**
     * Projects a raw world point onto the nearest edge in the graph -
     * mirrors the same clamped-projection math
     * RobotMission.progressAlongCurrentEdge() already uses for a
     * robot's own position, applied here to an arbitrary point.
     * Iterates every directed edge (including both directions of a
     * bidirectional pair); since both directions of one corridor sit on
     * the identical physical line, either may be returned - callers
     * that need a specific direction should not assume which one comes
     * back. Returns null only if the graph has no edges at all.
     */
    public EdgeProjection findNearestEdge(double worldX, double worldY) {
        GraphEdge bestEdge = null;
        double bestFraction = 0.0;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (List<GraphEdge> edgeList : adjacency.values()) {
            for (GraphEdge edge : edgeList) {
                GraphNode from = nodes.get(edge.getFromId());
                GraphNode to = nodes.get(edge.getToId());

                double edgeDx = to.getX() - from.getX();
                double edgeDy = to.getY() - from.getY();
                double edgeLengthSquared = edgeDx * edgeDx + edgeDy * edgeDy;

                double fraction;
                double closestX;
                double closestY;
                if (edgeLengthSquared < 1e-9) {
                    fraction = 0.0;
                    closestX = from.getX();
                    closestY = from.getY();
                } else {
                    double dx = worldX - from.getX();
                    double dy = worldY - from.getY();
                    double t = (dx * edgeDx + dy * edgeDy) / edgeLengthSquared;
                    fraction = Math.max(0.0, Math.min(1.0, t));
                    closestX = from.getX() + fraction * edgeDx;
                    closestY = from.getY() + fraction * edgeDy;
                }

                double perpDx = worldX - closestX;
                double perpDy = worldY - closestY;
                double distance = Math.sqrt(perpDx * perpDx + perpDy * perpDy);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestEdge = edge;
                    bestFraction = fraction;
                }
            }
        }

        if (bestEdge == null) {
            return null;
        }
        return new EdgeProjection(bestEdge.getFromId(), bestEdge.getToId(), bestFraction, bestDistance);
    }
}
