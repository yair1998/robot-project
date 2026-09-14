package com.robotproject.server.routing;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

public class AStarRouter {

    public List<GraphNode> findPath(RouteGraph graph, String startId, String goalId) {

        GraphNode goal = graph.getNode(goalId);
        if (goal == null) {
            throw new IllegalArgumentException("Unknown goal node: " + goalId);
        }

        Map<String, Double> gScore = new HashMap<>();
        Map<String, String> cameFrom = new HashMap<>();
        gScore.put(startId, 0.0);

        PriorityQueue<String> open = new PriorityQueue<>(
                Comparator.comparingDouble(id ->
                        gScore.getOrDefault(id, Double.POSITIVE_INFINITY)
                                + graph.getNode(id).distanceTo(goal)));
        open.add(startId);
        Set<String> visited = new HashSet<>();

        while (!open.isEmpty()) {
            String current = open.poll();

            if (current.equals(goalId)) {
                return reconstructPath(graph, cameFrom, current);
            }

            if (!visited.add(current)) {
                continue;
            }

            for (GraphEdge edge : graph.edgesFrom(current)) {
                double tentativeG = gScore.get(current) + edge.effectiveCost();
                if (tentativeG < gScore.getOrDefault(edge.getToId(), Double.POSITIVE_INFINITY)) {
                    gScore.put(edge.getToId(), tentativeG);
                    cameFrom.put(edge.getToId(), current);
                    open.add(edge.getToId());
                }
            }
        }

        return Collections.emptyList();
    }

    private List<GraphNode> reconstructPath(
            RouteGraph graph, Map<String, String> cameFrom, String current) {
        LinkedList<GraphNode> path = new LinkedList<>();
        path.addFirst(graph.getNode(current));
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.addFirst(graph.getNode(current));
        }
        return path;
    }
}
