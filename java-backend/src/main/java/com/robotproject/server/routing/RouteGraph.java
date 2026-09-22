package com.robotproject.server.routing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RouteGraph {

    private final Map<String, GraphNode> nodes = new HashMap<>();
    private final Map<String, List<GraphEdge>> adjacency = new HashMap<>();

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

        GraphEdge forward = new GraphEdge(fromId, toId, cost);
        forward.setPassableWidthMeters(widthMeters);
        adjacency.get(fromId).add(forward);

        if (bidirectional) {
            GraphEdge backward = new GraphEdge(toId, fromId, cost);
            backward.setPassableWidthMeters(widthMeters);
            adjacency.get(toId).add(backward);
        }
    }

    public GraphNode getNode(String id) {
        return nodes.get(id);
    }

    public List<GraphEdge> edgesFrom(String nodeId) {
        return adjacency.getOrDefault(nodeId, Collections.emptyList());
    }

    public void setEdgeCapacity(String fromId, String toId, double capacityFactor) {
        for (GraphEdge edge : adjacency.getOrDefault(fromId, Collections.emptyList())) {
            if (edge.getToId().equals(toId)) {
                edge.setCapacityFactor(capacityFactor);
            }
        }
    }

    public java.util.Collection<GraphNode> allNodes() {
        return nodes.values();
    }
}
