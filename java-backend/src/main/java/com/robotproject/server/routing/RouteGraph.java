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
        GraphNode from = nodes.get(fromId);
        GraphNode to = nodes.get(toId);
        if (from == null || to == null) {
            throw new IllegalArgumentException(
                    "Unknown node id in edge: " + fromId + " -> " + toId);
        }
        double cost = from.distanceTo(to);
        adjacency.get(fromId).add(new GraphEdge(fromId, toId, cost));
        if (bidirectional) {
            adjacency.get(toId).add(new GraphEdge(toId, fromId, cost));
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
