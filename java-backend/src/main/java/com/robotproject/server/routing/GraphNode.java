package com.robotproject.server.routing;

import com.robotproject.robotagent.Waypoint;

public class GraphNode {

    private final String id;
    private final double x;
    private final double y;

    public GraphNode(String id, double x, double y) {
        this.id = id;
        this.x = x;
        this.y = y;
    }

    public String getId() {
        return id;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double distanceTo(GraphNode other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public Waypoint toWaypoint() {
        return Waypoint.newBuilder()
                .setX(x)
                .setY(y)
                .build();
    }

    @Override
    public String toString() {
        return id + "(" + x + ", " + y + ")";
    }
}
