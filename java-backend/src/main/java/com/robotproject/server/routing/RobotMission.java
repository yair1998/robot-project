package com.robotproject.server.routing;

import java.util.List;

public class RobotMission {

    private final String robotId;
    private final String destinationNodeId;
    private List<GraphNode> remainingPath;

    public RobotMission(String robotId, String destinationNodeId, List<GraphNode> path) {
        this.robotId = robotId;
        this.destinationNodeId = destinationNodeId;
        this.remainingPath = path;
    }

    public String getRobotId() {
        return robotId;
    }

    public String getDestinationNodeId() {
        return destinationNodeId;
    }

    public String getLastConfirmedNodeId() {
        return remainingPath.isEmpty() ? null : remainingPath.get(0).getId();
    }

    public List<GraphNode> remainingNodes() {
        return remainingPath;
    }

    public GraphNode edgeFromNode() {
        return remainingPath.isEmpty() ? null : remainingPath.get(0);
    }

    public GraphNode currentTarget() {
        return remainingPath.size() > 1 ? remainingPath.get(1) : null;
    }

    public boolean isComplete() {
        return remainingPath.size() <= 1;
    }

    public void advance() {
        if (remainingPath.size() > 1) {
            remainingPath = remainingPath.subList(1, remainingPath.size());
        }
    }

    public void replacePath(List<GraphNode> newPath) {
        this.remainingPath = newPath;
    }

    public double progressAlongCurrentEdge(double robotX, double robotY) {
        GraphNode from = edgeFromNode();
        GraphNode to = currentTarget();
        if (from == null || to == null) {
            return 0.0;
        }

        double edgeDx = to.getX() - from.getX();
        double edgeDy = to.getY() - from.getY();
        double edgeLengthSquared = edgeDx * edgeDx + edgeDy * edgeDy;
        if (edgeLengthSquared < 1e-9) {
            return 1.0;
        }

        double robotDx = robotX - from.getX();
        double robotDy = robotY - from.getY();
        double t = (robotDx * edgeDx + robotDy * edgeDy) / edgeLengthSquared;

        return Math.max(0.0, Math.min(1.0, t));
    }

    public double remainingDistanceOnCurrentEdge(double robotX, double robotY) {
        GraphNode to = currentTarget();
        if (to == null) {
            return 0.0;
        }
        double dx = to.getX() - robotX;
        double dy = to.getY() - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
