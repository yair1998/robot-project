package com.robotproject.server.routing;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class MissionManager {

    private static final double ARRIVAL_THRESHOLD_METERS = 0.5;
    private static final double FULLY_BLOCKED_CAPACITY = 0.0001;

    private final RouteGraph graph;
    private final AStarRouter router;
    private final Map<String, RobotMission> missions = new ConcurrentHashMap<>();
    private final Map<String, RecoveryState> recovering = new ConcurrentHashMap<>();

    // Tracks the single directed edge each robot is physically traversing
    // right now, in whichever direction that actually is - including the
    // REVERSED direction while retreating. RobotMission's own
    // edgeFromNode()/currentTarget() are frozen during recovery (by
    // design - see RecoveryState), so they can't answer "what edge am I
    // on right now" during a retreat; this map is the one place that
    // question is always answered correctly, for both the normal-mission
    // case and the recovery case, via a single check instead of two.
    private final Map<String, EdgeInProgress> currentEdges = new ConcurrentHashMap<>();

    public MissionManager(RouteGraph graph, AStarRouter router) {
        this.graph = graph;
        this.router = router;
    }

    public GraphNode onTelemetry(String robotId, double x, double y, String destinationNodeId) {
        RecoveryState recovery = recovering.get(robotId);
        if (recovery != null) {
            return handleRecovery(robotId, recovery, x, y);
        }

        RobotMission mission = missions.computeIfAbsent(
                robotId, id -> startNewMission(id, x, y, destinationNodeId));

        if (mission.isComplete()) {
            return null;
        }

        GraphNode target = mission.currentTarget();
        double distanceToTarget = distance(x, y, target.getX(), target.getY());

        if (distanceToTarget < ARRIVAL_THRESHOLD_METERS) {
            mission.advance();
            System.out.println(
                    "Robot " + robotId + " confirmed at " + mission.getLastConfirmedNodeId());
            syncCurrentEdge(mission);
            return mission.isComplete() ? null : mission.currentTarget();
        }

        return null;
    }

    private GraphNode handleRecovery(String robotId, RecoveryState recovery, double x, double y) {
        double distanceToRetreatTarget =
                distance(x, y, recovery.retreatTarget.getX(), recovery.retreatTarget.getY());

        if (distanceToRetreatTarget < ARRIVAL_THRESHOLD_METERS) {
            RobotMission mission = missions.get(robotId);
            List<GraphNode> newPath =
                    router.findPath(graph, recovery.retreatTarget.getId(), mission.getDestinationNodeId());
            mission.replacePath(newPath);
            syncCurrentEdge(mission);
            recovering.remove(robotId);

            System.out.println(
                    "Robot " + robotId + " confirmed retreat to " + recovery.retreatTarget.getId()
                            + " - replanning to " + mission.getDestinationNodeId());

            return mission.isComplete() ? null : mission.currentTarget();
        }

        if (!recovery.dispatched) {
            recovery.dispatched = true;
            return recovery.retreatTarget;
        }

        return null;
    }

    public GraphNode initialTargetFor(String robotId) {
        RobotMission mission = missions.get(robotId);
        return mission == null ? null : mission.currentTarget();
    }

    /**
     * Backward-compatible overload - delegates with no known blockage
     * location, preserving the blind-retreat-on-full-block default.
     */
    public void onGraphCapacityChanged(
            String fromId, String toId, double newCapacity, double robotX, double robotY) {
        onGraphCapacityChanged(fromId, toId, newCapacity, robotX, robotY, null);
    }

    /**
     * blockagePositionFraction: 0.0 (at fromId) to 1.0 (at toId) along
     * the changed edge - same convention RobotMission.progressAlongCurrentEdge()
     * already uses for the robot's own position, so the two are directly
     * comparable. Pass null when the location isn't known.
     */
    public void onGraphCapacityChanged(
            String fromId, String toId, double newCapacity, double robotX, double robotY,
            Double blockagePositionFraction) {

        graph.setEdgeCapacity(fromId, toId, newCapacity);

        for (RobotMission mission : missions.values()) {
            handleGraphChangeForMission(
                    mission, fromId, toId, newCapacity, robotX, robotY, blockagePositionFraction);
        }
    }

    private void handleGraphChangeForMission(
            RobotMission mission, String changedFrom, String changedTo,
            double newCapacity, double robotX, double robotY, Double blockagePositionFraction) {

        String robotId = mission.getRobotId();
        EdgeInProgress inProgress = currentEdges.get(robotId);

        boolean isCurrentEdge =
                inProgress != null
                        && inProgress.fromId.equals(changedFrom)
                        && inProgress.toId.equals(changedTo);

        if (isCurrentEdge) {
            boolean fullyBlocked = newCapacity <= FULLY_BLOCKED_CAPACITY;

            if (!fullyBlocked) {
                System.out.println(
                        "Current edge (" + changedFrom + "->" + changedTo
                                + ") degraded but still passable - finishing this leg.");
                if (!recovering.containsKey(robotId)) {
                    GraphNode target = mission.currentTarget();
                    if (target != null) {
                        replanFrom(mission, target.getId());
                    }
                }
                return;
            }

            if (recovering.containsKey(robotId)) {
                // The edge the robot is CURRENTLY RETREATING ALONG has
                // itself just been reported fully blocked. Both directions
                // of this corridor are now blocked at once - there is no
                // known-safe routing action for this, and inventing one
                // (e.g. reversing the retreat again) risks oscillating
                // the robot between two blocked directions. Flagged, not
                // handled - this plausibly belongs to the safety layer
                // (the robot may be physically trapped), a decision left
                // for explicit review rather than guessed at here.
                System.out.println(
                        "WARNING: retreat edge (" + changedFrom + "->" + changedTo
                                + ") became fully blocked while robot " + robotId
                                + " was already retreating along it. No automatic routing "
                                + "action taken - this case needs explicit design review.");
                return;
            }

            if (blockagePositionFraction != null) {
                double robotProgress = mission.progressAlongCurrentEdge(robotX, robotY);
                if (blockagePositionFraction < robotProgress) {
                    System.out.println(
                            "Current edge fully blocked, but behind the robot's position - "
                                    + "continuing forward to " + mission.currentTarget().getId()
                                    + " instead of retreating.");
                    replanFrom(mission, mission.currentTarget().getId());
                    return;
                }
            }

            GraphNode currentFrom = mission.edgeFromNode();
            GraphNode currentTo = mission.currentTarget();
            System.out.println(
                    "Current edge fully blocked - retreating to "
                            + currentFrom.getId() + " before replanning.");
            recovering.put(robotId, new RecoveryState(currentFrom));
            currentEdges.put(robotId, new EdgeInProgress(currentTo.getId(), currentFrom.getId()));

        } else if (pathStillUsesEdge(mission, changedFrom, changedTo)) {
            // Deliberately still reads the mission's own (possibly
            // frozen-during-recovery) remaining path - a different
            // question ("is this edge anywhere in my planned route")
            // than isCurrentEdge above. Staleness here during a retreat
            // is a separate, smaller concern not addressed by this change.
            System.out.println("A future edge on the planned route changed - replanning ahead.");
            replanFrom(mission, mission.currentTarget().getId());
        }
    }

    private boolean pathStillUsesEdge(RobotMission mission, String fromId, String toId) {
        List<GraphNode> path = mission.remainingNodes();
        for (int i = 0; i < path.size() - 1; i++) {
            if (path.get(i).getId().equals(fromId) && path.get(i + 1).getId().equals(toId)) {
                return true;
            }
        }
        return false;
    }

    private void replanFrom(RobotMission mission, String fromNodeId) {
        List<GraphNode> continuation =
                router.findPath(graph, fromNodeId, mission.getDestinationNodeId());

        List<GraphNode> newPath = new ArrayList<>();
        newPath.add(mission.edgeFromNode());

        if (continuation.isEmpty()) {
            newPath.add(mission.currentTarget());
        } else {
            newPath.addAll(continuation);
        }

        mission.replacePath(newPath);
        syncCurrentEdge(mission);
    }

    private RobotMission startNewMission(
            String robotId, double x, double y, String destinationNodeId) {
        String startNodeId = nearestNode(x, y).getId();

        List<GraphNode> path = router.findPath(graph, startNodeId, destinationNodeId);
        System.out.println(
                "New mission for " + robotId + ": " + startNodeId + " -> " + destinationNodeId);
        RobotMission mission = new RobotMission(robotId, destinationNodeId, path);
        syncCurrentEdge(mission);
        return mission;
    }

    /**
     * Keeps currentEdges in sync with the mission's own forward-facing
     * state after any normal advance/replan. NOT used when recovery
     * starts - that's the one case where the tracked edge (reversed) is
     * deliberately different from what the (frozen) mission would say.
     */
    private void syncCurrentEdge(RobotMission mission) {
        GraphNode from = mission.edgeFromNode();
        GraphNode to = mission.currentTarget();
        if (from == null || to == null) {
            currentEdges.remove(mission.getRobotId());
        } else {
            currentEdges.put(mission.getRobotId(), new EdgeInProgress(from.getId(), to.getId()));
        }
    }

    private GraphNode nearestNode(double x, double y) {
        return graph.allNodes().stream()
                .min((a, b) -> Double.compare(
                        distance(x, y, a.getX(), a.getY()),
                        distance(x, y, b.getX(), b.getY())))
                .orElseThrow();
    }

    private double distance(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static class RecoveryState {
        final GraphNode retreatTarget;
        boolean dispatched;

        RecoveryState(GraphNode retreatTarget) {
            this.retreatTarget = retreatTarget;
            this.dispatched = false;
        }
    }

    private static class EdgeInProgress {
        final String fromId;
        final String toId;

        EdgeInProgress(String fromId, String toId) {
            this.fromId = fromId;
            this.toId = toId;
        }
    }
}
