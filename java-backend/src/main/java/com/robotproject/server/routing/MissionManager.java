package com.robotproject.server.routing;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class MissionManager {

    private static final double ARRIVAL_THRESHOLD_METERS = 0.5;
    private static final double FULLY_BLOCKED_CAPACITY = 0.0001;
    private static final double PROXIMITY_CORROBORATION_TOLERANCE_METERS = 2.0;

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

        verifyStaleObstructionsPassed(robotId, mission, x, y);

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

    private void handleGraphChangeForMission(
            RobotMission mission, String changedFrom, String changedTo,
            double robotX, double robotY) {

        GraphNode currentTo = mission.currentTarget();
        if (currentTo == null) {
            return; // mission already complete
        }

        String robotId = mission.getRobotId();
        EdgeInProgress inProgress = currentEdges.get(robotId);
        GraphEdge trackedEdge = inProgress != null
                ? graph.getEdge(inProgress.fromId, inProgress.toId) : null;
        GraphEdge changedEdge = graph.getEdge(changedFrom, changedTo);

        boolean isCurrentEdge = trackedEdge != null && changedEdge != null
                && trackedEdge.sharesCorridorWith(changedEdge);

        if (isCurrentEdge) {
            // Derived live from the shared corridor (blocking events only) - nothing stored.
            boolean fullyBlocked = trackedEdge.getCapacityFactor() <= FULLY_BLOCKED_CAPACITY;

            if (!fullyBlocked) {
                System.out.println(
                        "Current edge (" + changedFrom + "->" + changedTo
                                + ") degraded but still passable - finishing this leg.");
                if (!recovering.containsKey(robotId)) {
                    replanFrom(mission, currentTo.getId());
                }
                return;
            }

            if (recovering.containsKey(robotId)) {
                System.out.println(
                        "WARNING: retreat edge (" + changedFrom + "->" + changedTo
                                + ") became fully blocked while robot " + robotId
                                + " was already retreating along it. No automatic routing "
                                + "action taken - this case needs explicit design review.");
                return;
            }

            double robotProgress = mission.progressAlongCurrentEdge(robotX, robotY);
            boolean somethingAheadIsBlocking = GraphEdge.computeCapacityFromWidth(
                    trackedEdge.getEffectiveWidthAheadOfPosition(robotProgress),
                    GraphEdge.DEFAULT_ROBOT_WIDTH_METERS) <= 0.0;
            boolean somethingBehindIsBlocking = GraphEdge.computeCapacityFromWidth(
                    trackedEdge.getEffectiveWidthBehindPosition(robotProgress),
                    GraphEdge.DEFAULT_ROBOT_WIDTH_METERS) <= 0.0;

            if (somethingAheadIsBlocking && somethingBehindIsBlocking) {
                System.out.println(
                        "WARNING: both directions of the current edge appear blocked for robot "
                                + robotId + " - no automatic routing action taken, "
                                + "this case needs explicit design review.");
                return;
            }

            if (!somethingAheadIsBlocking && !somethingBehindIsBlocking) {
                // Cannot happen for validly positioned events - defensive, never silent.
                System.out.println(
                        "WARNING: edge is fully blocked but no blocking event was located on "
                                + "either side of robot " + robotId + " - no automatic routing "
                                + "action taken, this case needs explicit design review.");
                return;
            }

            if (!somethingAheadIsBlocking) {
                System.out.println(
                        "Current edge fully blocked, but nothing severe ahead of the robot's "
                                + "position - continuing forward to " + currentTo.getId()
                                + " instead of retreating.");
                replanFrom(mission, currentTo.getId());
                return;
            }

            GraphNode currentFrom = mission.edgeFromNode();
            System.out.println(
                    "Current edge fully blocked - retreating to "
                            + currentFrom.getId() + " before replanning.");
            recovering.put(robotId, new RecoveryState(currentFrom));
            currentEdges.put(robotId,
                    new EdgeInProgress(currentTo.getId(), currentFrom.getId()));

        } else if (pathStillUsesEdge(mission, changedFrom, changedTo)) {
            System.out.println("A future edge on the planned route changed - replanning ahead.");
            replanFrom(mission, currentTo.getId());
        }
    }

    private boolean pathStillUsesEdge(RobotMission mission, String fromId, String toId) {
        GraphEdge changedEdgeObj = graph.getEdge(fromId, toId);
        if (changedEdgeObj == null) {
            return false;
        }
        List<GraphNode> path = mission.remainingNodes();
        for (int i = 0; i < path.size() - 1; i++) {
            GraphEdge pathEdgeObj = graph.getEdge(path.get(i).getId(), path.get(i + 1).getId());
            if (pathEdgeObj != null && pathEdgeObj.sharesCorridorWith(changedEdgeObj)) {
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

    public void reportEdgeObstruction(
            String fromId, String toId, String eventId, double remainingWidthMeters,
            double positionFraction, double robotX, double robotY) {
        reportEdgeObstruction(fromId, toId, eventId, remainingWidthMeters, positionFraction,
                null, null, robotX, robotY);
    }

    public void reportEdgeObstruction(
            String fromId, String toId, String eventId, double remainingWidthMeters,
            double positionFraction, ObstructionKind kind, Instant observedAt,
            double robotX, double robotY) {

        if (positionFraction < 0.0 || positionFraction > 1.0) {
            throw new IllegalArgumentException(
                    "positionFraction must be within 0.0-1.0, got " + positionFraction);
        }
        GraphEdge edge = graph.getEdge(fromId, toId);
        if (edge == null) {
            System.out.println("reportEdgeObstruction: no such edge " + fromId + "->" + toId);
            return;
        }
        if (edge.reportObstructionEvent(eventId, remainingWidthMeters, positionFraction,
                kind, observedAt)) {
            reactToCorridorChange(fromId, toId, robotX, robotY);
        }
    }

    public void clearEdgeObstruction(
            String fromId, String toId, String eventId, double robotX, double robotY) {
        GraphEdge edge = graph.getEdge(fromId, toId);
        if (edge == null) {
            System.out.println("clearEdgeObstruction: no such edge " + fromId + "->" + toId);
            return;
        }
        edge.clearObstructionEvent(eventId);
        reactToCorridorChange(fromId, toId, robotX, robotY);
    }

    private static final double MAX_PROJECTION_DISTANCE_METERS = 2.0;
    private static final java.time.Duration TOMBSTONE_RETENTION = java.time.Duration.ofMinutes(10);

    /** Latest accepted sighting of a world obstruction, or a tombstone left by an explicit clear / passage verification. */
    private static final class WorldEventRecord {
        final GraphEdge edge;      // null => tombstone
        final Instant observedAt;  // live: latest accepted observation
        final Instant clearedAt;   // tombstone: when it was cleared

        private WorldEventRecord(GraphEdge edge, Instant observedAt, Instant clearedAt) {
            this.edge = edge;
            this.observedAt = observedAt;
            this.clearedAt = clearedAt;
        }

        static WorldEventRecord live(GraphEdge edge, Instant observedAt) {
            return new WorldEventRecord(edge, observedAt, null);
        }

        static WorldEventRecord tombstone(Instant clearedAt) {
            return new WorldEventRecord(null, null, clearedAt);
        }

        boolean isTombstone() {
            return edge == null;
        }
    }

    private final Map<String, WorldEventRecord> worldEventRegistry = new ConcurrentHashMap<>();

    /**
     * The REALISTIC entry point: raw world coordinates of a detected
     * obstacle plus an estimate of the obstacle's own width - nothing
     * about edges, directions, or topology, which Python has no
     * business knowing (that knowledge belongs to Java alone, same
     * principle already applied everywhere else in this project). This
     * performs the ONE translation step - projecting the world point
     * onto the nearest known edge - that turns a raw perception fact
     * into the internal representation the existing width mechanism
     * already understands, then delegates to it unchanged.
     */
    public void reportWorldObstruction(
            String eventId, double obstacleWorldX, double obstacleWorldY,
            double obstacleWidthMeters, double robotX, double robotY) {
        reportWorldObstruction(WorldObstructionReport.basic(
                eventId, obstacleWorldX, obstacleWorldY, obstacleWidthMeters), robotX, robotY);
    }

    public void reportWorldObstruction(WorldObstructionReport report, double robotX, double robotY) {
        RouteGraph.EdgeProjection projection = graph.findNearestEdge(report.worldX, report.worldY);
        if (projection == null
                || projection.perpendicularDistanceMeters > MAX_PROJECTION_DISTANCE_METERS) {
            System.out.println(
                    "reportWorldObstruction: world point (" + report.worldX + ", " + report.worldY
                            + ") is not close enough to any known edge (nearest is "
                            + (projection == null ? "none" : projection.perpendicularDistanceMeters + "m away")
                            + ") - ignoring.");
            return;
        }
        GraphEdge edge = graph.getEdge(projection.fromId, projection.toId);
        if (edge == null) {
            return;
        }

        // A measured free width wins; else structural minus obstacle if the structural width is KNOWN;
        // else an explicit, configurable, conservative assumption - never "1000m minus the obstacle".
        final boolean widthAssumed;
        final double assumedWidth;
        double remaining;
        if (report.measuredFreeWidthMeters != null) {
            remaining = report.measuredFreeWidthMeters;
            widthAssumed = false;
            assumedWidth = 0.0;
        } else if (edge.isPassableWidthKnown()) {
            remaining = edge.getPassableWidthMeters() - report.obstacleWidthMeters;
            widthAssumed = false;
            assumedWidth = 0.0;
        } else {
            assumedWidth = graph.getAssumedUnknownCorridorWidthMeters();
            remaining = assumedWidth - report.obstacleWidthMeters;
            widthAssumed = true;
        }
        final double remainingWidth = Math.max(0.0, remaining);

        final Instant now = graph.now();
        final Instant stamp = (report.observedAt == null || report.observedAt.isAfter(now))
                ? now : report.observedAt;
        final ObstructionKind kind = report.kind == null ? ObstructionKind.UNKNOWN : report.kind;
        final ExpiryPolicy policy = graph.getExpiryPolicy();

        final GraphEdge[] movedFrom = {null};
        final String[] ignoredBecause = {null};

        // Atomic per event id: decide acceptance FIRST, touch corridors only if accepted.
        worldEventRegistry.compute(report.eventId, (id, rec) -> {
            if (rec != null && rec.isTombstone() && !stamp.isAfter(rec.clearedAt)) {
                ignoredBecause[0] = "it was observed no later than the moment this obstruction was cleared";
                return rec;
            }
            if (rec != null && !rec.isTombstone() && rec.observedAt.isAfter(stamp)) {
                ignoredBecause[0] = "a newer observation of it is already recorded";
                return rec;
            }
            if (policy.stateOf(kind, java.time.Duration.between(stamp, now)) == ExpiryPolicy.State.GONE) {
                ignoredBecause[0] = "it is already expired";
                return rec;
            }
            GraphEdge old = (rec != null && !rec.isTombstone()) ? rec.edge : null;
            if (!edge.reportObstructionEvent(id, remainingWidth, projection.positionFraction, kind, stamp)) {
                ignoredBecause[0] = "the corridor holds a newer observation of it";
                return rec;
            }
            if (old != null && !old.sharesCorridorWith(edge)) {
                old.clearObstructionEventAs(id, ObstructionHistoryEntry.EndReason.MOVED);
                movedFrom[0] = old;
            }
            return WorldEventRecord.live(edge, stamp);
        });

        if (ignoredBecause[0] != null) {
            System.out.println("World obstruction '" + report.eventId + "' ignored: " + ignoredBecause[0] + ".");
            return;
        }
        if (widthAssumed) {
            System.out.println("World obstruction '" + report.eventId
                    + "': corridor width unknown and no measured free width - assuming a "
                    + assumedWidth + "m corridor.");
        }
        if (movedFrom[0] != null) {
            System.out.println("Obstruction '" + report.eventId + "' moved from corridor "
                    + movedFrom[0].getFromId() + "->" + movedFrom[0].getToId() + " to "
                    + projection.fromId + "->" + projection.toId + " - cleared from the old one.");
            reactToCorridorChange(movedFrom[0].getFromId(), movedFrom[0].getToId(), robotX, robotY);
        }
        System.out.println(
                "World obstruction '" + report.eventId + "' at (" + report.worldX + ", " + report.worldY
                        + ") mapped to edge " + projection.fromId + "->" + projection.toId
                        + " at fraction " + projection.positionFraction
                        + " (perpendicular distance " + projection.perpendicularDistanceMeters + "m)");
        reactToCorridorChange(projection.fromId, projection.toId, robotX, robotY);
    }

    /**
     * Clears a previously-reported world obstruction by its own event
     * id - Python only needs to remember the id it originally chose,
     * never the edge/direction Java derived internally for it.
     */
    public void clearWorldObstruction(String eventId, double robotX, double robotY) {
        clearWorldObstruction(eventId, null, robotX, robotY);
    }

    /** clearedAt = when the clear was observed (null = now). Leaves a tombstone, even for an id never seen. */
    public void clearWorldObstruction(String eventId, Instant clearedAt, double robotX, double robotY) {
        final Instant now = graph.now();
        final Instant stamp = (clearedAt == null || clearedAt.isAfter(now)) ? now : clearedAt;
        final GraphEdge[] clearedFrom = {null};
        final String[] ignoredBecause = {null};

        worldEventRegistry.compute(eventId, (id, rec) -> {
            if (rec != null && rec.isTombstone()) {
                return stamp.isAfter(rec.clearedAt) ? WorldEventRecord.tombstone(stamp) : rec;
            }
            if (rec != null && rec.observedAt.isAfter(stamp)) {
                ignoredBecause[0] = "it is older than the latest sighting of this obstruction";
                return rec;
            }
            if (rec != null) {
                rec.edge.clearObstructionEvent(id);
                clearedFrom[0] = rec.edge;
            }
            return WorldEventRecord.tombstone(stamp);
        });

        if (ignoredBecause[0] != null) {
            System.out.println("clearWorldObstruction '" + eventId + "' ignored: " + ignoredBecause[0] + ".");
            return;
        }
        if (clearedFrom[0] == null) {
            System.out.println("clearWorldObstruction: no active event '" + eventId
                    + "' found - recorded, so an older report of it cannot revive it.");
            return;
        }
        reactToCorridorChange(clearedFrom[0].getFromId(), clearedFrom[0].getToId(), robotX, robotY);
    }

    private void reactToCorridorChange(
            String fromId, String toId, double robotX, double robotY) {
        for (RobotMission mission : missions.values()) {
            handleGraphChangeForMission(mission, fromId, toId, robotX, robotY);
        }
    }

    /** Passage evidence: a robot that moved beyond a STALE obstruction's recorded position without incident has verified it gone. */
    private void verifyStaleObstructionsPassed(
            String robotId, RobotMission mission, double x, double y) {
        GraphNode from = mission.edgeFromNode();
        GraphNode to = mission.currentTarget();
        if (from == null || to == null) {
            return;
        }
        GraphEdge edge = graph.getEdge(from.getId(), to.getId());
        if (edge == null) {
            return;
        }
        announceCleared(robotId, edge.clearStaleObstructionsPassedBy(
                mission.progressAlongCurrentEdge(x, y)));
        forgetWorldObstructions(edge.purgeExpiredObstructions());
    }

    private void announceCleared(String robotId, List<String> clearedIds) {
        Instant now = graph.now();
        for (String id : clearedIds) {
            // Verified gone by passage: tombstone it NOW so an older report cannot revive it.
            worldEventRegistry.computeIfPresent(id,
                    (k, rec) -> rec.isTombstone() ? rec : WorldEventRecord.tombstone(now));
            System.out.println("Stale obstruction '" + id + "' cleared: robot " + robotId
                    + " passed its location without incident.");
        }
    }

    private void forgetWorldObstructions(List<String> expiredIds) {
        for (String id : expiredIds) {
            worldEventRegistry.computeIfPresent(id, (k, rec) -> rec.isTombstone() ? rec : null);
        }
    }

    /** Memory hygiene for a future scheduler: drops expired events and tombstones past their retention. */
    public void purgeExpiredObstructions() {
        for (GraphEdge edge : graph.allEdges()) {
            forgetWorldObstructions(edge.purgeExpiredObstructions());
        }
        Instant now = graph.now();
        worldEventRegistry.entrySet().removeIf(entry -> entry.getValue().isTombstone()
                && java.time.Duration.between(entry.getValue().clearedAt, now)
                        .compareTo(TOMBSTONE_RETENTION) > 0);
    }

    int trackedWorldObstructionCount() {
        int live = 0;
        for (WorldEventRecord rec : worldEventRegistry.values()) {
            if (!rec.isTombstone()) {
                live++;
            }
        }
        return live;
    }

    int trackedTombstoneCount() {
        int tombstones = 0;
        for (WorldEventRecord rec : worldEventRegistry.values()) {
            if (rec.isTombstone()) {
                tombstones++;
            }
        }
        return tombstones;
    }

    /**
     * Opportunistic verification by a normal robot's proximity reading. If it matches (within
     * tolerance) the predicted distance to a STALE obstruction ahead on the robot's current edge,
     * that obstruction is re-confirmed (back to FRESH) and the normal reaction runs.
     * NOT WIRED to RobotChannelService - the safety chain currently pre-empts routing within
     * 3.0m of an obstacle, so a retreat command would not be dispatched there; decide that first.
     */
    public void observeProximity(
            String robotId, double robotX, double robotY, Double nearestObstacleDistance) {
        if (!graph.isStaleProbingEnabled()) {
            return;
        }
        if (nearestObstacleDistance == null) {
            return;
        }
        RobotMission mission = missions.get(robotId);
        EdgeInProgress inProgress = currentEdges.get(robotId);
        if (mission == null || mission.isComplete() || inProgress == null
                || recovering.containsKey(robotId)) {
            return;
        }
        GraphEdge edge = graph.getEdge(inProgress.fromId, inProgress.toId);
        GraphNode from = graph.getNode(inProgress.fromId);
        GraphNode to = graph.getNode(inProgress.toId);
        if (edge == null || from == null || to == null) {
            return;
        }
        double edgeLength = from.distanceTo(to);
        double progress = mission.progressAlongCurrentEdge(robotX, robotY);

        ObstructionView best = null;
        double bestDelta = Double.POSITIVE_INFINITY;
        for (ObstructionView view : edge.getObstructionViews()) {
            if (view.state != ExpiryPolicy.State.STALE || view.positionFraction == null
                    || view.positionFraction < progress) {
                continue;
            }
            double predicted = (view.positionFraction - progress) * edgeLength;
            double delta = Math.abs(predicted - nearestObstacleDistance);
            if (delta <= PROXIMITY_CORROBORATION_TOLERANCE_METERS && delta < bestDelta) {
                best = view;
                bestDelta = delta;
            }
        }
        if (best != null && edge.confirmObstructionEvent(best.eventId)) {
            System.out.println("Stale obstruction '" + best.eventId
                    + "' corroborated by a proximity reading of " + nearestObstacleDistance
                    + "m - re-confirmed.");
            reactToCorridorChange(inProgress.fromId, inProgress.toId, robotX, robotY);
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
