package com.robotproject.server.routing;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.ReentrantLock;

public class MissionManager {

    private static final double ARRIVAL_THRESHOLD_METERS = 0.5;
    private static final double FULLY_BLOCKED_CAPACITY = 0.0001;
    private static final double PROXIMITY_CORROBORATION_TOLERANCE_METERS = 2.0;

    public enum ReportOutcome { APPLIED, REFRESHED, IGNORED_OLDER, IGNORED_EXPIRED, IGNORED_TOO_FAR, IGNORED_AFTER_CLEAR }
    public enum ClearOutcome { CLEARED, RECORDED_ONLY, IGNORED_OLDER }

    /** A target the robot should be sent, tagged with the epoch in which it was decided. */
    public static final class Dispatch {
        public final GraphNode target;      // null = nothing to send
        public final long epoch;
        Dispatch(GraphNode target, long epoch) { this.target = target; this.epoch = epoch; }
    }

    public interface DispatchListener {
        /** Something is waiting to be sent to this robot (a retreat was just decided). Called with NO lock held. */
        void onDispatchNeeded(String robotId);
    }

    private static final class RobotPosition {
        final double x; final double y; final Instant at;
        RobotPosition(double x, double y, Instant at) { this.x = x; this.y = y; this.at = at; }
    }

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

    // Lock rule L1: one lock per robot guards its mission, recovery state, tracked edge, held
    // dispatch and pending reassessments. Never call reactToCorridorChange or a listener while
    // holding a robot lock; never hold two robot locks.
    private final Map<String, ReentrantLock> robotLocks = new ConcurrentHashMap<>();
    private final Map<String, RobotPosition> lastPositions = new ConcurrentHashMap<>();
    private final Map<String, AtomicLong> dispatchEpochs = new ConcurrentHashMap<>();
    private final Map<String, Dispatch> heldDispatches = new ConcurrentHashMap<>();
    private final Map<String, Set<String>> pendingReassessment = new ConcurrentHashMap<>();   // guarded by the robot lock
    private volatile Duration positionMaxAge = Duration.ofSeconds(3);
    private volatile DispatchListener dispatchListener;

    public MissionManager(RouteGraph graph, AStarRouter router) {
        this.graph = graph;
        this.router = router;
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    private ReentrantLock lockFor(String robotId) { return robotLocks.computeIfAbsent(robotId, id -> new ReentrantLock()); }
    private AtomicLong epochOf(String robotId) { return dispatchEpochs.computeIfAbsent(robotId, id -> new AtomicLong()); }
    public long currentEpoch(String robotId) { return epochOf(robotId).get(); }                 // lock-free
    public void setDispatchListener(DispatchListener l) { this.dispatchListener = l; }
    public void setPositionMaxAge(Duration d) { this.positionMaxAge = java.util.Objects.requireNonNull(d); }
    public Duration getPositionMaxAge() { return positionMaxAge; }
    public Instant now() { return graph.now(); }
    RouteGraph graph() { return graph; }                                                        // for tests in this package
    public List<String> corridorsWithUnknownWidth() { return graph.corridorsWithUnknownWidth(); } // for RobotServer's startup line

    /** Records where a robot is, with the receipt time. Called for EVERY telemetry message before any decision. Never decides. */
    public void updateRobotPosition(String robotId, double x, double y) {
        lastPositions.put(robotId, new RobotPosition(x, y, graph.now()));
    }
    public Duration positionAge(String robotId) {                       // null if never recorded
        RobotPosition p = lastPositions.get(robotId);
        return p == null ? null : Duration.between(p.at, graph.now());
    }
    public double[] lastKnownPosition(String robotId) {                 // null if never recorded; may be stale
        RobotPosition p = lastPositions.get(robotId);
        return p == null ? null : new double[] {p.x, p.y};
    }
    private RobotPosition freshPositionOf(String robotId) {             // null when missing or older than positionMaxAge
        RobotPosition p = lastPositions.get(robotId);
        if (p == null || Duration.between(p.at, graph.now()).compareTo(positionMaxAge) > 0) { return null; }
        return p;
    }
    public RobotMission.Status missionStatus(String robotId) {          // null if no mission; takes the robot lock
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try { RobotMission m = missions.get(robotId); return m == null ? null : m.status(); }
        finally { lock.unlock(); }
    }
    /** For tests: structural invariants of one robot's state. Empty list = healthy. */
    List<String> checkInvariants(String robotId) {
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try {
            List<String> v = new ArrayList<>();
            RobotMission m = missions.get(robotId);
            if (m == null) { return v; }
            if (m.remainingNodes().isEmpty()) { v.add("empty path"); }
            boolean tracked = currentEdges.get(robotId) != null;
            if (m.status() == RobotMission.Status.EN_ROUTE) {
                if (m.currentTarget() == null) { v.add("EN_ROUTE without a current target"); }
                if (!tracked) { v.add("EN_ROUTE without a tracked current edge"); }
            } else if (tracked && !recovering.containsKey(robotId)) {
                v.add(m.status() + " but a current edge is still tracked");
            }
            return v;
        } finally { lock.unlock(); }
    }

    // ------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------
    public GraphNode onTelemetry(String robotId, double x, double y, String destinationNodeId) {
        return onTelemetryDispatch(robotId, x, y, destinationNodeId).target;
    }

    public Dispatch onTelemetryDispatch(String robotId, double x, double y, String destinationNodeId) {
        updateRobotPosition(robotId, x, y);
        ReentrantLock lock = lockFor(robotId);
        lock.lock();
        try {
            GraphNode target = tickLocked(robotId, x, y, destinationNodeId);
            return new Dispatch(target, epochOf(robotId).get());
        } finally { lock.unlock(); }
    }

    private GraphNode tickLocked(String robotId, double x, double y, String destinationNodeId) {
        RobotMission existing = missions.get(robotId);
        if (existing != null) { drainPendingReassessments(robotId, existing); }   // decisions deferred for lack of a fresh position

        RecoveryState recovery = recovering.get(robotId);
        if (recovery != null) { return handleRecovery(robotId, recovery, x, y); }

        RobotMission mission = missions.computeIfAbsent(robotId, id -> startNewMission(id, x, y, destinationNodeId));
        RobotMission.Status status = mission.status();
        if (status == RobotMission.Status.AT_DESTINATION) { return null; }
        if (status == RobotMission.Status.NO_ROUTE) { return retryStrandedMission(robotId, mission); }

        verifyStaleObstructionsPassed(robotId, mission, x, y);

        GraphNode target = mission.currentTarget();
        if (distance(x, y, target.getX(), target.getY()) < ARRIVAL_THRESHOLD_METERS) {
            mission.advance();
            System.out.println("Robot " + robotId + " confirmed at " + mission.getLastConfirmedNodeId());
            syncCurrentEdge(mission);
            if (mission.status() == RobotMission.Status.NO_ROUTE) {
                System.out.println("Robot " + robotId + " is at " + mission.getLastConfirmedNodeId()
                        + " but has no route to " + mission.getDestinationNodeId() + " - holding position and retrying.");
                return retryStrandedMission(robotId, mission);
            }
            return mission.currentTarget();          // null when the destination was reached
        }
        return null;
    }

    /** A stranded robot re-plans on every telemetry tick (the graph is small) and dispatches only when a route exists. */
    private GraphNode retryStrandedMission(String robotId, RobotMission mission) {
        String from = mission.getLastConfirmedNodeId();
        if (from == null) { return null; }
        List<GraphNode> path = router.findPath(graph, from, mission.getDestinationNodeId());
        if (path.size() < 2) { return null; }
        mission.replacePath(new ArrayList<>(path));
        syncCurrentEdge(mission);
        System.out.println("Robot " + robotId + ": a route to " + mission.getDestinationNodeId()
                + " exists again from " + from + " - resuming.");
        return mission.currentTarget();
    }

    private GraphNode handleRecovery(String robotId, RecoveryState recovery, double x, double y) {
        double d = distance(x, y, recovery.retreatTarget.getX(), recovery.retreatTarget.getY());
        if (d < ARRIVAL_THRESHOLD_METERS) {
            RobotMission mission = missions.get(robotId);
            List<GraphNode> newPath = router.findPath(graph, recovery.retreatTarget.getId(), mission.getDestinationNodeId());
            recovering.remove(robotId);
            if (newPath.isEmpty()) {
                mission.replacePath(new ArrayList<>(List.of(recovery.retreatTarget)));
                syncCurrentEdge(mission);
                System.out.println("Robot " + robotId + " confirmed retreat to " + recovery.retreatTarget.getId()
                        + " - but no route to " + mission.getDestinationNodeId() + " exists; holding position and retrying.");
                return null;
            }
            mission.replacePath(newPath);
            syncCurrentEdge(mission);
            System.out.println("Robot " + robotId + " confirmed retreat to " + recovery.retreatTarget.getId()
                    + " - replanning to " + mission.getDestinationNodeId());
            return mission.currentTarget();
        }
        if (!recovery.dispatched) { recovery.dispatched = true; return recovery.retreatTarget; }
        return null;
    }

    // ------------------------------------------------------------------
    // Dispatch API
    // ------------------------------------------------------------------
    /** A retreat decided but not yet handed to anyone: claims it (dispatched = true). Null if there is none. */
    public Dispatch takeRetreatDispatch(String robotId) {
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try {
            RecoveryState recovery = recovering.get(robotId);
            if (recovery == null || recovery.dispatched) { return null; }
            recovery.dispatched = true;
            return new Dispatch(recovery.retreatTarget, epochOf(robotId).get());
        } finally { lock.unlock(); }
    }

    /** One held slot per robot; the latest wins. Ignores null and a null target. */
    public void holdDispatch(String robotId, Dispatch d) {
        if (d == null || d.target == null) { return; }
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try { heldDispatches.put(robotId, d); }
        finally { lock.unlock(); }
    }

    public Dispatch takeHeldDispatch(String robotId) {
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try { return heldDispatches.remove(robotId); }
        finally { lock.unlock(); }
    }

    public boolean hasHeldDispatch(String robotId) {                    // lock-free
        return heldDispatches.containsKey(robotId);
    }

    /** The retreat target while retreating, else the mission's current target (null if none). */
    public GraphNode desiredTargetFor(String robotId) {
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try { return desiredTargetLocked(robotId); }
        finally { lock.unlock(); }
    }

    public Dispatch desiredDispatchFor(String robotId) {
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try { return new Dispatch(desiredTargetLocked(robotId), epochOf(robotId).get()); }
        finally { lock.unlock(); }
    }

    private GraphNode desiredTargetLocked(String robotId) {
        RecoveryState recovery = recovering.get(robotId);
        if (recovery != null) { return recovery.retreatTarget; }
        RobotMission mission = missions.get(robotId);
        return mission == null ? null : mission.currentTarget();
    }

    /** Connection lost: a retreat must be re-sent to the next connection, a held slot is dropped, earlier decisions are superseded. */
    public void rearmDispatch(String robotId) {
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try {
            RecoveryState recovery = recovering.get(robotId);
            if (recovery != null) { recovery.dispatched = false; }
            heldDispatches.remove(robotId);
            epochOf(robotId).incrementAndGet();
        } finally { lock.unlock(); }
    }

    public GraphNode initialTargetFor(String robotId) {
        return desiredTargetFor(robotId);
    }

    // ------------------------------------------------------------------
    // Reaction to corridor changes
    // ------------------------------------------------------------------
    private void reactToCorridorChange(String fromId, String toId) {
        List<String> needDispatch = new ArrayList<>();
        for (RobotMission mission : missions.values()) {
            if (handleGraphChangeForMission(mission, fromId, toId)) { needDispatch.add(mission.getRobotId()); }
        }
        DispatchListener listener = dispatchListener;
        if (listener != null) {                                           // AFTER every robot lock is released (L1)
            for (String robotId : needDispatch) { listener.onDispatchNeeded(robotId); }
        }
    }

    /** Returns true only when a retreat was started (something now waits to be sent). */
    private boolean handleGraphChangeForMission(RobotMission mission, String changedFrom, String changedTo) {
        String robotId = mission.getRobotId();
        ReentrantLock lock = lockFor(robotId); lock.lock();
        try {
            if (mission.currentTarget() == null) { return false; }       // at the destination or stranded
            RobotPosition position = freshPositionOf(robotId);
            if (position == null) {
                pendingReassessment.computeIfAbsent(robotId, id -> new LinkedHashSet<>()).add(changedFrom + "->" + changedTo);
                System.out.println("Robot " + robotId + " has no fresh position - the decision about corridor "
                        + changedFrom + "->" + changedTo + " is deferred until its next telemetry.");
                return false;
            }
            return decideLocked(mission, changedFrom, changedTo, position);
        } finally { lock.unlock(); }
    }

    private void drainPendingReassessments(String robotId, RobotMission mission) {   // robot lock held
        Set<String> pending = pendingReassessment.remove(robotId);
        if (pending == null) { return; }
        RobotPosition position = lastPositions.get(robotId);       // just updated by this telemetry: fresh by construction
        for (String key : pending) {
            int arrow = key.indexOf("->");
            decideLocked(mission, key.substring(0, arrow), key.substring(arrow + 2), position);
        }
        // a retreat started here is dispatched by the recovery check that follows in tickLocked
    }

    /** The decision itself (robot lock held). Returns true only when a retreat was started. */
    private boolean decideLocked(
            RobotMission mission, String changedFrom, String changedTo, RobotPosition position) {

        GraphNode currentTo = mission.currentTarget();
        if (currentTo == null) {
            return false; // mission already complete
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
                return false;
            }

            if (recovering.containsKey(robotId)) {
                System.out.println(
                        "WARNING: retreat edge (" + changedFrom + "->" + changedTo
                                + ") became fully blocked while robot " + robotId
                                + " was already retreating along it. No automatic routing "
                                + "action taken - this case needs explicit design review.");
                return false;
            }

            double robotProgress = mission.progressAlongCurrentEdge(position.x, position.y);
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
                return false;
            }

            if (!somethingAheadIsBlocking && !somethingBehindIsBlocking) {
                // Cannot happen for validly positioned events - defensive, never silent.
                System.out.println(
                        "WARNING: edge is fully blocked but no blocking event was located on "
                                + "either side of robot " + robotId + " - no automatic routing "
                                + "action taken, this case needs explicit design review.");
                return false;
            }

            if (!somethingAheadIsBlocking) {
                System.out.println(
                        "Current edge fully blocked, but nothing severe ahead of the robot's "
                                + "position - continuing forward to " + currentTo.getId()
                                + " instead of retreating.");
                replanFrom(mission, currentTo.getId());
                return false;
            }

            GraphNode currentFrom = mission.edgeFromNode();
            System.out.println(
                    "Current edge fully blocked - retreating to "
                            + currentFrom.getId() + " before replanning.");
            recovering.put(robotId, new RecoveryState(currentFrom));
            currentEdges.put(robotId,
                    new EdgeInProgress(currentTo.getId(), currentFrom.getId()));
            epochOf(robotId).incrementAndGet();      // anything decided earlier for this robot is now superseded
            return true;

        } else if (pathStillUsesEdge(mission, changedFrom, changedTo)) {
            System.out.println("A future edge on the planned route changed - replanning ahead.");
            replanFrom(mission, currentTo.getId());
        }
        return false;
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
        List<GraphNode> continuation = router.findPath(graph, fromNodeId, mission.getDestinationNodeId());
        List<GraphNode> newPath = new ArrayList<>();
        newPath.add(mission.edgeFromNode());
        if (continuation.isEmpty()) {
            // No route onward: finish the in-flight leg the robot is committed to, hold at its end (NO_ROUTE) and retry.
            System.out.println("No route to " + mission.getDestinationNodeId() + " from " + fromNodeId
                    + " right now - robot " + mission.getRobotId() + " will finish its current leg, hold there and retry.");
            newPath.add(mission.currentTarget());
        } else {
            newPath.addAll(continuation);
        }
        mission.replacePath(newPath);
        syncCurrentEdge(mission);
    }

    private RobotMission startNewMission(String robotId, double x, double y, String destinationNodeId) {
        GraphNode start = nearestNode(x, y);
        List<GraphNode> path = router.findPath(graph, start.getId(), destinationNodeId);
        System.out.println("New mission for " + robotId + ": " + start.getId() + " -> " + destinationNodeId);
        if (path.isEmpty()) {
            System.out.println("No route from " + start.getId() + " to " + destinationNodeId
                    + " yet - robot " + robotId + " will hold position and retry.");
            path = new ArrayList<>(List.of(start));
        }
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

    // ------------------------------------------------------------------
    // Edge-id API
    // ------------------------------------------------------------------
    public void reportEdgeObstruction(
            String fromId, String toId, String eventId, double remainingWidthMeters,
            double positionFraction) {
        reportEdgeObstruction(fromId, toId, eventId, remainingWidthMeters, positionFraction, null, null);
    }

    public void reportEdgeObstruction(
            String fromId, String toId, String eventId, double remainingWidthMeters,
            double positionFraction, ObstructionKind kind, Instant observedAt) {

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
            reactToCorridorChange(fromId, toId);
        }
    }

    public void clearEdgeObstruction(String fromId, String toId, String eventId) {
        GraphEdge edge = graph.getEdge(fromId, toId);
        if (edge == null) {
            System.out.println("clearEdgeObstruction: no such edge " + fromId + "->" + toId);
            return;
        }
        edge.clearObstructionEvent(eventId);
        reactToCorridorChange(fromId, toId);
    }

    // ------------------------------------------------------------------
    // World API
    // ------------------------------------------------------------------
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
    public ReportOutcome reportWorldObstruction(
            String eventId, double worldX, double worldY, double obstacleWidthMeters) {
        return reportWorldObstruction(WorldObstructionReport.basic(eventId, worldX, worldY, obstacleWidthMeters));
    }

    public ReportOutcome reportWorldObstruction(WorldObstructionReport report) {
        RouteGraph.EdgeProjection projection = graph.findNearestEdge(report.worldX, report.worldY);
        if (projection == null
                || projection.perpendicularDistanceMeters > MAX_PROJECTION_DISTANCE_METERS) {
            System.out.println(
                    "reportWorldObstruction: world point (" + report.worldX + ", " + report.worldY
                            + ") is not close enough to any known edge (nearest is "
                            + (projection == null ? "none" : projection.perpendicularDistanceMeters + "m away")
                            + ") - ignoring.");
            return ReportOutcome.IGNORED_TOO_FAR;
        }
        GraphEdge edge = graph.getEdge(projection.fromId, projection.toId);
        if (edge == null) {
            return ReportOutcome.IGNORED_TOO_FAR;
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
        final ReportOutcome[] outcome = {ReportOutcome.APPLIED};

        // Atomic per event id: decide acceptance FIRST, touch corridors only if accepted.
        // L3: nothing in here takes a robot lock.
        worldEventRegistry.compute(report.eventId, (id, rec) -> {
            if (rec != null && rec.isTombstone() && !stamp.isAfter(rec.clearedAt)) {
                ignoredBecause[0] = "it was observed no later than the moment this obstruction was cleared";
                outcome[0] = ReportOutcome.IGNORED_AFTER_CLEAR;
                return rec;
            }
            if (rec != null && !rec.isTombstone() && rec.observedAt.isAfter(stamp)) {
                ignoredBecause[0] = "a newer observation of it is already recorded";
                outcome[0] = ReportOutcome.IGNORED_OLDER;
                return rec;
            }
            if (policy.stateOf(kind, java.time.Duration.between(stamp, now)) == ExpiryPolicy.State.GONE) {
                ignoredBecause[0] = "it is already expired";
                outcome[0] = ReportOutcome.IGNORED_EXPIRED;
                return rec;
            }
            GraphEdge old = (rec != null && !rec.isTombstone()) ? rec.edge : null;
            boolean sameCorridor = old != null && old.sharesCorridorWith(edge);
            ObstructionView before = sameCorridor ? viewOf(edge, id) : null;
            if (!edge.reportObstructionEvent(id, remainingWidth, projection.positionFraction, kind, stamp)) {
                ignoredBecause[0] = "the corridor holds a newer observation of it";
                outcome[0] = ReportOutcome.IGNORED_OLDER;
                return rec;
            }
            if (old != null && !sameCorridor) {
                old.clearObstructionEventAs(id, ObstructionHistoryEntry.EndReason.MOVED);
                movedFrom[0] = old;
            }
            if (sameCorridor && !materiallyDifferent(before, viewOf(edge, id))) {
                outcome[0] = ReportOutcome.REFRESHED;
            }
            return WorldEventRecord.live(edge, stamp);
        });

        if (ignoredBecause[0] != null) {
            System.out.println("World obstruction '" + report.eventId + "' ignored: " + ignoredBecause[0] + ".");
            return outcome[0];
        }
        if (outcome[0] == ReportOutcome.REFRESHED) {
            return ReportOutcome.REFRESHED;              // G5: an unchanged refresh is quiet and makes no robot react
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
            reactToCorridorChange(movedFrom[0].getFromId(), movedFrom[0].getToId());
        }
        System.out.println(
                "World obstruction '" + report.eventId + "' at (" + report.worldX + ", " + report.worldY
                        + ") mapped to edge " + projection.fromId + "->" + projection.toId
                        + " at fraction " + projection.positionFraction
                        + " (perpendicular distance " + projection.perpendicularDistanceMeters + "m)");
        reactToCorridorChange(projection.fromId, projection.toId);
        return ReportOutcome.APPLIED;
    }

    private ObstructionView viewOf(GraphEdge edge, String eventId) {
        for (ObstructionView v : edge.getObstructionViews()) {
            if (v.eventId.equals(eventId)) {
                return v;
            }
        }
        return null;
    }

    private boolean isBlocking(ObstructionView v) {
        return v != null && (v.state == ExpiryPolicy.State.FRESH
                || (v.state == ExpiryPolicy.State.STALE && !graph.isStaleProbingEnabled()));
    }

    private boolean materiallyDifferent(ObstructionView a, ObstructionView b) {   // widths, position (null-aware), blocking effect
        if (a == null || b == null) return true;
        if (Math.abs(a.remainingWidthMeters - b.remainingWidthMeters) > 1e-9) return true;
        if ((a.positionFraction == null) != (b.positionFraction == null)) return true;
        if (a.positionFraction != null && Math.abs(a.positionFraction - b.positionFraction) > 1e-9) return true;
        return isBlocking(a) != isBlocking(b);
    }

    /**
     * Clears a previously-reported world obstruction by its own event
     * id - Python only needs to remember the id it originally chose,
     * never the edge/direction Java derived internally for it.
     */
    public ClearOutcome clearWorldObstruction(String eventId) {
        return clearWorldObstruction(eventId, null);
    }

    /** clearedAt = when the clear was observed (null = now). Leaves a tombstone, even for an id never seen. */
    public ClearOutcome clearWorldObstruction(String eventId, Instant clearedAt) {
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
            return ClearOutcome.IGNORED_OLDER;
        }
        if (clearedFrom[0] == null) {
            System.out.println("clearWorldObstruction: no active event '" + eventId
                    + "' found - recorded, so an older report of it cannot revive it.");
            return ClearOutcome.RECORDED_ONLY;
        }
        reactToCorridorChange(clearedFrom[0].getFromId(), clearedFrom[0].getToId());
        return ClearOutcome.CLEARED;
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
     * The matching runs under the robot lock; the reaction runs only after it is released (L1).
     */
    public void observeProximity(
            String robotId, double robotX, double robotY, Double nearestObstacleDistance) {
        updateRobotPosition(robotId, robotX, robotY);
        if (!graph.isStaleProbingEnabled()) {
            return;
        }
        if (nearestObstacleDistance == null) {
            return;
        }
        String reactFrom = null;
        String reactTo = null;
        ReentrantLock lock = lockFor(robotId);
        lock.lock();
        try {
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
                reactFrom = inProgress.fromId;
                reactTo = inProgress.toId;
            }
        } finally {
            lock.unlock();
        }
        if (reactFrom != null) {
            reactToCorridorChange(reactFrom, reactTo);
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
