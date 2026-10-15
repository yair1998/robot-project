package com.robotproject.server.routing;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

/**
 * One directed connection between two nodes. Passability belongs to the physical corridor (a
 * shared Corridor), never to the edge: there is no stored or settable capacity here, so the two
 * directions cannot disagree. A*'s per-edge cost is DERIVED on every call.
 */
public class GraphEdge {

    public static final double UNCONSTRAINED_WIDTH_METERS = 1000.0;
    public static final double DEFAULT_ROBOT_WIDTH_METERS = 0.6;
    private static final double SAFETY_MARGIN_METERS = 0.3;

    private final String fromId;
    private final String toId;
    private final double baseCost;

    private final Corridor corridor;
    private final boolean reversedRelativeToCorridor;

    public GraphEdge(String fromId, String toId, double baseCost) {
        this(fromId, toId, baseCost, new Corridor(new ObstructionContext()), false);
    }

    // Package-private: RouteGraph uses this so both halves of a bidirectional pair share one Corridor.
    GraphEdge(String fromId, String toId, double baseCost,
            Corridor corridor, boolean reversedRelativeToCorridor) {
        this.fromId = fromId;
        this.toId = toId;
        this.baseCost = baseCost;
        this.corridor = corridor;
        this.reversedRelativeToCorridor = reversedRelativeToCorridor;
    }

    public String getFromId() {
        return fromId;
    }

    public String getToId() {
        return toId;
    }

    /** DERIVED, read-only 0.0-1.0 factor from blocking obstructions and the structural width. No setter exists on purpose. */
    public double getCapacityFactor() {
        return computeCapacityFromWidth(
                corridor.snapshot(DEFAULT_ROBOT_WIDTH_METERS).effectiveWidthMeters,
                DEFAULT_ROBOT_WIDTH_METERS);
    }

    /** Expected extra cost (meters) from STALE obstructions on this corridor. */
    public double getStalePenaltyMeters() {
        return corridor.snapshot(DEFAULT_ROBOT_WIDTH_METERS).stalePenaltyMeters;
    }

    public double effectiveCost() {
        // ONE snapshot: capacity and stale penalty come from the same instant.
        Corridor.Passability p = corridor.snapshot(DEFAULT_ROBOT_WIDTH_METERS);
        double capacity = computeCapacityFromWidth(p.effectiveWidthMeters, DEFAULT_ROBOT_WIDTH_METERS);
        if (capacity <= 0.0001) {
            return Double.POSITIVE_INFINITY;
        }
        return baseCost / capacity + p.stalePenaltyMeters;
    }

    public double getPassableWidthMeters() {
        return corridor.getPassableWidthMeters();
    }

    /** True once a real structural width was set; false means "unknown" (the unconstrained placeholder). */
    public boolean isPassableWidthKnown() {
        return corridor.getPassableWidthMeters() < UNCONSTRAINED_WIDTH_METERS;
    }

    public void setPassableWidthMeters(double widthMeters) {
        corridor.setPassableWidthMeters(widthMeters);
    }

    /** Position unknown, kind UNKNOWN, observed now. */
    public boolean reportObstructionEvent(String eventId, double remainingWidthMeters) {
        return reportObstructionEvent(eventId, remainingWidthMeters, null, null, null);
    }

    /** Position given in THIS edge's own convention (0 at its fromId, 1 at its toId); kind UNKNOWN, observed now. */
    public boolean reportObstructionEventAt(
            String eventId, double remainingWidthMeters, double positionFraction) {
        return reportObstructionEvent(eventId, remainingWidthMeters, positionFraction, null, null);
    }

    /** Full form. Returns false if ignored (older than the stored observation, or already expired). */
    public boolean reportObstructionEvent(String eventId, double remainingWidthMeters,
            Double positionFraction, ObstructionKind kind, Instant observedAt) {
        return corridor.reportObstructionEvent(eventId, remainingWidthMeters,
                toCanonicalPosition(positionFraction), kind, observedAt);
    }

    public boolean confirmObstructionEvent(String eventId) {
        return corridor.confirmObstructionEvent(eventId);
    }

    public void clearObstructionEvent(String eventId) {
        corridor.clearObstructionEvent(eventId);
    }

    // Package-private: lets MissionManager record WHY an obstruction left (e.g. MOVED).
    void clearObstructionEventAs(String eventId, ObstructionHistoryEntry.EndReason reason) {
        corridor.clearObstructionEvent(eventId, reason);
    }

    /** Clears STALE events lying behind positionFraction (THIS edge's convention) in its direction of travel. */
    public List<String> clearStaleObstructionsPassedBy(double positionFraction) {
        return corridor.clearStaleEventsPassed(
                toCanonicalPosition(positionFraction), !reversedRelativeToCorridor);
    }

    public List<String> purgeExpiredObstructions() {
        return corridor.purgeExpired();
    }

    public int physicalObstructionCount() {
        return corridor.physicalEventCount();
    }

    /** Snapshots with positionFraction converted to THIS edge's convention. */
    public List<ObstructionView> getObstructionViews() {
        List<ObstructionView> local = new ArrayList<>();
        for (ObstructionView v : corridor.listEvents()) {
            Double position = v.positionFraction == null ? null
                    : (reversedRelativeToCorridor ? 1.0 - v.positionFraction : v.positionFraction);
            local.add(new ObstructionView(v.eventId, v.remainingWidthMeters, position,
                    v.kind, v.state, v.existenceProbability));
        }
        return local;
    }

    public double getCurrentEffectiveWidthMeters() {
        return corridor.getCurrentEffectiveWidthMeters();
    }

    public double getEffectiveWidthAheadOfPosition(double positionFraction) {
        double canonicalRobotPos = toCanonicalPosition(positionFraction);
        double canonicalAheadEnd = reversedRelativeToCorridor ? 0.0 : 1.0;
        return corridor.getEffectiveWidthInCanonicalRange(canonicalRobotPos, canonicalAheadEnd);
    }

    public double getEffectiveWidthBehindPosition(double positionFraction) {
        double canonicalRobotPos = toCanonicalPosition(positionFraction);
        double canonicalBehindEnd = reversedRelativeToCorridor ? 1.0 : 0.0;
        return corridor.getEffectiveWidthInCanonicalRange(canonicalRobotPos, canonicalBehindEnd);
    }

    /** Reference equality on the shared Corridor, deliberately not id matching. */
    boolean sharesCorridorWith(GraphEdge other) {
        return other != null && this.corridor == other.corridor;
    }

    private Double toCanonicalPosition(Double localPositionFraction) {
        if (localPositionFraction == null) {
            return null;
        }
        return reversedRelativeToCorridor ? (1.0 - localPositionFraction) : localPositionFraction;
    }

    public static double computeCapacityFromWidth(
            double effectiveWidthMeters, double robotWidthMeters) {
        if (effectiveWidthMeters <= robotWidthMeters) {
            return 0.0;
        }
        if (effectiveWidthMeters >= robotWidthMeters + SAFETY_MARGIN_METERS) {
            return 1.0;
        }
        return (effectiveWidthMeters - robotWidthMeters) / SAFETY_MARGIN_METERS;
    }
}
