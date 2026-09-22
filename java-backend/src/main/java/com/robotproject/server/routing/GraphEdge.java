package com.robotproject.server.routing;

public class GraphEdge {

    private final String fromId;
    private final String toId;
    private final double baseCost;
    private double capacityFactor = 1.0;

    public GraphEdge(String fromId, String toId, double baseCost) {
        this.fromId = fromId;
        this.toId = toId;
        this.baseCost = baseCost;
    }

    public String getFromId() {
        return fromId;
    }

    public String getToId() {
        return toId;
    }

    public double getCapacityFactor() {
        return capacityFactor;
    }

    public void setCapacityFactor(double capacityFactor) {
        if (capacityFactor < 0.0 || capacityFactor > 1.0) {
            throw new IllegalArgumentException(
                    "capacityFactor must be between 0.0 and 1.0, got " + capacityFactor);
        }
        this.capacityFactor = capacityFactor;
    }

    public double effectiveCost() {
        if (capacityFactor <= 0.0001) {
            return Double.POSITIVE_INFINITY;
        }
        return baseCost / capacityFactor;
    }

    public static final double UNCONSTRAINED_WIDTH_METERS = 1000.0;
    public static final double DEFAULT_ROBOT_WIDTH_METERS = 0.6;
    private static final double SAFETY_MARGIN_METERS = 0.3;

    // Structural, static property of the corridor itself (e.g. a real
    // doorway that's always narrow) - unchanged from last round.
    // Optional: defaults to "effectively unconstrained".
    private double passableWidthMeters = UNCONSTRAINED_WIDTH_METERS;

    // NEW: currently-active dynamic obstruction reports, keyed by an
    // event id Python (or a test) supplies. A plain HashMap, consistent
    // with this class's existing lack of synchronization - the only
    // live caller today is a single manual/test call path, same
    // assumption already made by setCapacityFactor.
    private final java.util.Map<String, Double> activeObstructions = new java.util.HashMap<>();

    public double getPassableWidthMeters() {
        return passableWidthMeters;
    }

    public void setPassableWidthMeters(double widthMeters) {
        this.passableWidthMeters = widthMeters;
        recomputeCapacity(DEFAULT_ROBOT_WIDTH_METERS);
    }

    /**
     * Records (or updates) a dynamic obstruction by its own event id,
     * with the passable width remaining at the point it was detected.
     * Multiple different-id events can be active on the same edge at
     * once; the narrowest of all of them (plus the structural floor)
     * always wins. Recomputes and applies capacityFactor immediately
     * via the existing setCapacityFactor - AStarRouter/effectiveCost()
     * need zero changes, they already just read capacityFactor.
     */
    public void reportObstructionEvent(String eventId, double remainingWidthMeters) {
        reportObstructionEvent(eventId, remainingWidthMeters, DEFAULT_ROBOT_WIDTH_METERS);
    }

    public void reportObstructionEvent(
            String eventId, double remainingWidthMeters, double robotWidthMeters) {
        activeObstructions.put(eventId, remainingWidthMeters);
        recomputeCapacity(robotWidthMeters);
    }

    /**
     * Removes exactly this event's contribution. Any OTHER still-active
     * event continues to determine the effective width correctly - this
     * is precisely the case a single overridden scalar cannot handle.
     * If this was the last active event, the edge reverts to its
     * structural width (or fully open, if none was ever set) on its own.
     */
    public void clearObstructionEvent(String eventId) {
        clearObstructionEvent(eventId, DEFAULT_ROBOT_WIDTH_METERS);
    }

    public void clearObstructionEvent(String eventId, double robotWidthMeters) {
        activeObstructions.remove(eventId);
        recomputeCapacity(robotWidthMeters);
    }

    public double getCurrentEffectiveWidthMeters() {
        double narrowest = passableWidthMeters;
        for (double w : activeObstructions.values()) {
            narrowest = Math.min(narrowest, w);
        }
        return narrowest;
    }

    private void recomputeCapacity(double robotWidthMeters) {
        setCapacityFactor(
                computeCapacityFromWidth(getCurrentEffectiveWidthMeters(), robotWidthMeters));
    }

    /**
     * Pure, independently-testable width-to-capacity conversion - kept
     * as a static function for the same reason it was last round, just
     * simplified to take one already-resolved effective width instead
     * of two separate structural/temporary inputs (that merging now
     * happens in getCurrentEffectiveWidthMeters() above).
     */
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
