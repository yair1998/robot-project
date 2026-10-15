package com.robotproject.server.routing;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The physical corridor both directions of a pair traverse: the single source of truth for
 * passability, in METERS. Positions are canonical: 0.0 at the "forward" fromId, 1.0 at its toId; each
 * GraphEdge translates through its reversed flag. A one-way edge gets a private Corridor.
 * Thread safety: events are immutable and the map is a ConcurrentHashMap (weakly consistent iteration),
 * because A* reads while a perception thread writes.
 */
class Corridor {

    private static final double POSITION_EPSILON = 1e-9;

    /** One consistent reading at a single instant: one clock read, one pass over the events. */
    static final class Passability {
        final double effectiveWidthMeters;
        final double stalePenaltyMeters;

        Passability(double effectiveWidthMeters, double stalePenaltyMeters) {
            this.effectiveWidthMeters = effectiveWidthMeters;
            this.stalePenaltyMeters = stalePenaltyMeters;
        }
    }

    private static final class ObstructionEvent {
        final double remainingWidthMeters;
        final Double canonicalPositionFraction; // null = unknown location
        final ObstructionKind kind;
        final Instant observedAt;

        ObstructionEvent(double remainingWidthMeters, Double canonicalPositionFraction,
                ObstructionKind kind, Instant observedAt) {
            this.remainingWidthMeters = remainingWidthMeters;
            this.canonicalPositionFraction = canonicalPositionFraction;
            this.kind = kind;
            this.observedAt = observedAt;
        }
    }

    private final ObstructionContext context;
    private volatile double passableWidthMeters = GraphEdge.UNCONSTRAINED_WIDTH_METERS;
    private final Map<String, ObstructionEvent> activeObstructions = new ConcurrentHashMap<>();

    Corridor(ObstructionContext context) {
        this.context = context;
    }

    double getPassableWidthMeters() {
        return passableWidthMeters;
    }

    void setPassableWidthMeters(double widthMeters) {
        this.passableWidthMeters = widthMeters;
    }

    private Duration ageOf(ObstructionEvent event, Instant now) {
        Duration age = Duration.between(event.observedAt, now);
        return age.isNegative() ? Duration.ZERO : age;
    }

    private ExpiryPolicy.State stateOf(ObstructionEvent event, Instant now) {
        return context.policy().stateOf(event.kind, ageOf(event, now));
    }

    /** FRESH always blocks; STALE blocks only while stale probing is switched off. */
    private boolean blocks(ObstructionEvent event, Instant now) {
        ExpiryPolicy.State state = stateOf(event, now);
        return state == ExpiryPolicy.State.FRESH
                || (state == ExpiryPolicy.State.STALE && !context.staleProbingEnabled());
    }

    /**
     * Records or refreshes an event. Returns false (nothing changed) if an existing event with the same
     * id was observed LATER than this report, or if this report is already past its hard TTL.
     * observedAt in the future is clamped to now; null means now.
     */
    boolean reportObstructionEvent(String eventId, double remainingWidthMeters,
            Double canonicalPositionFraction, ObstructionKind kind, Instant observedAt) {
        Instant now = context.now();
        Instant stamp = (observedAt == null || observedAt.isAfter(now)) ? now : observedAt;
        ObstructionKind effectiveKind = (kind == null) ? ObstructionKind.UNKNOWN : kind;
        ObstructionEvent incoming = new ObstructionEvent(
                remainingWidthMeters, canonicalPositionFraction, effectiveKind, stamp);

        if (stateOf(incoming, now) == ExpiryPolicy.State.GONE) {
            return false;
        }
        boolean[] applied = {false};
        activeObstructions.compute(eventId, (id, existing) -> {
            if (existing != null && existing.observedAt.isAfter(stamp)) {
                return existing;
            }
            applied[0] = true;
            return incoming;
        });
        return applied[0];
    }

    /** Re-stamps an existing, not-yet-GONE event as observed now. False if absent or GONE. */
    boolean confirmObstructionEvent(String eventId) {
        Instant now = context.now();
        boolean[] confirmed = {false};
        activeObstructions.computeIfPresent(eventId, (id, existing) -> {
            if (stateOf(existing, now) == ExpiryPolicy.State.GONE) {
                return existing;
            }
            confirmed[0] = true;
            return new ObstructionEvent(existing.remainingWidthMeters,
                    existing.canonicalPositionFraction, existing.kind, now);
        });
        return confirmed[0];
    }

    void clearObstructionEvent(String eventId) {
        clearObstructionEvent(eventId, ObstructionHistoryEntry.EndReason.CLEARED);
    }

    void clearObstructionEvent(String eventId, ObstructionHistoryEntry.EndReason reason) {
        ObstructionEvent removed = activeObstructions.remove(eventId);
        if (removed != null) {
            context.recordEnded(eventId, removed.kind, removed.observedAt, reason);
        }
    }

    /** One clock read and one pass: narrowest blocking width plus the worst STALE expected-encounter cost. */
    Passability snapshot(double robotWidthMeters) {
        Instant now = context.now();
        ExpiryPolicy policy = context.policy();
        boolean probing = context.staleProbingEnabled();
        double structural = passableWidthMeters;
        double narrowest = structural;
        double worstPenalty = 0.0;
        for (ObstructionEvent event : activeObstructions.values()) {
            Duration age = ageOf(event, now);
            ExpiryPolicy.State state = policy.stateOf(event.kind, age);
            if (state == ExpiryPolicy.State.FRESH || (state == ExpiryPolicy.State.STALE && !probing)) {
                narrowest = Math.min(narrowest, event.remainingWidthMeters);
            } else if (state == ExpiryPolicy.State.STALE) {
                double probability = policy.existenceProbability(event.kind, age);
                double widthIfReal = Math.min(structural, event.remainingWidthMeters);
                double capacityIfReal = GraphEdge.computeCapacityFromWidth(widthIfReal, robotWidthMeters);
                worstPenalty = Math.max(worstPenalty,
                        policy.getEncounterPenaltyMeters() * probability * (1.0 - capacityIfReal));
            }
        }
        return new Passability(narrowest, worstPenalty);
    }

    /** Narrowest width among the structural floor and blocking events (position ignored). */
    double getCurrentEffectiveWidthMeters() {
        return snapshot(GraphEdge.DEFAULT_ROBOT_WIDTH_METERS).effectiveWidthMeters;
    }

    /**
     * Same, restricted to blocking events within [a,b] of canonical position (either order). The
     * structural floor is always included (no position); an event with unknown position is always
     * included (treated conservatively as "could be anywhere").
     */
    double getEffectiveWidthInCanonicalRange(double a, double b) {
        double lo = Math.min(a, b);
        double hi = Math.max(a, b);
        Instant now = context.now();
        double narrowest = passableWidthMeters;
        for (ObstructionEvent event : activeObstructions.values()) {
            if (!blocks(event, now)) {
                continue;
            }
            boolean inRange = event.canonicalPositionFraction == null
                    || (event.canonicalPositionFraction >= lo && event.canonicalPositionFraction <= hi);
            if (inRange) {
                narrowest = Math.min(narrowest, event.remainingWidthMeters);
            }
        }
        return narrowest;
    }

    double getStaleEncounterPenaltyMeters(double robotWidthMeters) {
        return snapshot(robotWidthMeters).stalePenaltyMeters;
    }

    /**
     * Passage evidence: removes STALE events with a known position lying strictly behind the robot's
     * canonical position, given its direction of travel. Only when stale probing is ON.
     */
    List<String> clearStaleEventsPassed(double canonicalRobotPosition, boolean movingTowardHigherPositions) {
        List<String> cleared = new ArrayList<>();
        if (!context.staleProbingEnabled()) {
            return cleared;
        }
        Instant now = context.now();
        for (Map.Entry<String, ObstructionEvent> entry : activeObstructions.entrySet()) {
            ObstructionEvent event = entry.getValue();
            if (event.canonicalPositionFraction == null
                    || stateOf(event, now) != ExpiryPolicy.State.STALE) {
                continue;
            }
            boolean passed = movingTowardHigherPositions
                    ? event.canonicalPositionFraction < canonicalRobotPosition - POSITION_EPSILON
                    : event.canonicalPositionFraction > canonicalRobotPosition + POSITION_EPSILON;
            if (passed && activeObstructions.remove(entry.getKey(), event)) {
                context.recordEnded(entry.getKey(), event.kind, event.observedAt,
                        ObstructionHistoryEntry.EndReason.PASSED);
                cleared.add(entry.getKey());
            }
        }
        return cleared;
    }

    /** Physically removes GONE events (queries already ignore them). Returns the removed ids. */
    List<String> purgeExpired() {
        Instant now = context.now();
        List<String> removed = new ArrayList<>();
        for (Map.Entry<String, ObstructionEvent> entry : activeObstructions.entrySet()) {
            ObstructionEvent event = entry.getValue();
            if (stateOf(event, now) == ExpiryPolicy.State.GONE
                    && activeObstructions.remove(entry.getKey(), event)) {
                context.recordEnded(entry.getKey(), event.kind, event.observedAt,
                        ObstructionHistoryEntry.EndReason.EXPIRED);
                removed.add(entry.getKey());
            }
        }
        return removed;
    }

    int physicalEventCount() {
        return activeObstructions.size();
    }

    /** Snapshot of every physically present event (incl. GONE not yet purged), canonical positions. */
    List<ObstructionView> listEvents() {
        Instant now = context.now();
        ExpiryPolicy policy = context.policy();
        List<ObstructionView> views = new ArrayList<>();
        for (Map.Entry<String, ObstructionEvent> entry : activeObstructions.entrySet()) {
            ObstructionEvent event = entry.getValue();
            Duration age = ageOf(event, now);
            views.add(new ObstructionView(entry.getKey(), event.remainingWidthMeters,
                    event.canonicalPositionFraction, event.kind,
                    policy.stateOf(event.kind, age), policy.existenceProbability(event.kind, age)));
        }
        return views;
    }
}
