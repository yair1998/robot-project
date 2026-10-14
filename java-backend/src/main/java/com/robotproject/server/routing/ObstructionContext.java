package com.robotproject.server.routing;

import java.time.Clock;
import java.time.Instant;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Per-graph shared state for obstruction ageing: clock, policy, feature flag, assumptions, history. */
final class ObstructionContext {

    private static final int HISTORY_CAPACITY = 500;
    static final double DEFAULT_ASSUMED_UNKNOWN_CORRIDOR_WIDTH_METERS = 1.5;

    private volatile Clock clock = Clock.systemUTC();
    private volatile ExpiryPolicy policy = ExpiryPolicy.defaults();
    // OFF by default: a STALE obstruction keeps blocking like a FRESH one until its hard TTL.
    private volatile boolean staleProbingEnabled = false;
    private volatile double assumedUnknownCorridorWidthMeters = DEFAULT_ASSUMED_UNKNOWN_CORRIDOR_WIDTH_METERS;
    private final ArrayDeque<ObstructionHistoryEntry> history = new ArrayDeque<>();

    Instant now() { return clock.instant(); }
    ExpiryPolicy policy() { return policy; }
    void setClock(Clock clock) { this.clock = Objects.requireNonNull(clock); }
    void setPolicy(ExpiryPolicy policy) { this.policy = Objects.requireNonNull(policy); }

    boolean staleProbingEnabled() { return staleProbingEnabled; }
    void setStaleProbingEnabled(boolean enabled) { this.staleProbingEnabled = enabled; }

    double assumedUnknownCorridorWidthMeters() { return assumedUnknownCorridorWidthMeters; }
    void setAssumedUnknownCorridorWidthMeters(double widthMeters) {
        if (!(widthMeters > 0.0)) {
            throw new IllegalArgumentException("assumed width must be > 0, got " + widthMeters);
        }
        this.assumedUnknownCorridorWidthMeters = widthMeters;
    }

    synchronized void recordEnded(String eventId, ObstructionKind kind, Instant lastObservedAt,
            ObstructionHistoryEntry.EndReason reason) {
        history.addLast(new ObstructionHistoryEntry(eventId, kind, lastObservedAt, now(), reason));
        while (history.size() > HISTORY_CAPACITY) {
            history.pollFirst();
        }
    }

    synchronized List<ObstructionHistoryEntry> historySnapshot() {
        return new ArrayList<>(history);
    }
}
