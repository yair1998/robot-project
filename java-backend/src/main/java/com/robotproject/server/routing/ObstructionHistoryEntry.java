package com.robotproject.server.routing;

import java.time.Instant;

/** An obstruction that left the active set, and how. Bounded history, for calibrating expiry later. */
public final class ObstructionHistoryEntry {

    public enum EndReason {
        EXPIRED,  // passed its hard TTL and was purged
        PASSED,   // a robot moved beyond its recorded position without incident
        CLEARED,  // explicitly cleared by the reporting side
        MOVED     // re-registered on another corridor
    }

    public final String eventId;
    public final ObstructionKind kind;
    public final Instant lastObservedAt;
    public final Instant endedAt;
    public final EndReason reason;

    ObstructionHistoryEntry(String eventId, ObstructionKind kind, Instant lastObservedAt,
            Instant endedAt, EndReason reason) {
        this.eventId = eventId;
        this.kind = kind;
        this.lastObservedAt = lastObservedAt;
        this.endedAt = endedAt;
        this.reason = reason;
    }
}
