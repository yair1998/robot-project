package com.robotproject.server.routing;

import java.time.Instant;

/**
 * Everything the perception side may tell Java about ONE obstruction. Deliberately contains no
 * edge id, direction or fraction: topology belongs to Java alone, which derives it by projecting
 * the world point onto the graph.
 */
public final class WorldObstructionReport {
    public final String eventId;                  // stable across re-observations of the same physical obstruction
    public final double worldX;
    public final double worldY;
    public final double obstacleWidthMeters;
    public final Double measuredFreeWidthMeters;  // optional: width actually left free; overrides structural-minus-obstacle
    public final ObstructionKind kind;            // optional, null = UNKNOWN
    public final Instant observedAt;              // optional, null = receipt time; the moment it was SEEN

    public WorldObstructionReport(String eventId, double worldX, double worldY,
            double obstacleWidthMeters, Double measuredFreeWidthMeters,
            ObstructionKind kind, Instant observedAt) {
        if (eventId == null || eventId.isEmpty()) {
            throw new IllegalArgumentException("eventId is required");
        }
        if (obstacleWidthMeters < 0.0
                || (measuredFreeWidthMeters != null && measuredFreeWidthMeters < 0.0)) {
            throw new IllegalArgumentException("widths must be >= 0");
        }
        this.eventId = eventId;
        this.worldX = worldX;
        this.worldY = worldY;
        this.obstacleWidthMeters = obstacleWidthMeters;
        this.measuredFreeWidthMeters = measuredFreeWidthMeters;
        this.kind = kind;
        this.observedAt = observedAt;
    }

    public static WorldObstructionReport basic(
            String eventId, double worldX, double worldY, double obstacleWidthMeters) {
        return new WorldObstructionReport(
                eventId, worldX, worldY, obstacleWidthMeters, null, null, null);
    }
}
