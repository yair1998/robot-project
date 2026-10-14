package com.robotproject.server.routing;

public final class ObstructionView {
    public final String eventId;
    public final double remainingWidthMeters;
    public final Double positionFraction; // in the convention of the object that produced the view; null = unknown
    public final ObstructionKind kind;
    public final ExpiryPolicy.State state;
    public final double existenceProbability;

    ObstructionView(String eventId, double remainingWidthMeters, Double positionFraction,
            ObstructionKind kind, ExpiryPolicy.State state, double existenceProbability) {
        this.eventId = eventId;
        this.remainingWidthMeters = remainingWidthMeters;
        this.positionFraction = positionFraction;
        this.kind = kind;
        this.state = state;
        this.existenceProbability = existenceProbability;
    }
}
