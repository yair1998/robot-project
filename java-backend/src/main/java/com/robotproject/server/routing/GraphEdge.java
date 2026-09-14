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
}
