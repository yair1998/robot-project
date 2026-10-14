package com.robotproject.server.routing;

import java.time.Duration;
import java.util.EnumMap;
import java.util.Map;

/**
 * How long an obstruction report stays believable. All numbers in defaults() are UNCALIBRATED
 * placeholders; real values must come from observed behavior.
 */
public final class ExpiryPolicy {

    public enum State { FRESH, STALE, GONE }

    public static final class Lifetime {
        private final Duration softTtl; // null (with hardTtl null) = permanent
        private final Duration hardTtl;

        private Lifetime(Duration softTtl, Duration hardTtl) {
            this.softTtl = softTtl;
            this.hardTtl = hardTtl;
        }

        public static Lifetime permanent() {
            return new Lifetime(null, null);
        }

        public static Lifetime of(Duration softTtl, Duration hardTtl) {
            if (softTtl == null || hardTtl == null || softTtl.isNegative() || softTtl.isZero()
                    || hardTtl.compareTo(softTtl) <= 0) {
                throw new IllegalArgumentException(
                        "need 0 < softTtl < hardTtl, got " + softTtl + " / " + hardTtl);
            }
            return new Lifetime(softTtl, hardTtl);
        }

        boolean isPermanent() {
            return softTtl == null;
        }
    }

    private final Map<ObstructionKind, Lifetime> lifetimes;
    private final double encounterPenaltyMeters;

    public ExpiryPolicy(Map<ObstructionKind, Lifetime> lifetimes, double encounterPenaltyMeters) {
        if (encounterPenaltyMeters < 0.0) {
            throw new IllegalArgumentException("encounterPenaltyMeters must be >= 0");
        }
        EnumMap<ObstructionKind, Lifetime> copy = new EnumMap<>(ObstructionKind.class);
        for (ObstructionKind kind : ObstructionKind.values()) {
            Lifetime lifetime = lifetimes.get(kind);
            if (lifetime == null) {
                throw new IllegalArgumentException("no lifetime given for " + kind);
            }
            copy.put(kind, lifetime);
        }
        this.lifetimes = copy;
        this.encounterPenaltyMeters = encounterPenaltyMeters;
    }

    public static ExpiryPolicy defaults() {
        Map<ObstructionKind, Lifetime> m = new EnumMap<>(ObstructionKind.class);
        m.put(ObstructionKind.TRANSIENT, Lifetime.of(Duration.ofSeconds(5), Duration.ofSeconds(30)));
        m.put(ObstructionKind.SEMI_PERMANENT, Lifetime.of(Duration.ofMinutes(5), Duration.ofMinutes(30)));
        m.put(ObstructionKind.PERMANENT, Lifetime.permanent());
        m.put(ObstructionKind.UNKNOWN, Lifetime.of(Duration.ofMinutes(2), Duration.ofMinutes(10)));
        return new ExpiryPolicy(m, 25.0);
    }

    /** Extra route length (meters) a robot is assumed to lose if it meets a stale obstruction that is still there. */
    public double getEncounterPenaltyMeters() {
        return encounterPenaltyMeters;
    }

    public State stateOf(ObstructionKind kind, Duration age) {
        Lifetime lifetime = lifetimes.get(kind);
        if (lifetime.isPermanent() || age.compareTo(lifetime.softTtl) <= 0) {
            return State.FRESH;
        }
        if (age.compareTo(lifetime.hardTtl) < 0) {
            return State.STALE;
        }
        return State.GONE;
    }

    /** 1.0 while FRESH, linear decay to 0.0 between soft and hard TTL, 0.0 once GONE. */
    public double existenceProbability(ObstructionKind kind, Duration age) {
        State state = stateOf(kind, age);
        if (state == State.FRESH) {
            return 1.0;
        }
        if (state == State.GONE) {
            return 0.0;
        }
        Lifetime lifetime = lifetimes.get(kind);
        double span = lifetime.hardTtl.minus(lifetime.softTtl).toMillis();
        double into = age.minus(lifetime.softTtl).toMillis();
        return 1.0 - into / span;
    }
}
