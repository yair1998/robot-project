package com.robotproject.server.routing;

import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneId;
import java.time.ZoneOffset;

final class MutableClock extends Clock {
    private volatile Instant now;

    MutableClock(Instant start) { this.now = start; }
    void advance(Duration d) { this.now = this.now.plus(d); }
    void set(Instant t) { this.now = t; }

    @Override public ZoneId getZone() { return ZoneOffset.UTC; }
    @Override public Clock withZone(ZoneId zone) { return this; }
    @Override public Instant instant() { return now; }
}
