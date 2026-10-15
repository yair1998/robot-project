package com.robotproject.server;

import com.robotproject.robotagent.Command;
import com.robotproject.robotagent.CommandType;
import com.robotproject.robotagent.RobotChannelGrpc;
import com.robotproject.robotagent.Telemetry;

import io.grpc.stub.StreamObserver;

import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

import com.robotproject.server.routing.AStarRouter;
import com.robotproject.server.routing.GraphNode;
import com.robotproject.server.routing.MapLoader;
import com.robotproject.server.routing.MissionManager;

public class RobotChannelService
        extends RobotChannelGrpc.RobotChannelImplBase {

    /** The safety chain's verdict for one telemetry message - the single source for both the command and the dispatch gate. */
    enum SafetyVerdict { CLEAR, SLOW, STOP_PROXIMITY, STOP_EMERGENCY }

    private static final double SLOW_SPEED = 1.0;
    /** The commanded speed in the CLEAR band. Never taken from telemetry: a measured speed sent back as the commanded
     *  speed ratchets down (a slowed or stopped robot reports a low speed and is then told to keep it). */
    private static final double CRUISE_SPEED = 1.0;
    private static final Duration HELD_WARNING_AFTER = Duration.ofSeconds(10);

    private final MissionManager missionManager = new MissionManager(
            loadMissionGraph(), new AStarRouter());

    private final Map<String, RobotConnection> connections = new ConcurrentHashMap<>();

    private volatile Runnable beforeRoutingSendHook;

    // Lock rule L2: a connection write lock is never held together with a robot lock, and while it
    // is held nothing calls into MissionManager except the lock-free currentEpoch().
    private static class RobotConnection {
        final StreamObserver<Command> responseObserver;
        final java.util.concurrent.locks.Lock lock =
                new java.util.concurrent.locks.ReentrantLock(true); // fair
        // ONE id sequence per stream, shared by the telemetry path and every proactive send.
        final AtomicLong nextCommandId = new AtomicLong(1L);
        volatile SafetyVerdict verdict = SafetyVerdict.CLEAR;
        volatile Instant lastTelemetryAt;
        volatile Instant heldSince;
        volatile boolean heldWarningLogged;

        RobotConnection(StreamObserver<Command> responseObserver) {
            this.responseObserver = responseObserver;
        }
    }

    public RobotChannelService() {
        missionManager.setDispatchListener(this::onDispatchNeeded);
    }

    public MissionManager getMissionManager() {
        return missionManager;
    }

    /** Test hook: runs after a telemetry-path routing target was obtained, before the connection lock is taken. Null in production. */
    public void setBeforeRoutingSendHookForTests(Runnable r) {
        this.beforeRoutingSendHook = r;
    }

    /** The last safety verdict seen for this robot's registered connection, or "NO_CONNECTION". */
    public String debugVerdict(String robotId) {
        RobotConnection c = connections.get(robotId);
        return c == null ? "NO_CONNECTION" : c.verdict.name();
    }

    static SafetyVerdict evaluateSafety(Telemetry t) {
        if (t.getEmergencyStop()) {
            return SafetyVerdict.STOP_EMERGENCY;
        }
        if (t.hasNearestObstacleDistance() && t.getNearestObstacleDistance() < 1.0) {
            return SafetyVerdict.STOP_PROXIMITY;
        }
        if (t.hasNearestObstacleDistance() && t.getNearestObstacleDistance() < 3.0) {
            return SafetyVerdict.SLOW;
        }
        return SafetyVerdict.CLEAR;
    }

    private static boolean verdictAllowsRouting(SafetyVerdict v) {
        return v == SafetyVerdict.CLEAR || v == SafetyVerdict.SLOW;
    }

    /**
     * Entry point for a future Python-triggered urgent stop (or any
     * proactive command) - not wired to anything live yet. Exercised
     * only from a manual test/demo until a real trigger exists, the
     * same "prove standalone before wiring live" pattern already used
     * for MissionManager's obstruction-reporting entry points.
     */
    public void sendUrgentCommand(String robotId, Command command) {
        RobotConnection connection = connections.get(robotId);
        if (connection == null) {
            System.out.println("Cannot send urgent command - no active connection for robot " + robotId);
            return;
        }
        connection.lock.lock();
        try {
            if (!connections.containsKey(robotId)) {
                System.out.println(
                        "Connection for robot " + robotId + " closed before urgent command could be sent");
                return;
            }
            connection.responseObserver.onNext(command);
            System.out.println("Sent urgent command to robot " + robotId + ": " + command.getType());
        } finally {
            connection.lock.unlock();
        }
    }

    /** Removes THIS stream's connection (never a newer one for the same robot), then re-arms dispatch outside the lock (L2). */
    private void removeConnection(String robotId, RobotConnection connection) {
        if (robotId == null || connection == null) return;
        boolean removed;
        connection.lock.lock();
        try {
            removed = connections.remove(robotId, connection);
        } finally {
            connection.lock.unlock();
        }
        if (removed) {
            missionManager.rearmDispatch(robotId);
        }
    }

    private static com.robotproject.server.routing.RouteGraph loadMissionGraph() {
        try {
            return MapLoader.loadFromClasspath("maps/mission_map.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    // ------------------------------------------------------------------
    // Proactive path
    // ------------------------------------------------------------------
    private void onDispatchNeeded(String robotId) {
        MissionManager.Dispatch d = missionManager.takeRetreatDispatch(robotId);
        if (d == null) { return; }                                  // the telemetry path claimed it first
        deliver(robotId, connections.get(robotId), d, "urgent");
    }

    private void deliver(String robotId, RobotConnection c, MissionManager.Dispatch d, String origin) {
        if (c == null || !gateAllows(c)) {                          // no connection, stale telemetry, or a STOP verdict
            String reason = c == null ? "no connection"
                    : !verdictAllowsRouting(c.verdict) ? "safety " + c.verdict
                    : "stale telemetry";
            hold(robotId, c, d, reason);                            // NOT under the connection lock (L2)
            return;
        }
        sendRouting(robotId, c, d, origin);
    }

    /** Verdict CLEAR or SLOW and telemetry not older than the position max age. */
    private boolean gateAllows(RobotConnection c) {
        return verdictAllowsRouting(c.verdict) && telemetryFresh(c);
    }

    private boolean telemetryFresh(RobotConnection c) {
        Instant at = c.lastTelemetryAt;
        return at != null
                && Duration.between(at, missionManager.now()).compareTo(missionManager.getPositionMaxAge()) <= 0;
    }

    private void hold(String robotId, RobotConnection c, MissionManager.Dispatch d, String reason) {
        if (d == null || d.target == null) { return; }
        boolean first = !missionManager.hasHeldDispatch(robotId);
        missionManager.holdDispatch(robotId, d);
        if (c != null && c.heldSince == null) {
            c.heldSince = missionManager.now();
            c.heldWarningLogged = false;
        }
        if (first) {
            String waits = reason.startsWith("safety") ? "waits for safety to release." : "waits for the next telemetry.";
            System.out.println("Dispatch held (" + reason + "): " + robotId + " REROUTE -> "
                    + coords(d.target) + " " + waits);
        }
    }

    /** The ONLY method that writes a routing command. Inside the connection lock it only calls MissionManager.currentEpoch (L2). */
    private boolean sendRouting(String robotId, RobotConnection c, MissionManager.Dispatch d, String origin) {
        c.lock.lock();
        try {
            if (connections.get(robotId) != c) {
                System.out.println("Dispatch dropped (connection closed): " + robotId + " REROUTE -> " + coords(d.target));
                return false;
            }
            if (d.epoch < missionManager.currentEpoch(robotId)) {
                System.out.println("Dispatch dropped (superseded): " + robotId + " REROUTE -> " + coords(d.target));
                return false;
            }
            double speed = routingSpeed(c);
            Command command = Command.newBuilder()
                    .setRobotId(robotId)
                    .setCommandId(c.nextCommandId.getAndIncrement())
                    .setType(CommandType.COMMAND_TYPE_REROUTE)
                    .setTarget(d.target.toWaypoint())
                    .setTargetSpeed(speed)
                    .build();
            System.out.println("Sending command: " + command.getType() + ", id=" + command.getCommandId());
            c.responseObserver.onNext(command);
            System.out.println("Dispatch (" + origin + "): " + robotId + " REROUTE -> " + coords(d.target)
                    + " id=" + command.getCommandId() + " speed=" + speed);
            c.heldSince = null;
            c.heldWarningLogged = false;
            return true;
        } finally {
            c.lock.unlock();
        }
    }

    /** SLOW band: the SLOW speed, never faster. CLEAR: the cruise speed, never the robot's reported speed. */
    private static double routingSpeed(RobotConnection c) {
        if (c.verdict == SafetyVerdict.SLOW) {
            return SLOW_SPEED;
        }
        return CRUISE_SPEED;
    }

    /** Non-routing commands (STOP, SLOW_DOWN, CONTINUE) - unchanged content, id drawn inside the connection lock. */
    private void sendPlain(RobotConnection c, String robotId, CommandType type, double speed) {
        c.lock.lock();
        try {
            Command command = Command.newBuilder()
                    .setRobotId(robotId)
                    .setCommandId(c.nextCommandId.getAndIncrement())
                    .setType(type)
                    .setTargetSpeed(speed)
                    .build();
            System.out.println("Sending command: " + command.getType() + ", id=" + command.getCommandId());
            c.responseObserver.onNext(command);
        } finally {
            c.lock.unlock();
        }
    }

    private static String coords(GraphNode n) {
        return "(" + n.getX() + ", " + n.getY() + ")";
    }

    // ------------------------------------------------------------------
    // Telemetry path
    // ------------------------------------------------------------------
    @Override
    public StreamObserver<Telemetry> connect(
            final StreamObserver<Command> responseObserver) {

        System.out.println("Robot connected");

        return new StreamObserver<Telemetry>() {

            // Per-stream state lives on THIS anonymous instance, never as a local
            // in connect() or a field on RobotChannelService.
            private boolean isFirstTelemetryOnThisStream = true;

            private volatile String thisRobotId;
            private volatile RobotConnection thisConnection;

            @Override
            public void onNext(Telemetry telemetry) {
                // a. Record the position first - it decides nothing.
                missionManager.updateRobotPosition(
                        telemetry.getRobotId(), telemetry.getPositionX(), telemetry.getPositionY());

                String robotId = telemetry.getRobotId();
                double x = telemetry.getPositionX();
                double y = telemetry.getPositionY();
                thisRobotId = robotId;

                SafetyVerdict verdict = evaluateSafety(telemetry);

                boolean firstOnStream = isFirstTelemetryOnThisStream;
                if (firstOnStream) {
                    // Registered BEFORE the first command of the stream is built.
                    thisConnection = new RobotConnection(responseObserver);
                    connections.put(robotId, thisConnection);
                    isFirstTelemetryOnThisStream = false;
                }
                RobotConnection connection = thisConnection;
                connection.verdict = verdict;
                connection.lastTelemetryAt = missionManager.now();

                System.out.println(
                        "Telemetry: robot=" + telemetry.getRobotId()
                                + ", position=("
                                + telemetry.getPositionX() + ", "
                                + telemetry.getPositionY() + ")"
                                + ", speed=" + telemetry.getSpeed()
                                + ", battery=" + telemetry.getBattery()
                                + ", emergency="
                                + telemetry.getEmergencyStop()
                );

                if (telemetry.hasNearestObstacleDistance()) {
                    System.out.println(
                            "Nearest obstacle: "
                                    + telemetry.getNearestObstacleDistance()
                    );
                } else {
                    System.out.println(
                            "Nearest obstacle: unavailable"
                    );
                }

                // b. The mission layer ticks in EVERY branch, not only CLEAR.
                // "E" is a stand-in for real per-robot mission assignment, which
                // doesn't exist yet - the caller (here) decides the destination,
                // not MissionManager, so this placeholder lives here rather than
                // being baked into MissionManager itself.
                MissionManager.Dispatch fresh = missionManager.onTelemetryDispatch(robotId, x, y, "E");

                // c. Proximity corroboration (a no-op unless stale probing is ON).
                if (telemetry.hasNearestObstacleDistance()) {
                    missionManager.observeProximity(robotId, x, y, telemetry.getNearestObstacleDistance());
                }

                // d. What to deliver this tick.
                MissionManager.Dispatch toDeliver = null;
                String origin = null;
                if (fresh.target != null) {
                    missionManager.takeHeldDispatch(robotId);           // the new decision supersedes any held one
                    toDeliver = fresh;
                    origin = "telemetry";
                } else if (firstOnStream) {
                    MissionManager.Dispatch desired = missionManager.desiredDispatchFor(robotId);
                    if (desired.target != null) {
                        toDeliver = desired;
                        origin = "telemetry";
                    }
                } else if (verdictAllowsRouting(verdict)) {
                    MissionManager.Dispatch held = missionManager.takeHeldDispatch(robotId);
                    if (held != null) {
                        toDeliver = held;
                        origin = "released";
                    }
                }

                // e./f. Build and send.
                if (verdict == SafetyVerdict.STOP_EMERGENCY || verdict == SafetyVerdict.STOP_PROXIMITY) {
                    if (toDeliver != null) {
                        hold(robotId, connection, toDeliver, "safety " + verdict);
                    }
                    sendPlain(connection, robotId, CommandType.COMMAND_TYPE_STOP, 0.0);   // a STOP never carries a target
                } else if (toDeliver != null) {
                    Runnable hook = beforeRoutingSendHook;
                    if (hook != null) {
                        hook.run();
                    }
                    sendRouting(robotId, connection, toDeliver, origin);
                } else if (verdict == SafetyVerdict.SLOW) {
                    sendPlain(connection, robotId, CommandType.COMMAND_TYPE_SLOW_DOWN, 1.0);
                } else {
                    sendPlain(connection, robotId, CommandType.COMMAND_TYPE_CONTINUE, CRUISE_SPEED);
                }

                // g. One warning when a dispatch stays held under a persisting STOP for more than 10 s.
                if (missionManager.hasHeldDispatch(robotId)) {
                    Instant since = connection.heldSince;
                    if (since == null) {
                        connection.heldSince = missionManager.now();
                    } else if (!connection.heldWarningLogged
                            && !verdictAllowsRouting(verdict)
                            && Duration.between(since, missionManager.now()).compareTo(HELD_WARNING_AFTER) > 0) {
                        connection.heldWarningLogged = true;
                        System.out.println("WARNING: a routing command for robot " + robotId
                                + " has been held for more than " + HELD_WARNING_AFTER.getSeconds()
                                + " s under a persisting safety " + verdict
                                + " - an obstacle that never moves needs an operator; nothing here overrides safety.");
                    }
                } else {
                    connection.heldSince = null;
                    connection.heldWarningLogged = false;
                }
            }

            @Override
            public void onError(Throwable t) {
                System.err.println(
                        "Robot stream failed: "
                                + t.getMessage()
                );
                removeConnection(thisRobotId, thisConnection);
            }

            @Override
            public void onCompleted() {
                System.out.println("Robot stream completed");
                responseObserver.onCompleted();
                removeConnection(thisRobotId, thisConnection);
            }
        };
    }
}
