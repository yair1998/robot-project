package com.robotproject.server;

import com.robotproject.robotagent.Command;
import com.robotproject.robotagent.CommandType;
import com.robotproject.robotagent.RobotChannelGrpc;
import com.robotproject.robotagent.Telemetry;

import io.grpc.stub.StreamObserver;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.robotproject.server.routing.AStarRouter;
import com.robotproject.server.routing.GraphNode;
import com.robotproject.server.routing.MapLoader;
import com.robotproject.server.routing.MissionManager;

public class RobotChannelService
        extends RobotChannelGrpc.RobotChannelImplBase {

    private final MissionManager missionManager = new MissionManager(
            loadMissionGraph(), new AStarRouter());

    private final Map<String, RobotConnection> connections = new ConcurrentHashMap<>();

    private static class RobotConnection {
        final StreamObserver<Command> responseObserver;
        final java.util.concurrent.locks.Lock lock =
                new java.util.concurrent.locks.ReentrantLock(true); // fair

        RobotConnection(StreamObserver<Command> responseObserver) {
            this.responseObserver = responseObserver;
        }
    }

    /**
     * Entry point for a future Python-triggered urgent stop (or any
     * proactive command) - not wired to anything live yet. Exercised
     * only from a manual test/demo until a real trigger exists, the
     * same "prove standalone before wiring live" pattern already used
     * for onGraphCapacityChanged.
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

    private void removeConnection(String robotId) {
        if (robotId == null) return;
        RobotConnection connection = connections.get(robotId);
        if (connection != null) {
            connection.lock.lock();
            try {
                connections.remove(robotId);
            } finally {
                connection.lock.unlock();
            }
        }
    }

    private static com.robotproject.server.routing.RouteGraph loadMissionGraph() {
        try {
            return MapLoader.loadFromClasspath("maps/mission_map.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public StreamObserver<Telemetry> connect(
            final StreamObserver<Command> responseObserver) {

        System.out.println("Robot connected");

        return new StreamObserver<Telemetry>() {

            // Field on THIS per-stream instance - see CRITICAL FIELD
            // PLACEMENT note above for why it can't be a local in
            // connect() or a field on RobotChannelService.
            private boolean isFirstTelemetryOnThisStream = true;

            private final java.util.concurrent.atomic.AtomicLong nextCommandId =
                    new java.util.concurrent.atomic.AtomicLong(1L);

            private volatile String thisRobotId;

            @Override
            public void onNext(Telemetry telemetry) {

                thisRobotId = telemetry.getRobotId();

                if (isFirstTelemetryOnThisStream) {
                    connections.put(telemetry.getRobotId(), new RobotConnection(responseObserver));
                }

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

                Command.Builder command =
                        Command.newBuilder()
                                .setRobotId(telemetry.getRobotId())
                                .setCommandId(
                                        nextCommandId.getAndIncrement()
                                );

                if (telemetry.getEmergencyStop()) {

                    command
                            .setType(
                                CommandType.COMMAND_TYPE_STOP
                            )
                            .setTargetSpeed(0.0);

                } else if (
                        telemetry.hasNearestObstacleDistance()
                        && telemetry.getNearestObstacleDistance() < 1.0
                ) {

                    command
                            .setType(
                                CommandType.COMMAND_TYPE_STOP
                            )
                            .setTargetSpeed(0.0);

                } else if (
                        telemetry.hasNearestObstacleDistance()
                        && telemetry.getNearestObstacleDistance() < 3.0
                ) {

                    command
                            .setType(
                                CommandType.COMMAND_TYPE_SLOW_DOWN
                            )
                            .setTargetSpeed(1.0);

                } else {

                    // "E" is a stand-in for real per-robot mission assignment, which
                    // doesn't exist yet - the caller (here) decides the destination,
                    // not MissionManager, so this placeholder lives here rather than
                    // being baked into MissionManager itself.
                    GraphNode routeTarget = missionManager.onTelemetry(
                            telemetry.getRobotId(), telemetry.getPositionX(), telemetry.getPositionY(), "E");

                    if (routeTarget == null && isFirstTelemetryOnThisStream) {
                        routeTarget = missionManager.initialTargetFor(telemetry.getRobotId());
                    }
                    isFirstTelemetryOnThisStream = false;

                    if (routeTarget != null) {
                        command
                                .setType(
                                    CommandType.COMMAND_TYPE_REROUTE
                                )
                                .setTarget(routeTarget.toWaypoint())
                                .setTargetSpeed(
                                    telemetry.getSpeed()
                                );
                    } else {
                        command
                                .setType(
                                    CommandType.COMMAND_TYPE_CONTINUE
                                )
                                .setTargetSpeed(
                                    telemetry.getSpeed()
                                );
                    }
                }

                Command result = command.build();

                System.out.println(
                        "Sending command: "
                                + result.getType()
                                + ", id="
                                + result.getCommandId()
                );

                RobotConnection connection = connections.get(telemetry.getRobotId());
                if (connection != null) {
                    connection.lock.lock();
                    try {
                        responseObserver.onNext(result);
                    } finally {
                        connection.lock.unlock();
                    }
                } else {
                    responseObserver.onNext(result); // connections map race fallback - stream just started
                }
            }

            @Override
            public void onError(Throwable t) {
                System.err.println(
                        "Robot stream failed: "
                                + t.getMessage()
                );
                removeConnection(thisRobotId);
            }

            @Override
            public void onCompleted() {
                System.out.println("Robot stream completed");
                responseObserver.onCompleted();
                removeConnection(thisRobotId);
            }
        };
    }
}