package com.robotproject.server;

import com.robotproject.robotagent.Command;
import com.robotproject.robotagent.CommandType;
import com.robotproject.robotagent.RobotChannelGrpc;
import com.robotproject.robotagent.Telemetry;

import io.grpc.stub.StreamObserver;

import java.util.concurrent.atomic.AtomicLong;

public class RobotChannelService
        extends RobotChannelGrpc.RobotChannelImplBase {

    private final AtomicLong nextCommandId = new AtomicLong(1);

    @Override
    public StreamObserver<Telemetry> connect(
            StreamObserver<Command> responseObserver) {

        System.out.println("Robot connected");

        return new StreamObserver<Telemetry>() {

            @Override
            public void onNext(Telemetry telemetry) {

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

                    command
                            .setType(
                                CommandType.COMMAND_TYPE_CONTINUE
                            )
                            .setTargetSpeed(
                                telemetry.getSpeed()
                            );
                }

                Command result = command.build();

                System.out.println(
                        "Sending command: "
                                + result.getType()
                                + ", id="
                                + result.getCommandId()
                );

                responseObserver.onNext(result);
            }

            @Override
            public void onError(Throwable t) {
                System.err.println(
                        "Robot stream failed: "
                                + t.getMessage()
                );
            }

            @Override
            public void onCompleted() {
                System.out.println("Robot stream completed");
                responseObserver.onCompleted();
            }
        };
    }
}