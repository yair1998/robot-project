package com.robotproject.server;

import io.grpc.Server;
import io.grpc.ServerBuilder;

public class RobotServer {

    public static void main(String[] args) throws Exception {

        int port = 50051;

        Server server = ServerBuilder
                .forPort(port)
                .addService(new RobotChannelService())
                .build()
                .start();

        System.out.println(
                "Robot gRPC server listening on port " + port
        );

        server.awaitTermination();
    }
}