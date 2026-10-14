package com.robotproject.server;

import io.grpc.Server;
import io.grpc.ServerBuilder;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.robotproject.server.routing.MissionManager;

public class RobotServer {

    public static void main(String[] args) throws Exception {

        int port = 50051;

        RobotChannelService service = new RobotChannelService();
        MissionManager missionManager = service.getMissionManager();

        List<String> unknownWidths = missionManager.corridorsWithUnknownWidth();
        if (unknownWidths.isEmpty()) {
            System.out.println("All corridors have a structural width.");
        } else {
            System.out.println("Corridors without a structural width (world reports there use the assumed width): "
                    + unknownWidths);
        }

        // Without this, expired events and tombstones would accumulate for ever on a long-running server.
        ScheduledExecutorService maintenance = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "mission-maintenance");
            t.setDaemon(true);
            return t;
        });
        maintenance.scheduleWithFixedDelay(() -> {
            try {
                missionManager.purgeExpiredObstructions();
            } catch (RuntimeException e) {
                // A scheduled task that throws is never run again - print and carry on.
                System.err.println("mission-maintenance: purge failed: " + e);
            }
        }, 30, 30, TimeUnit.SECONDS);

        Server server = ServerBuilder
                .forPort(port)
                .addService(service)
                .build()
                .start();

        System.out.println(
                "Robot gRPC server listening on port " + port
        );

        server.awaitTermination();
    }
}
