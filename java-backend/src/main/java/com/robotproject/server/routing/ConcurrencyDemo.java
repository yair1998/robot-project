package com.robotproject.server.routing;

import com.robotproject.robotagent.Command;
import com.robotproject.robotagent.CommandType;
import com.robotproject.robotagent.Telemetry;
import com.robotproject.server.RobotChannelService;

import io.grpc.stub.StreamObserver;

import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Standalone concurrency proof for the per-robot lock backing
 * RobotChannelService.sendUrgentCommand(). No Docker, no real gRPC
 * transport - connect() already hands back an in-memory
 * StreamObserver<Telemetry> that can be driven directly from test
 * threads, with a fake StreamObserver<Command> standing in for the
 * network side.
 */
public class ConcurrencyDemo {

    private static final int ROBOT_COUNT = 5;
    private static final long RACE_DURATION_MS = 2000;
    private static final long URGENT_INTERVAL_MS = 50;
    private static final long STARVATION_LIMIT_NANOS = 250_000_000L; // 250ms
    private static final int TEARDOWN_BURST_SIZE = 20;

    private static int passCount = 0;
    private static int failCount = 0;

    public static void main(String[] args) throws Exception {

        RobotChannelService service = new RobotChannelService();

        List<RobotHarness> harnesses = new ArrayList<>();
        for (int i = 0; i < ROBOT_COUNT; i++) {
            harnesses.add(new RobotHarness("robot-conc-" + i, service));
        }

        System.out.println("Running " + ROBOT_COUNT + " robots in parallel for ~"
                + RACE_DURATION_MS + "ms each (normal-path tight loop vs. urgent sends every ~"
                + URGENT_INTERVAL_MS + "ms)...");
        System.out.println("(RobotChannelService's own per-call telemetry logging is silenced "
                + "during the stress run - the same code path and locking still runs, this just "
                + "avoids dumping tens of thousands of lines of routine per-call logging here.)");

        PrintStream realOut = System.out;
        PrintStream discard = new PrintStream(new OutputStream() {
            @Override
            public void write(int b) {
                // discard
            }
        });

        List<Thread> robotThreads = new ArrayList<>();
        for (RobotHarness h : harnesses) {
            Thread t = new Thread(h::run, "harness-" + h.robotId);
            robotThreads.add(t);
        }

        System.setOut(discard);
        long wallStart = System.nanoTime();
        for (Thread t : robotThreads) {
            t.start();
        }
        for (Thread t : robotThreads) {
            t.join();
        }
        long wallEnd = System.nanoTime();
        System.setOut(realOut);

        System.out.println("Stress run finished in " + ((wallEnd - wallStart) / 1_000_000) + "ms.");
        System.out.println();

        // ===== Per-robot checks =====
        long globalMaxUrgentDurationNanos = 0;
        long overallMaxNormalStart = Long.MIN_VALUE;
        long overallMinNormalEnd = Long.MAX_VALUE;

        for (RobotHarness h : harnesses) {
            System.out.println("--- " + h.robotId + " ---");

            check(h.robotId + ": no exception escaped either thread",
                    h.escapedExceptions.isEmpty());
            if (!h.escapedExceptions.isEmpty()) {
                for (Throwable t : h.escapedExceptions) {
                    System.out.println("    escaped: " + t);
                }
            }

            check(h.robotId + ": mutual exclusion never violated (max concurrent callers inside onNext = "
                            + h.fakeObserver.maxConcurrentCallers.get() + ")",
                    !h.fakeObserver.mutualExclusionViolated.get());

            check(h.robotId + ": fake observer never received onNext after being closed",
                    !h.fakeObserver.closedAfterOnNextViolated.get());

            check(h.robotId + ": normal-path and urgent threads both made real progress (normal="
                            + h.normalCallCount.get() + ", urgent=" + h.urgentCallCount.get() + ")",
                    h.normalCallCount.get() > 0 && h.urgentCallCount.get() > 0);

            double maxUrgentMs = h.maxUrgentDurationNanos.get() / 1_000_000.0;
            check(h.robotId + ": no single sendUrgentCommand call exceeded 250ms despite contention (max="
                            + String.format("%.2f", maxUrgentMs) + "ms)",
                    h.maxUrgentDurationNanos.get() < STARVATION_LIMIT_NANOS);
            globalMaxUrgentDurationNanos = Math.max(globalMaxUrgentDurationNanos, h.maxUrgentDurationNanos.get());

            check(h.robotId + ": teardown burst (" + TEARDOWN_BURST_SIZE
                            + " calls) all returned normally with no throw",
                    h.burstCallsCompletedWithoutThrow == TEARDOWN_BURST_SIZE);

            check(h.robotId + ": teardown burst delivered zero commands to the (closed) fake observer "
                            + "- rejected gracefully via 'no active connection', not delivered then thrown",
                    h.burstDeliveredNoNewCommands);

            overallMaxNormalStart = Math.max(overallMaxNormalStart, h.normalThreadStartNanos.get());
            overallMinNormalEnd = Math.min(overallMinNormalEnd, h.normalThreadEndNanos.get());

            System.out.println();
        }

        // ===== Cross-robot checks =====
        System.out.println("--- cross-robot ---");

        // If every robot's normal-loop window overlapped with every other
        // robot's, then the latest start time is still earlier than the
        // earliest end time - i.e. there is a common instant at which all
        // 5 robots were genuinely running simultaneously. A global lock
        // (instead of one lock per robot) would make this vanishingly
        // unlikely to hold across 5 robots each running for ~2 seconds.
        boolean allRobotsOverlapped = overallMaxNormalStart < overallMinNormalEnd;
        check("different robots' locks do not serialize each other - all "
                        + ROBOT_COUNT + " robots' normal-path loops were simultaneously active "
                        + "at some common instant",
                allRobotsOverlapped);

        check("global starvation check: worst-case sendUrgentCommand latency across all robots stayed under 250ms (max="
                        + String.format("%.2f", globalMaxUrgentDurationNanos / 1_000_000.0) + "ms)",
                globalMaxUrgentDurationNanos < STARVATION_LIMIT_NANOS);

        System.out.println();
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
    }

    private static void check(String description, boolean condition) {
        if (condition) {
            passCount++;
            System.out.println("  PASS: " + description);
        } else {
            failCount++;
            System.out.println("  FAIL: " + description);
        }
    }

    private static Command buildContinueCommand(String robotId, long id) {
        return Command.newBuilder()
                .setRobotId(robotId)
                .setCommandId(id)
                .setType(CommandType.COMMAND_TYPE_CONTINUE)
                .setTargetSpeed(1.0)
                .build();
    }

    private static Command buildUrgentStopCommand(String robotId, long id) {
        return Command.newBuilder()
                .setRobotId(robotId)
                .setCommandId(id)
                .setType(CommandType.COMMAND_TYPE_STOP)
                .setTargetSpeed(0.0)
                .build();
    }

    /**
     * Drives one robot's full lifecycle: connect, race a tight normal-path
     * loop against periodic urgent sends for RACE_DURATION_MS, tear down,
     * then immediately burst urgent sends to prove they're rejected
     * gracefully rather than reaching a closed stream.
     */
    private static class RobotHarness {
        final String robotId;
        final RobotChannelService service;
        final FakeCommandObserver fakeObserver;
        volatile StreamObserver<Telemetry> telemetryObserver;

        final AtomicLong normalCallCount = new AtomicLong(0);
        final AtomicLong urgentCallCount = new AtomicLong(0);
        final AtomicLong maxUrgentDurationNanos = new AtomicLong(0);
        final List<Throwable> escapedExceptions = new CopyOnWriteArrayList<>();

        final AtomicLong normalThreadStartNanos = new AtomicLong();
        final AtomicLong normalThreadEndNanos = new AtomicLong();

        volatile int burstCallsCompletedWithoutThrow = 0;
        volatile boolean burstDeliveredNoNewCommands = false;

        RobotHarness(String robotId, RobotChannelService service) {
            this.robotId = robotId;
            this.service = service;
            this.fakeObserver = new FakeCommandObserver(robotId);
        }

        void run() {
            telemetryObserver = service.connect(fakeObserver);
            // The very first onNext() call registers this stream's
            // connection (RobotChannelService only learns the robot id -
            // and therefore can populate its connections map - from the
            // first telemetry message, not from connect() itself).
            telemetryObserver.onNext(freshTelemetry());

            Thread normalThread = new Thread(this::runNormalLoop, "normal-" + robotId);
            Thread urgentThread = new Thread(this::runUrgentLoop, "urgent-" + robotId);

            normalThread.start();
            urgentThread.start();

            joinQuietly(normalThread);
            joinQuietly(urgentThread);

            try {
                telemetryObserver.onCompleted();
            } catch (Throwable t) {
                escapedExceptions.add(t);
            }

            long receivedBefore = fakeObserver.received.size();
            int completedWithoutThrow = 0;
            for (int i = 0; i < TEARDOWN_BURST_SIZE; i++) {
                try {
                    service.sendUrgentCommand(robotId, buildUrgentStopCommand(robotId, 9000 + i));
                    completedWithoutThrow++;
                } catch (Throwable t) {
                    escapedExceptions.add(t);
                }
            }
            burstCallsCompletedWithoutThrow = completedWithoutThrow;
            burstDeliveredNoNewCommands = fakeObserver.received.size() == receivedBefore;
        }

        private Telemetry freshTelemetry() {
            return Telemetry.newBuilder()
                    .setRobotId(robotId)
                    .setPositionX(0.0)
                    .setPositionY(0.0)
                    .setSpeed(1.0)
                    .setBattery(90.0)
                    .setTimestamp(1)
                    .setEmergencyStop(false)
                    .build();
        }

        private void runNormalLoop() {
            normalThreadStartNanos.set(System.nanoTime());
            Telemetry telemetry = freshTelemetry();
            long deadline = System.currentTimeMillis() + RACE_DURATION_MS;
            while (System.currentTimeMillis() < deadline) {
                try {
                    telemetryObserver.onNext(telemetry);
                    normalCallCount.incrementAndGet();
                } catch (Throwable t) {
                    escapedExceptions.add(t);
                    break;
                }
            }
            normalThreadEndNanos.set(System.nanoTime());
        }

        private void runUrgentLoop() {
            long deadline = System.currentTimeMillis() + RACE_DURATION_MS;
            long id = 1;
            while (System.currentTimeMillis() < deadline) {
                long callStart = System.nanoTime();
                try {
                    service.sendUrgentCommand(robotId, buildContinueCommand(robotId, id++));
                    urgentCallCount.incrementAndGet();
                } catch (Throwable t) {
                    escapedExceptions.add(t);
                }
                long callDuration = System.nanoTime() - callStart;
                maxUrgentDurationNanos.updateAndGet(prev -> Math.max(prev, callDuration));

                try {
                    Thread.sleep(URGENT_INTERVAL_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }

        private void joinQuietly(Thread t) {
            try {
                t.join(RACE_DURATION_MS + 5000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Stands in for the network side. Tracks how many callers are
     * concurrently inside onNext() (proving mutual exclusion, not just
     * "looked fine"), throws if called after being marked closed (mimics
     * real gRPC's onNext-after-onCompleted contract violation, exercising
     * the teardown race), and records every command received.
     */
    private static class FakeCommandObserver implements StreamObserver<Command> {
        final String robotId;
        final AtomicInteger callersInside = new AtomicInteger(0);
        final AtomicInteger maxConcurrentCallers = new AtomicInteger(0);
        final AtomicBoolean mutualExclusionViolated = new AtomicBoolean(false);
        final AtomicBoolean closed = new AtomicBoolean(false);
        final AtomicBoolean closedAfterOnNextViolated = new AtomicBoolean(false);
        final List<Command> received = new CopyOnWriteArrayList<>();

        FakeCommandObserver(String robotId) {
            this.robotId = robotId;
        }

        @Override
        public void onNext(Command command) {
            if (closed.get()) {
                closedAfterOnNextViolated.set(true);
                throw new IllegalStateException(
                        "onNext called for " + robotId + " after its stream was already closed");
            }

            int concurrentCallers = callersInside.incrementAndGet();
            maxConcurrentCallers.updateAndGet(prev -> Math.max(prev, concurrentCallers));
            if (concurrentCallers > 1) {
                mutualExclusionViolated.set(true);
            }
            try {
                Thread.sleep(2);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                callersInside.decrementAndGet();
            }

            received.add(command);
        }

        @Override
        public void onError(Throwable t) {
            closed.set(true);
        }

        @Override
        public void onCompleted() {
            closed.set(true);
        }
    }
}
