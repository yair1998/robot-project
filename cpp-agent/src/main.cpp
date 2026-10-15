#include <grpcpp/grpcpp.h>
#include "robot_channel.grpc.pb.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "core/clock.h"
#include "core/command_mailbox.h"
#include "core/control_core.h"
#include "core/local_perception.h"
#include "core/log.h"
#include "core/motion_backend.h"
#include "core/python_perception_adapter.h"
#include "core/robot_config.h"
#include "core/simulated_sensor_provider.h"
#include "core/telemetry_channel.h"

using grpc::ClientContext;
using grpc::ClientReaderWriter;

using robotagent::Command;
using robotagent::RobotChannel;
using robotagent::Telemetry;

// Threads (nothing else touches robot state):
//   control  (main thread)  - the ONLY mutator of robot state. Fixed period (config.control_period_s, 200 ms).
//                              Each tick: command mailbox -> sensing + fusion -> navigation -> safety -> motion
//                              backend -> telemetry snapshot. It never calls the network.
//   gRPC reader             - Read() -> decode -> CommandMailbox (latest-wins waypoint, flags, bounded state).
//   telemetry writer        - the ONLY caller of Write(). Waits for a newer snapshot; if Write() blocks under flow
//                              control the control thread keeps running and the writer later skips to the newest.
//   perception ingress      - whatever calls PythonPerceptionAdapter::offer(). Nothing yet: no receiver in this build.
// Locks: CommandMailbox, TelemetryChannel, the adapter slot and the log mutex are leaf locks. No thread ever holds two
// of them at once and none is held across Read()/Write(), so there is no lock order to get wrong.
// Failure: a Read() or Write() failure marks the stream broken; the next tick latches a permanent local stop
// (logged). Automatic reconnect is out of scope.
namespace {

std::mutex g_logMutex;

void logLine(const std::string& line) {
    std::lock_guard<std::mutex> lock(g_logMutex);
    std::cout << line << std::endl;
}

robot::JavaCommand decode(const Command& command) {
    robot::JavaCommand c;
    switch (command.type()) {
        case robotagent::COMMAND_TYPE_CONTINUE: c.type = robot::JavaCommandType::CONTINUE; break;
        case robotagent::COMMAND_TYPE_SLOW_DOWN: c.type = robot::JavaCommandType::SLOW_DOWN; break;
        case robotagent::COMMAND_TYPE_STOP: c.type = robot::JavaCommandType::STOP; break;
        case robotagent::COMMAND_TYPE_RESUME: c.type = robot::JavaCommandType::RESUME; break;
        case robotagent::COMMAND_TYPE_REROUTE: c.type = robot::JavaCommandType::REROUTE; break;
        default: c.type = robot::JavaCommandType::UNSPECIFIED; break;
    }
    c.id = command.command_id();
    c.target_speed = command.target_speed();
    if (command.has_target()) {
        c.waypoint = robot::Point{command.target().x(), command.target().y()};
    }
    return c;
}

Telemetry encode(const robot::TelemetrySnapshot& s, const std::string& robotId, int64_t timestamp) {
    Telemetry telemetry;
    telemetry.set_robot_id(robotId);
    telemetry.set_position_x(s.position.x);
    telemetry.set_position_y(s.position.y);
    telemetry.set_speed(s.measured_speed_mps);   // measured; the requested speed is not reported (one speed field)
    telemetry.set_battery(s.battery);
    telemetry.set_timestamp(timestamp);
    telemetry.set_emergency_stop(s.emergency_stop);
    if (s.nearest_obstacle_m) {
        telemetry.set_nearest_obstacle_distance(*s.nearest_obstacle_m);   // absent when no valid reading
    }
    return telemetry;
}

}  // namespace

int main() {
    const char* envTarget = std::getenv("GRPC_TARGET");
    std::string target = envTarget ? envTarget : "host.docker.internal:50051";
    const std::string robotId = "robot-1";

    logLine("Connecting to " + target);

    robot::RobotConfig config;
    robot::SteadyClock clock;
    robot::LogSink log = logLine;

    robot::SimulatedSensorProvider sim(config.sim, log);
    robot::PythonPerceptionAdapter python(config.python,
                                          config.nav.robot_half_width_m + config.nav.clearance_margin_m, clock);
    robot::LocalPerception perception(config.perception,
                                      {{&sim, robot::SourceRole::REQUIRED_FOR_SAFETY},
                                       {&python, robot::SourceRole::SUPPLEMENTARY}});
    robot::SimulatedMotionBackend motion(config.motion);
    robot::CommandMailbox mailbox;
    robot::TelemetryChannel telemetryChannel;
    robot::ControlCore core(config, clock, motion, mailbox, perception, &sim, telemetryChannel, log);

    logLine("[perception] " + sim.id() + ": REQUIRED_FOR_SAFETY; " + python.id()
            + ": SUPPLEMENTARY, UNAVAILABLE until a receiver feeds it (none in this build)");

    auto channel = grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
    std::unique_ptr<RobotChannel::Stub> stub = RobotChannel::NewStub(channel);
    ClientContext context;
    std::shared_ptr<ClientReaderWriter<Telemetry, Command>> stream(stub->Connect(&context));

    std::thread reader([stream, &mailbox]() {
        Command command;
        while (stream->Read(&command)) {
            logLine("Command received: id=" + std::to_string(command.command_id()) + " type="
                    + robotagent::CommandType_Name(command.type()));
            robot::JavaCommand decoded = decode(command);
            if (decoded.waypoint) {
                std::ostringstream line;   // same format as the old agent
                line << "New target: (" << decoded.waypoint->x << ", " << decoded.waypoint->y << ")";
                logLine(line.str());
            }
            if (mailbox.post(decoded)) {
                logLine(std::string("[command] ") + robot::toString(decoded.type) + " id=" + std::to_string(decoded.id)
                        + " carries target_speed " + robot::fmt(decoded.target_speed)
                        + " <= 0 - treated as unspecified, speed limit unchanged");
            }
        }
        logLine("[stream] Read() ended - the Java stream is closed");
        mailbox.markStreamBroken("Read() returned false");
    });

    std::thread writer([stream, &telemetryChannel, &mailbox, robotId]() {
        uint64_t lastSeq = 0;
        int64_t timestamp = 0;
        while (true) {
            std::optional<robot::TelemetrySnapshot> snapshot = telemetryChannel.waitNewer(lastSeq);
            if (!snapshot) {
                return;
            }
            lastSeq = snapshot->seq;
            if (!stream->Write(encode(*snapshot, robotId, ++timestamp))) {
                logLine("[stream] Write() failed - the Java stream is broken");
                mailbox.markStreamBroken("Write() failed");
                return;
            }
        }
    });

    // Runs until the process is killed (graceful shutdown remains a known gap, as before).
    auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(config.control_period_s));
    auto next = std::chrono::steady_clock::now();
    while (true) {
        core.tick();
        next += period;
        std::this_thread::sleep_until(next);
    }
}
