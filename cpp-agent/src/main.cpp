#include <grpcpp/grpcpp.h>
#include "robot_channel.grpc.pb.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using grpc::ClientContext;
using grpc::ClientReaderWriter;
using grpc::Status;

using robotagent::Command;
using robotagent::RobotChannel;
using robotagent::Telemetry;

namespace {

constexpr double kFakeSpeedMetersPerSecond = 1.0;
constexpr double kArrivalThresholdMeters = 0.3;
constexpr auto kTelemetryInterval = std::chrono::milliseconds(200);

// Shared between the reader thread (writes new targets as commands
// arrive) and the main loop (reads the target to move toward it).
// This is the only shared state in the client - deliberately small.
struct RobotState {
    std::mutex mutex;

    double position_x = 0.0;
    double position_y = 0.0;

    bool has_target = false;
    double target_x = 0.0;
    double target_y = 0.0;

    bool stopped = false;
};

void applyCommand(RobotState& state, const Command& command) {
    std::lock_guard<std::mutex> lock(state.mutex);

    if (command.type() == robotagent::COMMAND_TYPE_STOP) {
        state.stopped = true;
        return;
    }

    state.stopped = false;

    if (command.has_target()) {
        state.has_target = true;
        state.target_x = command.target().x();
        state.target_y = command.target().y();

        std::cout
            << "New target: (" << state.target_x << ", " << state.target_y << ")"
            << std::endl;
    }
}

// Moves position toward the current target by one tick's worth of
// distance. No map, no obstacle awareness - exactly the boundary
// already agreed on: Java decides where, C++ only knows how to get
// a little closer to there.
void advancePosition(RobotState& state, double dtSeconds) {
    std::lock_guard<std::mutex> lock(state.mutex);

    if (state.stopped || !state.has_target) {
        return;
    }

    double dx = state.target_x - state.position_x;
    double dy = state.target_y - state.position_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < kArrivalThresholdMeters) {
        std::cout
            << "Arrived at target (" << state.target_x << ", " << state.target_y << ")"
            << std::endl;
        state.has_target = false;
        return;
    }

    double step = kFakeSpeedMetersPerSecond * dtSeconds;
    double ratio = step / distance;

    state.position_x += dx * ratio;
    state.position_y += dy * ratio;
}

Telemetry buildTelemetry(RobotState& state, const std::string& robotId, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(state.mutex);

    Telemetry telemetry;
    telemetry.set_robot_id(robotId);
    telemetry.set_position_x(state.position_x);
    telemetry.set_position_y(state.position_y);
    telemetry.set_speed(state.stopped ? 0.0 : kFakeSpeedMetersPerSecond);
    telemetry.set_battery(90.0);
    telemetry.set_timestamp(timestamp);
    telemetry.set_emergency_stop(false);

    return telemetry;
}

}  // namespace

int main() {
    const char* envTarget = std::getenv("GRPC_TARGET");
    std::string target = envTarget ? envTarget : "host.docker.internal:50051";

    std::cout << "Connecting to " << target << std::endl;

    auto channel = grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
    std::unique_ptr<RobotChannel::Stub> stub = RobotChannel::NewStub(channel);

    ClientContext context;
    std::shared_ptr<ClientReaderWriter<Telemetry, Command>> stream(stub->Connect(&context));

    RobotState state;

    std::thread reader([stream, &state]() {
        Command command;
        while (stream->Read(&command)) {
            std::cout
                << "Command received: id=" << command.command_id()
                << " type=" << robotagent::CommandType_Name(command.type())
                << std::endl;
            applyCommand(state, command);
        }
    });

    int64_t timestamp = 0;
    const std::string robotId = "robot-1";
    const double tickSeconds =
        std::chrono::duration<double>(kTelemetryInterval).count();

    // Runs indefinitely - this is the real client loop now, not a
    // fixed test sequence. Stop with Ctrl+C for now; graceful shutdown
    // (WritesDone/Finish on signal) is a known gap, not an oversight -
    // not worth solving before there's a real path to walk.
    while (true) {
        advancePosition(state, tickSeconds);

        Telemetry telemetry = buildTelemetry(state, robotId, ++timestamp);
        stream->Write(telemetry);

        std::this_thread::sleep_for(kTelemetryInterval);
    }
}
