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

constexpr double kLocalAvoidanceLegProgressStartMeters = 3.0;
constexpr double kLocalAvoidanceLegProgressEndMeters = 5.0;
constexpr double kEscalationLegProgressStartMeters = 6.0;
constexpr double kEscalationLegProgressEndMeters = 8.0;
constexpr double kLateralOffsetMeters = 0.8;
constexpr double kClearObstacleDistance = 8.0;
constexpr double kAvoidanceObstacleDistance = 1.5; // stays ABOVE
    // Java's 1.0m STOP threshold on purpose - normal local
    // avoidance must never by itself trigger Java's safety stop
constexpr double kEscalationObstacleDistance = 0.5; // deliberately
    // BELOW Java's 1.0m STOP threshold - this is how escalation
    // reaches Java: through the EXISTING, unmodified safety
    // branch, not a new message type
constexpr double kEscalationClearAfterSeconds = 3.0; // the
    // simulated obstacle clears this long after being detected,
    // regardless of whether the robot is moving

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

    double target_speed = kFakeSpeedMetersPerSecond;

    double leg_start_x = 0.0;
    double leg_start_y = 0.0;
    bool avoiding_obstacle = false;
    double lateral_offset_meters = 0.0;
    double sensed_obstacle_distance = kClearObstacleDistance;
    bool in_escalation_zone = false;
    bool escalation_handled_this_leg = false;
    std::chrono::steady_clock::time_point escalation_zone_entered_at;
};

void applyCommand(RobotState& state, const Command& command) {
    std::lock_guard<std::mutex> lock(state.mutex);

    state.target_speed = command.target_speed();

    if (command.type() == robotagent::COMMAND_TYPE_STOP) {
        state.stopped = true;
        return;
    }

    state.stopped = false;

    if (command.has_target()) {
        state.leg_start_x = state.position_x;
        state.leg_start_y = state.position_y;
        state.in_escalation_zone = false;
        state.escalation_handled_this_leg = false;

        state.has_target = true;
        state.target_x = command.target().x();
        state.target_y = command.target().y();

        std::cout
            << "New target: (" << state.target_x << ", " << state.target_y << ")"
            << std::endl;
    }
}

double legProgressMetersLocked(const RobotState& state) {
    double dx = state.position_x - state.leg_start_x;
    double dy = state.position_y - state.leg_start_y;
    return std::sqrt(dx * dx + dy * dy);
}

// Sensing runs unconditionally every tick, regardless of stopped/has_target -
// a real proximity sensor doesn't stop sensing just because the robot isn't
// moving. Since there's no real sensor yet, this drives two fixed simulated
// zones along whichever leg is active, purely to exercise the end-to-end
// path: local avoidance (handled entirely here, Java never sees it) and a
// severe-obstacle escalation (reported to Java via sensed_obstacle_distance,
// through the existing safety branch). The escalation zone clears based on
// ELAPSED TIME rather than distance traveled - if it depended on distance, a
// full STOP (which halts movement) would prevent the very progress needed to
// clear the zone, deadlocking permanently.
void updateSimulatedSensorLocked(RobotState& state) {
    if (!state.has_target) {
        state.avoiding_obstacle = false;
        state.lateral_offset_meters = 0.0;
        state.sensed_obstacle_distance = kClearObstacleDistance;
        return;
    }

    double legProgress = legProgressMetersLocked(state);

    bool inAvoidanceZone = legProgress >= kLocalAvoidanceLegProgressStartMeters
            && legProgress < kLocalAvoidanceLegProgressEndMeters;
    bool inEscalationZoneByPosition = legProgress >= kEscalationLegProgressStartMeters
            && legProgress < kEscalationLegProgressEndMeters;

    bool wasAvoiding = state.avoiding_obstacle;
    state.avoiding_obstacle = inAvoidanceZone;
    state.lateral_offset_meters = inAvoidanceZone ? kLateralOffsetMeters : 0.0;

    if (inAvoidanceZone && !wasAvoiding) {
        std::cout << "[local-nav] Simulated obstacle ahead - deviating "
                  << state.lateral_offset_meters << "m to clear it" << std::endl;
    }
    if (!inAvoidanceZone && wasAvoiding) {
        std::cout << "[local-nav] Clear - converging back to the direct path" << std::endl;
    }

    // Entry: only while this zone hasn't already fired-and-cleared once on
    // this same leg (prevents re-triggering on the tail of the zone once the
    // robot resumes moving after a time-based clear).
    if (inEscalationZoneByPosition && !state.in_escalation_zone
            && !state.escalation_handled_this_leg) {
        state.in_escalation_zone = true;
        state.escalation_zone_entered_at = std::chrono::steady_clock::now();
        std::cout << "[local-nav] Simulated severe obstacle - no clear path, reporting to Java" << std::endl;
    }

    // Clearing: TIME-based, runs regardless of whether the robot is
    // currently stopped - this is what breaks the deadlock. A real dynamic
    // obstacle clears on its own over time; it doesn't wait for the robot it
    // just forced to stop to somehow keep moving.
    if (state.in_escalation_zone) {
        double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - state.escalation_zone_entered_at).count();
        if (elapsed >= kEscalationClearAfterSeconds) {
            state.in_escalation_zone = false;
            state.escalation_handled_this_leg = true;
            std::cout << "[local-nav] Simulated obstacle cleared after "
                      << elapsed << "s" << std::endl;
        }
    }

    if (state.in_escalation_zone) {
        state.sensed_obstacle_distance = kEscalationObstacleDistance;
    } else if (inAvoidanceZone) {
        state.sensed_obstacle_distance = kAvoidanceObstacleDistance;
    } else {
        state.sensed_obstacle_distance = kClearObstacleDistance;
    }
}

// Moves position toward the current target by one tick's worth of
// distance. No map, no long-range obstacle awareness - exactly the
// boundary already agreed on: Java decides where, C++ only knows how to
// get a little closer to there (now with a local reactive deviation
// layered on top, see updateSimulatedSensorLocked - the target itself
// never changes here, only the path taken to it).
void advancePosition(RobotState& state, double dtSeconds) {
    std::lock_guard<std::mutex> lock(state.mutex);

    updateSimulatedSensorLocked(state);

    if (state.stopped || !state.has_target) {
        return;
    }

    double aimX = state.target_x;
    double aimY = state.target_y;

    if (state.lateral_offset_meters != 0.0) {
        double dirX = state.target_x - state.position_x;
        double dirY = state.target_y - state.position_y;
        double dirLen = std::sqrt(dirX * dirX + dirY * dirY);
        if (dirLen > 1e-6) {
            double perpX = -dirY / dirLen;
            double perpY = dirX / dirLen;
            aimX += perpX * state.lateral_offset_meters;
            aimY += perpY * state.lateral_offset_meters;
        }
    }

    // Arrival is checked against the REAL target, never the
    // laterally-offset aim point - the offset only shapes the path
    // taken, it never changes what "arrived" means, and it never
    // changes what target Java thinks is active.
    double dxReal = state.target_x - state.position_x;
    double dyReal = state.target_y - state.position_y;
    double distanceToRealTarget = std::sqrt(dxReal * dxReal + dyReal * dyReal);

    if (distanceToRealTarget < kArrivalThresholdMeters) {
        std::cout
            << "Arrived at target (" << state.target_x << ", " << state.target_y << ")"
            << std::endl;
        state.has_target = false;
        return;
    }

    double dx = aimX - state.position_x;
    double dy = aimY - state.position_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > 1e-6) {
        double step = state.target_speed * dtSeconds;
        double ratio = step / distance;
        state.position_x += dx * ratio;
        state.position_y += dy * ratio;
    }
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
    telemetry.set_nearest_obstacle_distance(state.sensed_obstacle_distance);

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
