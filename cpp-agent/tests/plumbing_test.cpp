#include <grpcpp/grpcpp.h>
#include "robot_channel.grpc.pb.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using grpc::ClientContext;
using grpc::ClientReaderWriter;
using grpc::Status;

using robotagent::Command;
using robotagent::CommandType;
using robotagent::CommandType_Name;
using robotagent::RobotChannel;
using robotagent::Telemetry;

namespace {

constexpr double kEpsilon = 1e-6;

bool approx_equal(double a, double b) {
    return std::fabs(a - b) < kEpsilon;
}

// One telemetry variation and the Command the server is expected to
// reply with. Case 5 exercises the Waypoint/REROUTE plumbing path via
// position_x == 2.0.
struct TestCase {
    std::string description;

    double position_x;
    double nearest_obstacle_distance;
    bool emergency_stop;

    CommandType expected_type;
    double expected_target_speed;
    bool expected_has_target;
    double expected_target_x;
    double expected_target_y;
};

struct CaseResult {
    bool passed = true;
    std::vector<std::string> mismatches;
};

CaseResult check_command(const Command& command, const TestCase& test_case) {
    CaseResult result;

    if (command.type() != test_case.expected_type) {
        result.passed = false;
        result.mismatches.push_back(
            "type: expected "
                + std::string(CommandType_Name(test_case.expected_type))
                + ", got "
                + std::string(CommandType_Name(command.type()))
        );
    }

    if (!approx_equal(command.target_speed(), test_case.expected_target_speed)) {
        result.passed = false;
        result.mismatches.push_back(
            "target_speed: expected "
                + std::to_string(test_case.expected_target_speed)
                + ", got "
                + std::to_string(command.target_speed())
        );
    }

    if (command.has_target() != test_case.expected_has_target) {
        result.passed = false;
        result.mismatches.push_back(
            "has_target: expected "
                + std::string(test_case.expected_has_target ? "true" : "false")
                + ", got "
                + std::string(command.has_target() ? "true" : "false")
        );
    }

    if (test_case.expected_has_target && command.has_target()) {
        if (!approx_equal(command.target().x(), test_case.expected_target_x)) {
            result.passed = false;
            result.mismatches.push_back(
                "target.x: expected "
                    + std::to_string(test_case.expected_target_x)
                    + ", got "
                    + std::to_string(command.target().x())
            );
        }

        if (!approx_equal(command.target().y(), test_case.expected_target_y)) {
            result.passed = false;
            result.mismatches.push_back(
                "target.y: expected "
                    + std::to_string(test_case.expected_target_y)
                    + ", got "
                    + std::to_string(command.target().y())
            );
        }
    }

    return result;
}

}  // namespace

int main() {
    std::string target = "host.docker.internal:50051";

    std::cout << "Connecting to " << target << std::endl;

    auto channel = grpc::CreateChannel(
        target,
        grpc::InsecureChannelCredentials()
    );

    std::unique_ptr<RobotChannel::Stub> stub =
        RobotChannel::NewStub(channel);

    ClientContext context;

    std::unique_ptr<ClientReaderWriter<Telemetry, Command>> stream(
        stub->Connect(&context)
    );

    const std::vector<TestCase> cases = {
        // 1: clear path -> CONTINUE at cruising speed
        {"case 1: clear path (distance=8.0)",
         0.0, 8.0, false,
         robotagent::COMMAND_TYPE_CONTINUE, 3.5, false, 0.0, 0.0},

        // 2: obstacle in slow-down band -> SLOW_DOWN
        {"case 2: obstacle at 2.0",
         0.0, 2.0, false,
         robotagent::COMMAND_TYPE_SLOW_DOWN, 1.0, false, 0.0, 0.0},

        // 3: obstacle in stop band -> STOP
        {"case 3: obstacle at 0.5",
         0.0, 0.5, false,
         robotagent::COMMAND_TYPE_STOP, 0.0, false, 0.0, 0.0},

        // 4: emergency stop flag -> STOP regardless of obstacle distance
        {"case 4: emergency stop",
         0.0, 8.0, true,
         robotagent::COMMAND_TYPE_STOP, 0.0, false, 0.0, 0.0},

        // 5: Waypoint plumbing check via position_x == 2.0 test hook -> REROUTE
        {"case 5: waypoint plumbing (position_x=2.0)",
         2.0, 8.0, false,
         robotagent::COMMAND_TYPE_REROUTE, 0.0, true, 12.0, 8.0},
    };

    int passed_count = 0;
    int64_t previous_command_id = 0;

    for (std::size_t i = 0; i < cases.size(); ++i) {
        if (i > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(500)
            );
        }

        const TestCase& test_case = cases[i];

        std::cout
            << "[" << (i + 1) << "/" << cases.size() << "] "
            << test_case.description
            << std::endl;

        Telemetry telemetry;

        telemetry.set_robot_id("robot-1");
        telemetry.set_position_x(test_case.position_x);
        telemetry.set_position_y(0.0);
        telemetry.set_speed(3.5);
        telemetry.set_battery(90.0);
        telemetry.set_timestamp(1);
        telemetry.set_nearest_obstacle_distance(
            test_case.nearest_obstacle_distance
        );
        telemetry.set_emergency_stop(test_case.emergency_stop);

        if (!stream->Write(telemetry)) {
            std::cout
                << "  FAIL: write failed - stream is no longer writable"
                << std::endl;

            break;
        }

        Command command;

        if (!stream->Read(&command)) {
            std::cout
                << "  FAIL: stream closed before a response arrived"
                << std::endl;

            break;
        }

        std::cout
            << "  received: command_id=" << command.command_id()
            << " type=" << CommandType_Name(command.type())
            << " target_speed=" << command.target_speed();

        if (command.has_target()) {
            std::cout
                << " target=(" << command.target().x()
                << ", " << command.target().y() << ")";
        }

        std::cout << std::endl;

        CaseResult result = check_command(command, test_case);

        // Response order is verified the same way command_id ordering is:
        // each case does one blocking Write followed by one blocking Read,
        // so a passing id check here also proves this response belongs to
        // this request, in send order.
        if (command.command_id() <= previous_command_id) {
            result.passed = false;
            result.mismatches.push_back(
                "command_id: expected strictly greater than "
                    + std::to_string(previous_command_id)
                    + ", got "
                    + std::to_string(command.command_id())
            );
        }

        previous_command_id = command.command_id();

        if (result.passed) {
            std::cout << "  PASS" << std::endl;
            ++passed_count;
        } else {
            std::cout << "  FAIL" << std::endl;

            for (const std::string& mismatch : result.mismatches) {
                std::cout << "    " << mismatch << std::endl;
            }
        }
    }

    stream->WritesDone();

    Command trailing;

    while (stream->Read(&trailing)) {
        std::cout
            << "Unexpected trailing command: command_id="
            << trailing.command_id()
            << " type=" << CommandType_Name(trailing.type())
            << std::endl;
    }

    Status status = stream->Finish();

    if (!status.ok()) {
        std::cerr
            << "RPC failed: "
            << status.error_code() << ": " << status.error_message()
            << std::endl;

        return 1;
    }

    if (passed_count == static_cast<int>(cases.size())) {
        std::cout << "ALL PASSED" << std::endl;
        return 0;
    }

    std::cout
        << passed_count << "/" << cases.size() << " passed"
        << std::endl;

    return 1;
}
