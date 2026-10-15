#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "geometry.h"

namespace robot {

enum class JavaCommandType { UNSPECIFIED, CONTINUE, SLOW_DOWN, STOP, RESUME, REROUTE };

inline const char* toString(JavaCommandType t) {
    switch (t) {
        case JavaCommandType::UNSPECIFIED: return "UNSPECIFIED";
        case JavaCommandType::CONTINUE: return "CONTINUE";
        case JavaCommandType::SLOW_DOWN: return "SLOW_DOWN";
        case JavaCommandType::STOP: return "STOP";
        case JavaCommandType::RESUME: return "RESUME";
        case JavaCommandType::REROUTE: return "REROUTE";
    }
    return "?";
}

// A Java command, already decoded from protobuf by the gRPC reader thread (the core never sees protobuf).
struct JavaCommand {
    JavaCommandType type = JavaCommandType::UNSPECIFIED;
    int64_t id = 0;
    double target_speed = 0.0;
    std::optional<Point> waypoint;
};

// Everything Java asked for since the control thread last looked - the net effect, not a queue.
struct JavaDirectives {
    int commands = 0;
    std::optional<Point> new_waypoint;      // latest waypoint wins
    std::optional<bool> java_stop;          // state after the last STOP / non-STOP command
    std::optional<double> requested_speed;  // latest valid (> 0) speed from CONTINUE / RESUME / REROUTE
    std::optional<bool> slow_down;          // SLOW_DOWN -> true; CONTINUE / RESUME -> false
    bool slow_down_speed_cleared = false;   // a CONTINUE / RESUME lifted the SLOW_DOWN, its speed included
    std::optional<double> slow_down_speed;  // latest valid (> 0) SLOW_DOWN speed since then
    int ignored_nonpositive_speed = 0;
    bool stream_broken = false;             // sticky once set
    std::string stream_broken_reason;
};

// Written by the gRPC reader thread, drained once per tick by the control thread. Bounded by construction: the
// pending state is one JavaDirectives value, whatever the number of commands. The mutex is a leaf lock (nothing
// else is locked while it is held) and is never held across a network call.
class CommandMailbox {
public:
    // Applies the command to the pending directives. Returns true when a non-STOP command's target_speed <= 0 was
    // treated as "unspecified" (the caller logs it): such a command never sets the speed limit to zero.
    bool post(const JavaCommand& command);

    void markStreamBroken(const std::string& reason);

    JavaDirectives take();

    uint64_t posted() const;

private:
    mutable std::mutex mutex_;
    JavaDirectives pending_;
    bool stream_broken_ = false;
    std::string stream_broken_reason_;
    uint64_t posted_ = 0;
};

}  // namespace robot
