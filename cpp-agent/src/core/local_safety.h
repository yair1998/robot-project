#pragma once

#include <optional>
#include <string>

#include "clock.h"
#include "local_navigation.h"
#include "local_perception.h"
#include "log.h"
#include "robot_config.h"

namespace robot {

enum class SafetyStatus {
    NORMAL,               // the proposal passes unchanged
    SPEED_LIMITED,        // normal slowing: speed reduced to what the verified distance allows
    TEMPORARY_BLOCK,      // speed 0 while no safe way forward exists; lifts as soon as one does (no latch)
    JAVA_STOP,            // speed 0 because Java's STOP is active (and nothing local is)
    LOCAL_STOP_LATCHED    // speed 0, latched; released only by the release rule (never by Java)
};

inline const char* toString(SafetyStatus s) {
    switch (s) {
        case SafetyStatus::NORMAL: return "NORMAL";
        case SafetyStatus::SPEED_LIMITED: return "SPEED_LIMITED";
        case SafetyStatus::TEMPORARY_BLOCK: return "TEMPORARY_BLOCK";
        case SafetyStatus::JAVA_STOP: return "JAVA_STOP";
        case SafetyStatus::LOCAL_STOP_LATCHED: return "LOCAL_STOP_LATCHED";
    }
    return "?";
}

struct SafetyInputs {
    const NavDecision* nav = nullptr;
    const LocalPerceptionState* perception = nullptr;
    double measured_speed_mps = 0.0;
    bool has_target = false;
    bool java_stop = false;               // independent of the local stop: Java can neither set nor clear a local stop
    bool stream_broken = false;
    std::string stream_broken_reason;
    SteadyTime now{};
    double dt_s = 0.2;
};

struct SafetyOutput {
    double speed_mps = 0.0;
    double heading_rad = 0.0;
    SafetyStatus status = SafetyStatus::NORMAL;
    bool local_stop_active = false;       // reported to Java as Telemetry.emergency_stop
    std::optional<double> hazard_m;       // the hazard that governs the motion direction (AHEAD and motion sector)
    double d_stop_m = 0.0;                // d_stop at the measured speed
    std::string reason;
};

// Reviews and constrains EVERY proposed motion before it reaches the motion backend; nothing bypasses it.
// Precedence, highest first:
//   (1) local emergency / hardware safety condition, a broken Java stream, invalid required perception, a hazard
//       within d_stop(measured speed)  -> LOCAL_STOP, latched
//   (2) other local restrictions: navigation BLOCKED, or no verified free distance (AHEAD and in the motion
//       sector) for any speed -> TEMPORARY_BLOCK
//   (3) an active Java STOP
//   (4) the navigation decision (heading, speed)
//   (5) Java's waypoint and requested speed (already inside the navigation decision)
// A Java REROUTE / RESUME / CONTINUE clears only the Java flag, never a local stop. Recovery motion that Java
// requests goes through the same checks. A simulated safety controller is not proof of physical safety.
class LocalSafetyController {
public:
    LocalSafetyController(SafetyConfig config, double cruise_speed_mps, MotionConfig motion, LogSink log);

    SafetyOutput review(const SafetyInputs& in);

    bool latched() const { return latched_; }
    const std::string& latchReason() const { return latch_reason_; }
    double releaseThreshold() const;

private:
    void latch(const std::string& reason, bool permanent);
    void tryRelease(const SafetyInputs& in, const std::optional<double>& hazard);

    SafetyConfig config_;
    double cruise_speed_mps_;
    MotionConfig motion_;
    LogSink log_;

    bool latched_ = false;
    bool permanent_ = false;
    std::string latch_reason_;
    int release_count_ = 0;
    SteadyTime release_since_{};
};

}  // namespace robot
