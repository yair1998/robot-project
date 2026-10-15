#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "clock.h"
#include "command_mailbox.h"
#include "geometry.h"
#include "local_navigation.h"
#include "local_perception.h"
#include "local_safety.h"
#include "log.h"
#include "motion_backend.h"
#include "robot_config.h"
#include "simulated_sensor_provider.h"
#include "telemetry_channel.h"

namespace robot {

// One control tick, run by the control thread - the ONLY mutator of robot state:
//   mailbox -> perception (sense at the current pose, fuse) -> navigation -> safety -> motion backend -> arrival
//   check against the real waypoint -> telemetry snapshot.
// Sensing happens at the pose BEFORE the step, as in the old loop, so the reported reading belongs to the decision
// that produced the reported motion. Tests drive tick() directly with a ManualClock: no threads, no sleeps.
class ControlCore {
public:
    ControlCore(const RobotConfig& config, const IClock& clock, IMotionBackend& motion, CommandMailbox& mailbox,
                LocalPerception& perception, SimulatedSensorProvider* sim, TelemetryChannel& telemetry, LogSink log);

    void tick();

    bool hasTarget() const { return has_target_; }
    Point target() const { return target_; }
    Point legStart() const { return leg_start_; }
    bool javaStop() const { return java_stop_; }
    bool slowDownActive() const { return slow_down_active_; }
    double requestedSpeed() const { return requested_speed_; }
    double effectiveRequestedSpeed() const;
    bool streamBroken() const { return stream_broken_; }
    uint64_t legId() const { return leg_id_; }
    uint64_t ignoredNonPositiveSpeeds() const { return ignored_nonpositive_speeds_; }
    const NavDecision& lastNav() const { return last_nav_; }
    const SafetyOutput& lastSafety() const { return last_safety_; }
    const LocalPerceptionState& lastPerception() const { return last_perception_; }
    const LocalSafetyController& safety() const { return safety_; }

private:
    void applyDirectives(const JavaDirectives& d);

    RobotConfig config_;
    const IClock& clock_;
    IMotionBackend& motion_;
    CommandMailbox& mailbox_;
    LocalPerception& perception_;
    SimulatedSensorProvider* sim_;
    TelemetryChannel& telemetry_;
    LogSink log_;
    LocalNavigationController nav_;
    LocalSafetyController safety_;

    bool has_target_ = false;
    Point target_;
    Point leg_start_;
    uint64_t leg_id_ = 0;
    bool java_stop_ = false;
    bool slow_down_active_ = false;
    std::optional<double> slow_down_speed_;
    double requested_speed_;
    bool stream_broken_ = false;
    std::string stream_broken_reason_;
    uint64_t ignored_nonpositive_speeds_ = 0;
    uint64_t seq_ = 0;

    NavDecision last_nav_;
    SafetyOutput last_safety_;
    LocalPerceptionState last_perception_;
};

}  // namespace robot
