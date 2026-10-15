#include "control_core.h"

#include <algorithm>
#include <sstream>
#include <utility>

namespace robot {

ControlCore::ControlCore(const RobotConfig& config, const IClock& clock, IMotionBackend& motion,
                         CommandMailbox& mailbox, LocalPerception& perception, SimulatedSensorProvider* sim,
                         TelemetryChannel& telemetry, LogSink log)
    : config_(config),
      clock_(clock),
      motion_(motion),
      mailbox_(mailbox),
      perception_(perception),
      sim_(sim),
      telemetry_(telemetry),
      log_(std::move(log)),
      nav_(config.nav, config.perception.ahead_half_angle_deg, config.safety.stopping, log_),
      safety_(config.safety, config.cruise_speed_mps, config.motion, log_),
      requested_speed_(config.cruise_speed_mps) {}

double ControlCore::effectiveRequestedSpeed() const {
    if (!slow_down_active_) {
        return requested_speed_;
    }
    double cap = config_.slow_down_speed_mps;
    if (slow_down_speed_) {
        cap = std::min(cap, *slow_down_speed_);
    }
    return std::min(requested_speed_, cap);
}

void ControlCore::applyDirectives(const JavaDirectives& d) {
    if (d.stream_broken && !stream_broken_) {
        stream_broken_ = true;
        stream_broken_reason_ = d.stream_broken_reason;
    }
    ignored_nonpositive_speeds_ += static_cast<uint64_t>(d.ignored_nonpositive_speed);
    if (d.java_stop) {
        java_stop_ = *d.java_stop;
    }
    if (d.requested_speed) {
        requested_speed_ = *d.requested_speed;
    }
    if (d.slow_down_speed_cleared) {
        slow_down_speed_.reset();
    }
    if (d.slow_down_speed) {
        slow_down_speed_ = d.slow_down_speed;
    }
    if (d.slow_down) {
        slow_down_active_ = *d.slow_down;
    }
    if (d.new_waypoint) {
        // Stored even while stopped; it is acted on once every stop has cleared (safety keeps speed 0 until then).
        has_target_ = true;
        target_ = *d.new_waypoint;
        leg_start_ = motion_.pose().position;
        ++leg_id_;
        nav_.onNewLeg();
    }
}

void ControlCore::tick() {
    SteadyTime now = clock_.now();
    applyDirectives(mailbox_.take());

    Pose pose = motion_.pose();
    if (sim_ != nullptr) {
        sim_->setContext({has_target_, pose.position, leg_start_, leg_id_});
    }
    last_perception_ = perception_.update(now);

    NavInputs navIn;
    navIn.has_target = has_target_;
    navIn.position = pose.position;
    navIn.waypoint = target_;
    navIn.leg_start = leg_start_;
    navIn.requested_speed_mps = effectiveRequestedSpeed();
    navIn.perception = &last_perception_;
    navIn.now = now;
    navIn.dt_s = config_.control_period_s;
    last_nav_ = nav_.decide(navIn);

    SafetyInputs safetyIn;
    safetyIn.nav = &last_nav_;
    safetyIn.perception = &last_perception_;
    safetyIn.measured_speed_mps = motion_.measuredSpeed();
    safetyIn.has_target = has_target_;
    safetyIn.java_stop = java_stop_;
    safetyIn.stream_broken = stream_broken_;
    safetyIn.stream_broken_reason = stream_broken_reason_;
    safetyIn.now = now;
    safetyIn.dt_s = config_.control_period_s;
    last_safety_ = safety_.review(safetyIn);

    motion_.apply(last_safety_.speed_mps, last_safety_.heading_rad);
    motion_.step(config_.control_period_s);

    Pose after = motion_.pose();
    if (has_target_ && distance(after.position, target_) < config_.nav.arrival_threshold_m) {
        std::ostringstream line;   // same format as the old agent: the live scripts wait for this exact line
        line << "Arrived at target (" << target_.x << ", " << target_.y << ")";
        log_(line.str());
        has_target_ = false;
    }

    TelemetrySnapshot snapshot;
    snapshot.seq = ++seq_;
    snapshot.position = after.position;
    snapshot.measured_speed_mps = motion_.measuredSpeed();
    snapshot.nearest_obstacle_m = last_perception_.nearest_m;
    snapshot.emergency_stop = last_safety_.local_stop_active;
    snapshot.battery = config_.battery;
    snapshot.nav_state = last_nav_.state;
    snapshot.safety_status = last_safety_.status;
    telemetry_.publish(snapshot);
}

}  // namespace robot
