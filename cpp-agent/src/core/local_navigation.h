#pragma once

#include <optional>
#include <string>
#include <vector>

#include "clock.h"
#include "geometry.h"
#include "local_perception.h"
#include "log.h"
#include "robot_config.h"

namespace robot {

enum class NavState { IDLE_NO_TARGET, TRACKING_WAYPOINT, SLOWING_DOWN, LOCAL_AVOIDANCE, BLOCKED };

inline const char* toString(NavState s) {
    switch (s) {
        case NavState::IDLE_NO_TARGET: return "IDLE_NO_TARGET";
        case NavState::TRACKING_WAYPOINT: return "TRACKING_WAYPOINT";
        case NavState::SLOWING_DOWN: return "SLOWING_DOWN";
        case NavState::LOCAL_AVOIDANCE: return "LOCAL_AVOIDANCE";
        case NavState::BLOCKED: return "BLOCKED";
    }
    return "?";
}

struct NavInputs {
    bool has_target = false;
    Point position;
    Point waypoint;                       // the REAL Java waypoint - never replaced by an aim point
    Point leg_start;
    double requested_speed_mps = 0.0;     // Java's requested speed, already capped by an active SLOW_DOWN
    const LocalPerceptionState* perception = nullptr;
    SteadyTime now{};
    double dt_s = 0.2;
};

struct NavDecision {
    NavState state = NavState::IDLE_NO_TARGET;
    double heading_rad = 0.0;
    double offset_rad = 0.0;              // heading - direct bearing (non-zero only in LOCAL_AVOIDANCE)
    Sector motion_sector = Sector::AHEAD;
    double speed_mps = 0.0;
    std::string reason;
};

// Waypoint tracking plus local decisions, separate from the movement simulator. It never rewrites Java's mission
// and never reports a different target. Replaces the old fixed 0.8 m aim-point offset:
//   - TRACKING_WAYPOINT only while AHEAD is verified free for the stopping distance (UNKNOWN is never "free");
//   - SLOWING_DOWN while an AHEAD hazard is inside the slowing range or the verified free distance limits speed;
//   - LOCAL_AVOIDANCE only toward a FREE_VERIFIED sector with enough verified clearance, smallest offset first,
//     never beyond max_lateral_deviation_m from the leg line or max_avoidance_duration_s;
//   - BLOCKED (speed 0) when no acceptable heading exists - never an arbitrary lateral offset;
//   - once max_avoidance_duration_s is used up, BLOCKED until the direct path is clear again or a new waypoint arrives
//     (no fresh avoidance window on the same blockage);
//   - back to TRACKING only after AHEAD was FREE_VERIFIED for reconverge_clear_observations ticks in a row.
class LocalNavigationController {
public:
    LocalNavigationController(NavConfig config, double ahead_half_angle_deg, StoppingModel stopping, LogSink log);

    NavDecision decide(const NavInputs& in);

    // A new Java waypoint: drop all avoidance bookkeeping so no stale deviation carries into the new leg.
    void onNewLeg();

    NavState state() const { return state_; }

private:
    std::optional<double> pickCandidate(const NavInputs& in, double direct, double speed) const;
    NavDecision blocked(NavDecision d, const std::string& reason);
    NavDecision avoid(NavDecision d, const NavInputs& in, double direct, double offset, double speed);
    void setState(NavState next, const std::string& reason);

    NavConfig config_;
    double ahead_half_angle_rad_;
    StoppingModel stopping_;
    LogSink log_;
    std::vector<double> offsets_rad_;     // ordered: smallest first, preferred side first

    NavState state_ = NavState::IDLE_NO_TARGET;
    std::string last_reason_;
    SteadyTime avoidance_started_at_{};
    std::optional<double> avoidance_offset_rad_;
    int clear_count_ = 0;
    bool avoidance_used_up_ = false;      // the duration limit ran out; cleared by normal progress or a new leg
};

}  // namespace robot
