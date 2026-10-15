#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "perception_types.h"
#include "stopping_model.h"

namespace robot {

// Every tunable of the agent in one place. The defaults keep today's live behaviour comparable: same loop period,
// cruise speed, arrival threshold, zone positions and distances, instant stops.

struct SimSensorConfig {
    std::string source_id = "sim-proximity";
    // Zones along the active leg, by distance from the leg start (today's values).
    double avoidance_start_m = 3.0;
    double avoidance_end_m = 5.0;
    double avoidance_hazard_m = 1.5;            // above Java's 1.0 m STOP threshold: avoidance alone never stops Java
    Sector avoidance_free_side = Sector::LEFT;  // the side verified free in the avoidance zone (today's deviation side)
    double escalation_start_m = 6.0;
    double escalation_end_m = 8.0;
    double escalation_hazard_m = 0.5;           // below Java's threshold: escalation reaches Java's safety chain
    double escalation_clear_after_s = 3.0;      // clears by elapsed TIME, never by robot progress (no deadlock)
    double clear_range_m = 8.0;                 // verified free range everywhere else (today's "clear" reading)
    double range_uncertainty_m = 0.0;           // a perfect simulated ranger
};

struct PerceptionConfig {
    double ahead_half_angle_deg = 4.0;          // headings within +-this of the reference heading are AHEAD
    double min_confidence_for_free = 0.7;       // a FREE report below this confidence never verifies free space
    double max_required_age_s = 0.5;            // a required source older than this counts as STALE
};

struct PythonAdapterConfig {
    std::string source_id = "python-perception";
    std::string expected_robot_id = "robot-1";
    uint32_t schema_version = 1;
    double stale_after_s = 0.5;                 // observation age (receive age + sender age) above this: STALE
    double unavailable_after_s = 2.0;           // above this, or nothing received: UNAVAILABLE
};

struct NavConfig {
    double avoidance_trigger_range_m = 2.0;     // an AHEAD hazard this close blocks the direct path
    double slowing_range_m = 3.0;               // an AHEAD hazard this close (beyond the trigger) slows tracking
    double min_slowing_fraction = 0.3;          // SLOWING_DOWN keeps at least this fraction of the requested speed
    std::vector<double> candidate_offsets_deg = {6.0, 15.0};  // tried smallest first, preferred side first
    Sector preferred_side = Sector::LEFT;       // deterministic tie-break between equal offsets
    double max_lateral_deviation_m = 0.3;       // from the leg line; LiveHarness's 0.5 m tolerance must stay >= this
    double max_avoidance_duration_s = 8.0;
    int reconverge_clear_observations = 3;
    double robot_half_width_m = 0.25;
    double clearance_margin_m = 0.2;
    double arrival_threshold_m = 0.3;           // judged against the real waypoint; must stay <= Java's 0.5 m
};

struct SafetyConfig {
    StoppingModel stopping;
    double release_extra_m = 0.5;               // release threshold = d_stop(cruise) + this (hysteresis above stop)
    int release_clear_observations = 3;
    double release_hold_s = 0.3;
};

struct MotionConfig {
    double max_accel_mps2 = 0.0;                // 0 = unlimited (today's behaviour)
    double max_decel_mps2 = 0.0;                // 0 = unlimited: an instant stop, as today
};

struct RobotConfig {
    double control_period_s = 0.2;              // today's loop period
    double cruise_speed_mps = 1.0;              // requested speed until Java says otherwise (today's constant)
    double slow_down_speed_mps = 0.5;           // the cap a Java SLOW_DOWN puts on motion
    double battery = 90.0;                      // reported unchanged (no battery model)
    SimSensorConfig sim;
    PerceptionConfig perception;
    PythonAdapterConfig python;
    NavConfig nav;
    SafetyConfig safety;
    MotionConfig motion;
};

}  // namespace robot
