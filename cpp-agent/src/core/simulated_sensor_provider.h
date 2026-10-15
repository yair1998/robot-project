#pragma once

#include <cstdint>

#include "geometry.h"
#include "log.h"
#include "robot_config.h"
#include "sensor_provider.h"

namespace robot {

// What the simulated sensor needs to know about the robot; set by the control thread before every snapshot().
struct SimRobotContext {
    bool has_target = false;
    Point position;
    Point leg_start;
    uint64_t leg_id = 0;   // changes whenever Java supplies a new waypoint
};

// The zone script that used to live in main.cpp's updateSimulatedSensorLocked(), moved behind ISensorProvider and
// extended to per-sector readings. Deterministic: zones are by leg progress, the escalation clears by elapsed time
// on the injected clock. Sensing runs on every tick - also while the robot is stopped or has no target.
//
//   avoidance zone : AHEAD OCCUPIED at 1.5 m, the configured side FREE_VERIFIED to 8.0 m, the other side UNKNOWN
//   escalation zone: AHEAD OCCUPIED at 0.5 m, no side FREE_VERIFIED; clears 3.0 s after it was first sensed
//   everywhere else: all sectors FREE_VERIFIED to 8.0 m
class SimulatedSensorProvider final : public ISensorProvider {
public:
    SimulatedSensorProvider(SimSensorConfig config, LogSink log);

    const std::string& id() const override { return config_.source_id; }

    void setContext(const SimRobotContext& context) { context_ = context; }

    SensorObservation snapshot(SteadyTime now) override;

private:
    void fillClear(SensorObservation& obs) const;

    SimSensorConfig config_;
    LogSink log_;
    SimRobotContext context_;
    uint64_t zone_leg_id_ = 0;
    bool in_avoidance_zone_ = false;
    bool escalation_active_ = false;
    bool escalation_handled_this_leg_ = false;
    SteadyTime escalation_entered_at_{};
};

}  // namespace robot
