#pragma once

#include <string>

#include "clock.h"
#include "perception_types.h"

namespace robot {

// A source of local perception. Implemented now: SimulatedSensorProvider (the scripted zones) and
// PythonPerceptionAdapter (latest-wins slot fed by offer()). A hardware provider (lidar, sonar ring, bumpers) would
// implement the same two methods: convert its latest driver frame into robot-relative sector readings, stamp it with
// the local monotonic acquisition time and report STALE/INVALID/UNAVAILABLE honestly - never a made-up value.
class ISensorProvider {
public:
    virtual ~ISensorProvider() = default;
    virtual const std::string& id() const = 0;
    // The latest observation as of `now`. Called from the control thread every tick: must not block on I/O.
    virtual SensorObservation snapshot(SteadyTime now) = 0;
};

}  // namespace robot
