#pragma once

#include <algorithm>
#include <cmath>

namespace robot {

// d_stop(v) = v * t_reaction + v^2 / (2 * a_brake) + margin
//
// The defaults satisfy the required relationships (asserted in tests/local_nav_test.cpp):
//   d_stop(1.0) = 0.2 + 0.5 + 0.35 = 1.05 m
//   - escalation hazard 0.5 m <= 1.05 -> LOCAL_STOP at cruise
//   - avoidance hazard 1.5 m  >  1.05 -> no stop at cruise (avoidance stays possible)
//   - 1.05 >= Java's STOP threshold 1.0 m -> local safety never waits on Java for a hazard Java would stop for
struct StoppingModel {
    double reaction_time_s = 0.2;      // >= one control period: a hazard sensed now is acted on within one tick
    double braking_decel_mps2 = 1.0;   // the deceleration the drive is assumed to achieve under all conditions
    double margin_m = 0.35;            // distance still kept after a full stop
};

inline double stoppingDistance(const StoppingModel& m, double speed) {
    double v = std::max(0.0, speed);
    return v * m.reaction_time_s + v * v / (2.0 * m.braking_decel_mps2) + m.margin_m;
}

// The largest speed whose stopping distance fits in `distance`: the inverse of stoppingDistance().
// 0 when even a standing robot would be inside the margin.
inline double maxSpeedForDistance(const StoppingModel& m, double distance) {
    double usable = distance - m.margin_m;
    if (usable <= 0.0) {
        return 0.0;
    }
    double a = m.braking_decel_mps2;
    double t = m.reaction_time_s;
    return a * (-t + std::sqrt(t * t + 2.0 * usable / a));
}

}  // namespace robot
