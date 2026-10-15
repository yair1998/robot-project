#include "motion_backend.h"

#include <algorithm>
#include <cmath>

namespace robot {

void SimulatedMotionBackend::apply(double speed_mps, double heading_rad) {
    command_speed_ = std::max(0.0, speed_mps);
    command_heading_ = heading_rad;
}

void SimulatedMotionBackend::step(double dt_s) {
    double v0 = speed_;
    double target = command_speed_;
    double limit = target > v0 ? config_.max_accel_mps2 : config_.max_decel_mps2;
    double v1;
    double travelled;
    if (limit <= 0.0 || dt_s <= 0.0) {
        v1 = target;   // unlimited: the new speed holds for the whole step (the old loop's behaviour)
        travelled = target * dt_s;
    } else {
        double change = std::fabs(target - v0);
        double timeToTarget = change / limit;
        if (timeToTarget >= dt_s) {
            v1 = v0 + (target > v0 ? 1.0 : -1.0) * limit * dt_s;
            travelled = (v0 + v1) / 2.0 * dt_s;
        } else {
            v1 = target;
            travelled = (v0 + target) / 2.0 * timeToTarget + target * (dt_s - timeToTarget);
        }
    }
    pose_.heading_rad = command_heading_;
    pose_.position = advance(pose_.position, pose_.heading_rad, travelled);
    speed_ = v1;
    measured_speed_ = dt_s > 0.0 ? travelled / dt_s : 0.0;
}

}  // namespace robot
