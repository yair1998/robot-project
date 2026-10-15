#pragma once

#include "geometry.h"
#include "robot_config.h"

namespace robot {

struct Pose {
    Point position;
    double heading_rad = 0.0;
};

// The last stage of the pipeline: Java waypoint -> LocalNavigationController -> LocalSafetyController -> backend.
// A hardware drive would implement the same interface (send the speed/heading setpoint, integrate odometry).
class IMotionBackend {
public:
    virtual ~IMotionBackend() = default;
    virtual void apply(double speed_mps, double heading_rad) = 0;   // the setpoint for the next step
    virtual void step(double dt_s) = 0;
    virtual Pose pose() const = 0;
    virtual double measuredSpeed() const = 0;   // actual displacement of the last step / dt, not the setpoint
};

// A holonomic point robot. With the default (unlimited) limits the speed changes instantly, exactly like the
// old loop (it moved target_speed * dt per tick and stopped dead on STOP). Finite limits are used by the tests that
// prove the stopping-distance model is conservative.
class SimulatedMotionBackend final : public IMotionBackend {
public:
    explicit SimulatedMotionBackend(MotionConfig config, Point start = {}) : config_(config) { pose_.position = start; }

    void apply(double speed_mps, double heading_rad) override;
    void step(double dt_s) override;
    Pose pose() const override { return pose_; }
    double measuredSpeed() const override { return measured_speed_; }
    double currentSpeed() const { return speed_; }

private:
    MotionConfig config_;
    Pose pose_;
    double speed_ = 0.0;
    double measured_speed_ = 0.0;
    double command_speed_ = 0.0;
    double command_heading_ = 0.0;
};

}  // namespace robot
