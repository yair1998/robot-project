#include "simulated_sensor_provider.h"

#include <utility>

namespace robot {

namespace {

SectorReading freeTo(double range, double uncertainty) {
    SectorReading r;
    r.state = SectorState::FREE_VERIFIED;
    r.free_range_m = range;
    r.range_uncertainty_m = uncertainty;
    r.confidence = 1.0;
    return r;
}

// A ranging sensor's return: the hazard at `range`, and free space verified up to it.
SectorReading occupiedAt(double range, double uncertainty) {
    SectorReading r;
    r.state = SectorState::OCCUPIED;
    r.hazard_range_m = range;
    r.free_range_m = range;
    r.range_uncertainty_m = uncertainty;
    r.confidence = 1.0;
    return r;
}

}  // namespace

SimulatedSensorProvider::SimulatedSensorProvider(SimSensorConfig config, LogSink log)
    : config_(std::move(config)), log_(std::move(log)) {}

void SimulatedSensorProvider::fillClear(SensorObservation& obs) const {
    for (SectorReading& s : obs.sectors) {
        s = freeTo(config_.clear_range_m, config_.range_uncertainty_m);
    }
}

SensorObservation SimulatedSensorProvider::snapshot(SteadyTime now) {
    SensorObservation obs;
    obs.source_id = config_.source_id;
    obs.frame = Frame::ROBOT;
    obs.acquired_at = now;
    obs.validity = Validity::VALID;
    obs.uncertainty_m = config_.range_uncertainty_m;
    obs.confidence = 1.0;

    if (context_.leg_id != zone_leg_id_) {
        // A new waypoint starts a new leg: the escalation can fire once again (today's applyCommand reset).
        zone_leg_id_ = context_.leg_id;
        escalation_active_ = false;
        escalation_handled_this_leg_ = false;
    }

    if (!context_.has_target) {
        in_avoidance_zone_ = false;
        fillClear(obs);
        return obs;
    }

    double progress = distance(context_.leg_start, context_.position);
    bool inAvoidanceZone = progress >= config_.avoidance_start_m && progress < config_.avoidance_end_m;
    bool inEscalationZone = progress >= config_.escalation_start_m && progress < config_.escalation_end_m;

    if (inAvoidanceZone != in_avoidance_zone_) {
        in_avoidance_zone_ = inAvoidanceZone;
        if (inAvoidanceZone) {
            log_(std::string("[sim-sensor] obstacle ") + fmt(config_.avoidance_hazard_m) + " m ahead, "
                 + toString(config_.avoidance_free_side) + " verified free (avoidance zone)");
        } else {
            log_("[sim-sensor] avoidance-zone obstacle no longer ahead");
        }
    }

    if (inEscalationZone && !escalation_active_ && !escalation_handled_this_leg_) {
        escalation_active_ = true;
        escalation_entered_at_ = now;
        log_("[sim-sensor] severe obstacle " + fmt(config_.escalation_hazard_m)
             + " m ahead, no free side (escalation zone)");
    }

    // TIME-based clear, also while the robot is stopped: a progress-based clear would deadlock (the STOP blocks the
    // very progress the clear would need).
    if (escalation_active_) {
        double elapsed = secondsBetween(escalation_entered_at_, now);
        if (elapsed >= config_.escalation_clear_after_s) {
            escalation_active_ = false;
            escalation_handled_this_leg_ = true;
            log_("[sim-sensor] simulated obstacle cleared after " + fmt(elapsed, 3) + "s");
        }
    }

    if (escalation_active_) {
        obs.sectors[index(Sector::AHEAD)] = occupiedAt(config_.escalation_hazard_m, config_.range_uncertainty_m);
        obs.sectors[index(Sector::LEFT)] = SectorReading{};
        obs.sectors[index(Sector::RIGHT)] = SectorReading{};
    } else if (inAvoidanceZone) {
        obs.sectors[index(Sector::AHEAD)] = occupiedAt(config_.avoidance_hazard_m, config_.range_uncertainty_m);
        Sector freeSide = config_.avoidance_free_side == Sector::RIGHT ? Sector::RIGHT : Sector::LEFT;
        Sector otherSide = freeSide == Sector::LEFT ? Sector::RIGHT : Sector::LEFT;
        obs.sectors[index(freeSide)] = freeTo(config_.clear_range_m, config_.range_uncertainty_m);
        obs.sectors[index(otherSide)] = SectorReading{};
    } else {
        fillClear(obs);
    }
    return obs;
}

}  // namespace robot
