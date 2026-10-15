#include "local_safety.h"

#include <algorithm>
#include <utility>

#include "stopping_model.h"

namespace robot {

namespace {

// The hazard that governs motion along `sector`: anything AHEAD always counts (it is in front of the robot whatever
// the exact heading), plus the hazard of the sector the robot is moving into.
std::optional<double> governingHazard(const LocalPerceptionState& p, Sector motion) {
    std::optional<double> hazard;
    for (Sector s : {Sector::AHEAD, motion}) {
        const FusedSector& f = p.sectors[index(s)];
        if (f.state == SectorState::OCCUPIED) {
            double h = f.hazard_m.value_or(0.0);
            if (!hazard || h < *hazard) {
                hazard = h;
            }
        }
    }
    return hazard;
}

}  // namespace

LocalSafetyController::LocalSafetyController(SafetyConfig config, double cruise_speed_mps, MotionConfig motion,
                                             LogSink log)
    : config_(config), cruise_speed_mps_(cruise_speed_mps), motion_(motion), log_(std::move(log)) {}

double LocalSafetyController::releaseThreshold() const {
    return stoppingDistance(config_.stopping, cruise_speed_mps_) + config_.release_extra_m;
}

void LocalSafetyController::latch(const std::string& reason, bool permanent) {
    if (permanent && !permanent_) {
        permanent_ = true;
        if (latched_) {
            log_("[local-safety] LOCAL_STOP is now permanent: " + reason);
        }
    }
    if (!latched_) {
        latched_ = true;
        latch_reason_ = reason;
        log_("[local-safety] LOCAL_STOP: " + reason);
    }
    release_count_ = 0;
}

void LocalSafetyController::tryRelease(const SafetyInputs& in, const std::optional<double>& hazard) {
    const LocalPerceptionState& p = *in.perception;
    double threshold = releaseThreshold();
    const FusedSector& ahead = p.sectors[index(Sector::AHEAD)];
    bool valid = p.required_valid && !p.emergency;
    bool coverage = ahead.free_m && *ahead.free_m > threshold;   // verified free space, not just "no hazard seen"
    bool hazardBeyond = !hazard || *hazard > threshold;
    if (!(valid && coverage && hazardBeyond)) {
        release_count_ = 0;
        return;
    }
    if (release_count_ == 0) {
        release_since_ = in.now;
    }
    ++release_count_;
    double held = secondsBetween(release_since_, in.now);
    if (release_count_ >= config_.release_clear_observations && held >= config_.release_hold_s - 1e-9) {
        latched_ = false;
        log_("[local-safety] local stop released: " + std::to_string(release_count_) + " consecutive clear observations over "
             + fmt(held, 1) + " s, verified free beyond " + fmt(threshold) + " m");
        release_count_ = 0;
    }
}

SafetyOutput LocalSafetyController::review(const SafetyInputs& in) {
    const LocalPerceptionState& p = *in.perception;
    const NavDecision& nav = *in.nav;
    SafetyOutput out;
    out.heading_rad = nav.heading_rad;
    out.d_stop_m = stoppingDistance(config_.stopping, in.measured_speed_mps);
    std::optional<double> hazard = governingHazard(p, nav.motion_sector);
    out.hazard_m = hazard;

    // (1) latch triggers
    bool triggered = false;
    if (in.stream_broken) {
        latch("Java stream ended (" + in.stream_broken_reason + ") - controlled stop, no automatic reconnect", true);
        triggered = true;
    }
    if (p.emergency) {
        latch("hardware emergency input", false);
        triggered = true;
    }
    if (!p.required_valid) {
        latch("required perception not valid (" + p.invalid_reason + ")", false);
        triggered = true;
    }
    if (hazard && *hazard <= out.d_stop_m) {
        latch("hazard " + fmt(*hazard) + " m <= d_stop " + fmt(out.d_stop_m) + " m at " + fmt(in.measured_speed_mps)
              + " m/s", false);
        triggered = true;
    }
    if (latched_ && !permanent_ && !triggered) {
        tryRelease(in, hazard);
    }
    if (latched_) {
        out.speed_mps = 0.0;
        out.status = SafetyStatus::LOCAL_STOP_LATCHED;
        out.local_stop_active = true;
        out.reason = latch_reason_;
        return out;
    }

    // (2) temporary local restrictions
    bool wantsToMove = in.has_target && nav.state != NavState::IDLE_NO_TARGET;
    const FusedSector& motionSector = p.sectors[index(nav.motion_sector)];
    // Verified free space must cover AHEAD too, like its hazard: at avoidance offsets almost all of the swept path
    // still lies in front of the robot, so an UNKNOWN AHEAD is never driven through as if it were clear.
    double coverage = std::min(motionSector.free_m.value_or(0.0),
                               p.sectors[index(Sector::AHEAD)].free_m.value_or(0.0));
    double limitDistance = hazard ? std::min(coverage, *hazard) : coverage;
    double vMax = maxSpeedForDistance(config_.stopping, limitDistance);
    if (wantsToMove && nav.state == NavState::BLOCKED) {
        out.speed_mps = 0.0;
        out.status = SafetyStatus::TEMPORARY_BLOCK;
        out.local_stop_active = true;
        out.reason = nav.reason;
        return out;
    }
    if (wantsToMove && nav.speed_mps > 0.0 && vMax <= 0.0) {
        out.speed_mps = 0.0;
        out.status = SafetyStatus::TEMPORARY_BLOCK;
        out.local_stop_active = true;
        out.reason = "verified free distance " + fmt(limitDistance) + " m along "
                     + (nav.motion_sector == Sector::AHEAD ? std::string("AHEAD")
                                                           : std::string("AHEAD and ") + toString(nav.motion_sector))
                     + " leaves no safe speed";
        return out;
    }

    // (3) Java STOP
    if (in.java_stop) {
        out.speed_mps = 0.0;
        out.status = SafetyStatus::JAVA_STOP;
        out.reason = "Java STOP";
        return out;
    }

    // (4)/(5) the navigation proposal, limited by the verified distance and the acceleration limit
    double v = nav.speed_mps;
    out.status = SafetyStatus::NORMAL;
    if (v > vMax) {
        v = vMax;
        out.status = SafetyStatus::SPEED_LIMITED;
        out.reason = "limited to " + fmt(vMax) + " m/s by " + fmt(limitDistance) + " m";
    }
    if (motion_.max_accel_mps2 > 0.0) {
        v = std::min(v, in.measured_speed_mps + motion_.max_accel_mps2 * in.dt_s);
    }
    out.speed_mps = std::max(0.0, v);
    return out;
}

}  // namespace robot
