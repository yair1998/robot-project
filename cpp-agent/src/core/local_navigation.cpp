#include "local_navigation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "stopping_model.h"

namespace robot {

LocalNavigationController::LocalNavigationController(NavConfig config, double ahead_half_angle_deg,
                                                     StoppingModel stopping, LogSink log)
    : config_(std::move(config)),
      ahead_half_angle_rad_(degToRad(ahead_half_angle_deg)),
      stopping_(stopping),
      log_(std::move(log)) {
    std::vector<double> magnitudes = config_.candidate_offsets_deg;
    std::sort(magnitudes.begin(), magnitudes.end());
    double preferred = config_.preferred_side == Sector::RIGHT ? -1.0 : 1.0;
    for (double m : magnitudes) {
        offsets_rad_.push_back(preferred * degToRad(std::fabs(m)));
        offsets_rad_.push_back(-preferred * degToRad(std::fabs(m)));
    }
}

void LocalNavigationController::onNewLeg() {
    avoidance_offset_rad_.reset();
    clear_count_ = 0;
    avoidance_used_up_ = false;
    if (state_ == NavState::LOCAL_AVOIDANCE || state_ == NavState::SLOWING_DOWN || state_ == NavState::BLOCKED) {
        setState(NavState::TRACKING_WAYPOINT, "new waypoint - avoidance state reset");
    }
}

void LocalNavigationController::setState(NavState next, const std::string& reason) {
    if (next != state_) {
        log_(std::string("[local-nav] ") + toString(state_) + " -> " + toString(next) + ": " + reason);
        state_ = next;
    }
    last_reason_ = reason;
}

NavDecision LocalNavigationController::blocked(NavDecision d, const std::string& reason) {
    avoidance_offset_rad_.reset();
    clear_count_ = 0;
    setState(NavState::BLOCKED, reason);
    d.state = state_;
    d.speed_mps = 0.0;
    d.offset_rad = 0.0;
    d.motion_sector = Sector::AHEAD;
    d.reason = reason;
    return d;
}

NavDecision LocalNavigationController::avoid(NavDecision d, const NavInputs& in, double direct, double offset,
                                             double speed) {
    Sector sector = sectorOfOffset(offset, ahead_half_angle_rad_);
    const FusedSector& s = in.perception->sectors[index(sector)];
    std::string reason = std::string("AHEAD blocked; heading ") + (offset >= 0.0 ? "+" : "") + fmt(radToDeg(offset), 0)
                         + " deg into " + toString(sector) + " (verified free to " + fmt(s.free_m.value_or(0.0)) + " m)";
    if (state_ != NavState::LOCAL_AVOIDANCE || !avoidance_offset_rad_ || *avoidance_offset_rad_ != offset) {
        if (state_ == NavState::LOCAL_AVOIDANCE) {
            log_("[local-nav] LOCAL_AVOIDANCE: " + reason);
        }
        setState(NavState::LOCAL_AVOIDANCE, reason);
    }
    avoidance_offset_rad_ = offset;
    d.state = state_;
    d.heading_rad = direct + offset;
    d.offset_rad = offset;
    d.motion_sector = sector;
    d.speed_mps = speed;
    d.reason = reason;
    return d;
}

std::optional<double> LocalNavigationController::pickCandidate(const NavInputs& in, double direct,
                                                               double speed) const {
    double requiredClearance = config_.robot_half_width_m + stoppingDistance(stopping_, speed) + config_.clearance_margin_m;
    for (double offset : offsets_rad_) {
        Sector sector = sectorOfOffset(offset, ahead_half_angle_rad_);
        if (sector == Sector::AHEAD) {
            continue;   // the blocked direction itself
        }
        const FusedSector& s = in.perception->sectors[index(sector)];
        if (s.state != SectorState::FREE_VERIFIED || !s.free_m || *s.free_m < requiredClearance) {
            continue;
        }
        Point next = advance(in.position, direct + offset, speed * in.dt_s);
        if (std::fabs(signedLateralOffset(in.leg_start, in.waypoint, next)) > config_.max_lateral_deviation_m) {
            continue;
        }
        return offset;
    }
    return std::nullopt;
}

NavDecision LocalNavigationController::decide(const NavInputs& in) {
    NavDecision d;
    if (!in.has_target) {
        avoidance_offset_rad_.reset();
        clear_count_ = 0;
        avoidance_used_up_ = false;
        setState(NavState::IDLE_NO_TARGET, "no target");
        d.state = state_;
        d.reason = "no target";
        return d;
    }

    const LocalPerceptionState& p = *in.perception;
    double direct = bearing(in.position, in.waypoint);
    d.heading_rad = direct;
    if (!p.required_valid) {
        return blocked(d, "required perception not valid (" + p.invalid_reason + ")");
    }

    double speed = std::max(0.0, in.requested_speed_mps);
    const FusedSector& ahead = p.sectors[index(Sector::AHEAD)];
    double aheadHazard = ahead.state == SectorState::OCCUPIED ? ahead.hazard_m.value_or(0.0)
                                                              : std::numeric_limits<double>::infinity();
    bool aheadBlocked = ahead.state == SectorState::OCCUPIED && aheadHazard <= config_.avoidance_trigger_range_m;
    bool aheadVerifiedFree = ahead.state == SectorState::FREE_VERIFIED && ahead.free_m
                             && *ahead.free_m >= stoppingDistance(stopping_, speed);

    if (state_ == NavState::LOCAL_AVOIDANCE) {
        clear_count_ = aheadVerifiedFree ? clear_count_ + 1 : 0;
        if (clear_count_ < config_.reconverge_clear_observations) {
            if (secondsBetween(avoidance_started_at_, in.now) > config_.max_avoidance_duration_s) {
                avoidance_used_up_ = true;
                return blocked(d, "avoidance exceeded " + fmt(config_.max_avoidance_duration_s, 1) + " s");
            }
            std::optional<double> offset = pickCandidate(in, direct, speed);
            if (!offset) {
                return blocked(d, "no verified-free heading within " + fmt(config_.max_lateral_deviation_m)
                                  + " m of the leg line");
            }
            return avoid(d, in, direct, *offset, speed);
        }
        avoidance_offset_rad_.reset();
        clear_count_ = 0;
        setState(NavState::TRACKING_WAYPOINT, "AHEAD verified free for "
                 + std::to_string(config_.reconverge_clear_observations) + " observations - reconverging");
    }

    if (aheadBlocked) {
        if (avoidance_used_up_) {
            return blocked(d, "avoidance time limit used up on this leg - waiting for AHEAD to clear or a new waypoint");
        }
        std::optional<double> offset = pickCandidate(in, direct, speed);
        if (!offset) {
            return blocked(d, "AHEAD occupied at " + fmt(aheadHazard) + " m and no verified-free heading");
        }
        avoidance_started_at_ = in.now;
        clear_count_ = 0;
        return avoid(d, in, direct, *offset, speed);
    }

    if (ahead.state == SectorState::UNKNOWN || !ahead.free_m) {
        return blocked(d, std::string("forward space not verified (AHEAD ") + toString(ahead.state) + ")");
    }

    NavState next = NavState::TRACKING_WAYPOINT;
    std::string reason = "AHEAD verified free to " + fmt(*ahead.free_m) + " m";
    double v = speed;
    if (ahead.state == SectorState::OCCUPIED && aheadHazard < config_.slowing_range_m) {
        double span = config_.slowing_range_m - config_.avoidance_trigger_range_m;
        double fraction = span > 0.0 ? (aheadHazard - config_.avoidance_trigger_range_m) / span : 1.0;
        fraction = std::clamp(fraction, config_.min_slowing_fraction, 1.0);
        v = speed * fraction;
        next = NavState::SLOWING_DOWN;
        reason = "AHEAD hazard at " + fmt(aheadHazard) + " m";
    }
    double coverageLimit = maxSpeedForDistance(stopping_, *ahead.free_m);
    if (v > coverageLimit) {
        v = coverageLimit;
        next = NavState::SLOWING_DOWN;
        reason = "verified free distance " + fmt(*ahead.free_m) + " m limits speed to " + fmt(coverageLimit) + " m/s";
    }
    if (v <= 0.0 && speed > 0.0) {
        return blocked(d, "verified free distance " + fmt(*ahead.free_m) + " m is inside the stopping margin");
    }
    avoidance_used_up_ = false;   // normal progress on the direct path: a later blockage gets its own window
    setState(next, reason);
    d.state = state_;
    d.speed_mps = v;
    d.reason = reason;
    return d;
}

}  // namespace robot
