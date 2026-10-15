#include "python_perception_adapter.h"

#include <cmath>
#include <utility>

#include "geometry.h"

namespace robot {

namespace {

bool finite(double v) { return std::isfinite(v); }

void markOccupied(SectorReading& s, double range, double uncertainty, double confidence) {
    // Keep the reading whose conservative range (range - uncertainty) is the smallest.
    bool closer = s.state != SectorState::OCCUPIED || !s.hazard_range_m
                  || range - uncertainty < *s.hazard_range_m - s.range_uncertainty_m.value_or(0.0);
    s.state = SectorState::OCCUPIED;
    if (closer) {
        s.hazard_range_m = range;
        s.range_uncertainty_m = uncertainty;
        s.confidence = confidence;
    }
    s.free_range_m.reset();   // a detection list never verifies the space in front of the obstacle
}

}  // namespace

PythonPerceptionAdapter::PythonPerceptionAdapter(PythonAdapterConfig config, double corridor_half_width_m,
                                                 const IClock& clock)
    : config_(std::move(config)), corridor_half_width_m_(corridor_half_width_m), clock_(clock) {}

std::optional<RejectReason> PythonPerceptionAdapter::validateLocked(const PerceptionMessage& m) const {
    if (m.schema_version != config_.schema_version) return RejectReason::UNKNOWN_SCHEMA_VERSION;
    if (m.robot_id != config_.expected_robot_id) return RejectReason::WRONG_ROBOT_ID;
    if (m.frame != "ROBOT") return RejectReason::UNSUPPORTED_FRAME;

    if (!finite(m.sender_age_at_send_ms)) return RejectReason::NON_FINITE_VALUE;
    if (m.sender_age_at_send_ms < 0.0) return RejectReason::NEGATIVE_VALUE;
    for (const PerceivedObstacle& o : m.obstacles) {
        if (!finite(o.bearing_rad) || !finite(o.range_m) || !finite(o.range_uncertainty_m) || !finite(o.confidence)
                || (o.width_m && !finite(*o.width_m))) {
            return RejectReason::NON_FINITE_VALUE;
        }
        if (o.range_m < 0.0 || o.range_uncertainty_m < 0.0 || (o.width_m && *o.width_m < 0.0)) {
            return RejectReason::NEGATIVE_VALUE;
        }
        if (o.confidence < 0.0 || o.confidence > 1.0) return RejectReason::CONFIDENCE_OUT_OF_RANGE;
    }
    if (m.sector_summary) {
        for (const SectorSummaryEntry& e : *m.sector_summary) {
            if (!finite(e.range_uncertainty_m) || !finite(e.confidence) || (e.range_m && !finite(*e.range_m))) {
                return RejectReason::NON_FINITE_VALUE;
            }
            if (e.range_uncertainty_m < 0.0 || (e.range_m && *e.range_m < 0.0)) return RejectReason::NEGATIVE_VALUE;
            if (e.confidence < 0.0 || e.confidence > 1.0) return RejectReason::CONFIDENCE_OUT_OF_RANGE;
        }
    }

    // A new session (a restarted sender) resets sequence tracking, so it is never locked out.
    if (m.session_id == session_id_ && last_sequence_ && m.sequence <= *last_sequence_) {
        return RejectReason::OUT_OF_ORDER;
    }
    return std::nullopt;
}

std::optional<RejectReason> PythonPerceptionAdapter::offer(const PerceptionMessage& message) {
    SteadyTime receivedAt = clock_.now();
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.offered;
    std::optional<RejectReason> reject = validateLocked(message);
    if (reject) {
        ++stats_.rejected[static_cast<int>(*reject)];
        return reject;
    }
    if (slot_ && !slot_->consumed) {
        ++stats_.replaced_unconsumed;
    }
    session_id_ = message.session_id;
    last_sequence_ = message.sequence;
    slot_ = Slot{message, receivedAt, false};
    ++stats_.accepted;
    return std::nullopt;
}

SensorObservation PythonPerceptionAdapter::snapshot(SteadyTime now) {
    SensorObservation obs;
    obs.source_id = config_.source_id;
    obs.frame = Frame::ROBOT;

    std::optional<Slot> slot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.snapshots;
        if (slot_) {
            slot_->consumed = true;
            slot = slot_;
        }
    }
    if (!slot) {
        obs.validity = Validity::UNAVAILABLE;
        obs.detail = "no message received";
        return obs;
    }

    double senderAge = slot->message.sender_age_at_send_ms / 1000.0;
    double age = secondsBetween(slot->received_at, now) + senderAge;   // never sender clock vs local clock
    obs.acquired_at = plusSeconds(slot->received_at, -senderAge);
    if (age <= config_.stale_after_s) {
        obs.validity = Validity::VALID;
    } else if (age <= config_.unavailable_after_s) {
        obs.validity = Validity::STALE;
        obs.detail = "age " + std::to_string(age) + " s";
    } else {
        obs.validity = Validity::UNAVAILABLE;
        obs.detail = "silent for " + std::to_string(age) + " s";
    }

    for (const PerceivedObstacle& o : slot->message.obstacles) {
        if (std::fabs(o.bearing_rad) > kPi / 2.0) {
            continue;   // behind the robot: outside every sector
        }
        double lateral = o.range_m * std::sin(o.bearing_rad);
        Sector sector = std::fabs(lateral) <= corridor_half_width_m_ ? Sector::AHEAD
                        : (o.bearing_rad > 0.0 ? Sector::LEFT : Sector::RIGHT);
        markOccupied(obs.sectors[index(sector)], o.range_m, o.range_uncertainty_m, o.confidence);
    }
    if (slot->message.sector_summary) {
        for (int i = 0; i < kSectorCount; ++i) {
            const SectorSummaryEntry& e = (*slot->message.sector_summary)[i];
            SectorReading& s = obs.sectors[i];
            if (e.state == SectorState::OCCUPIED) {
                markOccupied(s, e.range_m.value_or(0.0), e.range_uncertainty_m, e.confidence);
            } else if (e.state == SectorState::FREE_VERIFIED && e.range_m && s.state != SectorState::OCCUPIED) {
                // Detections win over a free summary; a free claim without a range verifies nothing.
                s.state = SectorState::FREE_VERIFIED;
                s.free_range_m = *e.range_m;
                s.range_uncertainty_m = e.range_uncertainty_m;
                s.confidence = e.confidence;
            }
        }
    }
    return obs;
}

AdapterStats PythonPerceptionAdapter::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

}  // namespace robot
