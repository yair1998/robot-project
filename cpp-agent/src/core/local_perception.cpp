#include "local_perception.h"

#include <algorithm>
#include <utility>

namespace robot {

namespace {

void takeMin(std::optional<double>& target, double value) {
    if (!target || value < *target) {
        target = value;
    }
}

}  // namespace

LocalPerception::LocalPerception(PerceptionConfig config, std::vector<PerceptionSource> sources)
    : config_(config), sources_(std::move(sources)) {}

LocalPerceptionState LocalPerception::update(SteadyTime now) {
    LocalPerceptionState state;
    std::vector<SensorObservation> valid;
    bool anyRequired = false;
    bool requiredOk = true;

    for (const PerceptionSource& source : sources_) {
        SensorObservation obs = source.provider->snapshot(now);
        SourceStatus status;
        status.id = source.provider->id();
        status.role = source.role;
        status.validity = obs.validity;
        status.age_s = obs.validity == Validity::UNAVAILABLE ? 0.0 : secondsBetween(obs.acquired_at, now);
        status.detail = obs.detail;

        if (obs.frame != Frame::ROBOT) {
            status.validity = Validity::INVALID;
            status.detail = "unsupported frame";
        } else if (status.validity == Validity::VALID && source.role == SourceRole::REQUIRED_FOR_SAFETY
                   && status.age_s > config_.max_required_age_s) {
            status.validity = Validity::STALE;
            status.detail = "older than " + std::to_string(config_.max_required_age_s) + " s";
        }
        if (obs.emergency_input) {
            state.emergency = true;   // a hardware safety condition counts even from a degraded source
        }
        if (source.role == SourceRole::REQUIRED_FOR_SAFETY) {
            anyRequired = true;
            if (status.validity != Validity::VALID) {
                requiredOk = false;
                if (state.invalid_reason.empty()) {
                    state.invalid_reason = status.id + " " + toString(status.validity)
                                           + (status.detail.empty() ? "" : " (" + status.detail + ")");
                }
            }
        }
        if (status.validity == Validity::VALID) {
            valid.push_back(obs);
        }
        state.sources.push_back(status);
    }
    state.required_valid = anyRequired && requiredOk;
    if (!anyRequired) {
        state.invalid_reason = "no required source configured";
    }

    for (int i = 0; i < kSectorCount; ++i) {
        FusedSector& fused = state.sectors[i];
        bool occupied = false;
        bool confidentFree = false;
        std::optional<double> freeEvidence;   // conservative free distance any confident valid source verified

        for (const SensorObservation& obs : valid) {
            const SectorReading& r = obs.sectors[i];
            double uncertainty = r.range_uncertainty_m.value_or(0.0);
            bool confident = r.confidence && *r.confidence >= config_.min_confidence_for_free;
            if (r.state == SectorState::OCCUPIED) {
                occupied = true;
                // A hazard without a range is treated as immediate.
                takeMin(fused.hazard_m, std::max(0.0, r.hazard_range_m.value_or(0.0) - uncertainty));
            } else if (r.state == SectorState::FREE_VERIFIED && confident && r.free_range_m) {
                confidentFree = true;
            }
            if (confident && r.free_range_m && r.state != SectorState::UNKNOWN) {
                takeMin(freeEvidence, std::max(0.0, *r.free_range_m - uncertainty));
            }
        }

        if (occupied) {
            fused.state = SectorState::OCCUPIED;
            // Free space is verified only as far as a confident source saw it, and never beyond the hazard.
            if (freeEvidence) {
                fused.free_m = std::min(*freeEvidence, *fused.hazard_m);
            }
        } else if (confidentFree) {
            fused.state = SectorState::FREE_VERIFIED;
            fused.free_m = freeEvidence;
        }

        if (fused.hazard_m) {
            takeMin(state.nearest_m, *fused.hazard_m);
        } else if (fused.state == SectorState::FREE_VERIFIED && fused.free_m) {
            takeMin(state.nearest_m, *fused.free_m);
        }
    }
    return state;
}

}  // namespace robot
