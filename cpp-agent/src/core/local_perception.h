#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "perception_types.h"
#include "robot_config.h"
#include "sensor_provider.h"

namespace robot {

enum class SourceRole {
    REQUIRED_FOR_SAFETY,   // if it is not VALID, local safety stops the robot
    SUPPLEMENTARY          // may add hazards or verify free space while VALID; its loss alone never stops the robot
};

struct PerceptionSource {
    ISensorProvider* provider = nullptr;
    SourceRole role = SourceRole::SUPPLEMENTARY;
};

struct SourceStatus {
    std::string id;
    SourceRole role = SourceRole::SUPPLEMENTARY;
    Validity validity = Validity::UNAVAILABLE;
    double age_s = 0.0;
    std::string detail;
};

struct FusedSector {
    SectorState state = SectorState::UNKNOWN;
    std::optional<double> hazard_m;   // conservative (range - uncertainty), minimum over valid sources
    std::optional<double> free_m;     // conservative verified-free distance, never beyond hazard_m
};

struct LocalPerceptionState {
    std::array<FusedSector, kSectorCount> sectors{};
    std::optional<double> nearest_m;  // nearest valid distance (hazard, or verified-free range); absent if none
    bool required_valid = false;      // every required source VALID (and there is at least one)
    bool emergency = false;           // any source reports a hardware emergency input
    std::string invalid_reason;
    std::vector<SourceStatus> sources;
};

// Conservative fusion of all provider snapshots - no grid, no probabilities:
//   - a sector is OCCUPIED if ANY valid source reports a hazard there (whatever its confidence);
//   - it is FREE_VERIFIED only if a valid source with confidence >= min_confidence_for_free verifies it free AND no
//     valid source reports it occupied; otherwise UNKNOWN;
//   - ranges are conservative: range - uncertainty, the MINIMUM across valid sources, never an average;
//   - a missing measurement is not a clear path: no valid range means no value and an UNKNOWN sector.
// Invariant (tested): a credible direct-sensor hazard is never overridden by another source reporting clear.
class LocalPerception {
public:
    LocalPerception(PerceptionConfig config, std::vector<PerceptionSource> sources);

    LocalPerceptionState update(SteadyTime now);

private:
    PerceptionConfig config_;
    std::vector<PerceptionSource> sources_;
};

}  // namespace robot
