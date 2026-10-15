#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "perception_types.h"

namespace robot {

// Logical contract for local perception produced by a separate (Python) process. Transport-agnostic: whatever
// receiver is added later (see the transport comparison in the Part B report) decodes into this struct and calls
// PythonPerceptionAdapter::offer(). No images - only already-extracted, robot-relative facts.
//
// Units and conventions:
//   - distances in meters, angles in radians, ages in milliseconds;
//   - frame must be "ROBOT": bearings are relative to the robot's reference heading, positive to the LEFT;
//   - sender_age_at_send_ms is how old the underlying measurement was when the sender sent the message, measured
//     on the SENDER's own monotonic clock. The receiver never compares sender wall-clock time with its own clock.
//   - session_id is random per sender start; sequence increases within a session.
struct PerceivedObstacle {
    double bearing_rad = 0.0;
    double range_m = 0.0;
    double range_uncertainty_m = 0.0;
    std::optional<double> width_m;
    double confidence = 0.0;   // 0..1
};

struct SectorSummaryEntry {
    SectorState state = SectorState::UNKNOWN;
    std::optional<double> range_m;   // FREE_VERIFIED: verified free up to here; OCCUPIED: the nearest hazard
    double range_uncertainty_m = 0.0;
    double confidence = 0.0;         // 0..1
};

struct PerceptionMessage {
    uint32_t schema_version = 0;
    std::string robot_id;
    std::string session_id;
    uint64_t sequence = 0;
    double sender_age_at_send_ms = 0.0;
    std::string source_id;
    std::string frame;
    std::vector<PerceivedObstacle> obstacles;
    std::optional<std::array<SectorSummaryEntry, kSectorCount>> sector_summary;   // indexed by Sector
};

enum class RejectReason {
    UNKNOWN_SCHEMA_VERSION,
    WRONG_ROBOT_ID,
    UNSUPPORTED_FRAME,
    NON_FINITE_VALUE,
    NEGATIVE_VALUE,           // a negative range, uncertainty, width or age
    CONFIDENCE_OUT_OF_RANGE,
    OUT_OF_ORDER,             // same session and sequence <= the last accepted one
    COUNT
};

inline const char* toString(RejectReason r) {
    switch (r) {
        case RejectReason::UNKNOWN_SCHEMA_VERSION: return "UNKNOWN_SCHEMA_VERSION";
        case RejectReason::WRONG_ROBOT_ID: return "WRONG_ROBOT_ID";
        case RejectReason::UNSUPPORTED_FRAME: return "UNSUPPORTED_FRAME";
        case RejectReason::NON_FINITE_VALUE: return "NON_FINITE_VALUE";
        case RejectReason::NEGATIVE_VALUE: return "NEGATIVE_VALUE";
        case RejectReason::CONFIDENCE_OUT_OF_RANGE: return "CONFIDENCE_OUT_OF_RANGE";
        case RejectReason::OUT_OF_ORDER: return "OUT_OF_ORDER";
        case RejectReason::COUNT: break;
    }
    return "?";
}

}  // namespace robot
