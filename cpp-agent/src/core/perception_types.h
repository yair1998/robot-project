#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include "clock.h"

namespace robot {

// Sectors are relative to the robot's reference heading. In this simulated holonomic base the reference heading is
// the bearing to the active waypoint (the robot "faces" its waypoint and may translate sideways), so AHEAD is the
// direct path. Bearings are robot-relative, positive to the LEFT. Anything behind the robot (|bearing| > 90 deg) is
// outside all three sectors.
enum class Sector { AHEAD = 0, LEFT = 1, RIGHT = 2 };
constexpr int kSectorCount = 3;

inline int index(Sector s) { return static_cast<int>(s); }

enum class SectorState {
    UNKNOWN,         // nothing valid says anything about this sector - never treated as clear
    FREE_VERIFIED,   // a valid, confident source verified free space up to free_range_m
    OCCUPIED         // a valid source reports a hazard
};

enum class Validity { VALID, STALE, INVALID, UNAVAILABLE };

// Only robot-relative data is accepted for now; anything in another frame is rejected at the boundary.
enum class Frame { ROBOT };

struct SectorReading {
    SectorState state = SectorState::UNKNOWN;
    std::optional<double> hazard_range_m;       // nearest hazard in the sector (OCCUPIED)
    std::optional<double> free_range_m;         // free space verified up to this range (FREE_VERIFIED, or before a hazard)
    std::optional<double> range_uncertainty_m;  // applies to both ranges
    std::optional<double> confidence;           // 0..1; absent = not provided, which can never verify free space
};

// One provider's view at one moment. A provider fills in only what it can really measure.
struct SensorObservation {
    std::string source_id;
    Frame frame = Frame::ROBOT;
    SteadyTime acquired_at{};                   // local monotonic time of the measurement
    Validity validity = Validity::UNAVAILABLE;
    std::optional<double> uncertainty_m;
    std::optional<double> confidence;
    std::array<SectorReading, kSectorCount> sectors{};
    bool emergency_input = false;               // a hardware safety condition (e-stop, bumper contact)
    std::string detail;                         // why the observation is not VALID, for logs
};

// Which sector a heading offset (radians, relative to the reference heading) falls into.
inline Sector sectorOfOffset(double offset_rad, double ahead_half_angle_rad) {
    if (std::fabs(offset_rad) <= ahead_half_angle_rad) {
        return Sector::AHEAD;
    }
    return offset_rad > 0.0 ? Sector::LEFT : Sector::RIGHT;
}

inline const char* toString(Sector s) {
    switch (s) {
        case Sector::AHEAD: return "AHEAD";
        case Sector::LEFT: return "LEFT";
        case Sector::RIGHT: return "RIGHT";
    }
    return "?";
}

inline const char* toString(SectorState s) {
    switch (s) {
        case SectorState::UNKNOWN: return "UNKNOWN";
        case SectorState::FREE_VERIFIED: return "FREE_VERIFIED";
        case SectorState::OCCUPIED: return "OCCUPIED";
    }
    return "?";
}

inline const char* toString(Validity v) {
    switch (v) {
        case Validity::VALID: return "VALID";
        case Validity::STALE: return "STALE";
        case Validity::INVALID: return "INVALID";
        case Validity::UNAVAILABLE: return "UNAVAILABLE";
    }
    return "?";
}

}  // namespace robot
