#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "perception_message.h"
#include "robot_config.h"
#include "sensor_provider.h"

namespace robot {

struct AdapterStats {
    uint64_t offered = 0;
    uint64_t accepted = 0;
    uint64_t replaced_unconsumed = 0;   // accepted messages overwritten before any snapshot used them (burst)
    uint64_t snapshots = 0;
    std::array<uint64_t, static_cast<int>(RejectReason::COUNT)> rejected{};

    uint64_t rejectedTotal() const {
        uint64_t n = 0;
        for (uint64_t r : rejected) n += r;
        return n;
    }
};

// ISensorProvider fed by an external perception process. ONE slot, latest wins: a burst can never build a backlog of
// old frames, and offer() never waits for the consumer. snapshot() reports VALID / STALE / UNAVAILABLE from the
// observation's age, where age = (now - local receive time) + sender_age_at_send. Implemented and tested in this
// task; only the transport that would call offer() (socket / gRPC receiver) is not part of it.
class PythonPerceptionAdapter final : public ISensorProvider {
public:
    // corridor_half_width_m: an obstacle whose lateral offset is within this is in the robot's forward swept
    // corridor and therefore AHEAD, whatever its angle (robot half width + clearance margin).
    PythonPerceptionAdapter(PythonAdapterConfig config, double corridor_half_width_m, const IClock& clock);

    const std::string& id() const override { return config_.source_id; }

    // Validates and, if valid, replaces the slot. Returns the reject reason, or nullopt when accepted.
    std::optional<RejectReason> offer(const PerceptionMessage& message);

    SensorObservation snapshot(SteadyTime now) override;

    AdapterStats stats() const;

private:
    std::optional<RejectReason> validateLocked(const PerceptionMessage& message) const;

    PythonAdapterConfig config_;
    double corridor_half_width_m_;
    const IClock& clock_;

    mutable std::mutex mutex_;   // guards everything below; held only for a validation and one message copy
    struct Slot {
        PerceptionMessage message;
        SteadyTime received_at{};
        bool consumed = false;
    };
    std::optional<Slot> slot_;
    std::string session_id_;
    std::optional<uint64_t> last_sequence_;
    AdapterStats stats_;
};

}  // namespace robot
