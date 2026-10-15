#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "geometry.h"
#include "local_navigation.h"
#include "local_safety.h"

namespace robot {

// What one control tick tells Java (and the tests). Maps onto the EXISTING Telemetry fields only.
struct TelemetrySnapshot {
    uint64_t seq = 0;                         // control tick number
    Point position;
    double measured_speed_mps = 0.0;          // actual displacement / dt - never the commanded speed
    std::optional<double> nearest_obstacle_m; // fused conservative nearest valid distance; absent when none
    bool emergency_stop = false;              // any local safety stop active (latched or temporary block)
    double battery = 90.0;
    NavState nav_state = NavState::IDLE_NO_TARGET;
    SafetyStatus safety_status = SafetyStatus::NORMAL;
};

// Hand-over from the control thread (publish) to the telemetry writer thread (waitNewer), latest wins: if a Write()
// blocks under gRPC flow control, the writer simply skips to the newest snapshot - the control thread never waits.
class TelemetryChannel {
public:
    void publish(const TelemetrySnapshot& snapshot) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = snapshot;
        }
        cv_.notify_all();
    }

    // Blocks until a snapshot newer than `after_seq` exists or close() was called (then nullopt).
    std::optional<TelemetrySnapshot> waitNewer(uint64_t after_seq) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return closed_ || (latest_ && latest_->seq > after_seq); });
        if (closed_) {
            return std::nullopt;
        }
        return latest_;
    }

    std::optional<TelemetrySnapshot> latest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        cv_.notify_all();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<TelemetrySnapshot> latest_;
    bool closed_ = false;
};

}  // namespace robot
