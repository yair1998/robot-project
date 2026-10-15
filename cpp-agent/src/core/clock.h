#pragma once

#include <chrono>
#include <mutex>

namespace robot {

using SteadyTime = std::chrono::steady_clock::time_point;
using Seconds = std::chrono::duration<double>;

inline double secondsBetween(SteadyTime from, SteadyTime to) {
    return Seconds(to - from).count();
}

inline SteadyTime plusSeconds(SteadyTime t, double seconds) {
    return t + std::chrono::duration_cast<std::chrono::steady_clock::duration>(Seconds(seconds));
}

// Monotonic time source. Every age and timeout in the agent goes through this interface, so tests can drive time
// explicitly and nothing ever compares a timestamp from another process with the local clock.
class IClock {
public:
    virtual ~IClock() = default;
    virtual SteadyTime now() const = 0;
};

class SteadyClock final : public IClock {
public:
    SteadyTime now() const override { return std::chrono::steady_clock::now(); }
};

// Test clock: time moves only when advance() is called. Thread-safe, so the stress test can read it from any thread.
class ManualClock final : public IClock {
public:
    SteadyTime now() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return now_;
    }

    void advance(double seconds) {
        std::lock_guard<std::mutex> lock(mutex_);
        now_ = plusSeconds(now_, seconds);
    }

private:
    mutable std::mutex mutex_;
    SteadyTime now_{};   // only differences are ever used
};

}  // namespace robot
