#include "command_mailbox.h"

#include <utility>

namespace robot {

bool CommandMailbox::post(const JavaCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++posted_;
    ++pending_.commands;
    bool ignoredSpeed = false;
    switch (command.type) {
        case JavaCommandType::STOP:
            pending_.java_stop = true;   // a STOP's speed is irrelevant
            break;
        case JavaCommandType::CONTINUE:
        case JavaCommandType::RESUME:
            pending_.java_stop = false;
            pending_.slow_down = false;
            pending_.slow_down_speed_cleared = true;   // same net effect as one command per tick
            pending_.slow_down_speed.reset();
            if (command.target_speed > 0.0) {
                pending_.requested_speed = command.target_speed;
            } else {
                ignoredSpeed = true;
            }
            break;
        case JavaCommandType::SLOW_DOWN:
            pending_.java_stop = false;
            pending_.slow_down = true;
            if (command.target_speed > 0.0) {
                pending_.slow_down_speed = command.target_speed;
            } else {
                ignoredSpeed = true;
            }
            break;
        case JavaCommandType::REROUTE:
            // A REROUTE leaves any SLOW_DOWN cap in place: the next CONTINUE lifts it.
            pending_.java_stop = false;
            if (command.waypoint) {
                pending_.new_waypoint = command.waypoint;
            }
            if (command.target_speed > 0.0) {
                pending_.requested_speed = command.target_speed;
            } else {
                ignoredSpeed = true;
            }
            break;
        case JavaCommandType::UNSPECIFIED:
            break;
    }
    if (ignoredSpeed) {
        ++pending_.ignored_nonpositive_speed;
    }
    return ignoredSpeed;
}

void CommandMailbox::markStreamBroken(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_broken_) {
        stream_broken_ = true;
        stream_broken_reason_ = reason;
    }
}

JavaDirectives CommandMailbox::take() {
    std::lock_guard<std::mutex> lock(mutex_);
    JavaDirectives out = std::move(pending_);
    pending_ = JavaDirectives{};
    out.stream_broken = stream_broken_;
    out.stream_broken_reason = stream_broken_reason_;
    return out;
}

uint64_t CommandMailbox::posted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return posted_;
}

}  // namespace robot
