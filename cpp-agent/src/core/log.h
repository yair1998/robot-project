#pragma once

#include <cstdio>
#include <functional>
#include <string>

namespace robot {

// Where the agent's log lines go. main() prints them to stdout under one mutex; tests capture or drop them.
using LogSink = std::function<void(const std::string&)>;

// Fixed-precision formatting for log lines (iostream defaults would print 0.30000000000000004).
inline std::string fmt(double value, int decimals = 2) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return buffer;
}

}  // namespace robot
