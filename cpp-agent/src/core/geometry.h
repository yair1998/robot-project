#pragma once

#include <cmath>

namespace robot {

constexpr double kPi = 3.14159265358979323846;

struct Point {
    double x = 0.0;
    double y = 0.0;
};

inline double distance(Point a, Point b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

inline double bearing(Point from, Point to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

inline double degToRad(double degrees) { return degrees * kPi / 180.0; }
inline double radToDeg(double radians) { return radians * 180.0 / kPi; }

inline Point advance(Point from, double heading, double length) {
    return {from.x + length * std::cos(heading), from.y + length * std::sin(heading)};
}

// Signed distance of p from the infinite line through start -> end; positive = left of that direction.
// A degenerate line (start == end) has no direction, so every point counts as on it.
inline double signedLateralOffset(Point start, Point end, Point p) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double length = std::hypot(dx, dy);
    if (length < 1e-9) {
        return 0.0;
    }
    return (dx * (p.y - start.y) - dy * (p.x - start.x)) / length;
}

}  // namespace robot
