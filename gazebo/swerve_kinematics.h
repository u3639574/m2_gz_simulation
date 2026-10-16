#pragma once

#include <cmath>

namespace m2::simulation {
constexpr double pi = 3.14159265358979323846;

struct ModuleTarget {
    double angle;
    double speed;  // wheel perimeter speed, m/s
};

// Rigid-body inverse kinematics, as in the referenced swerve controller.
// Choose the equivalent wheel direction requiring at most 90 degrees of steering.
inline ModuleTarget module_target(double vx, double vy, double wz,
                                  double x, double y, double measured_angle) {
    const double wheel_x = vx - wz * y;
    const double wheel_y = vy + wz * x;
    double speed = std::hypot(wheel_x, wheel_y);
    if (speed < 1e-6) return {measured_angle, 0.0};
    double error = std::remainder(std::atan2(wheel_y, wheel_x) - measured_angle, 2*pi);
    if (std::abs(error) > pi/2) {
        error = std::remainder(error + pi, 2*pi);
        speed = -speed;
    }
    return {measured_angle + error, speed};
}
}  // namespace m2::simulation
