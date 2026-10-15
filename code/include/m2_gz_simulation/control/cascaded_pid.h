#pragma once
#include "m2_gz_simulation/control/position_pid.h"

namespace m2::gz_simulation {
// SI-unit position -> velocity -> effort cascade. The VESC foc_math.c supplied
// by the user informed the topology; no firmware code or ERPM scaling is copied.
class cascaded_pid {
public:
    cascaded_pid(position_pid outer, double kp, double ki, double kd,
                 double integral_limit, double effort_limit, double feedforward = 0.0)
        : outer_(outer), kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit),
          effort_limit_(effort_limit), feedforward_(feedforward) {}
    void reset() noexcept
    { outer_.reset(); integral_ = 0; previous_velocity_ = 0; derivative_ = 0; started_ = false; velocity_target_ = 0; }
    double compute(double target, double position, double velocity, double dt) noexcept
    {
        if (!std::isfinite(target) || !std::isfinite(position) || !std::isfinite(velocity) ||
            !std::isfinite(dt) || dt <= 0) { reset(); return 0; }
        velocity_target_ = outer_.compute(target, position, velocity, dt);
        const double error = velocity_target_ - velocity;
        // Derivative on measured velocity avoids a kick on target changes.
        const double raw = started_ ? (velocity - previous_velocity_) / dt : 0;
        derivative_ += dt / (0.02 + dt) * (raw - derivative_);
        previous_velocity_ = velocity; started_ = true;
        const double candidate = std::clamp(integral_ + ki_ * error * dt, -integral_limit_, integral_limit_);
        const double output = feedforward_ + kp_ * error + candidate - kd_ * derivative_;
        const double bounded = std::clamp(output, -effort_limit_, effort_limit_);
        if (output == bounded || (output > bounded && error < 0) || (output < bounded && error > 0)) { integral_ = candidate; }
        return bounded;
    }
    double velocity_target() const noexcept { return velocity_target_; }
private:
    position_pid outer_;
    double kp_, ki_, kd_, integral_limit_, effort_limit_, feedforward_;
    double integral_{0}, previous_velocity_{0}, derivative_{0}, velocity_target_{0};
    bool started_{false};
};
}  // namespace m2::gz_simulation
