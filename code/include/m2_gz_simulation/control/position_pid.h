#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace m2::gz_simulation {

// Outer position loop. Derivative on measured velocity avoids setpoint kick.
class position_pid {
public:
    position_pid(double kp, double ki, double kd, double max_velocity,
        double integral_limit, double max_acceleration)
        : kp_(kp), ki_(ki), kd_(kd), limit_(max_velocity),
          integral_limit_(integral_limit), acceleration_(max_acceleration)
    {
        for (double value : {kp, ki, kd, max_velocity, integral_limit, max_acceleration}) {
            if (!std::isfinite(value) || value < 0.0) {
                throw std::invalid_argument("PID parameters must be finite and nonnegative");
            }
        }
        if (max_velocity == 0.0 || max_acceleration == 0.0) {
            throw std::invalid_argument("PID velocity and acceleration limits must be positive");
        }
    }

    double compute(double target, double position, double velocity, double dt) noexcept
    {
        if (!std::isfinite(target) || !std::isfinite(position) ||
            !std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0.0) {
            return 0.0;
        }
        const double error = target - position;
        const double candidate = std::clamp(integral_ + error * dt,
            -integral_limit_, integral_limit_);
        const double raw = kp_ * error + ki_ * candidate - kd_ * velocity;
        const double lower = std::max(-limit_, output_ - acceleration_ * dt);
        const double upper = std::min(limit_, output_ + acceleration_ * dt);
        const double limited = std::clamp(raw, lower, upper);
        // Integrate only when unsaturated, or when error unwinds saturation.
        if ((raw - limited) * error <= 0.0) {
            integral_ = candidate;
        }
        output_ = limited;
        return output_;
    }

    void reset() noexcept { integral_ = 0.0; output_ = 0.0; }

private:
    double kp_, ki_, kd_, limit_, integral_limit_, acceleration_;
    double integral_{0.0};
    double output_{0.0};
};
}  // namespace m2::gz_simulation
