#ifndef M2_GZ_SIMULATION_CONTROL_POSITION_PID_H
#define M2_GZ_SIMULATION_CONTROL_POSITION_PID_H

#include <algorithm>
#include <cmath>

namespace m2::gz_simulation {

class position_pid {
public:
    position_pid(
        double kp,
        double ki,
        double kd,
        double integral_limit,
        double max_velocity,
        double max_acceleration) noexcept
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , integral_limit_(integral_limit)
        , max_velocity_(max_velocity)
        , max_acceleration_(max_acceleration)
    {
    }

    void reset() noexcept
    {
        integral_ = 0.0;
        previous_output_ = 0.0;
    }

    double compute(
        double target_position,
        double measured_position,
        double measured_velocity,
        double dt) noexcept
    {
        if (!std::isfinite(target_position) ||
            !std::isfinite(measured_position) ||
            !std::isfinite(measured_velocity) ||
            !std::isfinite(dt) ||
            dt <= 0.0) {
            return 0.0;
        }

        const double error =
            target_position - measured_position;

        const double candidate_integral =
            std::clamp(
                integral_ + error * dt,
                -integral_limit_,
                integral_limit_);

        const double candidate_output =
            kp_ * error +
            ki_ * candidate_integral -
            kd_ * measured_velocity;

        const double velocity_limited_output =
            std::clamp(
                candidate_output,
                -max_velocity_,
                max_velocity_);

        // Integrate when the output is not saturated, or when the error
        // is driving the controller back toward the valid output range.
        const bool output_saturated =
            candidate_output != velocity_limited_output;

        const bool unwinding_integral =
            (candidate_output > max_velocity_ && error < 0.0) ||
            (candidate_output < -max_velocity_ && error > 0.0);

        if (!output_saturated || unwinding_integral) {
            integral_ = candidate_integral;
        }

        const double maximum_change =
            max_acceleration_ * dt;

        const double acceleration_limited_output =
            std::clamp(
                velocity_limited_output,
                previous_output_ - maximum_change,
                previous_output_ + maximum_change);

        previous_output_ = acceleration_limited_output;

        return acceleration_limited_output;
    }

private:
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};

    double integral_limit_{0.0};
    double max_velocity_{0.0};
    double max_acceleration_{0.0};

    double integral_{0.0};
    double previous_output_{0.0};
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_CONTROL_POSITION_PID_H