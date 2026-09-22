#pragma once

#include "m2_gz_simulation/subsystems/position_subsystem.h"
#include "m2_gz_simulation/control/position_pid.h"
#include <std_msgs/msg/float64.hpp>

namespace m2::gz_simulation {
class lift_subsystem : public position_subsystem {
public:
    lift_subsystem(rclcpp::Node& node, joint_feedback& feedback);
    bool accepts(double target) const noexcept override;
    void set_target(double target, fsm::command_instance_id_t owner) noexcept override;
    bool at_target() const noexcept override;
    void hold() noexcept override;
    void reset() noexcept override;
    void tick() noexcept override;
    void close() noexcept { publisher_.reset(); }
private:
    void publish(double velocity) noexcept;
    position_pid pid_;
    double min_, max_, tolerance_, velocity_tolerance_;
    std::optional<double> target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};
}  // namespace m2::gz_simulation
