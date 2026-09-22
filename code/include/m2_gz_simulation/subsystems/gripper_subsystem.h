#pragma once

#include "m2_gz_simulation/subsystems/position_subsystem.h"
#include <std_msgs/msg/float64.hpp>

namespace m2::gz_simulation {
class gripper_subsystem : public position_subsystem {
public:
    gripper_subsystem(rclcpp::Node& node, joint_feedback& feedback);
    bool accepts(double target) const noexcept override;
    void set_target(double target, fsm::command_instance_id_t owner) noexcept override;
    bool at_target() const noexcept override;
    void hold() noexcept override;
    void reset() noexcept override;
    void tick() noexcept override;
    void close() noexcept { left_publisher_.reset(); right_publisher_.reset(); }
private:
    void publish() noexcept;
    double max_, tolerance_, velocity_tolerance_;
    std::optional<double> left_target_, right_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_publisher_, right_publisher_;
};
}  // namespace m2::gz_simulation
