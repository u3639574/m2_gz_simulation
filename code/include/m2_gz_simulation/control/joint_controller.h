#pragma once
#include "m2_gz_simulation/control/joint_feedback.h"
#include "m2_gz_simulation/control/cascaded_pid.h"
#include "std_msgs/msg/float64.hpp"
#include <optional>

namespace m2::gz_simulation {
class joint_controller {
public:
    joint_controller(std::shared_ptr<rclcpp::Node> node, joint_feedback& feedback,
                     const std::string& axis = "lift", bool angular = false);
    void tick() noexcept;
private:
    void receive_setpoint(const std_msgs::msg::Float64& msg) noexcept;
    void publish(double effort) noexcept;
    rclcpp::Node* node_;
    joint_feedback& feedback_;
    bool angular_;
    cascaded_pid pid_;
    std::optional<double> target_;
    double min_position_, max_position_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr effort_pub_, velocity_target_pub_;
};
}  // namespace m2::gz_simulation
