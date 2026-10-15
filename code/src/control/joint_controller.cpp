#include "m2_gz_simulation/control/joint_controller.h"
#include <cmath>

namespace m2::gz_simulation {
joint_controller::joint_controller(std::shared_ptr<rclcpp::Node> node, joint_feedback& feedback,
    const std::string& axis, bool angular)
    : node_(node.get()), feedback_(feedback), angular_(angular),
      pid_(position_pid(
          node_->declare_parameter<double>(axis + ".position.kp", 3.0),
          node_->declare_parameter<double>(axis + ".position.ki", 0.0),
          node_->declare_parameter<double>(axis + ".position.kd", 0.05),
          node_->declare_parameter<double>(axis + ".position.integral_limit", 0.2),
          node_->declare_parameter<double>(axis + ".max_velocity", angular ? 0.7 : 0.3),
          node_->declare_parameter<double>(axis + ".max_acceleration", angular ? 1.0 : 0.6)),
          node_->declare_parameter<double>(axis + ".velocity.kp", angular ? 12.0 : 100.0),
          node_->declare_parameter<double>(axis + ".velocity.ki", angular ? 8.0 : 180.0),
          node_->declare_parameter<double>(axis + ".velocity.kd", angular ? 0.1 : 0.3),
          node_->declare_parameter<double>(axis + ".velocity.integral_limit", angular ? 10.0 : 30.0),
          node_->declare_parameter<double>(axis + ".max_effort", angular ? 20.0 : 120.0),
          node_->declare_parameter<double>(axis + ".feedforward", angular ? 0.0 : 39.24))
{
    min_position_ = node_->declare_parameter<double>(axis + ".min_position", 0.0);
    max_position_ = node_->declare_parameter<double>(axis + ".max_position", angular ? 6.283185307179586 : 1.1);
    const auto input = node_->declare_parameter<std::string>(axis + ".p_setpoint_topic", axis + "/p_setpoint");
    const auto effort = node_->declare_parameter<std::string>(axis + ".effort_topic", axis + "/cmd_effort");
    setpoint_sub_ = node_->create_subscription<std_msgs::msg::Float64>(input, 1,
        [this](const std_msgs::msg::Float64& msg) { receive_setpoint(msg); });
    effort_pub_ = node_->create_publisher<std_msgs::msg::Float64>(effort, 1);
    // Diagnostic only: never bridged to an actuator/controller.
    velocity_target_pub_ = node_->create_publisher<std_msgs::msg::Float64>(axis + "/velocity_target", 1);
}
void joint_controller::receive_setpoint(const std_msgs::msg::Float64& msg) noexcept
{
    if (!std::isfinite(msg.data)) { return; }
    if (angular_ && (msg.data < 0.0 || msg.data >= 6.283185307179586)) { return; }
    // Keep both loops' state across heartbeats and changing goals; outer ramp
    // provides continuity. Reset only on disabled/invalid feedback/reset.
    target_ = angular_ ? msg.data : std::clamp(msg.data, min_position_, max_position_);
}
void joint_controller::publish(double effort) noexcept
{
    std_msgs::msg::Float64 msg; msg.data = effort; effort_pub_->publish(msg);
}
void joint_controller::tick() noexcept
{
    if (!feedback_.ready() || feedback_.discontinuity() || feedback_.stalled()) {
        pid_.reset(); publish(0.0); return;
    }
    if (feedback_.dt() <= 0.0) { return; }
    const auto& state = feedback_.lift();
    // Hold the measured startup position until commanded (including gravity).
    if (!target_) { target_ = angular_ ? 0.0 : std::clamp(state.position, min_position_, max_position_); }
    const double goal = angular_ ? state.position + std::remainder(*target_ - state.position, 6.283185307179586) : *target_;
    publish(pid_.compute(goal, state.position, state.velocity, feedback_.dt()));
    std_msgs::msg::Float64 v; v.data = pid_.velocity_target(); velocity_target_pub_->publish(v);
}
}  // namespace m2::gz_simulation
