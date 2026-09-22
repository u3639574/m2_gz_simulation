#include "m2_gz_simulation/subsystems/gripper_subsystem.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace m2::gz_simulation {
gripper_subsystem::gripper_subsystem(rclcpp::Node& node, joint_feedback& feedback)
    : position_subsystem("gripper", feedback)
{
    max_ = node.declare_parameter("gripper.max_displacement", 0.08);
    tolerance_ = node.declare_parameter("gripper.position_tolerance", 0.002);
    velocity_tolerance_ = node.declare_parameter("gripper.velocity_tolerance", 0.01);
    if (!std::isfinite(max_) || max_ <= 0 || max_ > 0.08 ||
        !std::isfinite(tolerance_) || tolerance_ <= 0 ||
        !std::isfinite(velocity_tolerance_) || velocity_tolerance_ <= 0) {
        throw std::invalid_argument("Invalid gripper limits or tolerances");
    }
    left_publisher_ = node.create_publisher<std_msgs::msg::Float64>(
        node.declare_parameter<std::string>("gripper.left_cmd_pos_topic",
            "/m2_gz_simulation/gripper/left/cmd_pos"), 1);
    right_publisher_ = node.create_publisher<std_msgs::msg::Float64>(
        node.declare_parameter<std::string>("gripper.right_cmd_pos_topic",
            "/m2_gz_simulation/gripper/right/cmd_pos"), 1);
}
bool gripper_subsystem::accepts(double target) const noexcept
{ return std::isfinite(target) && target >= 0.0 && target <= max_; }

void gripper_subsystem::set_target(double target, fsm::command_instance_id_t owner) noexcept
{ left_target_ = target; right_target_ = -target; owner_ = owner; }

bool gripper_subsystem::at_target() const noexcept
{
    return ready() && left_target_ && right_target_ &&
        std::abs(*left_target_ - feedback_.left().position) <= tolerance_ &&
        std::abs(*right_target_ - feedback_.right().position) <= tolerance_ &&
        std::abs(feedback_.left().velocity) <= velocity_tolerance_ &&
        std::abs(feedback_.right().velocity) <= velocity_tolerance_;
}
void gripper_subsystem::publish() noexcept
{
    if (!left_target_ || !right_target_ || !left_publisher_ || !right_publisher_ || !rclcpp::ok()) { return; }
    std_msgs::msg::Float64 message;
    message.data = *left_target_; left_publisher_->publish(message);
    message.data = *right_target_; right_publisher_->publish(message);
}
void gripper_subsystem::hold() noexcept
{
    // Last measured positions are also useful when feedback has just gone stale.
    left_target_ = std::clamp(feedback_.left().position, 0.0, max_);
    right_target_ = std::clamp(feedback_.right().position, -max_, 0.0);
    publish();
}
void gripper_subsystem::reset() noexcept
{ owner_.reset(); hold(); }

void gripper_subsystem::tick() noexcept
{
    if (!ready()) { reset(); return; }
    if (!left_target_) { hold(); }
    publish();
}
}  // namespace m2::gz_simulation
