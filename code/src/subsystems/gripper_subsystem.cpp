#include "m2_gz_simulation/subsystems/gripper_subsystem.h"

#include "trolly/log/trolly_logger_macro.h"

#include <utility>

namespace m2::gz_simulation {

gripper_subsystem::gripper_subsystem(
    std::shared_ptr<rclcpp::Node> node) noexcept
    : subsystem("gripper_subsystem")
    , node_(std::move(node))
{
    node_->declare_parameter<std::string>(
        "gripper.left_set_pos_topic",
        "/m2_gz_simulation/gripper/left/cmd_pos");

    node_->declare_parameter<std::string>(
        "gripper.right_set_pos_topic",
        "/m2_gz_simulation/gripper/right/cmd_pos");

    // Per-finger displacement in metres.
    //
    // closed_displacement = 0.0:
    //     left joint  =  0.0 m
    //     right joint = -0.0 m
    //
    // open_displacement = 0.08:
    //     left joint  =  0.08 m
    //     right joint = -0.08 m
    node_->declare_parameter<double>(
        "gripper.closed_displacement",
        0.0);

    node_->declare_parameter<double>(
        "gripper.open_displacement",
        0.08);

    closed_displacement_ =
        node_->get_parameter("gripper.closed_displacement").as_double();

    open_displacement_ =
        node_->get_parameter("gripper.open_displacement").as_double();

    const auto left_set_pos_topic =
        node_->get_parameter("gripper.left_set_pos_topic").as_string();

    const auto right_set_pos_topic =
        node_->get_parameter("gripper.right_set_pos_topic").as_string();

    left_pos_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            left_set_pos_topic,
            1);

    right_pos_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            right_set_pos_topic,
            1);

    // Start released / open.
    gripper_gripped_ = false;
    set_gripper(false);

    TROLLY_INFO(
        "[gripper_subsystem] Left position topic: %s",
        left_set_pos_topic.c_str());

    TROLLY_INFO(
        "[gripper_subsystem] Right position topic: %s",
        right_set_pos_topic.c_str());

    TROLLY_INFO(
        "[gripper_subsystem] Closed displacement: %.3f, open displacement: %.3f",
        closed_displacement_,
        open_displacement_);
}

void gripper_subsystem::set_gripper(bool grip) noexcept
{
    gripper_gripped_ = grip;

    if (gripper_gripped_) {
        left_setpoint_ = closed_displacement_;
        right_setpoint_ = -closed_displacement_;

        TROLLY_INFO(
            "[gripper_subsystem] Gripper command: grip");
    } else {
        left_setpoint_ = open_displacement_;
        right_setpoint_ = -open_displacement_;

        TROLLY_INFO(
            "[gripper_subsystem] Gripper command: release");
    }
}

void gripper_subsystem::gripper_set_grip() noexcept
{
    set_gripper(true);
}

void gripper_subsystem::gripper_set_release() noexcept
{
    set_gripper(false);
}

void gripper_subsystem::gripper_toggle() noexcept
{
    set_gripper(!gripper_gripped_);
}

bool gripper_subsystem::is_gripped() const noexcept
{
    return gripper_gripped_;
}

void gripper_subsystem::pub_gripper_pos() noexcept
{
    std_msgs::msg::Float64 left_msg;
    left_msg.data = left_setpoint_;
    left_pos_pub_->publish(left_msg);

    std_msgs::msg::Float64 right_msg;
    right_msg.data = right_setpoint_;
    right_pos_pub_->publish(right_msg);
}

void gripper_subsystem::tick() noexcept
{
    // Constantly publish the current pneumatic-cylinder state.
    //
    // Gazebo JointPositionController receives these position commands
    // directly, so the gripper needs no external PID object.
    pub_gripper_pos();
}

}  // namespace m2::gz_simulation