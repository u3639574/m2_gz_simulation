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
    // open_displacement = 0.22:
    //     left joint  =  0.22 m
    //     right joint = -0.22 m
    node_->declare_parameter<double>(
        "gripper.closed_displacement",
        0.0);

    node_->declare_parameter<double>(
        "gripper.open_displacement",
        0.22);

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

    completion_ = std::make_unique<motion_completion>(*node_,
        std::vector<std::string>{"gripper_left_joint", "gripper_right_joint"});
    left_contact_sub_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "gripper/left/contacts", rclcpp::SensorDataQoS(),
        [this](const ros_gz_interfaces::msg::Contacts& msg) { receive_contact(msg, left_contact_); });
    right_contact_sub_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "gripper/right/contacts", rclcpp::SensorDataQoS(),
        [this](const ros_gz_interfaces::msg::Contacts& msg) { receive_contact(msg, right_contact_); });
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
    left_contact_ = {}; right_contact_ = {};
    gripper_gripped_ = grip;
    completion_->invalidate();

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
    if (!left_pos_pub_ || !right_pos_pub_ || !rclcpp::ok()) { return; }
    std_msgs::msg::Float64 left_msg;
    left_msg.data = left_setpoint_;
    left_pos_pub_->publish(left_msg);

    std_msgs::msg::Float64 right_msg;
    right_msg.data = right_setpoint_;
    right_pos_pub_->publish(right_msg);
}

void gripper_subsystem::receive_contact(const ros_gz_interfaces::msg::Contacts& msg, contact_state& state) noexcept
{
    std::string block;
    for (const auto& contact : msg.contacts) {
        for (const auto& name : {contact.collision1.name, contact.collision2.name}) {
            const auto begin = name.find("earth_block_");
            if (begin != std::string::npos) { block = name.substr(begin, name.find("::", begin) - begin); }
        }
    }
    const auto now = std::chrono::steady_clock::now();
    if (block != state.block || std::chrono::duration<double>(now - state.received).count() > 0.25) { state.since = now; }
    state.block = block; state.received = now;
}

bool gripper_subsystem::holding_block() const noexcept
{
    const auto now = std::chrono::steady_clock::now();
    return !left_contact_.block.empty() && left_contact_.block == right_contact_.block &&
        std::chrono::duration<double>(now - left_contact_.received).count() < 0.25 &&
        std::chrono::duration<double>(now - right_contact_.received).count() < 0.25 &&
        std::chrono::duration<double>(now - left_contact_.since).count() >= 0.1 &&
        std::chrono::duration<double>(now - right_contact_.since).count() >= 0.1;
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
