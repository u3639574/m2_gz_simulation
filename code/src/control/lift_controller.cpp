#include "m2_gz_simulation/control/lift_controller.h"

#include "trolly/log/trolly_logger_macro.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace m2::gz_simulation {

lift_controller::lift_controller(
    std::shared_ptr<rclcpp::Node> node,
    joint_feedback& feedback)
    : node_(std::move(node))
    , feedback_(feedback)
    , pid_(
          node_->declare_parameter<double>("lift.kp", 3.0),
          node_->declare_parameter<double>("lift.ki", 0.1),
          node_->declare_parameter<double>("lift.kd", 0.1),
          node_->declare_parameter<double>("lift.integral_limit", 0.2),
          node_->declare_parameter<double>("lift.max_velocity", 0.25),
          node_->declare_parameter<double>("lift.max_acceleration", 0.8))
{
    node_->declare_parameter<std::string>(
        "lift.p_setpoint_topic",
        "lift/p_setpoint");

    node_->declare_parameter<std::string>(
        "lift.cmd_vel_topic",
        "/m2_gz_simulation/lift/cmd_vel");

    node_->declare_parameter<double>(
        "lift.min_position",
        -0.4);

    node_->declare_parameter<double>(
        "lift.max_position",
        0.4);

    const auto p_setpoint_topic =
        node_->get_parameter("lift.p_setpoint_topic").as_string();

    const auto cmd_vel_topic =
        node_->get_parameter("lift.cmd_vel_topic").as_string();

    min_position_ =
        node_->get_parameter("lift.min_position").as_double();

    max_position_ =
        node_->get_parameter("lift.max_position").as_double();

    setpoint_sub_ =
        node_->create_subscription<std_msgs::msg::Float64>(
            p_setpoint_topic,
            1,
            [this](const std_msgs::msg::Float64& msg) {
                receive_setpoint(msg);
            });

    velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            cmd_vel_topic,
            1);

    TROLLY_INFO(
        "[lift_controller] P setpoint topic: %s",
        p_setpoint_topic.c_str());

    TROLLY_INFO(
        "[lift_controller] Velocity command topic: %s",
        cmd_vel_topic.c_str());
}

void lift_controller::receive_setpoint(
    const std_msgs::msg::Float64& msg) noexcept
{
    if (!std::isfinite(msg.data)) {
        TROLLY_WARN(
            "[lift_controller] Ignoring non-finite p setpoint.");
        return;
    }

    p_setpoint_ =
        std::clamp(
            msg.data,
            min_position_,
            max_position_);

    pid_.reset();
}

void lift_controller::publish_velocity(double velocity) noexcept
{
    std_msgs::msg::Float64 msg;
    msg.data = velocity;

    velocity_pub_->publish(msg);
}

void lift_controller::tick() noexcept
{
    // Stop safely if Gazebo feedback is invalid, old, or after simulation reset.
    if (!feedback_.ready() || feedback_.discontinuity()) {
        pid_.reset();
        publish_velocity(0.0);
        return;
    }

    // No lift command has been issued yet.
    if (!p_setpoint_.has_value()) {
        pid_.reset();
        publish_velocity(0.0);
        return;
    }

    // Gazebo is paused or no new simulation clock value has arrived.
    if (feedback_.dt() <= 0.0) {
        if (feedback_.stalled()) {
            pid_.reset();
            publish_velocity(0.0);
        }

        return;
    }

    const auto& lift = feedback_.lift();

    double velocity =
        pid_.compute(
            p_setpoint_.value(),
            lift.position,
            lift.velocity,
            feedback_.dt());

    // Prevent a command from driving farther beyond a physical joint limit.
    if ((lift.position <= min_position_ && velocity < 0.0) ||
        (lift.position >= max_position_ && velocity > 0.0)) {
        velocity = 0.0;
        pid_.reset();
    }

    publish_velocity(velocity);
}

}  // namespace m2::gz_simulation