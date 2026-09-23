#include "m2_gz_simulation/subsystems/lift_subsystem.h"

#include "trolly/log/trolly_logger_macro.h"

#include <algorithm>
#include <cmath>

namespace m2::gz_simulation {

lift_subsystem::lift_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept
    : subsystem("lift_subsystem")
    , node_(node)
{
    node_->declare_parameter<std::string>(
        "lift.set_pos_topic",
        "lift/p_setpoint");

    node_->declare_parameter<double>("lift.min_pos", -0.4);
    node_->declare_parameter<double>("lift.max_pos", 0.4);

    min_pos_ = node_->get_parameter("lift.min_pos").as_double();
    max_pos_ = node_->get_parameter("lift.max_pos").as_double();

    const auto set_pos_topic =
        node_->get_parameter("lift.set_pos_topic").as_string();

    lift_pos_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(set_pos_topic, 1);

    // Initial target. The subsystem continuously publishes this value until
    // a command calls set_lift_pos().
    lift_pos_target_ = 0.0;

    TROLLY_INFO(
        "[lift_subsystem] Setpoint topic: %s, position range: [%.3f, %.3f]",
        set_pos_topic.c_str(),
        min_pos_,
        max_pos_);
}

void lift_subsystem::set_lift_pos(double pos) noexcept
{
    if (!std::isfinite(pos)) {
        TROLLY_WARN(
            "[lift_subsystem] Ignoring non-finite lift target position.");
        return;
    }

    if (pos < min_pos_ || pos > max_pos_) {
        TROLLY_WARN(
            "[lift_subsystem] Lift target %.3f is out of bounds [%.3f, %.3f]. "
            "Clamping target.",
            pos,
            min_pos_,
            max_pos_);
    }

    lift_pos_target_ = std::clamp(pos, min_pos_, max_pos_);

    TROLLY_INFO(
        "[lift_subsystem] Lift target set to %.3f",
        lift_pos_target_);
}

double lift_subsystem::lift_target_pos() const noexcept
{
    return lift_pos_target_;
}

void lift_subsystem::pub_lift_pos() noexcept
{
    std_msgs::msg::Float64 msg;
    msg.data = lift_pos_target_;

    lift_pos_pub_->publish(msg);
}

void lift_subsystem::tick() noexcept
{
    // Publish the current target every FSM update.
    // The control object receives this p_setpoint and performs PID control.
    pub_lift_pos();
}

}  // namespace m2::gz_simulation