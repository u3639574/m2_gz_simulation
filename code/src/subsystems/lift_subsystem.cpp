#include "m2_gz_simulation/subsystems/lift_subsystem.h"

namespace m2::gz_simulation {
lift_subsystem::lift_subsystem(rclcpp::Node& node, joint_feedback& feedback)
    : position_subsystem("lift", feedback),
      pid_(node.declare_parameter("lift.kp", 3.0),
          node.declare_parameter("lift.ki", 0.1),
          node.declare_parameter("lift.kd", 0.1),
          node.declare_parameter("lift.max_velocity", 0.25),
          node.declare_parameter("lift.integral_limit", 0.2),
          node.declare_parameter("lift.max_acceleration", 0.8))
{
    min_ = node.declare_parameter("lift.min_position", -0.4);
    max_ = node.declare_parameter("lift.max_position", 0.4);
    tolerance_ = node.declare_parameter("lift.position_tolerance", 0.003);
    velocity_tolerance_ = node.declare_parameter("lift.velocity_tolerance", 0.01);
    if (!std::isfinite(min_) || !std::isfinite(max_) || min_ >= max_ ||
        min_ < -0.4 || max_ > 0.4 || !std::isfinite(tolerance_) || tolerance_ <= 0 ||
        !std::isfinite(velocity_tolerance_) || velocity_tolerance_ <= 0) {
        throw std::invalid_argument("Invalid lift limits or tolerances");
    }
    publisher_ = node.create_publisher<std_msgs::msg::Float64>(
        node.declare_parameter<std::string>("lift.cmd_vel_topic", "/m2_gz_simulation/lift/cmd_vel"), 1);
}
bool lift_subsystem::accepts(double target) const noexcept
{ return std::isfinite(target) && target >= min_ && target <= max_; }

void lift_subsystem::set_target(double target, fsm::command_instance_id_t owner) noexcept
{
    target_ = target;
    owner_ = owner;
    pid_.reset();
}
bool lift_subsystem::at_target() const noexcept
{
    return ready() && target_ && std::abs(*target_ - feedback_.lift().position) <= tolerance_ &&
        std::abs(feedback_.lift().velocity) <= velocity_tolerance_;
}
void lift_subsystem::publish(double velocity) noexcept
{
    if (!publisher_ || !rclcpp::ok()) { return; }
    std_msgs::msg::Float64 message;
    message.data = velocity;
    publisher_->publish(message);
}
void lift_subsystem::reset() noexcept
{ target_.reset(); owner_.reset(); pid_.reset(); publish(0.0); }

void lift_subsystem::hold() noexcept
{
    pid_.reset();
    target_ = ready() ? std::optional<double>(std::clamp(feedback_.lift().position, min_, max_))
                      : std::nullopt;
    publish(0.0);
}
void lift_subsystem::tick() noexcept
{
    if (!ready()) { reset(); return; }
    if (!target_) { hold(); }
    if (dt() <= 0.0) {
        // A wall tick may arrive between clock messages during normal running.
        // Stop only after a sustained clock stall, not on every duplicate stamp.
        if (feedback_.stalled()) { publish(0.0); pid_.reset(); }
        return;
    }
    const auto& state = feedback_.lift();
    double velocity = pid_.compute(*target_, state.position, state.velocity, dt());
    if ((state.position <= min_ && velocity < 0.0) ||
        (state.position >= max_ && velocity > 0.0)) {
        velocity = 0.0;
        pid_.reset();
    }
    publish(velocity);
}
}  // namespace m2::gz_simulation
