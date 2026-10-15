#include "m2_gz_simulation/control/joint_feedback.h"

#include "trolly/log/trolly_logger_macro.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace m2::gz_simulation {

joint_feedback::joint_feedback(
    std::shared_ptr<rclcpp::Node> node, const std::string& prefix, const std::string& joint_name)
    : node_(node.get())
{
    if (!node_->has_parameter("joint_states_topic")) {
        node_->declare_parameter<std::string>("joint_states_topic", "/m2_gz_simulation/joint_states");
        node_->declare_parameter<double>("feedback_timeout", 0.25);
        node_->declare_parameter<double>("max_control_dt", 0.1);
    }
    node_->declare_parameter<std::string>(prefix + ".joint_name", joint_name);

    const auto joint_states_topic =
        node_->get_parameter("joint_states_topic").as_string();

    lift_joint_name_ =
        node_->get_parameter(prefix + ".joint_name").as_string();

    feedback_timeout_ =
        node_->get_parameter("feedback_timeout").as_double();

    max_control_dt_ =
        node_->get_parameter("max_control_dt").as_double();

    joint_state_sub_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState& msg) {
                receive(msg);
            });

    TROLLY_INFO(
        "[joint_feedback] Joint-state topic: %s",
        joint_states_topic.c_str());

    TROLLY_INFO(
        "[joint_feedback] Tracked joint: %s",
        lift_joint_name_.c_str());
}

void joint_feedback::receive(
    const sensor_msgs::msg::JointState& msg) noexcept
{
    const auto joint_it =
        std::find(msg.name.begin(), msg.name.end(), lift_joint_name_);

    if (joint_it == msg.name.end()) {
        feedback_received_ = false;
        ready_ = false;
        return;
    }

    const auto joint_index =
        static_cast<std::size_t>(
            std::distance(msg.name.begin(), joint_it));

    if (joint_index >= msg.position.size() ||
        joint_index >= msg.velocity.size()) {
        feedback_received_ = false;
        ready_ = false;
        return;
    }

    const double position = msg.position[joint_index];
    const double velocity = msg.velocity[joint_index];

    if (!std::isfinite(position) || !std::isfinite(velocity)) {
        feedback_received_ = false;
        ready_ = false;
        return;
    }

    const double stamp =
        rclcpp::Time(msg.header.stamp).seconds();

    // Ignore an old joint-state message that arrives after a newer one.
    if (feedback_received_ && stamp < feedback_stamp_) {
        return;
    }

    lift_state_.position = position;
    lift_state_.velocity = velocity;
    feedback_stamp_ = stamp;
    feedback_received_ = true;
}

void joint_feedback::advance(double now) noexcept
{
    const auto wall_now = std::chrono::steady_clock::now();

    const double elapsed =
        started_ ? now - now_ : 0.0;

    discontinuity_ =
        started_ && (elapsed < 0.0 || elapsed > max_control_dt_);

    if (!started_ || elapsed != 0.0) {
        last_clock_progress_ = wall_now;
    }

    stalled_ =
        std::chrono::duration<double>(
            wall_now - last_clock_progress_).count() > feedback_timeout_;

    now_ = now;
    started_ = true;

    if (discontinuity_) {
        dt_ = 0.0;
        feedback_received_ = false;
        ready_ = false;

        TROLLY_WARN(
            "[joint_feedback] Simulation-time discontinuity detected.");
        return;
    }

    dt_ = elapsed;

    const double feedback_age =
        now_ - feedback_stamp_;

    ready_ =
        feedback_received_ &&
        feedback_age >= -max_control_dt_ &&
        feedback_age <= feedback_timeout_;
}

bool joint_feedback::ready() const noexcept
{
    return ready_;
}

bool joint_feedback::stalled() const noexcept
{
    return stalled_;
}

bool joint_feedback::discontinuity() const noexcept
{
    return discontinuity_;
}

double joint_feedback::now() const noexcept
{
    return now_;
}

double joint_feedback::dt() const noexcept
{
    return dt_;
}

const joint_state_t& joint_feedback::lift() const noexcept
{
    return lift_state_;
}

}  // namespace m2::gz_simulation
