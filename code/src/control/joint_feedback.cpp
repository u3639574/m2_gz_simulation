#include "m2_gz_simulation/control/joint_feedback.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace m2::gz_simulation {
joint_feedback::joint_feedback(rclcpp::Node& node)
{
    timeout_ = node.declare_parameter("feedback_timeout", 0.25);
    max_dt_ = node.declare_parameter("max_control_dt", 0.1);
    if (!std::isfinite(timeout_) || !std::isfinite(max_dt_) || timeout_ <= 0 || max_dt_ <= 0) {
        throw std::invalid_argument("Feedback timeout and max_control_dt must be positive");
    }
    const auto topic = node.declare_parameter<std::string>("joint_states_topic",
        "/m2_gz_simulation/joint_states");
    subscription_ = node.create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { receive(*msg); });
}

void joint_feedback::receive(const sensor_msgs::msg::JointState& message)
{
    static const std::array<std::string, 3> names{
        "vertical_to_horizontal", "gripper_left_joint", "gripper_right_joint"};
    std::array<joint_sample, 3> next;
    for (size_t joint = 0; joint < names.size(); ++joint) {
        const auto it = std::find(message.name.begin(), message.name.end(), names[joint]);
        const auto index = static_cast<size_t>(std::distance(message.name.begin(), it));
        if (it == message.name.end() || index >= message.position.size() ||
            index >= message.velocity.size() ||
            std::count(message.name.begin(), message.name.end(), names[joint]) != 1 ||
            !std::isfinite(message.position[index]) || !std::isfinite(message.velocity[index])) {
            received_ = ready_ = false;
            return;
        }
        next[joint] = {message.position[index], message.velocity[index]};
    }
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    if (received_ && stamp < stamp_) { return; }  // reordered transport sample
    samples_ = next;
    stamp_ = stamp;
    received_ = true;
}

bool joint_feedback::advance(double now)
{
    const double elapsed = started_ ? now - now_ : 0.0;
    const bool discontinuity = started_ && (elapsed < 0.0 || elapsed > max_dt_);
    const auto wall_now = std::chrono::steady_clock::now();
    if (!started_ || elapsed != 0.0) { last_clock_progress_ = wall_now; }
    stalled_ = std::chrono::duration<double>(wall_now - last_clock_progress_).count() > timeout_;
    now_ = now;
    started_ = true;
    dt_ = discontinuity ? 0.0 : elapsed;
    if (discontinuity) { received_ = false; }
    // Gazebo feedback and /clock travel independently; tolerate one small lead.
    const double age = now_ - stamp_;
    ready_ = received_ && age >= -max_dt_ && age <= timeout_;
    return discontinuity;
}
}  // namespace m2::gz_simulation
