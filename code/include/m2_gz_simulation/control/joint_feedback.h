#pragma once

#include <array>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace m2::gz_simulation {

struct joint_sample { double position{0.0}; double velocity{0.0}; };

// Accessed only by the node's single-threaded executor. A complete message is
// accepted atomically, so lift and gripper never see a partially updated frame.
class joint_feedback {
public:
    explicit joint_feedback(rclcpp::Node& node);
    void receive(const sensor_msgs::msg::JointState& message);
    bool advance(double now);  // true on clock rewind or an excessive time gap
    bool ready() const noexcept { return ready_; }
    double now() const noexcept { return now_; }
    double dt() const noexcept { return dt_; }
    bool stalled() const noexcept { return stalled_; }
    const joint_sample& lift() const noexcept { return samples_[0]; }
    const joint_sample& left() const noexcept { return samples_[1]; }
    const joint_sample& right() const noexcept { return samples_[2]; }

private:
    std::array<joint_sample, 3> samples_{};
    bool received_{false}, ready_{false}, started_{false};
    double stamp_{0.0}, now_{0.0}, dt_{0.0};
    double timeout_, max_dt_;
    bool stalled_{false};
    std::chrono::steady_clock::time_point last_clock_progress_{std::chrono::steady_clock::now()};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
}  // namespace m2::gz_simulation
