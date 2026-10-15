#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace m2::gz_simulation {
// Read-only actuator status for command completion; no control law lives here.
// The subsystem's node outlives this helper. Used on the single ROS executor.
class motion_completion {
public:
    motion_completion(rclcpp::Node& node, std::vector<std::string> joints, bool angular = false)
        : node_(node), joints_(std::move(joints)), angular_(angular)
    {
        if (!node.has_parameter("completion.joint_states_topic")) {
            node.declare_parameter<std::string>("completion.joint_states_topic", "/m2_gz_simulation/joint_states");
            node.declare_parameter<double>("completion.position_tolerance", 0.003);
            node.declare_parameter<double>("completion.velocity_tolerance", 0.01);
            node.declare_parameter<double>("completion.feedback_timeout", 0.25);
        }
        position_tolerance_ = node.get_parameter("completion.position_tolerance").as_double();
        velocity_tolerance_ = node.get_parameter("completion.velocity_tolerance").as_double();
        timeout_ = node.get_parameter("completion.feedback_timeout").as_double();
        sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
            node.get_parameter("completion.joint_states_topic").as_string(), rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState& msg) {
                valid_ = false;
                positions_.clear(); velocities_.clear();
                for (const auto& name : joints_) {
                    const auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    const auto i = static_cast<std::size_t>(it - msg.name.begin());
                    if (it == msg.name.end() || i >= msg.position.size() || i >= msg.velocity.size() ||
                        !std::isfinite(msg.position[i]) || !std::isfinite(msg.velocity[i])) { return; }
                    positions_.push_back(msg.position[i]); velocities_.push_back(msg.velocity[i]);
                }
                stamp_ = rclcpp::Time(msg.header.stamp).seconds();
                received_ = std::chrono::steady_clock::now();
                valid_ = true;
            });
    }

    // A new request needs a new feedback sample, even if its goal is unchanged.
    void invalidate() noexcept { valid_ = false; }

    bool at_target(const std::vector<double>& targets) const noexcept
    {
        if (!valid_ || targets.size() != positions_.size()) { return false; }
        const double age = node_.now().seconds() - stamp_;
        const double wall_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - received_).count();
        if (age < -timeout_ || age > timeout_ || wall_age > timeout_) { return false; }
        for (std::size_t i = 0; i < targets.size(); ++i) {
            if (!std::isfinite(targets[i]) || std::abs(angular_ ? std::remainder(positions_[i] - targets[i], 6.283185307179586) : positions_[i] - targets[i]) > position_tolerance_ ||
                std::abs(velocities_[i]) > velocity_tolerance_) { return false; }
        }
        return true;
    }

private:
    rclcpp::Node& node_;
    std::vector<std::string> joints_;
    std::vector<double> positions_, velocities_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    double stamp_{0.0}, position_tolerance_, velocity_tolerance_, timeout_;
    std::chrono::steady_clock::time_point received_;
    bool valid_{false};
    bool angular_{false};
};
}  // namespace m2::gz_simulation
