#ifndef M2_GZ_SIMULATION_CONTROL_JOINT_FEEDBACK_H
#define M2_GZ_SIMULATION_CONTROL_JOINT_FEEDBACK_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace m2::gz_simulation {

struct joint_state_t {
    double position{0.0};
    double velocity{0.0};
};

class joint_feedback {
public:
    explicit joint_feedback(std::shared_ptr<rclcpp::Node> node);

    void advance(double now) noexcept;

    bool ready() const noexcept;

    bool stalled() const noexcept;

    bool discontinuity() const noexcept;

    double now() const noexcept;

    double dt() const noexcept;

    const joint_state_t& lift() const noexcept;

private:
    void receive(const sensor_msgs::msg::JointState& msg) noexcept;

    // The owning control_node outlives this helper.
    rclcpp::Node* node_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::string lift_joint_name_;

    joint_state_t lift_state_;

    double feedback_timeout_{0.25};
    double max_control_dt_{0.1};

    double now_{0.0};
    double dt_{0.0};
    double feedback_stamp_{0.0};

    bool started_{false};
    bool feedback_received_{false};
    bool ready_{false};
    bool stalled_{false};
    bool discontinuity_{false};

    std::chrono::steady_clock::time_point last_clock_progress_;
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_CONTROL_JOINT_FEEDBACK_H
