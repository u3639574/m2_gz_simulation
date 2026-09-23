#ifndef M2_GZ_SIMULATION_CONTROL_LIFT_CONTROLLER_H
#define M2_GZ_SIMULATION_CONTROL_LIFT_CONTROLLER_H

#include "m2_gz_simulation/control/joint_feedback.h"
#include "m2_gz_simulation/control/position_pid.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>
#include <optional>

namespace m2::gz_simulation {

class lift_controller {
public:
    lift_controller(
        std::shared_ptr<rclcpp::Node> node,
        joint_feedback& feedback);

    void tick() noexcept;

private:
    void receive_setpoint(
        const std_msgs::msg::Float64& msg) noexcept;

    void publish_velocity(double velocity) noexcept;

    std::shared_ptr<rclcpp::Node> node_;

    joint_feedback& feedback_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        setpoint_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        velocity_pub_;

    position_pid pid_;

    std::optional<double> p_setpoint_;

    double min_position_{-0.4};
    double max_position_{0.4};
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_CONTROL_LIFT_CONTROLLER_H