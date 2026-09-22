#pragma once

#include "m2_fsm_cpp/base/fsm_robot.h"
#include "m2_gz_simulation/commands/position_command.h"
#include <std_srvs/srv/trigger.hpp>

namespace m2::gz_simulation {
// One scheduler/robot per process, matching m2_fsm_cpp's singleton ownership.
class simulation_robot : public fsm::robot {
public:
    explicit simulation_robot(rclcpp::Node& node);
    ~simulation_robot() noexcept override;
    void initialize() noexcept override;
    void tick() noexcept override;
    void stop() noexcept;
private:
    bool validate_request(position_subsystem& subsystem, double target);
    rclcpp::Node& node_;
    joint_feedback feedback_;
    lift_subsystem* lift_;
    gripper_subsystem* gripper_;
    double command_timeout_, settle_time_;
    status_publisher status_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_request_, gripper_request_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};
}  // namespace m2::gz_simulation
