#pragma once

#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"
#include "m2_gz_simulation/subsystems/gripper_subsystem.h"
#include <std_msgs/msg/string.hpp>

namespace m2::gz_simulation {
using status_publisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;

// Common lifecycle for the two independent mechanisms. Commands set intent;
// subsystem ticks continue holding the final position after a command finishes.
class position_command : public fsm::command {
public:
    position_command(position_subsystem& subsystem, double target, double timeout,
        double settle_time, status_publisher status);
    void initialize() noexcept override;
    void execute() noexcept override {}
    bool is_finished() noexcept override;
    void end(bool interrupted) noexcept override;
private:
    void report(const std::string& state) noexcept;
    position_subsystem& subsystem_;
    double target_, timeout_, settle_time_, start_{0.0};
    std::optional<double> settled_since_;
    bool failed_{false};
    std::string result_;
    status_publisher status_;
};

class move_lift_command final : public position_command {
public:
    move_lift_command(lift_subsystem& lift, double target, double timeout,
        double settle, status_publisher status)
        : position_command(lift, target, timeout, settle, std::move(status)) {}
};

class set_gripper_command final : public position_command {
public:
    set_gripper_command(gripper_subsystem& gripper, double displacement, double timeout,
        double settle, status_publisher status)
        : position_command(gripper, displacement, timeout, settle, std::move(status)) {}
};
}  // namespace m2::gz_simulation
