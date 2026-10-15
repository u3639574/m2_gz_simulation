#pragma once
#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_gz_simulation/subsystems/yaw_subsystem.h"
namespace m2::gz_simulation {
class set_yaw_angle_command : public fsm::command {
public:
    explicit set_yaw_angle_command(double degrees) noexcept;
    void initialize() noexcept override;
    void execute() noexcept override {}
    void end(bool) noexcept override {}
    bool is_finished() noexcept override { return !accepted_ || yaw_->at_target(); }
private:
    yaw_subsystem* yaw_;
    double degrees_;
    bool accepted_{false};
};
class yaw_zero_command : public set_yaw_angle_command {
public: yaw_zero_command() noexcept : set_yaw_angle_command(0.0) {}
};
class yaw_half_turn_command : public set_yaw_angle_command {
public: yaw_half_turn_command() noexcept : set_yaw_angle_command(180.0) {}
};
}
