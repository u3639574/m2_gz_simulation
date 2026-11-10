#pragma once
#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_simlib/ros2/subsystems/localization_subsystem.h"
#include "m2_simlib/ros2/subsystems/move_base_subsystem.h"
#include "m2_simlib/utils/pid_control.h"
#include "m2_gz_simulation/subsystems/chassis_subsystem.h"
#include <functional>

namespace m2::gz_simulation {
class move_forward_until_command : public fsm::command {
public:
    using is_finished_func_t = std::function<bool()>;
    explicit move_forward_until_command(const float& speed, is_finished_func_t&& finished) noexcept;
    explicit move_forward_until_command(const float& speed, const int& timeout_ms) noexcept;
    void initialize() noexcept override;
    void execute() noexcept override;
    void end(bool interrupted) noexcept override;
    bool is_finished() noexcept override;
private:
    chassis_subsystem* chassis_subsystem_;
    simlib::move_base_subsystem* move_base_subsystem_;
    simlib::localization_subsystem* localization_subsystem_;
    float max_speed_, init_yaw_{0.0f};
    is_finished_func_t is_finished_func_;
    simlib::PIDController yaw_controller_;
    double start_time_{0.0};
    bool failed_{false};
};
}  // namespace m2::gz_simulation
