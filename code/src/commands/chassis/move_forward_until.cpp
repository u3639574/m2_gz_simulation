#include "m2_gz_simulation/commands/chassis/move_forward_until.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "trolly/log/trolly_logger_macro.h"
#include "trolly/math/trolly_math_util.h"
#include <cmath>

namespace m2::gz_simulation {
move_forward_until_command::move_forward_until_command(
    const float& speed, is_finished_func_t&& finished) noexcept
    : command("move_forward_until_command"), max_speed_(speed),
      is_finished_func_(std::move(finished))
{
    move_base_subsystem_ = FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem);
    localization_subsystem_ = FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem, simlib::localization_subsystem);
    chassis_subsystem_ = FSM_GET_SUBSYSTEM(g_subsystem_container.chassis_subsystem, chassis_subsystem);
    assert(move_base_subsystem_ && localization_subsystem_ && chassis_subsystem_);
    add_requirements({g_subsystem_container.move_base_subsystem, g_subsystem_container.chassis_subsystem});
}
move_forward_until_command::move_forward_until_command(const float& speed, const int& timeout_ms) noexcept
    : move_forward_until_command(speed, [this, timeout_ms]() {
        return (localization_subsystem_->now() - start_time_) * 1000.0 >= timeout_ms;
    }) {}
void move_forward_until_command::initialize() noexcept
{
    const auto params = chassis_subsystem_->get_pid_command_params("move_forward_yaw_params");
    yaw_controller_ = simlib::PIDController(params.pid_params);
    init_yaw_ = localization_subsystem_->current_yaw();
    start_time_ = localization_subsystem_->now();
    failed_ = !localization_subsystem_->fresh() || !std::isfinite(max_speed_) || !is_finished_func_;
    move_base_subsystem_->stop_trajectory(simlib::move_base_subsystem::STOP_MODE::PUBLISH_NONE);
    TROLLY_INFO("[move_forward_until] %.2f m/s, initial yaw %.3f", max_speed_, init_yaw_);
}
void move_forward_until_command::execute() noexcept
{
    failed_ = failed_ || !localization_subsystem_->fresh() || localization_subsystem_->now() < start_time_;
    geometry_msgs::msg::Twist cmd;
    if (!failed_ && !is_finished_func_()) {
        const auto error = trolly::math::wrapped_angle_difference(localization_subsystem_->current_yaw(), init_yaw_);
        cmd.linear.x = max_speed_;
        cmd.angular.z = yaw_controller_.compute(error); // ROS body yaw; Oogway's hardware uses angular.x.
    }
    move_base_subsystem_->set_fsm_cmd_vel(cmd);
}
void move_forward_until_command::end(bool /*interrupted*/) noexcept
{
    move_base_subsystem_->set_fsm_cmd_vel(geometry_msgs::msg::Twist{});
    TROLLY_INFO("[move_forward_until] %s", failed_ ? "Stopped: feedback/time invalid" : "Ended");
}
bool move_forward_until_command::is_finished() noexcept
{
    return failed_ || !localization_subsystem_->fresh() || is_finished_func_();
}
}  // namespace m2::gz_simulation
