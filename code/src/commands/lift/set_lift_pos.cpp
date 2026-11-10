#include "m2_gz_simulation/commands/lift/set_lift_pos.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "trolly/log/trolly_logger_macro.h"
#include <cmath>

namespace m2::gz_simulation {
set_lift_pos_command::set_lift_pos_command(double position) noexcept
    : command("set_lift_pos_command"), p_setpoint_(position),
      lift_subsystem_(FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem))
{
    assert(lift_subsystem_);
    add_requirements({g_subsystem_container.lift_subsystem});
}
void set_lift_pos_command::initialize() noexcept
{
    const auto profile = lift_subsystem_->motion_profile();
    invalid_ = !std::isfinite(p_setpoint_) || (profile != "pid" && profile != "sine");
    if (invalid_) { TROLLY_ERROR("Invalid lift target or motion profile"); return; }
    sine_ = profile == "sine";
    p_setpoint_ = lift_subsystem_->clamp_position(p_setpoint_);
    started_ = false;
    lift_subsystem_->begin_motion();
    // A sine reference starts from measured position, as in Nezha.
    if (!sine_) { lift_subsystem_->set_lift_pos(p_setpoint_); started_ = true; }
    revision_ = lift_subsystem_->target_revision();
    execute();
    TROLLY_INFO("[set_lift_pos_command] %s target %.3f m", profile.c_str(), p_setpoint_);
}
void set_lift_pos_command::execute() noexcept
{
    if (invalid_ || !sine_ || !lift_subsystem_->feedback_fresh()) { return; }
    const double now = lift_subsystem_->now();
    if (!started_ || now < start_time_) {
        start_position_ = lift_subsystem_->current_position();
        const auto params = lift_subsystem_->trajectory_params(p_setpoint_ >= start_position_);
        if (!std::isfinite(params.vel) || !std::isfinite(params.accel) || !std::isfinite(params.decel) ||
            params.vel <= 0 || params.accel <= 0 || params.decel <= 0) {
            invalid_ = true; TROLLY_ERROR("Invalid lift sine limits"); return;
        }
        trajectory_.compute_coefficients(p_setpoint_ - start_position_, params.vel, params.accel, params.decel, 0.0);
        start_time_ = now; started_ = true;
    }
    const double elapsed = now - start_time_;
    lift_subsystem_->set_lift_pos(elapsed >= trajectory_.get_total_duration() ? p_setpoint_ :
        start_position_ + trajectory_.get_output(elapsed).position, false);
    revision_ = lift_subsystem_->target_revision();
}
bool set_lift_pos_command::is_finished() noexcept
{
    if (invalid_) { return true; }
    return started_ && (!sine_ || lift_subsystem_->now() - start_time_ >= trajectory_.get_total_duration())
        && lift_subsystem_->at_target();
}
void set_lift_pos_command::end(bool interrupted) noexcept
{
    // A replacement can initialize before the old command ends. Do not overwrite it.
    if (interrupted && started_ && revision_ == lift_subsystem_->target_revision() && lift_subsystem_->feedback_fresh()) {
        lift_subsystem_->set_lift_pos(lift_subsystem_->current_position());
    }
    TROLLY_INFO("[set_lift_pos_command] %s", interrupted || invalid_ ? "Stopped" : "Reached target");
}
}  // namespace m2::gz_simulation
