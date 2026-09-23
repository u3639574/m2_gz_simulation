#include "m2_gz_simulation/commands/lift/set_lift_pos.h"

#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {

set_lift_pos_command::set_lift_pos_command(double p_setpoint) noexcept
    : command("set_lift_pos_command")
    , p_setpoint_(p_setpoint)
{
    lift_subsystem_ =
        FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem);

    assert(lift_subsystem_ != nullptr);

    add_requirements({
        g_subsystem_container.lift_subsystem,
    });
}

void set_lift_pos_command::initialize() noexcept
{
    lift_subsystem_->set_lift_pos(p_setpoint_);

    TROLLY_INFO(
        "[set_lift_pos_command] Lift p setpoint: %.3f",
        p_setpoint_);
}

void set_lift_pos_command::execute() noexcept
{
}

void set_lift_pos_command::end(bool interrupted) noexcept
{
    if (interrupted) {
        TROLLY_INFO("[set_lift_pos_command] Interrupted.");
    } else {
        TROLLY_INFO("[set_lift_pos_command] Completed.");
    }
}

bool set_lift_pos_command::is_finished() noexcept
{
    return true;
}

}  // namespace m2::gz_simulation