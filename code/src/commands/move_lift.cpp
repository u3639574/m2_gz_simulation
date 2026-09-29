#include "trolly/log/trolly_logger_macro.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::gz_simulation {
move_lift_command::move_lift_command(lift_endpoint endpoint) noexcept
    : command(endpoint == lift_endpoint::TOP ? "raise_lift_command" : "lower_lift_command"),
      endpoint_(endpoint),
      lift_(FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem))
{
    assert(lift_);
    add_requirements({g_subsystem_container.lift_subsystem});
}
void move_lift_command::initialize() noexcept
{
    lift_->set_lift_pos(endpoint_ == lift_endpoint::TOP ? lift_->top_position() : lift_->bottom_position());
}
bool move_lift_command::is_finished() noexcept { return lift_->at_target(); }
void move_lift_command::end(bool interrupted) noexcept
{
    // Targets remain latched; a conflicting command replaces the target.
    TROLLY_INFO("[%s] %s", name().c_str(), interrupted ? "Interrupted" : "Reached target");
}
}  // namespace m2::gz_simulation
