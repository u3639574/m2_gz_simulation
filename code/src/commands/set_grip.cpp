#include "trolly/log/trolly_logger_macro.h"
#include "m2_gz_simulation/commands/gripper/set_grip.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::gz_simulation {
set_grip_command::set_grip_command(bool closed) noexcept
    : command(closed ? "close_grip_command" : "open_grip_command"), closed_(closed),
      gripper_(FSM_GET_SUBSYSTEM(g_subsystem_container.gripper_subsystem, gripper_subsystem))
{
    assert(gripper_);
    add_requirements({g_subsystem_container.gripper_subsystem});
}
void set_grip_command::initialize() noexcept { gripper_->set_gripper(closed_); }
bool set_grip_command::is_finished() noexcept { return true; }
void set_grip_command::end(bool interrupted) noexcept
{
    TROLLY_INFO("[%s] %s", name().c_str(), interrupted ? "Interrupted" : "Reached target");
}
}  // namespace m2::gz_simulation
