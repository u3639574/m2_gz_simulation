#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
namespace m2::gz_simulation {
move_lift_command::move_lift_command(lift_endpoint endpoint) noexcept
    : set_lift_pos_command(endpoint == lift_endpoint::TOP
        ? FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem)->top_position()
        : FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem)->bottom_position())
{
    set_name(endpoint == lift_endpoint::TOP ? "raise_lift_command" : "lower_lift_command");
}
}  // namespace m2::gz_simulation
