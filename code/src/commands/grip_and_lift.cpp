#include "m2_gz_simulation/commands/composite/grip_and_lift.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/commands/gripper/set_grip.h"

namespace m2::gz_simulation {
close_then_raise_command::close_then_raise_command() noexcept
{
    set_name("close_then_raise_command");
    add_commands(std::make_unique<close_grip_command>(), std::make_unique<raise_lift_command>());
}
open_and_lower_command::open_and_lower_command() noexcept
{
    set_name("open_and_lower_command");
    add_commands(std::make_unique<open_grip_command>(), std::make_unique<lower_lift_command>());
}
}  // namespace m2::gz_simulation
