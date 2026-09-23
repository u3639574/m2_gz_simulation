#include "m2_gz_simulation/commands/gripper/toggle_grip.h"

#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_gz_simulation/subsystems/gripper_subsystem.h"

#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::gz_simulation {

gripper_toggle_grip_command::gripper_toggle_grip_command(
    fsm::subsystem_instance_id_t gripper_subsystem_id) noexcept
    : command("gripper_toggle_grip_command")
    , gripper_subsystem_(
          FSM_GET_SUBSYSTEM(gripper_subsystem_id, gripper_subsystem))
{
    assert(gripper_subsystem_ != nullptr);

    add_requirements({gripper_subsystem_id});
}

void gripper_toggle_grip_command::initialize() noexcept
{
    gripper_subsystem_->gripper_toggle();
}

void gripper_toggle_grip_command::execute() noexcept
{
}

void gripper_toggle_grip_command::end(bool /* interrupted */) noexcept
{
    TROLLY_INFO("[gripper_toggle_grip_command] End.");
}

bool gripper_toggle_grip_command::is_finished() noexcept
{
    return true;
}

}  // namespace m2::gz_simulation