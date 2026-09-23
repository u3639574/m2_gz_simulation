#ifndef M2_GZ_SIMULATION_COMMANDS_GRIPPER_TOGGLE_GRIP_H
#define M2_GZ_SIMULATION_COMMANDS_GRIPPER_TOGGLE_GRIP_H

#include "m2_fsm_cpp/base/fsm_command.h"

#include "m2_gz_simulation/base/gz_simulation_define.h"
#include "m2_gz_simulation/subsystems/gripper_subsystem.h"

namespace m2::gz_simulation {

class gripper_toggle_grip_command : public fsm::command {
public:
    explicit gripper_toggle_grip_command(
        fsm::subsystem_instance_id_t gripper_subsystem_id) noexcept;

    void initialize() noexcept;

    void execute() noexcept;

    void end(bool interrupted) noexcept;

    bool is_finished() noexcept;

private:
    gripper_subsystem* gripper_subsystem_;
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_COMMANDS_GRIPPER_TOGGLE_GRIP_H