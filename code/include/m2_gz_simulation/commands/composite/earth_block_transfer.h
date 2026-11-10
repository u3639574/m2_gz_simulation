#pragma once
#include "m2_fsm_cpp/commands/composite/fsm_sequential_command_group.h"
#include "m2_gz_simulation/subsystems/chassis_subsystem.h"
#include "m2_gz_simulation/subsystems/gripper_subsystem.h"
namespace m2::gz_simulation {
class earth_block_transfer_command : public fsm::sequential_command_group {
public:
    earth_block_transfer_command() noexcept;
    void initialize() noexcept override;
    void execute() noexcept override;
    void end(bool interrupted) noexcept override;
    bool is_finished() noexcept override;
private:
    void stage(const std::string& name, fsm::command_ptr command, bool carrying = false);
    chassis_subsystem* chassis_;
    gripper_subsystem* gripper_;
    simlib::localization_subsystem* localization_;
    simlib::move_base_subsystem* move_base_;
    double stage_start_{}, last_contact_{};
    bool carrying_{false}, halted_{false};
};
}  // namespace m2::gz_simulation
