#pragma once
#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_gz_simulation/subsystems/gripper_subsystem.h"

namespace m2::gz_simulation {
class set_grip_command : public fsm::command {
public:
    explicit set_grip_command(bool closed) noexcept;
    void initialize() noexcept override;
    void execute() noexcept override {}
    void end(bool interrupted) noexcept override;
    bool is_finished() noexcept override;
private:
    bool closed_;
    gripper_subsystem* gripper_;
};
class open_grip_command : public set_grip_command {
public:
    open_grip_command() noexcept : set_grip_command(false) {}
};
class close_grip_command : public set_grip_command {
public:
    close_grip_command() noexcept : set_grip_command(true) {}
};
}  // namespace m2::gz_simulation
