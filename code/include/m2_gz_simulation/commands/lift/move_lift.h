#pragma once
#include "m2_fsm_cpp/base/fsm_command.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"

namespace m2::gz_simulation {
enum class lift_endpoint { TOP, BOTTOM };
class move_lift_command : public fsm::command {
public:
    explicit move_lift_command(lift_endpoint endpoint) noexcept;
    void initialize() noexcept override;
    void execute() noexcept override {}
    void end(bool interrupted) noexcept override;
    bool is_finished() noexcept override;
private:
    lift_endpoint endpoint_;
    lift_subsystem* lift_;
};
class raise_lift_command : public move_lift_command {
public:
    raise_lift_command() noexcept : move_lift_command(lift_endpoint::TOP) {}
};
class lower_lift_command : public move_lift_command {
public:
    lower_lift_command() noexcept : move_lift_command(lift_endpoint::BOTTOM) {}
};
}  // namespace m2::gz_simulation
