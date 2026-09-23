#ifndef M2_GZ_SIMULATION_COMMANDS_LIFT_SET_LIFT_POS_H
#define M2_GZ_SIMULATION_COMMANDS_LIFT_SET_LIFT_POS_H

#include "m2_fsm_cpp/base/fsm_command.h"

#include "m2_gz_simulation/base/gz_simulation_define.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"

namespace m2::gz_simulation {

class set_lift_pos_command : public fsm::command {
public:
    explicit set_lift_pos_command(double p_setpoint) noexcept;

    void initialize() noexcept;

    void execute() noexcept;

    void end(bool interrupted) noexcept;

    bool is_finished() noexcept;

private:
    double p_setpoint_{0.0};

    lift_subsystem* lift_subsystem_;
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_COMMANDS_LIFT_SET_LIFT_POS_H