#include "m2_gz_simulation/commands/yaw/set_angle.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
namespace m2::gz_simulation {
set_yaw_angle_command::set_yaw_angle_command(double degrees) noexcept
    : command("set_yaw_angle_command"),
      yaw_(FSM_GET_SUBSYSTEM(g_subsystem_container.yaw_subsystem, yaw_subsystem)), degrees_(degrees)
{ assert(yaw_); add_requirements({g_subsystem_container.yaw_subsystem}); }
void set_yaw_angle_command::initialize() noexcept { accepted_ = yaw_->set_angle(degrees_); }
}
