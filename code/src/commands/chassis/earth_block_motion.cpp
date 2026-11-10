#include "m2_gz_simulation/commands/chassis/earth_block_motion.h"
#include "m2_gz_simulation/subsystems/chassis_subsystem.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"

namespace m2::gz_simulation {
namespace {
auto* chassis() { return FSM_GET_SUBSYSTEM(g_subsystem_container.chassis_subsystem, chassis_subsystem); }
auto* localization() { return FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem, simlib::localization_subsystem); }
std::string slope(const std::string& requested) {
    const auto& p = localization()->current_odom().pose.pose.position;
    return requested.empty() ? chassis()->nearest_slope(p.x, p.y) : requested;
}
fsm::command_ptr pid(const std::string& name) {
    auto* ss = chassis();
    auto cmd = std::make_unique<simlib::pid_pose_command>(ss->get_pid_command_params(name+"_x"),
        ss->get_pid_command_params(name+"_y"), ss->get_pid_command_params(name+"_yaw"), localization(),
        FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem),
        ss->get_motion_limits(name));
    cmd->add_requirements({g_subsystem_container.chassis_subsystem});
    return cmd;
}
}
move_to_earth_block_command::move_to_earth_block_command() noexcept {
    set_name("move_to_earth_block_command"); add_commands(pid("pickup"));
}
move_to_nearest_slope_command::move_to_nearest_slope_command(const std::string& side) noexcept {
    set_name("move_to_nearest_slope_command"); add_commands(pid(slope(side)+"_approach"));
}
move_up_slope_command::move_up_slope_command(const std::string& side) noexcept {
    set_name("move_up_slope_command"); add_commands(pid(slope(side)+"_ascent"));
}
}  // namespace m2::gz_simulation
