#include "m2_gz_simulation/base/simulation_robot.h"
#include "m2_gz_simulation/base/subsystem_container.h"

#include "m2_gz_simulation/modes/test_mode.h"

#include "m2_gz_simulation/subsystems/gripper_subsystem.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"

#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {

simulation_robot::simulation_robot(
    std::shared_ptr<rclcpp::Node> node, uint32_t hz,
    const std::string& input_topic) noexcept
    : fsm::ros2_ps_mode_mux_robot(node, hz, input_topic)
{
    auto& cs = fsm::command_scheduler::instance();

    // Register subsystems here.
    g_subsystem_container.gripper_subsystem =
        cs.register_subsystem<gripper_subsystem>(node);
    g_subsystem_container.lift_subsystem =
        cs.register_subsystem<lift_subsystem>(node);

    mode_store_type modes;

    // Add modes here.
    modes.emplace_back(std::make_unique<test_mode>());

    add_modes(modes);
}

}  // namespace m2::gz_simulation