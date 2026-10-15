#include "m2_gz_simulation/base/simulation_robot.h"
#include "m2_gz_simulation/base/subsystem_container.h"

#include "m2_gz_simulation/modes/test_mode.h"

#include "m2_gz_simulation/subsystems/gripper_subsystem.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"
#include "m2_gz_simulation/commands/yaw/set_angle.h"


#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {

simulation_robot::simulation_robot(
    std::shared_ptr<rclcpp::Node> node, uint32_t hz,
    const std::string& input_topic) noexcept
    : fsm::ros2_robot(node, hz)
{
    auto& cs = fsm::command_scheduler::instance();

    // Register subsystems here.
    g_subsystem_container.gripper_subsystem =
        cs.register_subsystem<gripper_subsystem>(node);
    g_subsystem_container.lift_subsystem =
        cs.register_subsystem<lift_subsystem>(node);

    g_subsystem_container.yaw_subsystem = cs.register_subsystem<yaw_subsystem>(node);
    // Public degree request goes through the scheduler, preserving requirements.
    yaw_request_ = node->create_subscription<std_msgs::msg::Float64>(
        "yaw/target_degrees", 1, [](const std_msgs::msg::Float64& msg) {
            if (std::isfinite(msg.data) && msg.data >= 0.0 && msg.data < 360.0) {
                fsm::command_scheduler::instance().schedule_command<set_yaw_angle_command>(msg.data);
            } else {
                TROLLY_WARN("Yaw target must be finite degrees in [0, 360).");
            }
        });
    keyboard_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        input_topic, 10, [this](const std_msgs::msg::Int32& msg) {
            mode_.on_input(static_cast<keyboard_key>(msg.data));
        });
}

void simulation_robot::initialize() noexcept { mode_.on_enter(); }

void simulation_robot::tick() noexcept
{
    fsm::ros2_robot::tick();
    mode_.tick();
}

void simulation_robot::close() noexcept
{
    if (closed_) { return; }
    closed_ = true;
    mode_.on_exit();
    keyboard_sub_.reset();
    yaw_request_.reset();
    auto& scheduler = fsm::command_scheduler::instance();
    scheduler.cancel_all_commands();
    scheduler.tick();
    // Scheduler subsystems have static lifetime; release ROS entities while
    // the node still exists, and break subsystem -> node ownership cycles.
    FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem)->close();
    FSM_GET_SUBSYSTEM(g_subsystem_container.yaw_subsystem, yaw_subsystem)->close();
    FSM_GET_SUBSYSTEM(g_subsystem_container.gripper_subsystem, gripper_subsystem)->close();
}

}  // namespace m2::gz_simulation
