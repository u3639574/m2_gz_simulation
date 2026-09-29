#include "m2_gz_simulation/base/simulation_robot.h"
#include "m2_gz_simulation/base/subsystem_container.h"

#include "m2_gz_simulation/modes/test_mode.h"

#include "m2_gz_simulation/subsystems/gripper_subsystem.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"
#include "m2_gz_simulation/commands/lift/set_lift_pos.h"
#include "m2_gz_simulation/commands/gripper/toggle_grip.h"

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

    keyboard_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        input_topic, 10, [this](const std_msgs::msg::Int32& msg) {
            mode_.on_input(static_cast<keyboard_key>(msg.data));
        });

    // Teaching/debug entry points schedule the same commands a mode uses.
    // They are intent topics; do not publish externally on the subsystem output.
    lift_request_ = node->create_subscription<std_msgs::msg::Float64>(
        "lift/target_position", 1,
        [](const std_msgs::msg::Float64& msg) {
            fsm::command_scheduler::instance().schedule_command<set_lift_pos_command>(msg.data);
        });
    toggle_request_ = node->create_service<std_srvs::srv::Trigger>(
        "gripper/toggle",
        [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            fsm::command_scheduler::instance().schedule_command<gripper_toggle_grip_command>(
                g_subsystem_container.gripper_subsystem);
            response->success = true;
            response->message = "Gripper toggle scheduled (not a motion-completion acknowledgement)";
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
    auto& scheduler = fsm::command_scheduler::instance();
    scheduler.cancel_all_commands();
    scheduler.tick();
    // Scheduler subsystems have static lifetime; release ROS entities while
    // the node still exists, and break subsystem -> node ownership cycles.
    FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem)->close();
    FSM_GET_SUBSYSTEM(g_subsystem_container.gripper_subsystem, gripper_subsystem)->close();
    lift_request_.reset();
    toggle_request_.reset();
}

}  // namespace m2::gz_simulation
