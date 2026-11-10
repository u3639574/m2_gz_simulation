#include "m2_gz_simulation/modes/test_mode.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/commands/yaw/set_angle.h"
#include "m2_gz_simulation/commands/gripper/set_grip.h"
#include "m2_gz_simulation/commands/composite/grip_and_lift.h"
#include "m2_gz_simulation/commands/chassis/move_forward_until.h"
#include "m2_gz_simulation/commands/chassis/earth_block_motion.h"
#include "m2_gz_simulation/commands/composite/earth_block_transfer.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {
void test_mode::on_enter() noexcept
{
    TROLLY_INFO("[test_mode] W/S: lift; E/D: open/close; Q/A: composites; R/F: arm yaw 0/180; arrows: chassis X/Y; Z/C: chassis yaw; X: stop chassis; V: forward 0.15 m/s for 2 simulation seconds; G: pickup pose; B: nearest ramp; H: ramp ascent; T: full Earth Block transfer.");
}
void test_mode::tick() noexcept
{
    const auto now = FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem, simlib::localization_subsystem)->now();
    if (now < last_tick_time_) {
        // A Gazebo reset starts a new run; a latched key must not restart the robot.
        manual_active_ = false; keyboard_velocity_ = geometry_msgs::msg::Twist{};
        FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem)->stop();
        for (const auto key : {keyboard_key::V, keyboard_key::G, keyboard_key::B, keyboard_key::H, keyboard_key::T}) {
            const auto active = active_commands_.find(key);
            if (active != active_commands_.end()) fsm::command_scheduler::instance().cancel_command(active->second);
        }
    }
    last_tick_time_ = now;
    // Keypress has no release event: latch manual motion until another drive key or X.
    if (manual_active_) {
        FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem)->set_keyboard_cmd_vel(keyboard_velocity_);
    }
}
void test_mode::on_exit() noexcept
{
    auto& scheduler = fsm::command_scheduler::instance();
    for (const auto& entry : active_commands_) { scheduler.cancel_command(entry.second); }
    active_commands_.clear();
    manual_active_ = false; keyboard_velocity_ = geometry_msgs::msg::Twist{};
    FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem)->stop();
}
void test_mode::on_input(keyboard_key key) noexcept
{
    auto& scheduler = fsm::command_scheduler::instance();
    auto* move_base = FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem);
    geometry_msgs::msg::Twist manual;
    bool drive_key = true;
    switch (key) {
    case keyboard_key::UP: manual.linear.x = 0.2; break;
    case keyboard_key::DOWN: manual.linear.x = -0.2; break;
    case keyboard_key::LEFT: manual.linear.y = 0.2; break;
    case keyboard_key::RIGHT: manual.linear.y = -0.2; break;
    case keyboard_key::Z: manual.angular.z = 0.4; break;
    case keyboard_key::C: manual.angular.z = -0.4; break;
    case keyboard_key::X: break;
    default: drive_key = false; break;
    }
    if (drive_key) {
        for (const auto action : {keyboard_key::V, keyboard_key::G, keyboard_key::B, keyboard_key::H, keyboard_key::T}) {
            const auto active = active_commands_.find(action);
            if (active != active_commands_.end()) scheduler.cancel_command(active->second);
        }
        move_base->stop();
        keyboard_velocity_ = manual; manual_active_ = key != keyboard_key::X;
        tick(); return;
    }
    // Do not clear a fault or velocity cache when an incoming command will be
    // rejected by the automatic sequence. X and drive keys above cancel it.
    const auto transfer = active_commands_.find(keyboard_key::T);
    if (transfer != active_commands_.end() && scheduler.get_command(transfer->second) != nullptr) return;
    const auto previous = active_commands_.find(key);
    // Keypress has no release event. Ignore autorepeat while this action is active.
    if (previous != active_commands_.end() && scheduler.get_command(previous->second) != nullptr) { return; }
    if (key == keyboard_key::G || key == keyboard_key::B || key == keyboard_key::H || key == keyboard_key::T) {
        manual_active_ = false; keyboard_velocity_ = geometry_msgs::msg::Twist{}; move_base->stop();
        FSM_GET_SUBSYSTEM(g_subsystem_container.chassis_subsystem, chassis_subsystem)->clear_fault();
    }
    fsm::command_ptr cmd;
    switch (key) {
    case keyboard_key::T: cmd = std::make_unique<earth_block_transfer_command>(); break;
    case keyboard_key::G: cmd = std::make_unique<move_to_earth_block_command>(); break;
    case keyboard_key::B: cmd = std::make_unique<move_to_nearest_slope_command>(); break;
    case keyboard_key::H: cmd = std::make_unique<move_up_slope_command>(); break;
    case keyboard_key::W: cmd = std::make_unique<raise_lift_command>(); break;
    case keyboard_key::S: cmd = std::make_unique<lower_lift_command>(); break;
    case keyboard_key::E: cmd = std::make_unique<open_grip_command>(); break;
    case keyboard_key::D: cmd = std::make_unique<close_grip_command>(); break;
    case keyboard_key::Q: cmd = std::make_unique<close_then_raise_command>(); break;
    case keyboard_key::A: cmd = std::make_unique<open_and_lower_command>(); break;
    case keyboard_key::R: cmd = std::make_unique<yaw_zero_command>(); break;
    case keyboard_key::F: cmd = std::make_unique<yaw_half_turn_command>(); break;
    case keyboard_key::V:
        manual_active_ = false; keyboard_velocity_ = geometry_msgs::msg::Twist{}; move_base->stop();
        cmd = std::make_unique<move_forward_until_command>(0.15f, 2000); break;
    default: return;
    }
    TROLLY_INFO("[test_mode] Scheduling %s", cmd->name().c_str());
    active_commands_[key] = scheduler.schedule_command(std::move(cmd));
}
}  // namespace m2::gz_simulation
