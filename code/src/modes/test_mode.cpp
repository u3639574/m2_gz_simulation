#include "m2_gz_simulation/modes/test_mode.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/commands/yaw/set_angle.h"
#include "m2_gz_simulation/commands/gripper/set_grip.h"
#include "m2_gz_simulation/commands/composite/grip_and_lift.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {
void test_mode::on_enter() noexcept
{
    TROLLY_INFO("[test_mode] W/S: lift; E/D: open/close; Q/A: composites; R/F: yaw 0/180; arrows: chassis; X: stop chassis.");
}
void test_mode::on_exit() noexcept
{
    auto& scheduler = fsm::command_scheduler::instance();
    for (const auto& entry : active_commands_) { scheduler.cancel_command(entry.second); }
    active_commands_.clear();
}
void test_mode::on_input(keyboard_key key) noexcept
{
    auto& scheduler = fsm::command_scheduler::instance();
    const auto previous = active_commands_.find(key);
    // Keypress has no release event. Ignore autorepeat while this action is active.
    if (previous != active_commands_.end() && scheduler.get_command(previous->second) != nullptr) { return; }
    fsm::command_ptr cmd;
    switch (key) {
    case keyboard_key::W: cmd = std::make_unique<raise_lift_command>(); break;
    case keyboard_key::S: cmd = std::make_unique<lower_lift_command>(); break;
    case keyboard_key::E: cmd = std::make_unique<open_grip_command>(); break;
    case keyboard_key::D: cmd = std::make_unique<close_grip_command>(); break;
    case keyboard_key::Q: cmd = std::make_unique<close_then_raise_command>(); break;
    case keyboard_key::A: cmd = std::make_unique<open_and_lower_command>(); break;
    case keyboard_key::R: cmd = std::make_unique<yaw_zero_command>(); break;
    case keyboard_key::F: cmd = std::make_unique<yaw_half_turn_command>(); break;
    default: return;
    }
    TROLLY_INFO("[test_mode] Scheduling %s", cmd->name().c_str());
    active_commands_[key] = scheduler.schedule_command(std::move(cmd));
}
}  // namespace m2::gz_simulation
