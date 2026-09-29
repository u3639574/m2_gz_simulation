#include "m2_gz_simulation/modes/test_mode.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/commands/gripper/set_grip.h"
#include "m2_gz_simulation/commands/composite/grip_and_lift.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {
void test_mode::on_enter() noexcept
{
    TROLLY_INFO("[test_mode] Up/Down: lift; Left/Right: open/close; Q: close then raise; A: open and lower.");
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
    case keyboard_key::UP: cmd = std::make_unique<raise_lift_command>(); break;
    case keyboard_key::DOWN: cmd = std::make_unique<lower_lift_command>(); break;
    case keyboard_key::LEFT: cmd = std::make_unique<open_grip_command>(); break;
    case keyboard_key::RIGHT: cmd = std::make_unique<close_grip_command>(); break;
    case keyboard_key::Q: cmd = std::make_unique<close_then_raise_command>(); break;
    case keyboard_key::A: cmd = std::make_unique<open_and_lower_command>(); break;
    default: return;
    }
    TROLLY_INFO("[test_mode] Scheduling %s", cmd->name().c_str());
    active_commands_[key] = scheduler.schedule_command(std::move(cmd));
}
}  // namespace m2::gz_simulation
