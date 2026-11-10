#include "m2_gz_simulation/commands/composite/earth_block_transfer.h"
#include "m2_gz_simulation/commands/composite/grip_and_lift.h"
#include "m2_gz_simulation/commands/chassis/earth_block_motion.h"
#include "m2_gz_simulation/commands/gripper/set_grip.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/commands/yaw/set_angle.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_fsm_cpp/base/fsm_command_scheduler.h"
#include "m2_fsm_cpp/commands/fsm_functional_command.h"
#include <cmath>

namespace m2::gz_simulation {
earth_block_transfer_command::earth_block_transfer_command() noexcept
{
    set_name("earth_block_transfer_command");
    chassis_ = FSM_GET_SUBSYSTEM(g_subsystem_container.chassis_subsystem, chassis_subsystem);
    gripper_ = FSM_GET_SUBSYSTEM(g_subsystem_container.gripper_subsystem, gripper_subsystem);
    localization_ = FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem, simlib::localization_subsystem);
    move_base_ = FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, simlib::move_base_subsystem);
    // Choose once so approach/ascent always refer to the same ramp.
    const auto& p = localization_->current_odom().pose.pose.position;
    const auto side = chassis_->nearest_slope(p.x, p.y);
    auto prepare = std::make_unique<fsm::parallel_command_group>();
    prepare->add_commands(std::make_unique<open_and_lower_command>(), std::make_unique<yaw_zero_command>());
    stage("prepare", std::move(prepare));
    stage("pickup", std::make_unique<move_to_earth_block_command>());
    stage("close_grip", std::make_unique<close_grip_command>());
    // Reaching the closed joint position with an empty gripper is not a successful grasp.
    stage("confirm_grasp", std::make_unique<fsm::functional_command>([] {}, [] {}, [](bool) {},
        [this] { return gripper_->holding_block(); }));
    stage("raise", std::make_unique<raise_lift_command>(), true);
    stage("approach_"+side, std::make_unique<move_to_nearest_slope_command>(side), true);
    stage("turn_arm", std::make_unique<yaw_half_turn_command>(), true);
    stage("ascend_"+side, std::make_unique<move_up_slope_command>(side), true);
    stage("release_and_lower", std::make_unique<open_and_lower_command>());
    // During an automatic sequence ignore other mechanism commands; X/manual driving cancels explicitly.
    set_interruption_behaviour(fsm::interruption_behaviour_t::CANCEL_INCOMING);
}
void earth_block_transfer_command::stage(const std::string& name, fsm::command_ptr command, bool carrying)
{
    add_commands(std::make_unique<fsm::functional_command>([this, name, carrying] {
        carrying_ = carrying; stage_start_ = last_contact_ = localization_->now(); chassis_->report(name);
    }, [] {}, [](bool) {}, [] { return true; }), std::move(command));
}
void earth_block_transfer_command::initialize() noexcept
{
    chassis_->clear_fault(); halted_ = false; carrying_ = false;
    move_base_->stop();
    stage_start_ = last_contact_ = localization_->now();
    sequential_command_group::initialize();
    if (!std::isfinite(chassis_->stage_timeout()) || chassis_->stage_timeout() <= 0)
        chassis_->fail("invalid stage timeout");
}
void earth_block_transfer_command::execute() noexcept
{
    const double now = localization_->now();
    if (!localization_->fresh()) chassis_->fail("odometry unavailable");
    if (now < stage_start_ || now-stage_start_ > chassis_->stage_timeout())
        chassis_->fail("stage timeout or clock reset");
    if (carrying_) {
        if (gripper_->holding_block()) last_contact_ = now;
        else if (now-last_contact_ > 0.6) chassis_->fail("payload contact lost");
    }
    if (chassis_->failed()) {
        if (!halted_) { sequential_command_group::end(true); halted_ = true; }
        move_base_->stop(); return;
    }
    sequential_command_group::execute();
}
bool earth_block_transfer_command::is_finished() noexcept
{
    return !chassis_->failed() && !halted_ && sequential_command_group::is_finished();
}
void earth_block_transfer_command::end(bool interrupted) noexcept
{
    if (!halted_) sequential_command_group::end(interrupted);
    move_base_->stop();
    if (!chassis_->failed()) chassis_->report(interrupted ? "cancelled" : "complete");
}
}  // namespace m2::gz_simulation
