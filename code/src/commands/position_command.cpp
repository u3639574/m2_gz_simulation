#include "m2_gz_simulation/commands/position_command.h"

namespace m2::gz_simulation {
position_command::position_command(position_subsystem& subsystem, double target,
    double timeout, double settle_time, status_publisher status)
    : command(subsystem.name() + "_position"), subsystem_(subsystem), target_(target),
      timeout_(timeout), settle_time_(settle_time), status_(std::move(status))
{
    add_requirements({subsystem.instance_id()});
}
void position_command::report(const std::string& state) noexcept
{
    if (!rclcpp::ok()) { return; }
    std_msgs::msg::String message;
    message.data = subsystem_.name() + ": " + state;
    status_->publish(message);
}
void position_command::initialize() noexcept
{
    start_ = subsystem_.now();
    settled_since_.reset();
    failed_ = !subsystem_.ready() || !subsystem_.accepts(target_);
    if (failed_) { result_ = "rejected (invalid target or unavailable feedback)"; return; }
    subsystem_.set_target(target_, instance_id());
    report("running target=" + std::to_string(target_));
}
bool position_command::is_finished() noexcept
{
    if (failed_) { return true; }
    if (!subsystem_.ready()) {
        failed_ = true; result_ = "aborted (feedback unavailable)"; return true;
    }
    if (subsystem_.dt() <= 0.0) { return false; }  // pause freezes completion and timeout
    if (subsystem_.now() - start_ >= timeout_) {
        failed_ = true; result_ = "timed out"; return true;
    }
    if (!subsystem_.at_target()) { settled_since_.reset(); return false; }
    if (!settled_since_) { settled_since_ = subsystem_.now(); }
    return subsystem_.now() - *settled_since_ >= settle_time_;
}
void position_command::end(bool interrupted) noexcept
{
    subsystem_.release(instance_id(), interrupted || failed_);
    report(interrupted ? "interrupted" : failed_ ? result_ : "succeeded");
}
}  // namespace m2::gz_simulation
