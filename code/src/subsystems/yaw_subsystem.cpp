#include "m2_gz_simulation/subsystems/yaw_subsystem.h"
namespace m2::gz_simulation {
yaw_subsystem::yaw_subsystem(std::shared_ptr<rclcpp::Node> node)
    : subsystem("yaw_subsystem"), node_(std::move(node))
{
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("yaw/p_setpoint", 1);
    completion_ = std::make_unique<motion_completion>(*node_, std::vector<std::string>{"base_to_vertical"}, true);
}
bool yaw_subsystem::set_angle(double degrees) noexcept
{
    if (!std::isfinite(degrees) || degrees < 0.0 || degrees >= 360.0) { return false; }
    target_ = degrees * 3.141592653589793 / 180.0;
    completion_->invalidate(); return true;
}
void yaw_subsystem::tick() noexcept
{
    if (!publisher_ || !rclcpp::ok()) { return; }
    std_msgs::msg::Float64 msg; msg.data = target_; publisher_->publish(msg);
}
}  // namespace m2::gz_simulation
