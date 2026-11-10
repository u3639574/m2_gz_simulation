#pragma once
#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_simlib/base/simlib_define.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "m2_simlib/ros2/commands/move_base/pid_pose.h"

namespace m2::gz_simulation {
// Oogway's parameter facade, without VESC, lightgate or active-caster hardware IO.
// simlib owns velocity selection; the Gazebo plugin owns the four swerve modules.
class chassis_subsystem : public fsm::subsystem {
public:
    explicit chassis_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept;
    void declare_pid_pose_params(const std::string& name) noexcept;
    simlib::pid_command_params_t get_pid_command_params(const std::string& name) noexcept;
    simlib::pid_pose_limits_t get_motion_limits(const std::string& name) noexcept;
    std::string nearest_slope(double x, double y) const noexcept;
    void report(const std::string& stage) noexcept;
    void fail(const std::string& reason) noexcept;
    void clear_fault() noexcept { fault_.clear(); }
    bool failed() const noexcept { return !fault_.empty(); }
    double stage_timeout() const noexcept { return node_->get_parameter("earth_transfer.stage_timeout").as_double(); }
    void tick() noexcept override {}
    void close() noexcept { status_.reset(); node_.reset(); }
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    std::string fault_;
};
}  // namespace m2::gz_simulation
