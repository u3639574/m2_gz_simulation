#include "m2_gz_simulation/subsystems/chassis_subsystem.h"
#include "trolly/log/trolly_logger_macro.h"
#include <cmath>
namespace m2::gz_simulation {
chassis_subsystem::chassis_subsystem(std::shared_ptr<rclcpp::Node> node) noexcept
    : subsystem("chassis_subsystem"), node_(node)
{
    declare_pid_pose_params("move_forward_yaw_params");
    node_->declare_parameter("earth_transfer.stage_timeout", 45.0);
    status_ = node_->create_publisher<std_msgs::msg::String>("chassis/autopath_status", rclcpp::QoS(1).transient_local());
    for (const std::string motion : {"pickup", "red_approach", "blue_approach", "red_ascent", "blue_ascent"}) {
        for (const std::string axis : {"x", "y", "yaw"}) {
            declare_pid_pose_params(motion+"_"+axis);
            node_->declare_parameter("motion."+motion+"."+axis+".acceleration", 0.3);
            node_->declare_parameter("motion."+motion+"."+axis+".deceleration", 0.4);
        }
        node_->declare_parameter("motion."+motion+".timeout", 40.0);
    }
}
void chassis_subsystem::report(const std::string& stage) noexcept
{
    std_msgs::msg::String msg; msg.data = stage; status_->publish(msg);
    TROLLY_INFO("[earth_transfer] %s", stage.c_str());
}
void chassis_subsystem::fail(const std::string& reason) noexcept
{
    if (!failed()) { fault_ = reason; report("failed: "+reason); }
}
std::string chassis_subsystem::nearest_slope(double x, double y) const noexcept
{
    const auto distance = [&](const std::string& side) {
        return std::hypot(x-node_->get_parameter("pid_pose."+side+"_approach_x.target").as_double(),
                          y-node_->get_parameter("pid_pose."+side+"_approach_y.target").as_double());
    };
    return distance("red") <= distance("blue") ? "red" : "blue";
}
simlib::pid_pose_limits_t chassis_subsystem::get_motion_limits(const std::string& name) noexcept
{
    simlib::pid_pose_limits_t limits;
    size_t i = 0;
    for (const std::string axis : {"x", "y", "yaw"}) {
        limits.acceleration[i] = node_->get_parameter("motion."+name+"."+axis+".acceleration").as_double();
        limits.deceleration[i++] = node_->get_parameter("motion."+name+"."+axis+".deceleration").as_double();
    }
    limits.timeout = node_->get_parameter("motion."+name+".timeout").as_double();
    limits.on_failure = [this, name](const std::string& reason) { fail(name+": "+reason); };
    return limits;
}
void chassis_subsystem::declare_pid_pose_params(const std::string& name) noexcept
{
    node_->declare_parameter("pid_pose." + name + ".target", 0.0);
    node_->declare_parameter("pid_pose." + name + ".kp", 0.0);
    node_->declare_parameter("pid_pose." + name + ".ki", 0.0);
    node_->declare_parameter("pid_pose." + name + ".kd", 0.0);
    node_->declare_parameter("pid_pose." + name + ".ki_deadzone", 0.0);
    node_->declare_parameter("pid_pose." + name + ".ki_sum_error_lim", 0.0);
    node_->declare_parameter("pid_pose." + name + ".max_abs_output", 0.0);
    node_->declare_parameter("pid_pose." + name + ".min_abs_output", 0.0);
    node_->declare_parameter("pid_pose." + name + ".threshold", 0.0);
    node_->declare_parameter("pid_pose." + name + ".stable_count_req", 0);
}

simlib::pid_command_params_t chassis_subsystem::get_pid_command_params(
    const std::string& param_name) noexcept
{
    simlib::pid_command_params_t params{};
    params.target =
        static_cast<float>(node_->get_parameter("pid_pose." + param_name + ".target").as_double());
    params.pid_params.kp =
        static_cast<float>(node_->get_parameter("pid_pose." + param_name + ".kp").as_double());
    params.pid_params.ki =
        static_cast<float>(node_->get_parameter("pid_pose." + param_name + ".ki").as_double());
    params.pid_params.kd =
        static_cast<float>(node_->get_parameter("pid_pose." + param_name + ".kd").as_double());
    params.pid_params.ki_deadzone = static_cast<float>(
        node_->get_parameter("pid_pose." + param_name + ".ki_deadzone").as_double());
    params.pid_params.ki_sum_error_lim = static_cast<float>(
        node_->get_parameter("pid_pose." + param_name + ".ki_sum_error_lim").as_double());
    params.pid_params.max_abs_output = static_cast<float>(
        node_->get_parameter("pid_pose." + param_name + ".max_abs_output").as_double());
    params.pid_params.min_abs_output = static_cast<float>(
        node_->get_parameter("pid_pose." + param_name + ".min_abs_output").as_double());
    params.threshold = static_cast<float>(
        node_->get_parameter("pid_pose." + param_name + ".threshold").as_double());
    params.stable_count_req = static_cast<uint16_t>(
        node_->get_parameter("pid_pose." + param_name + ".stable_count_req").as_int());
    return params;
}


}  // namespace m2::gz_simulation
