#include "m2_gz_simulation/base/simulation_robot.h"

namespace m2::gz_simulation {
simulation_robot::simulation_robot(rclcpp::Node& node) : node_(node), feedback_(node)
{
    command_timeout_ = node.declare_parameter("command_timeout", 10.0);
    settle_time_ = node.declare_parameter("settle_time", 0.2);
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
        !std::isfinite(settle_time_) || settle_time_ < 0.0 || settle_time_ >= command_timeout_) {
        throw std::invalid_argument("Invalid command_timeout or settle_time");
    }
    // Construct outside the scheduler's noexcept factory so bad configuration
    // produces a useful startup error instead of std::terminate.
    auto lift = std::make_unique<lift_subsystem>(node, feedback_);
    auto gripper = std::make_unique<gripper_subsystem>(node, feedback_);
    lift_ = lift.get(); gripper_ = gripper.get();
    auto& scheduler = fsm::command_scheduler::instance();
    scheduler.register_subsystem(std::move(lift));
    scheduler.register_subsystem(std::move(gripper));

    status_ = node.create_publisher<std_msgs::msg::String>("command_status", 10);
    lift_request_ = node.create_subscription<std_msgs::msg::Float64>("lift/target_position", 1,
        [this](std_msgs::msg::Float64::ConstSharedPtr message) {
            if (validate_request(*lift_, message->data)) {
                fsm::command_scheduler::instance().schedule_command<move_lift_command>(
                    *lift_, message->data, command_timeout_, settle_time_, status_);
            }
        });
    gripper_request_ = node.create_subscription<std_msgs::msg::Float64>("gripper/target_displacement", 1,
        [this](std_msgs::msg::Float64::ConstSharedPtr message) {
            if (validate_request(*gripper_, message->data)) {
                fsm::command_scheduler::instance().schedule_command<set_gripper_command>(
                    *gripper_, message->data, command_timeout_, settle_time_, status_);
            }
        });
    stop_service_ = node.create_service<std_srvs::srv::Trigger>("stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            stop();
            response->success = true;
            response->message = "Commands cancelled; holding measured positions where available";
        });
}
void simulation_robot::initialize() noexcept
{ RCLCPP_INFO(node_.get_logger(), "Waiting for Gazebo clock and complete joint feedback"); }

simulation_robot::~simulation_robot() noexcept
{
    // The scheduler owns subsystems until static destruction. Release their ROS
    // entities while the node still exists, and drain command-owned publishers.
    lift_->close(); gripper_->close();
    fsm::command_scheduler::instance().cancel_all_commands();
    fsm::robot::tick();
}

bool simulation_robot::validate_request(position_subsystem& subsystem, double target)
{
    if (subsystem.ready() && subsystem.accepts(target)) { return true; }
    std_msgs::msg::String message;
    message.data = subsystem.name() + ": rejected (invalid target or unavailable feedback)";
    status_->publish(message);
    RCLCPP_WARN(node_.get_logger(), "%s", message.data.c_str());
    return false;
}
void simulation_robot::stop() noexcept
{
    fsm::command_scheduler::instance().cancel_all_commands();
    lift_->hold();
    gripper_->hold();
}
void simulation_robot::tick() noexcept
{
    if (feedback_.advance(node_.now().seconds())) {
        RCLCPP_WARN(node_.get_logger(), "Simulation time discontinuity: resetting control and cancelling commands");
        fsm::command_scheduler::instance().cancel_all_commands();
        lift_->reset(); gripper_->reset();
    }
    fsm::robot::tick();
}
}  // namespace m2::gz_simulation
