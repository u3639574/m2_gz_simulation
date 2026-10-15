#include "m2_gz_simulation/nodes/control/control_node.h"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

#include <chrono>
#include <exception>
#include <memory>

namespace m2::gz_simulation {

control_node::control_node() noexcept
    : rclcpp::Node("control_node")
{
    declare_parameter<int64_t>("loop_freq", 100);
}

bool control_node::init() noexcept
{
    loop_freq_ = get_parameter("loop_freq").as_int();

    if (loop_freq_ < 1 || loop_freq_ > 1000) {
        TROLLY_ERROR(
            "[control_node] loop_freq must be in [1, 1000] Hz.");
        return false;
    }

    feedback_ =
        std::make_unique<joint_feedback>(shared_from_this());

    lift_controller_ =
        std::make_unique<lift_controller>(
            shared_from_this(),
            *feedback_);

    yaw_feedback_ = std::make_unique<joint_feedback>(shared_from_this(), "yaw", "base_to_vertical");
    yaw_controller_ = std::make_unique<joint_controller>(shared_from_this(), *yaw_feedback_, "yaw", true);

    timer_ =
        create_wall_timer(
            std::chrono::duration<double>(1.0 / static_cast<double>(loop_freq_)),
            [this]() {
                tick();
            });

    return true;
}

void control_node::tick() noexcept
{
    feedback_->advance(now().seconds());

    lift_controller_->tick();
    yaw_feedback_->advance(now().seconds());
    yaw_controller_->tick();
}

void control_node::run() noexcept
{
    rclcpp::spin(shared_from_this());
}

}  // namespace m2::gz_simulation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<m2::gz_simulation::control_node>();

    trolly::log::use_ros2_logger(
        node->get_logger(),
        node->get_clock());

    TROLLY_INFO("[control_node] Node init begin.");

    if (node->init()) {
        TROLLY_INFO("[control_node] Node init end.");
        node->run();
    } else {
        TROLLY_ERROR(
            "[control_node] Node init fail! Exiting...");
    }

    rclcpp::shutdown();
}
