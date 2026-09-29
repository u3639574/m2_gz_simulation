#include "m2_gz_simulation/nodes/fsm/fsm_node.h"

#include "trolly/log/trolly_logger_macro.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

namespace m2::gz_simulation {

fsm_node::fsm_node() noexcept
    : rclcpp::Node("fsm_node")
{
    declare_parameter<int64_t>("loop_freq", 100);
    declare_parameter<std::string>("input_topic", "/keyboard/keypress");
}

bool fsm_node::init() noexcept
{
    loop_freq_ = get_parameter("loop_freq").as_int();
    if (loop_freq_ < 1 || loop_freq_ > 1000) {
        TROLLY_ERROR("[fsm_node] loop_freq must be in [1, 1000] Hz.");
        return false;
    }
    input_topic_ = get_parameter("input_topic").as_string();

    robot_ = std::make_unique<simulation_robot>(
        shared_from_this(), loop_freq_, input_topic_);
    return true;
}

void fsm_node::run() noexcept
{
    robot_->start();
    robot_->close();
    robot_.reset();
}

}  // namespace m2::gz_simulation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<m2::gz_simulation::fsm_node>();

    trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
    TROLLY_INFO("[fsm_node] Node init begin.");

    if (node->init()) {
        TROLLY_INFO("[fsm_node] Node init end.");
        node->run();
    } else {
        TROLLY_ERROR("[fsm_node] Node init fail! Exiting...");
    }
}
