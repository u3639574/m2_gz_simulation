#include "m2_gz_simulation/base/simulation_robot.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

#include <chrono>
#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<rclcpp::Node>("fsm_node", "/m2_gz_simulation");
        const double frequency = node->declare_parameter("loop_frequency", 100.0);
        if (!std::isfinite(frequency) || frequency < 1.0 || frequency > 1000.0) {
            throw std::invalid_argument("loop_frequency must be in [1, 1000] Hz");
        }
        trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
        m2::gz_simulation::simulation_robot robot(*node);
        robot.initialize();
        auto timer = node->create_wall_timer(std::chrono::duration<double>(1.0 / frequency),
            [&robot]() { robot.tick(); });
        // All feedback, requests and scheduler ticks execute serially.
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        std::cerr << "m2_gz_simulation: " << error.what() << '\n';
        result = 1;
    }
    if (rclcpp::ok()) { rclcpp::shutdown(); }
    return result;
}
