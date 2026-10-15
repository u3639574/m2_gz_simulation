#ifndef M2_GZ_SIMULATION_SIMULATION_ROBOT_H
#define M2_GZ_SIMULATION_SIMULATION_ROBOT_H

#include "m2_gz_simulation/base/gz_simulation_define.h"

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_robot.h"
#include "m2_gz_simulation/modes/test_mode.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"


namespace m2::gz_simulation {

class simulation_robot : public fsm::ros2_robot {
public:
    explicit simulation_robot(
        std::shared_ptr<rclcpp::Node> node, uint32_t hz,
        const std::string& input_topic) noexcept;
    ~simulation_robot() noexcept = default;
    void close() noexcept;
    void initialize() noexcept override;
    void tick() noexcept override;

private:
    bool closed_{false};
    test_mode mode_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_request_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keyboard_sub_;
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_SIMULATION_ROBOT_H
