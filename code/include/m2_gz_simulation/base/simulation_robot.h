#ifndef M2_GZ_SIMULATION_SIMULATION_ROBOT_H
#define M2_GZ_SIMULATION_SIMULATION_ROBOT_H

#include "m2_gz_simulation/base/gz_simulation_define.h"

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode_mux_robot.h"

namespace m2::gz_simulation {

class simulation_robot : public fsm::ros2_ps_mode_mux_robot {
public:
    explicit simulation_robot(
        std::shared_ptr<rclcpp::Node> node, uint32_t hz,
        const std::string& input_topic) noexcept;
    ~simulation_robot() noexcept = default;
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_SIMULATION_ROBOT_H