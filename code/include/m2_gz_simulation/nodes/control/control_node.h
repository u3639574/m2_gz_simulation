#ifndef M2_GZ_SIMULATION_NODES_CONTROL_CONTROL_NODE_H
#define M2_GZ_SIMULATION_NODES_CONTROL_CONTROL_NODE_H

#include "m2_gz_simulation/control/joint_feedback.h"
#include "m2_gz_simulation/control/lift_controller.h"

#include "trolly/trolly_macro.h"

#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace m2::gz_simulation {

class control_node : public rclcpp::Node {
public:
    control_node() noexcept;

    bool init() noexcept;

    void run() noexcept;

private:
    void tick() noexcept;

    int64_t loop_freq_{100};

    std::unique_ptr<joint_feedback> feedback_;
    std::unique_ptr<lift_controller> lift_controller_;
    std::unique_ptr<joint_feedback> yaw_feedback_;
    std::unique_ptr<joint_controller> yaw_controller_;

    rclcpp::TimerBase::SharedPtr timer_;

    TROLLY_DISALLOW_COPY_AND_MOVE(control_node);
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_NODES_CONTROL_CONTROL_NODE_H