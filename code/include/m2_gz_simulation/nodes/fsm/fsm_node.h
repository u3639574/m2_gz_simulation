#ifndef M2_GZ_SIMULATION_NODES_FSM_FSM_NODE_H
#define M2_GZ_SIMULATION_NODES_FSM_FSM_NODE_H

#include "m2_gz_simulation/base/simulation_robot.h"
#include "trolly/trolly_macro.h"

#include "rclcpp/rclcpp.hpp"

#include <string>

namespace m2::gz_simulation {

class fsm_node : public rclcpp::Node {
public:
    fsm_node() noexcept;

    ~fsm_node() = default;

    bool init() noexcept;

    void run() noexcept;

private:
    int64_t loop_freq_{100};
    std::string input_topic_{"/keyboard/keypress"};
    std::unique_ptr<simulation_robot> robot_;

    TROLLY_DISALLOW_COPY_AND_MOVE(fsm_node);
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_NODES_FSM_FSM_NODE_H