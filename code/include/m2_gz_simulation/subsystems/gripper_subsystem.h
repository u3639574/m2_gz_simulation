#ifndef M2_GZ_SIMULATION_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H
#define M2_GZ_SIMULATION_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_subsystem.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>

namespace m2::gz_simulation {

class gripper_subsystem : public fsm::subsystem {
public:
    explicit gripper_subsystem(
        std::shared_ptr<rclcpp::Node> node) noexcept;

    ~gripper_subsystem() noexcept = default;

    // true  -> grip / close
    // false -> release / open
    void set_gripper(bool grip) noexcept;

    void gripper_set_grip() noexcept;

    void gripper_set_release() noexcept;

    void gripper_toggle() noexcept;

    bool is_gripped() const noexcept;

    void pub_gripper_pos() noexcept;

    void tick() noexcept override;

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;

    // Fixed configured positions for the pneumatic-cylinder simulation.
    double closed_displacement_{0.0};
    double open_displacement_{0.08};

    // Current targets that are constantly published in tick().
    double left_setpoint_{0.08};
    double right_setpoint_{-0.08};

    bool gripper_gripped_{false};
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H