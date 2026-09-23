#ifndef M2_GZ_SIMULATION_SUBSYSTEMS_LIFT_SUBSYSTEM_H
#define M2_GZ_SIMULATION_SUBSYSTEMS_LIFT_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_subsystem.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>

namespace m2::gz_simulation {

class lift_subsystem : public fsm::subsystem {
public:
    explicit lift_subsystem(
        std::shared_ptr<rclcpp::Node> node) noexcept;

    ~lift_subsystem() noexcept = default;

    void set_lift_pos(double pos) noexcept;

    double lift_target_pos() const noexcept;

    void pub_lift_pos() noexcept;

    void tick() noexcept override;

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_pos_pub_;

    double min_pos_{-0.4};
    double max_pos_{0.4};

    // This is the position target / p setpoint.
    // Commands update it; tick() republishes it continuously.
    double lift_pos_target_{0.0};
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_SUBSYSTEMS_LIFT_SUBSYSTEM_H