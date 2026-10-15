#ifndef M2_GZ_SIMULATION_SUBSYSTEMS_LIFT_SUBSYSTEM_H
#define M2_GZ_SIMULATION_SUBSYSTEMS_LIFT_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_subsystem.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>
#include "m2_gz_simulation/subsystems/motion_completion.h"

namespace m2::gz_simulation {

class lift_subsystem : public fsm::subsystem {
public:
    explicit lift_subsystem(
        std::shared_ptr<rclcpp::Node> node) noexcept;

    ~lift_subsystem() noexcept = default;

    void set_lift_pos(double pos) noexcept;

    double lift_target_pos() const noexcept;
    double top_position() const noexcept { return max_pos_; }
    double bottom_position() const noexcept { return bottom_pos_; }
    bool at_target() const noexcept
    { return has_target_ && completion_ && completion_->at_target({lift_pos_target_}); }

    void pub_lift_pos() noexcept;

    void tick() noexcept override;
    void close() noexcept { completion_.reset(); lift_pos_pub_.reset(); node_.reset(); }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<motion_completion> completion_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_pos_pub_;

    double bottom_pos_{0.0};
    double min_pos_{0.0};
    double max_pos_{1.1};

    // This is the position target / p setpoint.
    // Commands update it; tick() republishes it continuously.
    double lift_pos_target_{0.0};
    bool has_target_{false};
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_SUBSYSTEMS_LIFT_SUBSYSTEM_H
