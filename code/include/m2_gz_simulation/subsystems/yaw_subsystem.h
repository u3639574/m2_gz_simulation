#pragma once
#include "m2_fsm_cpp/base/fsm_subsystem.h"
#include "m2_gz_simulation/subsystems/motion_completion.h"
#include "std_msgs/msg/float64.hpp"

namespace m2::gz_simulation {
class yaw_subsystem : public fsm::subsystem {
public:
    explicit yaw_subsystem(std::shared_ptr<rclcpp::Node> node);
    // Degrees, chassis-relative: zero points the arm along chassis +X.
    // Reject out-of-range/NaN instead of silently changing a requested angle.
    bool set_angle(double degrees) noexcept;
    double target_degrees() const noexcept { return target_ * 180.0 / 3.141592653589793; }
    bool at_target() const noexcept { return completion_ && completion_->at_target({target_}); }
    void tick() noexcept override;
    void close() noexcept { completion_.reset(); publisher_.reset(); node_.reset(); }
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<motion_completion> completion_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    double target_{0.0};
};
}  // namespace m2::gz_simulation
