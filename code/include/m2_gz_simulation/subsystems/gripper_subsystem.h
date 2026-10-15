#ifndef M2_GZ_SIMULATION_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H
#define M2_GZ_SIMULATION_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H

#include "m2_fsm_cpp/base/fsm_subsystem.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "m2_gz_simulation/subsystems/motion_completion.h"

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
    bool at_target() const noexcept
    { return (completion_ && completion_->at_target({left_setpoint_, right_setpoint_})) || (gripper_gripped_ && holding_block()); }
    bool holding_block() const noexcept;

    void pub_gripper_pos() noexcept;

    void tick() noexcept override;
    void close() noexcept
    {
        left_contact_sub_.reset(); right_contact_sub_.reset();
        completion_.reset();
        left_pos_pub_.reset();
        right_pos_pub_.reset();
        node_.reset();
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<motion_completion> completion_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos_pub_;

    // Fixed configured positions for the pneumatic-cylinder simulation.
    double closed_displacement_{0.0};
    double open_displacement_{0.22};

    // Current targets that are constantly published in tick().
    double left_setpoint_{0.22};
    double right_setpoint_{-0.22};

    struct contact_state {
        std::string block;
        std::chrono::steady_clock::time_point since{}, received{};
    };
    void receive_contact(const ros_gz_interfaces::msg::Contacts& msg, contact_state& state) noexcept;
    contact_state left_contact_, right_contact_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr left_contact_sub_, right_contact_sub_;
    bool gripper_gripped_{false};
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_SUBSYSTEMS_GRIPPER_SUBSYSTEM_H
