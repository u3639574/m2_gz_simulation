#include <gtest/gtest.h>
#include <thread>
#include "m2_gz_simulation/base/simulation_robot.h"
#include "m2_gz_simulation/base/subsystem_container.h"
#include "m2_gz_simulation/commands/composite/grip_and_lift.h"
#include "m2_gz_simulation/commands/lift/move_lift.h"
#include "m2_gz_simulation/subsystems/gripper_subsystem.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"
#include "m2_gz_simulation/subsystems/yaw_subsystem.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

using namespace m2::gz_simulation;
namespace fsm = m2::fsm;
class ros_environment : public testing::Environment {
    void SetUp() override { int argc = 0; rclcpp::init(argc, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};
auto* const environment = testing::AddGlobalTestEnvironment(new ros_environment);

class keyboard_commands : public testing::Test {
protected:
    void SetUp() override
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("completion.joint_states_topic", "/keyboard_test/joints")});
        node = std::make_shared<rclcpp::Node>("keyboard_command_test", options);
        trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
        robot = std::make_unique<simulation_robot>(node, 100, "/keyboard_test/keys");
        lift = FSM_GET_SUBSYSTEM(g_subsystem_container.lift_subsystem, lift_subsystem);
        grip = FSM_GET_SUBSYSTEM(g_subsystem_container.gripper_subsystem, gripper_subsystem);
        joints = node->create_publisher<sensor_msgs::msg::JointState>("/keyboard_test/joints", 10);
        keys = node->create_publisher<std_msgs::msg::Int32>("/keyboard_test/keys", 10);
        executor.add_node(node);
        robot->initialize(); pump(0.1);
    }
    void TearDown() override { robot->close(); robot.reset(); executor.remove_node(node); }
    void pump(double seconds = 0.03)
    {
        const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        do { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
        while (std::chrono::steady_clock::now() < until);
    }
    void sample(double lift_pos, double finger, double velocity = 0.0, bool omit_right = false)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node->now();
        // Deliberately not SDF array order.
        msg.name = {"gripper_right_joint", "vertical_to_horizontal", "gripper_left_joint"};
        msg.position = {-finger, lift_pos, finger};
        msg.velocity = {velocity, velocity, velocity};
        if (omit_right) { msg.name[0] = "unrelated_joint"; }
        joints->publish(msg); pump();
    }
    void press(keyboard_key key)
    {
        std_msgs::msg::Int32 msg; msg.data = static_cast<int32_t>(key);
        keys->publish(msg); pump(); robot->tick();
    }
    rclcpp::Node::SharedPtr node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::unique_ptr<simulation_robot> robot;
    lift_subsystem* lift;
    gripper_subsystem* grip;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr keys;
};

TEST_F(keyboard_commands, Int32KeyboardMapsFourBasicActionsAndIgnoresUnknown)
{
    press(keyboard_key::W); EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 1.1);
    press(keyboard_key::S); EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    press(keyboard_key::D); EXPECT_TRUE(grip->is_gripped());
    press(keyboard_key::E); EXPECT_FALSE(grip->is_gripped());
    press(static_cast<keyboard_key>(-123)); EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    EXPECT_EQ(static_cast<int32_t>(keyboard_key::Q), 81);
    EXPECT_EQ(static_cast<int32_t>(keyboard_key::UP), 16777235);
}

TEST_F(keyboard_commands, SequentialWaitsForBothFingersBeforeRaising)
{
    lift->set_lift_pos(0.0);
    auto& scheduler = fsm::command_scheduler::instance();
    const auto id = scheduler.schedule_command<close_then_raise_command>();
    robot->tick();
    EXPECT_TRUE(grip->is_gripped());
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    sample(0.0, 0.22); robot->tick();
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    sample(0.0, 0.0, 0.0, true); robot->tick();
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    sample(0.0, 0.0); robot->tick();
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 1.1);
    EXPECT_TRUE(scheduler.is_scheduled(id));
    sample(1.1, 0.0); robot->tick();
    EXPECT_FALSE(scheduler.is_scheduled(id));
}

TEST_F(keyboard_commands, ParallelStartsTogetherAndWaitsForBoth)
{
    lift->set_lift_pos(1.1); grip->set_gripper(true);
    auto& scheduler = fsm::command_scheduler::instance();
    const auto id = scheduler.schedule_command<open_and_lower_command>();
    robot->tick();
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    EXPECT_FALSE(grip->is_gripped());
    sample(1.1, 0.22); robot->tick();
    EXPECT_TRUE(scheduler.is_scheduled(id));
    sample(0.0, 0.22); robot->tick();
    EXPECT_FALSE(scheduler.is_scheduled(id));
}

TEST_F(keyboard_commands, OppositeCommandCancelsSequenceBeforeItsRaiseStage)
{
    lift->set_lift_pos(0.0);
    press(keyboard_key::Q);
    press(keyboard_key::Q); // Repeated key cannot restart the active sequence.
    press(keyboard_key::S);
    sample(0.0, 0.0); robot->tick(); robot->tick();
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
    press(keyboard_key::A);
    EXPECT_FALSE(grip->is_gripped());
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), 0.0);
}

TEST_F(keyboard_commands, CompletionRequiresFreshStationaryFeedbackAfterRequest)
{
    lift->set_lift_pos(1.1);
    EXPECT_FALSE(lift->at_target());
    sample(1.1, 0.0, 0.1); EXPECT_FALSE(lift->at_target());
    sample(1.1, 0.0); EXPECT_TRUE(lift->at_target());
    pump(0.3); EXPECT_FALSE(lift->at_target());
    sample(1.1, 0.0); EXPECT_TRUE(lift->at_target());
    lift->set_lift_pos(1.1); EXPECT_FALSE(lift->at_target());
}

TEST_F(keyboard_commands, YawAcceptsDegreesAndMapsPresets)
{
    auto* yaw = FSM_GET_SUBSYSTEM(g_subsystem_container.yaw_subsystem, yaw_subsystem);
    ASSERT_TRUE(yaw->set_angle(359.0)); EXPECT_NEAR(yaw->target_degrees(),359,1e-10);
    EXPECT_FALSE(yaw->set_angle(360.0)); EXPECT_FALSE(yaw->set_angle(-1.0));
    EXPECT_FALSE(yaw->set_angle(NAN)); EXPECT_NEAR(yaw->target_degrees(),359,1e-10);
    press(keyboard_key::F); EXPECT_NEAR(yaw->target_degrees(),180,1e-10);
    press(keyboard_key::R); EXPECT_DOUBLE_EQ(yaw->target_degrees(),0);
}

TEST_F(keyboard_commands, ArrowKeysDoNotCommandTheArm)
{
    lift->set_lift_pos(.5);
    press(keyboard_key::UP); press(keyboard_key::DOWN);
    press(keyboard_key::LEFT); press(keyboard_key::RIGHT);
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(),.5); EXPECT_FALSE(grip->is_gripped());
}

TEST_F(keyboard_commands, GraspCompletionRequiresBothFingersOnTheSameBlock)
{
    auto left=node->create_publisher<ros_gz_interfaces::msg::Contacts>("gripper/left/contacts",10);
    auto right=node->create_publisher<ros_gz_interfaces::msg::Contacts>("gripper/right/contacts",10);
    pump(.1); grip->set_gripper(true);
    auto message=[](const std::string& name) {
        ros_gz_interfaces::msg::Contacts msg;
        ros_gz_interfaces::msg::Contact c; c.collision1.name="two_arm_robot::gripper_left::collision";
        c.collision2.name=name+"::block::collision"; msg.contacts.push_back(c); return msg;
    };
    left->publish(message("earth_block_1")); pump(.12);
    EXPECT_FALSE(grip->at_target());
    right->publish(message("earth_block_2")); pump(.12);
    EXPECT_FALSE(grip->at_target());
    left->publish(message("earth_block_1")); right->publish(message("earth_block_1")); pump(.12);
    EXPECT_TRUE(grip->at_target());
    pump(.3); EXPECT_FALSE(grip->at_target());
}
