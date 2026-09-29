#include "m2_gz_simulation/control/lift_controller.h"
#include "m2_gz_simulation/subsystems/lift_subsystem.h"
#include "trolly/ros2/log/trolly_logger_ros2.h"

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <thread>

using namespace m2::gz_simulation;

class ros_environment : public testing::Environment {
    void SetUp() override { int argc = 0; rclcpp::init(argc, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};
auto* const environment = testing::AddGlobalTestEnvironment(new ros_environment);

class controller_test : public testing::Test {
protected:
    void SetUp() override
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("joint_states_topic", "/controller_test/joints"),
            rclcpp::Parameter("lift.p_setpoint_topic", "/controller_test/setpoint"),
            rclcpp::Parameter("lift.cmd_vel_topic", "/controller_test/velocity"),
            rclcpp::Parameter("lift.set_pos_topic", "/controller_test/subsystem_setpoint")});
        node = std::make_shared<rclcpp::Node>("controller_test", options);
        trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
        feedback = std::make_unique<joint_feedback>(node);
        controller = std::make_unique<lift_controller>(node, *feedback);
        joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("/controller_test/joints", 1);
        setpoint_pub = node->create_publisher<std_msgs::msg::Float64>("/controller_test/setpoint", 1);
        velocity_sub = node->create_subscription<std_msgs::msg::Float64>(
            "/controller_test/velocity", 10,
            [this](const std_msgs::msg::Float64& msg) { output = msg.data; ++output_count; });
        executor.add_node(node);
        pump(0.1);
    }
    void TearDown() override
    {
        executor.remove_node(node);
        controller.reset(); feedback.reset();
    }
    void pump(double seconds = 0.02)
    {
        const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        do {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (std::chrono::steady_clock::now() < until);
    }
    void sample(double time, double position = 0.0)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
        msg.name = {"unrelated_joint", "vertical_to_horizontal"};
        msg.position = {99.0, position}; msg.velocity = {0.0, 0.0};
        joint_pub->publish(msg); pump(); feedback->advance(time);
    }
    void target(double value)
    {
        std_msgs::msg::Float64 msg; msg.data = value;
        setpoint_pub->publish(msg); pump();
    }
    void tick() { controller->tick(); pump(); }

    rclcpp::Node::SharedPtr node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::unique_ptr<joint_feedback> feedback;
    std::unique_ptr<lift_controller> controller;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;
    double output{0.0};
    int output_count{0};
};

TEST_F(controller_test, RepeatedSetpointsDoNotRestartAccelerationRamp)
{
    sample(1.0); ASSERT_TRUE(feedback->ready());
    for (int i = 1; i <= 20; ++i) {
        target(0.3); sample(1.0 + i * 0.01); tick();
        EXPECT_LE(std::abs(output), 0.25);
    }
    // Old code stayed at 0.008 m/s because every heartbeat reset previous_output.
    EXPECT_GT(output, 0.12);
    target(-0.3); sample(1.21); tick();
    EXPECT_LT(output, 0.0);
    EXPECT_NEAR(output, -0.008, 1e-6);
}

TEST_F(controller_test, RepeatedTargetRetainsIntegral)
{
    sample(1.0, 0.1); target(0.101);
    double initial = 0.0;
    for (int i = 1; i <= 20; ++i) {
        target(0.101); sample(1.0 + i * 0.01, 0.1); tick();
        if (i == 1) { initial = output; }
    }
    EXPECT_GT(output, initial + 0.00001);
}

TEST_F(controller_test, FeedbackLossAndResetStopVelocity)
{
    sample(1.0); target(0.3); sample(1.01); tick(); EXPECT_GT(output, 0.0);
    for (int i = 1; i <= 6; ++i) { feedback->advance(1.01 + i * 0.05); }
    tick(); EXPECT_FALSE(feedback->ready()); EXPECT_DOUBLE_EQ(output, 0.0);
    sample(1.32); tick(); EXPECT_TRUE(feedback->ready());
    feedback->advance(0.0); tick();
    EXPECT_TRUE(feedback->discontinuity()); EXPECT_DOUBLE_EQ(output, 0.0);
    sample(0.01); tick(); EXPECT_TRUE(feedback->ready());
}

TEST_F(controller_test, InvalidFeedbackStopsAndNaNTargetDoesNotReplaceGoal)
{
    sample(1.0); target(0.3); sample(1.01); tick();
    target(NAN); sample(1.02); tick(); EXPECT_GT(output, 0.0);
    sensor_msgs::msg::JointState msg;
    msg.name = {"vertical_to_horizontal"};  // missing position/velocity
    joint_pub->publish(msg); pump(); feedback->advance(1.03); tick();
    EXPECT_FALSE(feedback->ready()); EXPECT_DOUBLE_EQ(output, 0.0);
}

TEST_F(controller_test, SubsystemPublishesOnlyAfterAnExplicitCommand)
{
    lift_subsystem subsystem(node);
    int received = 0; double last_target = 0;
    auto sub = node->create_subscription<std_msgs::msg::Float64>(
        "/controller_test/subsystem_setpoint", 10,
        [&](const std_msgs::msg::Float64& msg) { ++received; last_target = msg.data; });
    pump(0.1);
    for (int i = 0; i < 3; ++i) { subsystem.tick(); pump(); }
    EXPECT_EQ(received, 0);
    subsystem.set_lift_pos(0.2);
    for (int i = 0; i < 3; ++i) { subsystem.tick(); pump(); }
    EXPECT_GE(received, 3); EXPECT_DOUBLE_EQ(last_target, 0.2);
    subsystem.close();
}

TEST(PositionPid, LimitsVelocityAccelerationAndRejectsInvalidDt)
{
    position_pid pid(3, 0.1, 0.1, 0.2, 0.25, 0.8);
    EXPECT_DOUBLE_EQ(pid.compute(1, 0, 0, 0), 0.0);
    EXPECT_DOUBLE_EQ(pid.compute(1, NAN, 0, 0.01), 0.0);
    EXPECT_NEAR(pid.compute(1, 0, 0, 0.01), 0.008, 1e-12);
    for (int i = 0; i < 100; ++i) { EXPECT_LE(pid.compute(1, 0, 0, 0.01), 0.25); }
}
