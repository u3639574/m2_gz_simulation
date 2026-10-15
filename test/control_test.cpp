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
            rclcpp::Parameter("lift.effort_topic", "/controller_test/effort"),
            rclcpp::Parameter("lift.set_pos_topic", "/controller_test/subsystem_setpoint")});
        node = std::make_shared<rclcpp::Node>("controller_test", options);
        trolly::log::use_ros2_logger(node->get_logger(), node->get_clock());
        feedback = std::make_unique<joint_feedback>(node);
        controller = std::make_unique<lift_controller>(node, *feedback);
        joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("/controller_test/joints", 1);
        setpoint_pub = node->create_publisher<std_msgs::msg::Float64>("/controller_test/setpoint", 1);
        velocity_sub = node->create_subscription<std_msgs::msg::Float64>(
            "/controller_test/effort", 10,
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

TEST_F(controller_test, RepeatedSetpointsPreserveBothLoops)
{
    sample(1.0);
    for (int i = 1; i <= 30; ++i) {
        target(0.8); sample(1.0 + i * 0.01); tick();
        EXPECT_LE(std::abs(output), 120.0);
    }
    EXPECT_GT(output, 55.0); // gravity + growing velocity-loop effort
}

TEST_F(controller_test, StaleFeedbackAndClockResetClearEffort)
{
    sample(1.0); target(0.8); sample(1.01); tick(); EXPECT_GT(output, 39.24);
    feedback->advance(2.0); tick(); EXPECT_DOUBLE_EQ(output, 0.0);
    sample(2.01); tick(); EXPECT_GT(output, 39.24);
    feedback->advance(0.0); tick(); EXPECT_DOUBLE_EQ(output, 0.0);
    sample(0.01); tick(); EXPECT_GT(output, 39.24);
}

TEST(Cascade, InnerLoopUsesMeasuredVelocity)
{
    auto make = [] { return cascaded_pid(position_pid(3,0,0,.2,.3,.6),100,10,.1,30,120,39.24); };
    auto stationary=make(), moving=make();
    EXPECT_GT(stationary.compute(.5,0,0,.01), moving.compute(.5,0,.2,.01));
}

TEST(Cascade, HoldsPayloadAgainstGravity)
{
    cascaded_pid pid(position_pid(3,0,.05,.2,.3,.6),100,180,.3,30,120,39.24);
    double position=0, velocity=0;
    constexpr double dt=.005, mass=4.6;
    for (int i=0;i<3000;++i) {
        const double force=pid.compute(.5,position,velocity,dt);
        velocity+=(force-mass*9.81)/mass*dt; position+=velocity*dt;
    }
    EXPECT_NEAR(position,.5,.003); EXPECT_NEAR(velocity,0,.01);
}

TEST(Cascade, VelocityRampEffortLimitsAndReset)
{
    cascaded_pid pid(position_pid(3,0,0,.2,.3,.6),100,180,.3,30,20);
    for (int i=1;i<=100;++i) {
        EXPECT_LE(std::abs(pid.compute(1,0,0,.01)),20);
        EXPECT_LE(pid.velocity_target(), std::min(.3,i*.006)+1e-10);
    }
    pid.reset(); pid.compute(1,0,0,.01); EXPECT_NEAR(pid.velocity_target(),.006,1e-10);
}

TEST(Cascade, RejectsNonfiniteStateAndInvalidTime)
{
    cascaded_pid pid(position_pid(3,0,0,.2,.3,.6),100,180,.3,30,120,39.24);
    EXPECT_EQ(pid.compute(NAN,0,0,.01),0);
    EXPECT_EQ(pid.compute(.5,INFINITY,0,.01),0);
    EXPECT_EQ(pid.compute(.5,0,0,0),0);
    EXPECT_EQ(pid.compute(.5,0,0,-1),0);
    EXPECT_EQ(pid.compute(.5,0,NAN,.01),0);
}
