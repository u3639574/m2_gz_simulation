#include "m2_gz_simulation/commands/position_command.h"
#include <gtest/gtest.h>
#include <limits>

using namespace m2::gz_simulation;

class ros_environment : public testing::Environment {
    void SetUp() override { int argc = 0; rclcpp::init(argc, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};
auto* const environment = testing::AddGlobalTestEnvironment(new ros_environment);

sensor_msgs::msg::JointState frame(double time, double lift = 0.0, double opening = 0.0)
{
    sensor_msgs::msg::JointState message;
    message.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
    // Deliberately reordered, with an unrelated fixed joint included.
    message.name = {"gripper_right_joint", "base_to_vertical", "vertical_to_horizontal", "gripper_left_joint"};
    message.position = {-opening, 99.0, lift, opening};
    message.velocity = {0.0, 0.0, 0.0, 0.0};
    return message;
}

TEST(PositionPid, SaturationDoesNotWindUpAndReverseRespondsImmediately)
{
    position_pid pid(2.0, 1.0, 0.0, 0.2, 5.0, 100.0);
    for (int i = 0; i < 1000; ++i) { EXPECT_DOUBLE_EQ(pid.compute(1, 0, 0, 0.01), 0.2); }
    EXPECT_NEAR(pid.compute(0, 0, 0, 0.01), 0.0, 1e-12);
    EXPECT_LT(pid.compute(-1, 0, 0, 0.01), 0.0);
}
TEST(PositionPid, PreservesIntegralAndLimitsAcceleration)
{
    position_pid pid(0.0, 1.0, 0.0, 1.0, 5.0, 10.0);
    EXPECT_NEAR(pid.compute(1, 0, 0, 0.01), 0.01, 1e-12);
    EXPECT_NEAR(pid.compute(1, 0, 0, 0.01), 0.02, 1e-12);
    pid.reset();
    EXPECT_NEAR(pid.compute(1, 0, 0, 0.01), 0.01, 1e-12);
    position_pid ramp(10, 0, 0, 1, 0, 0.5);
    EXPECT_NEAR(ramp.compute(1, 0, 0, 0.02), 0.01, 1e-12);
}
TEST(PositionPid, InvalidDtAndMeasurementsDoNotAdvanceControl)
{
    position_pid pid(0, 1, 0, 1, 5, 100);
    EXPECT_DOUBLE_EQ(pid.compute(1, 0, 0, 0), 0);
    EXPECT_DOUBLE_EQ(pid.compute(1, 0, 0, -1), 0);
    EXPECT_DOUBLE_EQ(pid.compute(1, NAN, 0, 0.1), 0);
    EXPECT_NEAR(pid.compute(1, 0, 0, 0.1), 0.1, 1e-12);
    EXPECT_THROW(position_pid(1, 0, 0, 0, 0, 1), std::invalid_argument);
}
TEST(PositionPid, DerivativeUsesMeasuredVelocity)
{
    position_pid pid(1, 0, 0.5, 10, 0, 1000);
    EXPECT_NEAR(pid.compute(1, 0, 0.2, 0.01), 0.9, 1e-12);
}
TEST(Feedback, ResolvesJointNamesAndRejectsPartialOrNonfiniteFrames)
{
    rclcpp::Node node("feedback_test");
    joint_feedback feedback(node);
    feedback.receive(frame(1, 0.2, 0.05));
    feedback.advance(1);
    ASSERT_TRUE(feedback.ready());
    EXPECT_DOUBLE_EQ(feedback.lift().position, 0.2);
    EXPECT_DOUBLE_EQ(feedback.right().position, -0.05);
    auto partial = frame(1.01); partial.velocity.clear();
    feedback.receive(partial); feedback.advance(1.01);
    EXPECT_FALSE(feedback.ready());
    auto invalid = frame(1.02); invalid.position[2] = NAN;
    feedback.receive(invalid); feedback.advance(1.02);
    EXPECT_FALSE(feedback.ready());
}
TEST(Feedback, PauseStalenessReorderingAndReset)
{
    rclcpp::Node node("feedback_time_test");
    joint_feedback feedback(node);
    feedback.receive(frame(1)); feedback.advance(1);
    feedback.advance(1); EXPECT_DOUBLE_EQ(feedback.dt(), 0); EXPECT_TRUE(feedback.ready());
    for (int i = 1; i <= 6; ++i) { feedback.advance(1 + 0.05 * i); }
    EXPECT_FALSE(feedback.ready());
    feedback.receive(frame(1.3, 0.2)); feedback.advance(1.3);
    feedback.receive(frame(1.2, 0.1)); feedback.advance(1.3);
    EXPECT_DOUBLE_EQ(feedback.lift().position, 0.2);
    EXPECT_TRUE(feedback.advance(0.0)); EXPECT_FALSE(feedback.ready());
    feedback.receive(frame(0.01)); feedback.advance(0.01); EXPECT_TRUE(feedback.ready());
    EXPECT_TRUE(feedback.advance(0.5)); EXPECT_FALSE(feedback.ready());
}
TEST(Subsystems, EnforceSdfUnitsAndLimits)
{
    rclcpp::Node node("limits_test"); joint_feedback feedback(node);
    lift_subsystem lift(node, feedback); gripper_subsystem gripper(node, feedback);
    EXPECT_TRUE(lift.accepts(-0.4)); EXPECT_TRUE(lift.accepts(0.4));
    EXPECT_FALSE(lift.accepts(0.41)); EXPECT_FALSE(lift.accepts(NAN));
    EXPECT_TRUE(gripper.accepts(0.08)); EXPECT_FALSE(gripper.accepts(-0.01));
    EXPECT_FALSE(gripper.accepts(0.081));
    feedback.receive(frame(1, 0.2, 0.05)); feedback.advance(1);
    gripper.set_target(0.05, 1); EXPECT_TRUE(gripper.at_target());
    lift.set_target(0.2, 2); EXPECT_TRUE(lift.at_target());
}
TEST(Commands, ReplacementIsNotOverwrittenByOldEnd)
{
    rclcpp::Node node("preemption_test"); joint_feedback feedback(node);
    lift_subsystem lift(node, feedback);
    feedback.receive(frame(1, 0.0)); feedback.advance(1);
    auto status = node.create_publisher<std_msgs::msg::String>("status", 10);
    move_lift_command old(lift, 0.1, 10, 0.2, status);
    move_lift_command replacement(lift, 0.3, 10, 0.2, status);
    old.set_instance_id(1); replacement.set_instance_id(2);
    old.initialize(); replacement.initialize(); old.end(true);
    feedback.receive(frame(1.01, 0.3)); feedback.advance(1.01);
    EXPECT_TRUE(lift.at_target());
}
TEST(Commands, CompletionNeedsSettledFeedbackAndPauseDoesNotFinish)
{
    rclcpp::Node node("settle_test"); joint_feedback feedback(node);
    lift_subsystem lift(node, feedback);
    feedback.receive(frame(1, 0.1)); feedback.advance(1);
    auto status = node.create_publisher<std_msgs::msg::String>("status", 10);
    move_lift_command command(lift, 0.1, 10, 0.1, status);
    command.set_instance_id(3); command.initialize();
    EXPECT_FALSE(command.is_finished());
    feedback.receive(frame(1.05, 0.1)); feedback.advance(1.05);
    EXPECT_FALSE(command.is_finished());
    feedback.advance(1.05); EXPECT_FALSE(command.is_finished());
    feedback.receive(frame(1.1, 0.1)); feedback.advance(1.1);
    EXPECT_FALSE(command.is_finished());
    feedback.receive(frame(1.16, 0.1)); feedback.advance(1.16);
    EXPECT_TRUE(command.is_finished()); command.end(false);
    EXPECT_TRUE(lift.at_target());
}
TEST(Commands, TimeoutHoldsMeasuredPositionAndMissingFeedbackAborts)
{
    rclcpp::Node node("timeout_test"); joint_feedback feedback(node);
    lift_subsystem lift(node, feedback);
    feedback.receive(frame(1, 0.0)); feedback.advance(1);
    auto status = node.create_publisher<std_msgs::msg::String>("status", 10);
    move_lift_command command(lift, 0.3, 0.05, 0.01, status);
    command.set_instance_id(4); command.initialize();
    feedback.receive(frame(1.06, 0.02)); feedback.advance(1.06);
    EXPECT_TRUE(command.is_finished()); command.end(false); EXPECT_TRUE(lift.at_target());
    move_lift_command next(lift, 0.3, 10, 0.1, status);
    next.set_instance_id(5); next.initialize();
    auto bad = frame(1.07); bad.name.clear(); feedback.receive(bad); feedback.advance(1.07);
    EXPECT_TRUE(next.is_finished()); next.end(false);
}
