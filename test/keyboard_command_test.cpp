#include <gtest/gtest.h>
#include <thread>
#include "m2_gz_simulation/commands/chassis/move_forward_until.h"
#include "m2_gz_simulation/commands/composite/earth_block_transfer.h"
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
        options.parameter_overrides({rclcpp::Parameter("completion.joint_states_topic", "/keyboard_test/joints"),
            rclcpp::Parameter("lift.motion_profile", "pid")});
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

TEST_F(keyboard_commands, ChassisKeysDoNotCommandTheArm)
{
    lift->set_lift_pos(.5);
    press(keyboard_key::UP); press(keyboard_key::DOWN);
    press(keyboard_key::LEFT); press(keyboard_key::RIGHT);
    press(keyboard_key::Z); press(keyboard_key::C); press(keyboard_key::X);
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

TEST_F(keyboard_commands, SineProfileStartsAtFeedbackAndParameterChangesApplyToNextCommand)
{
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("lift.motion_profile", "sine")).successful);
    sample(0.0, 0.22);
    auto& scheduler = fsm::command_scheduler::instance();
    const auto id = scheduler.schedule_command<set_lift_pos_command>(0.1);
    robot->tick();
    EXPECT_LT(lift->lift_target_pos(), 0.005);
    double previous = lift->lift_target_pos();
    for (int i = 0; i < 4; ++i) {
        pump(.08); sample(previous, .22); robot->tick();
        EXPECT_GE(lift->lift_target_pos(), previous);
        EXPECT_LT(lift->lift_target_pos(), .1);
        previous = lift->lift_target_pos();
    }
    EXPECT_GT(previous, .005);
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("lift.motion_profile", "pid")).successful);
    sample(previous, .22); robot->tick();
    EXPECT_LT(lift->lift_target_pos(), .1); // Existing command retains its chosen profile.
    scheduler.schedule_command<set_lift_pos_command>(.8);
    robot->tick();
    EXPECT_FALSE(scheduler.is_scheduled(id));
    EXPECT_DOUBLE_EQ(lift->lift_target_pos(), .8);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("lift.motion_profile", "unknown")).successful);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("lift.sine.rise.velocity", 0.0)).successful);
}

TEST(SineTrajectory, SignedEndpointsZeroDistanceAndSmoothReference)
{
    m2::simlib::SinTrajectory trajectory;
    for (const double distance : {0.0, 1e-8, .1, -.1, 1.1}) {
        trajectory.compute_coefficients(distance, .35, .6, .6, 0.0);
        const auto duration = trajectory.get_total_duration();
        EXPECT_TRUE(std::isfinite(duration));
        EXPECT_NEAR(trajectory.get_output(0).position, 0.0, 1e-12);
        EXPECT_NEAR(trajectory.get_output(duration + 1).position, distance, 1e-12);
        EXPECT_DOUBLE_EQ(trajectory.get_output(duration + 1).velocity, 0.0);
        double previous = 0.0;
        for (double t = 0; t < duration; t += .005) {
            const auto output = trajectory.get_output(t);
            EXPECT_TRUE(std::isfinite(output.acceleration));
            EXPECT_LE(std::abs(output.velocity), .36);
            EXPECT_LE(std::abs(output.acceleration), .62);
            EXPECT_GE(std::abs(output.position) + 1e-12, previous);
            previous = std::abs(output.position);
        }
    }
}

TEST_F(keyboard_commands, CommonlibPriorityZeroRelinquishesAndStaleSourcesStop)
{
    auto* base = FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, m2::simlib::move_base_subsystem);
    geometry_msgs::msg::Twist output;
    auto out = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist& msg) { output = msg; });
    auto planner = node->create_publisher<geometry_msgs::msg::Twist>("move_base/cmd_vel", 10);
    pump(.1);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = .1; base->set_fsm_cmd_vel(cmd);
    cmd.linear.x = .2; planner->publish(cmd); pump(.01);
    cmd.linear.x = .3; base->set_keyboard_cmd_vel(cmd);
    base->tick(); pump(.01); EXPECT_DOUBLE_EQ(output.linear.x, .3);
    base->set_keyboard_cmd_vel(geometry_msgs::msg::Twist{});
    base->tick(); pump(.01); EXPECT_DOUBLE_EQ(output.linear.x, .2);
    planner->publish(geometry_msgs::msg::Twist{}); pump(.01);
    base->tick(); pump(.01); EXPECT_DOUBLE_EQ(output.linear.x, .1);
    pump(.25); base->tick(); pump(.01); EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    cmd.linear.x = .2; base->set_fsm_cmd_vel(cmd); base->stop();
    planner->publish(cmd); pump(.01);
    base->tick(); pump(.01); EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
}

TEST_F(keyboard_commands, ForwardCommandUsesGroundTruthYawAndTimeout)
{
    auto* base = FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, m2::simlib::move_base_subsystem);
    auto* loc = FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem, m2::simlib::localization_subsystem);
    auto odom = node->create_publisher<nav_msgs::msg::Odometry>("chassis_odom", 10);
    geometry_msgs::msg::Twist output;
    auto out = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist& msg) { output = msg; });
    const auto feedback = [&](double yaw) {
        nav_msgs::msg::Odometry msg; msg.header.stamp = node->now(); msg.header.frame_id = "odom";
        msg.pose.pose.orientation.z = std::sin(yaw / 2);
        msg.pose.pose.orientation.w = std::cos(yaw / 2);
        msg.pose.pose.position.x = 1; msg.pose.pose.position.y = 2;
        odom->publish(msg); pump(.02);
    };
    pump(.1); feedback(.5); ASSERT_TRUE(loc->fresh());
    EXPECT_NEAR(loc->distance_to_point(4,6), 5.0, 1e-6);
    node->set_parameter(rclcpp::Parameter("pid_pose.move_forward_yaw_params.kp", 2.0));
    node->set_parameter(rclcpp::Parameter("pid_pose.move_forward_yaw_params.max_abs_output", .5));
    move_forward_until_command command(.15f, 250);
    command.initialize(); feedback(.6); command.execute(); base->tick(); pump(.01);
    EXPECT_NEAR(output.linear.x, .15, 1e-6);
    EXPECT_NEAR(output.angular.z, -.2, .001);
    EXPECT_DOUBLE_EQ(output.angular.x, 0.0);
    EXPECT_FALSE(command.is_finished());
    pump(.25); feedback(.6); command.execute(); base->tick(); pump(.01);
    EXPECT_TRUE(command.is_finished()); EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    command.end(false);
    move_forward_until_command stale(.15f, 2000);
    stale.initialize(); pump(.35); stale.execute(); base->tick(); pump(.01);
    EXPECT_TRUE(stale.is_finished()); EXPECT_DOUBLE_EQ(output.linear.x, 0.0);
    stale.end(true);
}

TEST_F(keyboard_commands, PidPoseDoesNotAdvanceSequenceOnStaleFeedback)
{
    auto* base = FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem, m2::simlib::move_base_subsystem);
    auto* loc = FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem, m2::simlib::localization_subsystem);
    auto odom = node->create_publisher<nav_msgs::msg::Odometry>("chassis_odom", 10);
    geometry_msgs::msg::Twist output;
    auto out = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        [&](const geometry_msgs::msg::Twist& msg) { output = msg; });
    pump(.1);
    nav_msgs::msg::Odometry msg; msg.header.stamp=node->now(); msg.pose.pose.orientation.w=1;
    odom->publish(msg); pump(.02); ASSERT_TRUE(loc->fresh());
    m2::simlib::pid_command_params_t axis{};
    axis.target=1; axis.threshold=.01; axis.pid_params.kp=2; axis.pid_params.max_abs_output=.5;
    axis.stable_count_req=3;
    m2::simlib::pid_pose_limits_t limits; limits.acceleration={.2,.1,.3};
    bool failed=false; limits.on_failure=[&](const std::string&) { failed=true; };
    m2::simlib::pid_pose_command command(axis,axis,axis,loc,base,limits);
    command.initialize(); pump(.02); command.execute(); base->tick(); pump(.01);
    EXPECT_GT(output.linear.x,0); EXPECT_LE(output.linear.x,.01);
    EXPECT_GT(output.linear.y,0); EXPECT_LE(output.linear.y,.005);
    EXPECT_FALSE(command.is_finished());
    pump(.35); command.execute(); base->tick(); pump(.01);
    EXPECT_TRUE(failed); EXPECT_FALSE(command.is_finished()); EXPECT_DOUBLE_EQ(output.linear.x,0);
    // Fresh feedback after a failure must not silently resume the unfinished move.
    msg.header.stamp=node->now(); odom->publish(msg); pump(.01);
    command.execute(); base->tick(); pump(.01);
    EXPECT_FALSE(command.is_finished()); EXPECT_DOUBLE_EQ(output.linear.x,0);
    command.end(true);
}

TEST_F(keyboard_commands, TransferReservesMechanismsAndStopCancelsWholeGroup)
{
    auto* chassis = FSM_GET_SUBSYSTEM(g_subsystem_container.chassis_subsystem, chassis_subsystem);
    press(keyboard_key::T); robot->tick(); // Scheduler initializes first, then executes on the next tick.
    EXPECT_TRUE(chassis->failed()); EXPECT_FALSE(grip->is_gripped());
    press(keyboard_key::D); EXPECT_FALSE(grip->is_gripped());
    press(keyboard_key::B); EXPECT_TRUE(chassis->failed()); // Rejected action cannot clear the fault.
    press(keyboard_key::X); // Cancellation releases the whole group's subsystem requirements.
    press(keyboard_key::D); EXPECT_TRUE(grip->is_gripped());
}

TEST_F(keyboard_commands, PidPoseCompletionRequiresPositionYawAndStationaryFeedback)
{
    auto* base=FSM_GET_SUBSYSTEM(g_subsystem_container.move_base_subsystem,m2::simlib::move_base_subsystem);
    auto* loc=FSM_GET_SUBSYSTEM(g_subsystem_container.localization_subsystem,m2::simlib::localization_subsystem);
    auto odom=node->create_publisher<nav_msgs::msg::Odometry>("chassis_odom",10); pump(.1);
    m2::simlib::pid_command_params_t axis{};
    axis.threshold=.01; axis.stable_count_req=2; axis.pid_params.kp=1; axis.pid_params.max_abs_output=.5;
    m2::simlib::pid_pose_command command(axis,axis,axis,loc,base);
    command.initialize();
    auto feedback=[&](double x, double velocity, double yaw) {
        nav_msgs::msg::Odometry msg; msg.header.stamp=node->now();
        msg.pose.pose.position.x=x; msg.pose.pose.orientation.w=std::cos(yaw/2);
        msg.pose.pose.orientation.z=std::sin(yaw/2); msg.twist.twist.linear.x=velocity;
        odom->publish(msg); pump(.01); command.execute();
    };
    feedback(0,.1,0); feedback(0,.1,0); EXPECT_FALSE(command.is_finished());
    feedback(.1,0,0); feedback(.1,0,0); EXPECT_FALSE(command.is_finished());
    feedback(0,0,.1); feedback(0,0,.1); EXPECT_FALSE(command.is_finished());
    feedback(0,0,0); EXPECT_FALSE(command.is_finished());
    feedback(0,0,0); EXPECT_TRUE(command.is_finished());
    command.end(false);
}
