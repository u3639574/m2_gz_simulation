"""Launch the Gazebo world, ROS/Gazebo bridge, FSM, and lift control node."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_nodes(context):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            *([] if gui else ['-s']),
            *([] if paused else ['-r']),
            '-v',
            '3',
            world,
        ],
        name='gazebo',
        output='screen',
        on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo exited')),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[
            {'config_file': LaunchConfiguration('bridge_config')},
        ],
        output='screen',
    )

    fsm = Node(
        package='m2_gz_simulation',
        executable='fsm_node',
        name='fsm_node',
        namespace='m2_gz_simulation',
        parameters=[
            LaunchConfiguration('fsm_config'),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    control = Node(
        package='m2_gz_simulation',
        executable='control_node',
        name='control_node',
        namespace='m2_gz_simulation',
        parameters=[
            LaunchConfiguration('control_config'),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # Gazebo's controller can retain its last command if the ROS-side
    # processes disappear, so stop the world if any required process exits.
    handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='Bridge, FSM, or control node exited'
                        )
                    ),
                ],
            )
        )
        for process in (bridge, fsm, control)
    ]

    return [*handlers, gazebo, bridge, fsm, control]


def generate_launch_description():
    share = FindPackageShare('m2_gz_simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                share, 'model', 'two_arm_robot.sdf',
            ]),
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'ros_gz_bridge.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'fsm_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'fsm_node_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'control_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'control_node_params.yaml',
            ]),
        ),
        OpaqueFunction(function=launch_nodes),
    ])