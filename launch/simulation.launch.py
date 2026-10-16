"""Launch the Gazebo world, ROS/Gazebo bridge, FSM, and lift control node."""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
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
    command_source = LaunchConfiguration('command_source').perform(context)

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            *(['--gui-config', LaunchConfiguration('gui_config').perform(context)] if gui else ['-s', '--headless-rendering']),
            *([] if paused else ['-r']),
            '-v',
            '3',
            world,
        ],
        additional_env={'GZ_SIM_RESOURCE_PATH': os.pathsep.join(filter(None, [
            PathJoinSubstitution([FindPackageShare('m2_gz_simulation'), 'model']).perform(context),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')])),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(filter(None, [
                os.path.join(get_package_prefix('m2_gz_simulation'), 'lib'),
                os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))},
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

    # Exactly one producer at each control layer. Terminal effort tests must
    # not compete with control_node; terminal position tests must not compete
    # with the FSM's continuously published setpoint.
    processes = [bridge]
    if command_source == 'fsm':
        processes.append(fsm)
    if command_source != 'effort':
        processes.append(control)

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
        for process in processes
    ]

    return [LogInfo(msg=f'Command source: {command_source}'), *handlers, gazebo, *processes]


def generate_launch_description():
    share = FindPackageShare('m2_gz_simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'command_source', default_value='fsm', choices=['fsm', 'position', 'effort'],
            description='fsm: commands/subsystems; position: direct p_setpoint; '
                        'effort: direct cmd_effort (no FSM or ROS PID)',
        ),
        DeclareLaunchArgument(
            'gui_config',
            default_value=PathJoinSubstitution([share, 'config', 'gazebo_gui.config']),
            description='Gazebo GUI configuration including keyboard publisher',
        ),
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
