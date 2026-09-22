"""Launch the complete arm world, topic bridge, and command-based controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_nodes(context):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    # Invoke directly without a shell so launch signals reach Gazebo itself.
    # The upstream gz_sim.launch.py shell wrapper can leave the server running
    # when another process exits and launch initiates shutdown.
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', *([] if gui else ['-s']),
             *([] if paused else ['-r']), '-v', '3', world],
        name='gazebo', output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        parameters=[{'config_file': LaunchConfiguration('bridge_config')}], output='screen',
    )
    controller = Node(
        package='m2_gz_simulation', executable='fsm_node', name='fsm_node',
        namespace='m2_gz_simulation',
        parameters=[LaunchConfiguration('controller_config'), {'use_sim_time': True}],
        output='screen',
    )
    # If a controller or bridge exits, terminate the world too: Gazebo's built-in
    # joint controller otherwise retains the last velocity command indefinitely.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='Controller or bridge exited'))],
    )) for process in (bridge, controller)]
    return [*handlers, gazebo, bridge, controller]


def generate_launch_description():
    share = FindPackageShare('m2_gz_simulation')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('paused', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            share, 'model', 'two_arm_robot.sdf'])),
        DeclareLaunchArgument('bridge_config', default_value=PathJoinSubstitution([
            share, 'config', 'ros_gz_bridge.yaml'])),
        DeclareLaunchArgument('controller_config', default_value=PathJoinSubstitution([
            share, 'config', 'fsm_node_params.yaml'])),
        OpaqueFunction(function=launch_nodes),
    ])
