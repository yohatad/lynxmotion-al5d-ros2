"""Simulate the Lynxmotion AL5D in Gazebo (gz sim) with ros2_control and brick management."""

# Replaces al5d_gazebo_control.launch of the ROS 1 package.
#
# Arguments:
#   gui:=true|false        show the Gazebo GUI (default true; forced off when headless)
#   headless:=true|false   run the server only, with headless rendering (default false)
#   paused:=true|false     start the simulation paused (default false)
#   camera_update_rate     rate of the overhead camera in Hz (default 1.0)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'al5d'
NAMESPACE = 'lynxmotion_al5d'


def _gz_sim(context):
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    paused = LaunchConfiguration('paused').perform(context) == 'true'
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world = os.path.join(
        get_package_share_directory('lynxmotion_al5d_description'), 'worlds', 'al5d.sdf')

    args = [] if paused else ['-r']
    if headless or not gui:
        args += ['-s']
    if headless:
        args += ['--headless-rendering']
    args += ['-v', '2', world]

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ' '.join(args), 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    pkg = get_package_share_directory('lynxmotion_al5d_description')
    xacro_file = os.path.join(pkg, 'urdf', 'lynxmotion_al5d.xacro')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' use_sim:=true camera_update_rate:=', LaunchConfiguration('camera_update_rate'),
        ]),
        value_type=str)

    # Let gz sim resolve package://lynxmotion_al5d_description/meshes/... and the models.
    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=NAMESPACE, output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', WORLD_NAME, '-name', 'lynxmotion_al5d',
                   '-topic', f'/{NAMESPACE}/robot_description'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity',
            f'/world/{WORLD_NAME}/remove@ros_gz_interfaces/srv/DeleteEntity',
            f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            f'/{NAMESPACE}/external_vision/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ])

    manager = Node(
        package='lynxmotion_al5d_description', executable='manager_node',
        name='sim_manager', output='screen',
        parameters=[{'world_name': WORLD_NAME, 'use_sim_time': True}])

    pose_relay = Node(
        package='lynxmotion_al5d_description', executable='gz_pose_relay',
        name='gz_pose_relay', output='screen',
        parameters=[{'world_name': WORLD_NAME, 'use_sim_time': True}])

    relay = Node(
        package='lynxmotion_al5d_description', executable='command_relay',
        name='command_relay', output='screen', parameters=[{'use_sim_time': True}])

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[controller, '-c', f'/{NAMESPACE}/controller_manager',
                       '--controller-manager-timeout', '60'])

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    arm_controller = spawner('arm_controller')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('camera_update_rate', default_value='1.0'),
        *resource_paths,
        OpaqueFunction(function=_gz_sim),
        robot_state_publisher,
        bridge,
        spawn_robot,
        manager,
        relay,
        pose_relay,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[arm_controller])),
    ])
