"""Show the AL5D model in RViz with joint sliders."""

# Replaces al5d.launch of the ROS 1 package.
#
# Arguments:
#   gui:=true|false    start joint_state_publisher_gui (default true; false = plain publisher)
#   rviz:=true|false   start RViz (default true)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('lynxmotion_al5d_description')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 os.path.join(pkg, 'urdf', 'lynxmotion_al5d.xacro')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui')),
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(LaunchConfiguration('gui')),
             parameters=[{'robot_description': robot_description}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(pkg, 'rviz', 'urdf.rviz')]),
    ])
