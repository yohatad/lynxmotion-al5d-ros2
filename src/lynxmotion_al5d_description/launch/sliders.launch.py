"""Move the simulated AL5D with sliders: a joint slider window that drives the Gazebo arm."""

# Start the simulation first (ros2 launch lynxmotion_al5d_description sim.launch.py), then this.
# The slider window publishes joint states on /arm_sliders/joint_states; slider_command turns a
# change of any slider into a command on /lynxmotion_al5d/joints_positions/commands.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('lynxmotion_al5d_description')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 os.path.join(pkg, 'urdf', 'lynxmotion_al5d.xacro')]),
        value_type=str)

    sliders = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        name='arm_sliders', output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', '/arm_sliders/joint_states')])

    command = Node(
        package='lynxmotion_al5d_description', executable='slider_command',
        name='slider_command', output='screen')

    return LaunchDescription([sliders, command])
