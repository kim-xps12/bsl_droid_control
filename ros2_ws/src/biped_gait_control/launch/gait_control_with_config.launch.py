"""
Launch file for biped gait pattern generator with YAML config file.

This launch file loads parameters from a YAML configuration file.

Usage:
    ros2 launch biped_gait_control gait_control_with_config.launch.py

    With custom config:
    ros2 launch biped_gait_control gait_control_with_config.launch.py config_file:=/path/to/config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('biped_gait_control')

    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'gait_params.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML configuration file'
    )

    # Gait pattern generator node with config file
    # Publishes directly to /joint_states (standard ROS 2 convention)
    gait_generator_node = Node(
        package='biped_gait_control',
        executable='gait_pattern_generator',
        name='gait_pattern_generator',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        gait_generator_node,
    ])
