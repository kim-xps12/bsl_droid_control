"""
Launch file for biped gait pattern generator.

This launch file starts the gait pattern generator node with configurable parameters.

Usage:
    ros2 launch biped_gait_control gait_control.launch.py

    With custom parameters:
    ros2 launch biped_gait_control gait_control.launch.py step_frequency:=1.0 step_height:=0.05
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

    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publishing rate in Hz'
    )

    step_height_arg = DeclareLaunchArgument(
        'step_height',
        default_value='0.04',
        description='Foot lift height in meters'
    )

    step_length_arg = DeclareLaunchArgument(
        'step_length',
        default_value='0.08',
        description='Stride length in meters'
    )

    step_frequency_arg = DeclareLaunchArgument(
        'step_frequency',
        default_value='0.5',
        description='Walking frequency in Hz'
    )

    leg_extension_ratio_arg = DeclareLaunchArgument(
        'leg_extension_ratio',
        default_value='0.90',
        description='Standing leg extension ratio (0.0-1.0)'
    )

    thigh_length_arg = DeclareLaunchArgument(
        'thigh_length',
        default_value='0.18',
        description='Thigh link length in meters'
    )

    shank_length_arg = DeclareLaunchArgument(
        'shank_length',
        default_value='0.20',
        description='Shank link length in meters'
    )

    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable/disable walking pattern generation'
    )

    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='false',
        description='Use YAML config file instead of launch arguments'
    )

    # Gait pattern generator node
    # Publishes directly to /joint_states (standard ROS 2 convention)
    gait_generator_node = Node(
        package='biped_gait_control',
        executable='gait_pattern_generator',
        name='gait_pattern_generator',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'step_height': LaunchConfiguration('step_height'),
            'step_length': LaunchConfiguration('step_length'),
            'step_frequency': LaunchConfiguration('step_frequency'),
            'leg_extension_ratio': LaunchConfiguration('leg_extension_ratio'),
            'thigh_length': LaunchConfiguration('thigh_length'),
            'shank_length': LaunchConfiguration('shank_length'),
            'enabled': LaunchConfiguration('enabled'),
        }]
    )

    return LaunchDescription([
        publish_rate_arg,
        step_height_arg,
        step_length_arg,
        step_frequency_arg,
        leg_extension_ratio_arg,
        thigh_length_arg,
        shank_length_arg,
        enabled_arg,
        use_config_arg,
        gait_generator_node,
    ])
