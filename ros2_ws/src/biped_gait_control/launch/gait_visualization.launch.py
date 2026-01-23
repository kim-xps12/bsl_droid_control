"""
Launch file for gait pattern generator with RViz visualization.

This launch file combines:
- gait_pattern_generator: Generates walking joint commands
- robot_state_publisher: Publishes TF from URDF
- rviz2: Visualization

The gait_pattern_generator publishes to /joint_states which
robot_state_publisher subscribes to for TF calculation.

Usage:
    ros2 launch biped_gait_control gait_visualization.launch.py

    With custom gait parameters:
    ros2 launch biped_gait_control gait_visualization.launch.py step_frequency:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    biped_description_share = FindPackageShare('biped_description')
    biped_gait_control_share = FindPackageShare('biped_gait_control')

    # URDF file path
    urdf_file = PathJoinSubstitution([
        biped_description_share, 'urdf', 'biped_digitigrade.urdf.xacro'
    ])

    # RViz config file
    rviz_config = PathJoinSubstitution([
        biped_description_share, 'rviz', 'biped_display.rviz'
    ])

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

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

    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable/disable walking pattern generation'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

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
            'thigh_length': 0.18,
            'shank_length': 0.20,
            'enabled': LaunchConfiguration('enabled'),
        }]
    )

    # Robot state publisher: Publishes TF from URDF and /joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    # RViz2: Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        publish_rate_arg,
        step_height_arg,
        step_length_arg,
        step_frequency_arg,
        leg_extension_ratio_arg,
        enabled_arg,
        gait_generator_node,
        robot_state_publisher_node,
        rviz_node,
    ])
