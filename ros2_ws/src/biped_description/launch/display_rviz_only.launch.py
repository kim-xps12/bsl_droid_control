"""
Launch file for RViz visualization only (no joint_state_publisher).

Use this launch file when joint states are provided by an external node
(e.g., gait_pattern_generator, ros2_control, etc.)

Usage:
    ros2 launch biped_description display_rviz_only.launch.py

    Then separately run your joint state publisher:
    ros2 launch biped_gait_control gait_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Package path
    pkg_share = FindPackageShare('biped_description')

    # URDF file path
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'biped_digitigrade.urdf.xacro'])

    # RViz config file
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'biped_display.rviz'])

    # use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # robot_state_publisher: Publishes TF from URDF
    # Subscribes to /joint_states from external source
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # RViz2: Visualization only
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
        robot_state_publisher_node,
        rviz_node,
    ])
