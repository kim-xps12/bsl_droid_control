"""
Launch file for trajectory replay with RViz visualization.

Supports multiple trajectory sources (foot, oscillation, waypoint)
selected via config_file argument.

Usage:
    # Foot trajectory (default):
    ros2 launch biped_gait_control trajectory_replay.launch.py

    # Single joint oscillation:
    ros2 launch biped_gait_control trajectory_replay.launch.py config_file:=replay_oscillation.yaml

    # Waypoint playback:
    ros2 launch biped_gait_control trajectory_replay.launch.py config_file:=replay_waypoint.yaml

    # Control mode (requires external aoba_hardware bringup):
    ros2 launch biped_gait_control trajectory_replay.launch.py mode:=control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    aoba_description_share = FindPackageShare('aoba_description')
    biped_gait_control_share = FindPackageShare('biped_gait_control')

    # URDF: aoba
    urdf_file = PathJoinSubstitution([
        aoba_description_share, 'urdf', 'aoba.xacro'
    ])

    rviz_config = PathJoinSubstitution([
        aoba_description_share, 'rviz', 'aoba_display.rviz'
    ])

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='replay_foot.yaml',
        description='Config YAML file name (replay_foot.yaml, replay_oscillation.yaml, replay_waypoint.yaml)'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='viz',
        description='Operating mode: viz or control'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    config_file_path = PathJoinSubstitution([
        biped_gait_control_share, 'config', LaunchConfiguration('config_file')
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_enabled = LaunchConfiguration('rviz')

    # Trajectory replay node
    trajectory_replay_node = Node(
        package='biped_gait_control',
        executable='trajectory_replay',
        name='trajectory_replay',
        output='screen',
        parameters=[
            config_file_path,
            {'mode': LaunchConfiguration('mode')},
        ],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # Robot state publisher
    # trajectory_replay_node publishes to /cmd/joint_states, so remap here
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
        }],
        remappings=[('joint_states', '/cmd/joint_states')],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_enabled),
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # Shutdown entire launch when main node exits (prevents orphan processes)
    shutdown_on_main_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_replay_node,
            on_exit=[Shutdown(reason='trajectory_replay node exited')],
        )
    )

    return LaunchDescription([
        config_file_arg,
        mode_arg,
        use_sim_time_arg,
        rviz_arg,
        trajectory_replay_node,
        robot_state_publisher_node,
        rviz_node,
        shutdown_on_main_exit,
    ])
