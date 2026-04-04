"""
RViz可視化専用のLaunchファイル（joint_state_publisherなし）.

外部ノードからjoint statesが供給される場合に使用する.

使い方:
    ros2 launch aoba_description display_rviz_only.launch.py

    別途，joint state publisherを起動すること.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # パッケージパス
    pkg_share = FindPackageShare('aoba_description')

    # URDFファイルパス
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'aoba.xacro'])

    # RViz設定ファイル
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'aoba_display.rviz'])

    # use_sim_time引数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # robot_state_publisher: URDFからTFを配信
    # /joint_statesは外部ノードから供給される前提
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': use_sim_time
        }],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # RViz2: 可視化のみ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        rviz_node,
    ])
