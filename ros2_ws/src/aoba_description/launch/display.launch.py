from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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

    # use_gui引数 (joint_state_publisher vs joint_state_publisher_gui)
    use_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui'
    )

    use_gui = LaunchConfiguration('gui')

    # robot_state_publisher: URDFからTFを配信
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

    # joint_state_publisher_gui: スライダーGUIで関節操作
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui),
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # joint_state_publisher: GUI無しバージョン
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gui),
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # RViz2: 可視化
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

    # RVizが閉じられたら全プロセスを終了
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[Shutdown(reason='rviz2 exited')],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
        shutdown_on_rviz_exit,
    ])
