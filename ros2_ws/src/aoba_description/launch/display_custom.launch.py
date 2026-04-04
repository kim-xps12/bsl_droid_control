from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
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

    # カスタムGUIスクリプト
    joint_gui_script = PathJoinSubstitution([pkg_share, 'scripts', 'joint_gui.py'])

    # PlotJuggler起動フラグ
    plot_arg = DeclareLaunchArgument(
        'plot',
        default_value='false',
        description='Launch PlotJuggler'
    )

    # use_sim_time引数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # カスタムGUI (PyQt5ベース、歩行待機姿勢ボタン付き)
    custom_joint_gui = ExecuteProcess(
        cmd=['python3', joint_gui_script],
        name='joint_gui',
        output='screen',
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

    # PlotJuggler（条件付き起動、レイアウト自動読み込み）
    layout_file = PathJoinSubstitution([pkg_share, 'config', 'plotjuggler_joint_states.xml'])
    plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler', '--layout', layout_file],
        name='plotjuggler',
        output='screen',
        condition=IfCondition(LaunchConfiguration('plot')),
    )

    # RVizが閉じられたら全プロセスを終了
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[Shutdown(reason='rviz2 exited')],
        )
    )

    return LaunchDescription([
        plot_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        custom_joint_gui,
        rviz_node,
        plotjuggler,
        shutdown_on_rviz_exit,
    ])
