from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # パッケージパス
    pkg_share = FindPackageShare('biped_description')

    # URDFファイルパス (BSL-Droid Simplified V2)
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'bsl_droid_simplified_v2.urdf.xacro'])

    # RViz設定ファイル
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'biped_display.rviz'])

    # カスタムGUIスクリプト (自動可動範囲取得版)
    joint_gui_script = PathJoinSubstitution([pkg_share, 'scripts', 'joint_gui.py'])

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
        }]
    )

    # カスタムGUI (自動可動範囲取得版)
    # robot_state_publisherが起動してからGUIを起動するため、遅延を入れる
    custom_joint_gui = TimerAction(
        period=1.0,  # 1秒待機してからGUIを起動
        actions=[
            ExecuteProcess(
                cmd=['python3', joint_gui_script],
                name='joint_gui',
                output='screen'
            )
        ]
    )

    # RViz2: 可視化
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
        custom_joint_gui,
        rviz_node,
    ])
