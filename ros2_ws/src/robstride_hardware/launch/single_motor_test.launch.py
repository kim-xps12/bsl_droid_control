#!/usr/bin/env python3
"""
Single Motor Test Launch File

単一モーター（CAN: can1, Motor ID: 127）のテスト用Launchファイル．
ros2_controlのController Managerを起動し，Forward Position Controllerで
位置指令を受け付ける．テスト指令ノードが目標位置へゆっくり移動し，
一定時間ホールド後に自動終了する．
テスト完了後，CAN 通信ログの統計分析を自動実行する．

起動例:
  # デフォルト（0 rad, 0.5 rad/s, 10秒ホールド）
  ros2 launch robstride_hardware single_motor_test.launch.py

  # パラメータ指定
  ros2 launch robstride_hardware single_motor_test.launch.py \
    target_position:=1.57 max_velocity:=0.3 hold_duration:=15.0
"""
import os
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Spawner起動待機時間 [秒] (Controller Managerの初期化完了を待つ)
SPAWNER_DELAY_SEC = 1.0

# analyze_timing_log.py のインストールパス
_ANALYZE_SCRIPT = os.path.join(
    get_package_prefix('robstride_hardware'),
    'lib', 'robstride_hardware', 'analyze_timing_log.py'
)


def generate_launch_description():
    # ============================================================
    # Launch arguments
    # ============================================================
    target_position_arg = DeclareLaunchArgument(
        'target_position', default_value='0.0',
        description='Target position [rad]')
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity', default_value='0.5',
        description='Maximum movement velocity [rad/s]')
    hold_duration_arg = DeclareLaunchArgument(
        'hold_duration', default_value='10.0',
        description='Hold duration after reaching target [s]')

    # ============================================================
    # 1. URDF生成 (single_motor_test.urdf.xacro)
    # ============================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robstride_hardware'),
            'urdf',
            'single_motor_test.urdf.xacro'
        ])
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ============================================================
    # 2. コントローラー設定ファイル (single_motor用)
    # ============================================================
    controller_config = PathJoinSubstitution([
        FindPackageShare('robstride_hardware'),
        'config',
        'single_motor_controllers.yaml'
    ])

    # ============================================================
    # 3. Controller Manager (200Hz制御ループ)
    # ============================================================
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='both',
    )

    # ============================================================
    # 4. Robot State Publisher (TF配信)
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # ============================================================
    # 5. Joint State Broadcaster
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='both',
    )

    # ============================================================
    # 6. Forward Position Controller
    # ============================================================
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
        output='both',
    )

    # ============================================================
    # 7. Test Commander Node
    # ============================================================
    test_commander = Node(
        package='robstride_hardware',
        executable='single_motor_test_commander.py',
        name='single_motor_test_commander',
        parameters=[{
            'target_position': LaunchConfiguration('target_position'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'hold_duration': LaunchConfiguration('hold_duration'),
            'joint_name': 'test_joint',
        }],
        output='both',
    )

    # ============================================================
    # 8. 統計分析プロセス（テスト完了後に自動起動）
    # ============================================================
    analyze_process = ExecuteProcess(
        cmd=[sys.executable, _ANALYZE_SCRIPT],
        output='screen',
        name='analyze_timing_log',
    )

    # ============================================================
    # 9. 起動シーケンス制御
    # ============================================================
    # Controller Manager初期化完了後にspawnerを起動
    delay_jsb = TimerAction(
        period=SPAWNER_DELAY_SEC,
        actions=[joint_state_broadcaster_spawner],
    )

    # joint_state_broadcaster spawner完了後 → forward_position_controller
    delay_fpc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    # forward_position_controller spawner完了後 → テスト指令ノード
    start_commander_after_fpc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[test_commander],
        )
    )

    # テスト指令ノード終了 → 統計分析
    run_analysis_on_complete = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=test_commander,
            on_exit=[analyze_process],
        )
    )

    # 統計分析完了 → 全体シャットダウン
    shutdown_on_analysis_complete = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=analyze_process,
            on_exit=[EmitEvent(event=Shutdown(reason='Analysis complete'))],
        )
    )

    return LaunchDescription([
        target_position_arg,
        max_velocity_arg,
        hold_duration_arg,
        control_node,
        robot_state_publisher,
        delay_jsb,
        delay_fpc,
        start_commander_after_fpc,
        run_analysis_on_complete,
        shutdown_on_analysis_complete,
    ])
