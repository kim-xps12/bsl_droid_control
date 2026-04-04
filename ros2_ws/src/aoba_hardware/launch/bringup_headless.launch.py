#!/usr/bin/env python3
"""
RobStride Hardware ros2_control Launch File (Headless)

bringup.launch.py から robot_state_publisher を除いたバージョン。
TF配信はMac側の display_custom.launch.py に任せる構成で使用する。

起動するもの:
1. Controller Manager (ros2_control_node) - 200Hz RTループ
2. joint_state_broadcaster - モーター状態を /joint_states に配信
3. forward_position_controller - /commands から目標位置を受信
"""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # 1. URDF生成 (xacroファイルをXMLに変換)
    # ============================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('aoba_hardware'),
            'urdf',
            'aoba_system.urdf.xacro'
        ])
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ============================================================
    # 2. コントローラー設定ファイル
    # ============================================================
    controller_config = PathJoinSubstitution([
        FindPackageShare('aoba_hardware'),
        'config',
        'controllers.yaml'
    ])

    # ============================================================
    # 3. Controller Manager (中核ノード)
    # ============================================================
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='both',
    )

    # ============================================================
    # 4. コントローラーのスポーン
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
    )

    # ============================================================
    # 5. 起動シーケンス制御
    # ============================================================
    delay_joint_state_broadcaster_after_control_node = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delay_forward_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    # ============================================================
    # 6. Launch Description
    # ============================================================
    # bringup.launch.py との差分: robot_state_publisher を含まない
    return LaunchDescription([
        control_node,
        delay_joint_state_broadcaster_after_control_node,
        delay_forward_controller_after_joint_state_broadcaster,
    ])
