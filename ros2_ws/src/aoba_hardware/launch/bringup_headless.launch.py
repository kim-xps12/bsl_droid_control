#!/usr/bin/env python3
"""
RobStride Hardware ros2_control Launch File (Headless)

bringup.launch.py から robot_state_publisher を除いたバージョン。
TF配信はMac側の display_custom.launch.py に任せる構成で使用する。

起動するもの:
1. robot_description_publisher - URDF配信 (/hw/robot_description)
2. Controller Manager (ros2_control_node) - 200Hz RTループ
3. joint_state_broadcaster - モーター状態配信（トピック名は引数で設定可能）
4. forward_position_controller - /commands から目標位置を受信

使い方:
    # デフォルト（/joint_statesに配信）
    ros2 launch aoba_hardware bringup_headless.launch.py

    # /hw/joint_statesに配信（Mac側GUIやtrajectory_replayとの競合回避）
    ros2 launch aoba_hardware bringup_headless.launch.py joint_states_topic:=/hw/joint_states
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # 0. Launch引数
    # ============================================================
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states',
        description='Output topic for joint_state_broadcaster'
    )

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
    # 3. URDF配信 (controller_manager用)
    # ============================================================
    # Jazzy (v4.x) の controller_manager は /robot_description トピックから
    # URDFを受信する。Mac側 robot_state_publisher が ros2_controlタグなし
    # URDFを /robot_description に配信するため、専用トピックで衝突回避。
    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_description_publisher',
        parameters=[robot_description],
        remappings=[('robot_description', '/hw/robot_description')],
        output='both',
    )

    # ============================================================
    # 4. Controller Manager (中核ノード)
    # ============================================================
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        remappings=[('robot_description', '/hw/robot_description')],
        output='both',
    )

    # ============================================================
    # 5. コントローラーのスポーン
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        remappings=[
            ('/joint_states', LaunchConfiguration('joint_states_topic')),
        ],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
    )

    # ============================================================
    # 6. 起動シーケンス制御
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
    # 7. Launch Description
    # ============================================================
    # bringup.launch.py との差分: robot_state_publisher を含まない
    return LaunchDescription([
        joint_states_topic_arg,
        robot_description_publisher,
        control_node,
        delay_joint_state_broadcaster_after_control_node,
        delay_forward_controller_after_joint_state_broadcaster,
    ])
