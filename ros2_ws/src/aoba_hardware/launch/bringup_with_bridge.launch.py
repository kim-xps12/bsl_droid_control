#!/usr/bin/env python3
"""
RobStride Hardware ros2_control Launch File (Headless + Bridge)

bringup_headless.launch.py に以下を追加した構成:
- joint_state_broadcaster の出力を /hw/joint_states にリマップ
  （Mac側GUIの /joint_states と競合しないようにする）
- joint_state_bridge ノードを起動
  （/joint_states → /forward_position_controller/commands 変換）

起動するもの:
1. robot_description_publisher - URDF配信 (/hw/robot_description)
2. Controller Manager (ros2_control_node) - 200Hz RTループ
3. joint_state_broadcaster - モーター状態を /hw/joint_states に配信
4. forward_position_controller - /commands から目標位置を受信
5. joint_state_bridge - GUIの /joint_states を hardware commands に変換
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
    # joint_state_broadcaster の出力を /hw/joint_states にリマップ
    # Mac側GUIが /joint_states に配信するため、競合を回避する
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controller_config,
            '--controller-manager', '/controller_manager',
        ],
        remappings=[
            ('/joint_states', '/hw/joint_states'),
        ],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
    )

    # ============================================================
    # 6. Joint State Bridge ノード
    # ============================================================
    joint_state_bridge_node = Node(
        package='biped_gait_control',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='both',
    )

    # ============================================================
    # 7. 起動シーケンス制御
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
    # 8. Launch Description
    # ============================================================
    return LaunchDescription([
        robot_description_publisher,
        control_node,
        delay_joint_state_broadcaster_after_control_node,
        delay_forward_controller_after_joint_state_broadcaster,
        joint_state_bridge_node,
    ])
