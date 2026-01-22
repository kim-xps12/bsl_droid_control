#!/usr/bin/env python3
"""
Launch file for RobStride Hardware with Sinusoidal Motion

このLaunchファイルは以下を起動:
1. Controller Manager + RobStride Hardware Interface (200Hz制御ループ)
2. Robot State Publisher (TF配信)
3. Joint State Broadcaster (状態配信)
4. Forward Position Controller (コマンド受信)
5. Sinusoidal Motion Publisher (50Hzで目標位置生成)
"""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================================
    # 1. URDF生成 (xacroファイルをXMLに変換)
    # ============================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robstride_hardware'),
            'urdf',
            'robstride_system.urdf.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # ============================================================
    # 2. コントローラー設定ファイル
    # ============================================================
    controller_config = PathJoinSubstitution([
        FindPackageShare('robstride_hardware'),
        'config',
        'controllers.yaml'
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
    # 5. Joint State Broadcaster (状態配信)
    # ============================================================
    # Controller Manager起動後2秒待ってからスポーン
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='both',
            )
        ]
    )
    
    # ============================================================
    # 6. Forward Position Controller (コマンド受信)
    # ============================================================
    # joint_state_broadcaster起動後1秒待ってスポーン
    forward_position_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['forward_position_controller'],
                output='both',
            )
        ]
    )
    
    # ============================================================
    # 7. Sinusoidal Motion Publisher (50Hzで目標位置生成)
    # ============================================================
    # forward_position_controller起動後1秒待ってから開始
    sinusoidal_motion_publisher = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robstride_hardware',
                executable='sinusoidal_motion_publisher.py',
                name='sinusoidal_motion_publisher',
                output='both',
                parameters=[
                    {'publish_rate': 50.0},          # 50 Hz (変更不要)
                    {'amplitude': 1.5708},           # 振幅: π/2 rad = 90 deg
                                                     # 例: 0.7854 = 45度, 3.1416 = 180度
                    {'frequency': 2.0},              # 周波数: 2Hz = 1秒に2回往復（元の設定）
                                                     # 例: 1.0 = 1秒1回, 0.5 = 2秒1回
                    {'topic_name': '/forward_position_controller/commands'}
                ]
            )
        ]
    )
    
    # ============================================================
    # 8. Launch Description
    # ============================================================
    # 起動順序 (TimerActionで時間差起動):
    # t=0s:   control_node, robot_state_publisher
    # t=2s:   joint_state_broadcaster
    # t=3s:   forward_position_controller
    # t=4s:   sinusoidal_motion_publisher (モーション開始)
    return LaunchDescription([
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        sinusoidal_motion_publisher,
    ])
