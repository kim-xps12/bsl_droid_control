#!/usr/bin/env python3
"""
RobStride Hardware ros2_control Launch File

このLaunchファイルは以下の処理を行う:
1. URDFロード (xacro → XML変換)
2. Controller Manager起動 (ros2_control_node)
3. Robot State Publisher起動 (TF配信)
4. コントローラーのスポーン (joint_state_broadcaster, forward_position_controller)
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
    # Command()は実行時にxacroコマンドを呼び出してURDFを生成
    # これによりros2_control設定、joint定義、hardware pluginが読み込まれる
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('aoba_hardware'),
            'urdf',
            'aoba_system.urdf.xacro'  # モーター設定 (CAN, motor_id, kp, kd)
        ])
    ])
    
    # ROS 2パラメータとして渡すための辞書形式
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # ============================================================
    # 2. コントローラー設定ファイル
    # ============================================================
    # update_rate (200Hz), コントローラー種類を定義
    controller_config = PathJoinSubstitution([
        FindPackageShare('aoba_hardware'),
        'config',
        'controllers.yaml'  # joint_state_broadcaster, forward_position_controller
    ])
    
    # ============================================================
    # 3. Controller Manager (中核ノード)
    # ============================================================
    # ros2_control_nodeは以下を実行:
    # - URDFからHardware Interface (AobaHardware) をロード
    # - 200Hzでread()/write()を呼び出すリアルタイムループ
    # - コントローラーのライフサイクル管理
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='both',  # stdout/stderrを両方表示
    )
    
    # ============================================================
    # 4. Robot State Publisher (TF配信)
    # ============================================================
    # URDFからTF (Transform) を計算して /tf トピックに配信
    # joint_statesトピックを購読し、各リンクの位置姿勢を計算
    # RViz等で可視化するために必要
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    # ============================================================
    # 5. コントローラーのスポーン (ロード + 起動)
    # ============================================================
    # JointStateBroadcaster: Hardware Interfaceの状態を /joint_states トピックに配信
    # これにより、robot_state_publisherがTFを計算できる
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',  # Controller Managerにコントローラーをロードするツール
        arguments=['joint_state_broadcaster'],
    )
    
    # ForwardPositionController: /commands トピックから目標位置を受信
    # Hardware Interfaceの command_interface に書き込む
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
    )
    
    # ============================================================
    # 6. 起動シーケンス制御 (依存関係の定義)
    # ============================================================
    # Controller Manager初期化完了後にspawnerを起動
    # control_nodeは長時間動作するためOnProcessExitではなくTimerActionを使用
    
    # ステップ1: 待機後 → joint_state_broadcaster起動
    delay_joint_state_broadcaster_after_control_node = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )
    
    # ステップ2: joint_state_broadcaster spawner完了後 → forward_position_controller起動
    # spawnerは短命プロセスなのでOnProcessExitで正しくチェーンできる
    delay_forward_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )
    
    # ============================================================
    # 7. Launch Description (起動シーケンス)
    # ============================================================
    # 実際の起動順序:
    # 1. control_node (Controller Manager) 起動
    # 2. robot_state_publisher 起動 (並行)
    # 3. control_node準備完了 → joint_state_broadcaster スポーン
    # 4. joint_state_broadcaster準備完了 → forward_position_controller スポーン
    return LaunchDescription([
        control_node,
        robot_state_publisher,
        delay_joint_state_broadcaster_after_control_node,
        delay_forward_controller_after_joint_state_broadcaster,
    ])
