#!/usr/bin/env python3
"""
Multi Motor Zero Test Launch File

複数モーターを同時にゼロ点（0 rad）へ移動し、CAN通信品質を定量評価するLaunchファイル。
モーター構成はlaunch引数で動的に指定可能。OpaqueFunction内でURDFとコントローラ設定を
動的に生成する。

起動例:
  # 2モーター on can1
  ros2 launch robstride_hardware multi_motor_zero_test.launch.py motors:='can1:11,12'

  # 4モーター on 2 buses
  ros2 launch robstride_hardware multi_motor_zero_test.launch.py \\
    motors:='can1:11,12 can2:21,22' hold_duration:=15.0

  # 全10モーター
  ros2 launch robstride_hardware multi_motor_zero_test.launch.py \\
    motors:='can1:11,12,13,14,15 can2:21,22,23,24,25'
"""
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

# Spawner起動待機時間 [秒] (Controller Managerの初期化完了を待つ)
SPAWNER_DELAY_SEC = 1.0

# analyze_timing_log.py のインストールパス
_ANALYZE_SCRIPT = os.path.join(
    get_package_prefix('robstride_hardware'),
    'lib', 'robstride_hardware', 'analyze_timing_log.py',
)


def _parse_motors(motors_str: str) -> list[tuple[str, int]]:
    """Parse motors argument string into list of (can_interface, motor_id) tuples.

    Format: "can1:11,12 can2:21,22" -> [("can1",11), ("can1",12), ("can2",21), ("can2",22)]
    """
    result: list[tuple[str, int]] = []
    for bus_spec in motors_str.strip().split():
        parts = bus_spec.split(':')
        if len(parts) != 2:
            raise ValueError(f"Invalid motor spec '{bus_spec}'. Expected 'canX:id1,id2,...'")
        can_interface = parts[0]
        for motor_id_str in parts[1].split(','):
            result.append((can_interface, int(motor_id_str)))
    return result


def _generate_urdf(motors: list[tuple[str, int]], kp: float, kd: float) -> str:
    """Generate URDF XML string for the given motors."""
    joints_xml = ''
    for can_if, motor_id in motors:
        joint_name = f'motor_{can_if}_{motor_id}'
        link_name = f'{joint_name}_link'
        joints_xml += f"""
  <link name="{link_name}">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </visual>
  </link>

  <joint name="{joint_name}" type="revolute">
    <parent link="base_link"/>
    <child link="{link_name}"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-12.566" upper="12.566" effort="17.0" velocity="44.0"/>
  </joint>
"""

    ros2_control_joints = ''
    for can_if, motor_id in motors:
        joint_name = f'motor_{can_if}_{motor_id}'
        ros2_control_joints += f"""
    <joint name="{joint_name}">
      <param name="can_interface">{can_if}</param>
      <param name="motor_id">{motor_id}</param>
      <param name="kp">{kp}</param>
      <param name="kd">{kd}</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
"""

    return f"""<?xml version="1.0"?>
<robot name="robstride_multi_motor_test">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
{joints_xml}
  <ros2_control name="robstride_system" type="system">
    <hardware>
      <plugin>robstride_hardware/RobStrideHardware</plugin>
    </hardware>
{ros2_control_joints}
  </ros2_control>

</robot>
"""


def _launch_setup(context: LaunchContext) -> list:
    """OpaqueFunction callback: generate URDF, controller config, and launch nodes."""
    motors_str = context.launch_configurations['motors']
    max_velocity = context.launch_configurations['max_velocity']
    hold_duration = context.launch_configurations['hold_duration']
    kp = float(context.launch_configurations['kp'])
    kd = float(context.launch_configurations['kd'])

    motors = _parse_motors(motors_str)
    if not motors:
        raise ValueError("No motors specified. Use motors:='can1:11,12 can2:21,22'")

    joint_names = [f'motor_{can_if}_{motor_id}' for can_if, motor_id in motors]

    # --- URDF ---
    urdf_content = _generate_urdf(motors, kp, kd)
    robot_description = {'robot_description': urdf_content}

    # --- Controller config (write to temp YAML) ---
    controller_params = {
        'controller_manager': {'ros__parameters': {
            'update_rate': 200,
            'use_realtime': True,
            'thread_priority': 90,
            'cpu_affinity': 2,
            'lock_memory': False,
            'joint_state_broadcaster': {
                'type': 'joint_state_broadcaster/JointStateBroadcaster',
            },
            'forward_position_controller': {
                'type': 'forward_command_controller/ForwardCommandController',
            },
        }},
        'forward_position_controller': {'ros__parameters': {
            'joints': joint_names,
            'interface_name': 'position',
        }},
        'joint_state_broadcaster': {'ros__parameters': {
            'publish_rate': 50.0,
            'use_local_topics': False,
        }},
    }
    config_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='multi_motor_ctrl_', delete=False,
    )
    yaml.dump(controller_params, config_file, default_flow_style=False)
    config_file.close()

    # --- Nodes ---
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, config_file.name],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='both',
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
        output='both',
    )

    test_commander = Node(
        package='robstride_hardware',
        executable='multi_motor_zero_commander.py',
        name='multi_motor_zero_commander',
        parameters=[{
            'joint_names': ','.join(joint_names),
            'max_velocity': float(max_velocity),
            'hold_duration': float(hold_duration),
        }],
        output='both',
    )

    analyze_process = ExecuteProcess(
        cmd=[sys.executable, _ANALYZE_SCRIPT],
        output='screen',
        name='analyze_timing_log',
    )

    # --- Launch sequence ---
    delay_jsb = TimerAction(
        period=SPAWNER_DELAY_SEC,
        actions=[joint_state_broadcaster_spawner],
    )

    delay_fpc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    start_commander_after_fpc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[test_commander],
        )
    )

    run_analysis_on_complete = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=test_commander,
            on_exit=[analyze_process],
        )
    )

    shutdown_on_analysis_complete = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=analyze_process,
            on_exit=[EmitEvent(event=Shutdown(reason='Analysis complete'))],
        )
    )

    return [
        control_node,
        robot_state_publisher,
        delay_jsb,
        delay_fpc,
        start_commander_after_fpc,
        run_analysis_on_complete,
        shutdown_on_analysis_complete,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'motors',
            description="Motor specification: 'can1:11,12 can2:21,22'",
        ),
        DeclareLaunchArgument(
            'max_velocity', default_value='0.5',
            description='Maximum movement velocity [rad/s]',
        ),
        DeclareLaunchArgument(
            'hold_duration', default_value='10.0',
            description='Hold duration after all joints reach zero [s]',
        ),
        DeclareLaunchArgument(
            'kp', default_value='30.0',
            description='Position gain',
        ),
        DeclareLaunchArgument(
            'kd', default_value='1.0',
            description='Damping gain',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
