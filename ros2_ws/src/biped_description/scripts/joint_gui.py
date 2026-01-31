#!/usr/bin/env python3
"""
カスタム関節操作GUI (自動可動範囲取得版)

robot_state_publisherが公開するrobot_descriptionパラメータから
URDFをパースし、関節の可動範囲を自動取得する。
"""

import sys
import math
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QSlider, QGroupBox, QPushButton, QMessageBox
    )
    from PyQt5.QtCore import Qt, QTimer
except ImportError:
    print("PyQt5 is required. Install with: pip install PyQt5")
    sys.exit(1)


def parse_joint_limits_from_urdf(urdf_xml: str) -> dict:
    """
    URDF XMLから関節の可動範囲を抽出する

    Returns:
        dict: {joint_name: (lower_rad, upper_rad)}
    """
    joints = {}
    try:
        root = ET.fromstring(urdf_xml)
        for joint in root.findall('.//joint'):
            joint_type = joint.get('type')
            if joint_type not in ('revolute', 'prismatic'):
                continue

            name = joint.get('name')
            limit = joint.find('limit')
            if limit is not None:
                lower = float(limit.get('lower', 0))
                upper = float(limit.get('upper', 0))
                joints[name] = (lower, upper)
    except ET.ParseError as e:
        print(f"URDF parse error: {e}")

    return joints


class JointSlider(QWidget):
    """個別の関節スライダーウィジェット"""

    def __init__(self, joint_name: str, min_rad: float, max_rad: float, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.min_rad = min_rad
        self.max_rad = max_rad

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # 関節名ラベル（短縮表示）
        short_name = joint_name.replace('_joint', '').replace('left_', 'L:').replace('right_', 'R:')
        self.name_label = QLabel(short_name)
        self.name_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.name_label)

        # 可動範囲表示
        min_deg = math.degrees(min_rad)
        max_deg = math.degrees(max_rad)
        range_label = QLabel(f"[{min_deg:.0f}° ~ {max_deg:.0f}°]")
        range_label.setAlignment(Qt.AlignCenter)
        range_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(range_label)

        # スライダー
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)
        self.slider.setValue(self._rad_to_slider(0.0))
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider)

        # 値表示ラベル
        self.value_label = QLabel("0.00°")
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)

    def _rad_to_slider(self, rad: float) -> int:
        """ラジアン値をスライダー値(0-1000)に変換"""
        if self.max_rad == self.min_rad:
            return 500
        ratio = (rad - self.min_rad) / (self.max_rad - self.min_rad)
        return int(max(0, min(1000, ratio * 1000)))

    def _slider_to_rad(self, val: int) -> float:
        """スライダー値(0-1000)をラジアン値に変換"""
        ratio = val / 1000.0
        return self.min_rad + ratio * (self.max_rad - self.min_rad)

    def _on_slider_changed(self, val: int):
        rad = self._slider_to_rad(val)
        deg = math.degrees(rad)
        self.value_label.setText(f"{deg:.1f}°")

    def get_position(self) -> float:
        """現在の関節角度をラジアンで取得"""
        return self._slider_to_rad(self.slider.value())

    def reset(self):
        """スライダーを0にリセット"""
        self.slider.setValue(self._rad_to_slider(0.0))


class JointGUI(QMainWindow):
    """メインGUIウィンドウ"""

    # 関節の物理的な接続順（股関節から足首へ）
    JOINT_ORDER = [
        'hip_yaw',
        'hip_roll',
        'hip_pitch',
        'knee_pitch',
        'ankle_pitch',
    ]

    def __init__(self, node: Node, joint_limits: dict):
        super().__init__()
        self.node = node
        self.joint_limits = joint_limits
        self.sliders: dict[str, JointSlider] = {}

        self.setWindowTitle("Biped Joint Control (Auto)")
        self.setMinimumSize(550, 450)

        # JointState パブリッシャー
        self.joint_pub = self.node.create_publisher(JointState, '/joint_states', 10)

        self._setup_ui()

        # 定期的にJointStateをパブリッシュ (50Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._publish_joint_states)
        self.timer.start(20)

    def _get_joint_sort_key(self, joint_name: str) -> int:
        """関節名からソートキーを取得（物理的な接続順）"""
        for i, suffix in enumerate(self.JOINT_ORDER):
            if suffix in joint_name:
                return i
        return len(self.JOINT_ORDER)  # 未知の関節は最後

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)

        # 左右の脚を横に並べる
        legs_layout = QHBoxLayout()

        # 左脚の関節を抽出（物理的な接続順でソート）
        left_joints = sorted(
            [(name, limits) for name, limits in self.joint_limits.items()
             if name.startswith('left_')],
            key=lambda x: self._get_joint_sort_key(x[0])
        )

        # 右脚の関節を抽出（物理的な接続順でソート）
        right_joints = sorted(
            [(name, limits) for name, limits in self.joint_limits.items()
             if name.startswith('right_')],
            key=lambda x: self._get_joint_sort_key(x[0])
        )

        # 左脚グループ
        left_group = QGroupBox(f"Left Leg ({len(left_joints)} joints)")
        left_layout = QVBoxLayout(left_group)
        for name, (lower, upper) in left_joints:
            slider = JointSlider(name, lower, upper)
            self.sliders[name] = slider
            left_layout.addWidget(slider)
        legs_layout.addWidget(left_group)

        # 右脚グループ
        right_group = QGroupBox(f"Right Leg ({len(right_joints)} joints)")
        right_layout = QVBoxLayout(right_group)
        for name, (lower, upper) in right_joints:
            slider = JointSlider(name, lower, upper)
            self.sliders[name] = slider
            right_layout.addWidget(slider)
        legs_layout.addWidget(right_group)

        main_layout.addLayout(legs_layout)

        # ボタン
        button_layout = QHBoxLayout()

        reset_btn = QPushButton("Reset All")
        reset_btn.clicked.connect(self._reset_all)
        button_layout.addWidget(reset_btn)

        center_btn = QPushButton("Center Pose")
        center_btn.clicked.connect(self._center_pose)
        button_layout.addWidget(center_btn)

        main_layout.addLayout(button_layout)

    def _publish_joint_states(self):
        """JointStateメッセージをパブリッシュ"""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        for name, slider in self.sliders.items():
            msg.name.append(name)
            msg.position.append(slider.get_position())
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

        self.joint_pub.publish(msg)

    def _reset_all(self):
        """全関節を0にリセット"""
        for slider in self.sliders.values():
            slider.reset()

    def _center_pose(self):
        """自然な立位姿勢（膝を軽く曲げた状態）"""
        poses = {
            'left_hip_pitch_joint': 15,
            'right_hip_pitch_joint': 15,
            'left_knee_pitch_joint': -30,
            'right_knee_pitch_joint': -30,
            'left_ankle_pitch_joint': 15,
            'right_ankle_pitch_joint': 15,
        }
        for name, deg in poses.items():
            if name in self.sliders:
                slider = self.sliders[name]
                rad = math.radians(deg)
                slider.slider.setValue(slider._rad_to_slider(rad))


def get_robot_description(node: Node, timeout_sec: float = 10.0) -> str:
    """
    robot_state_publisherからrobot_descriptionパラメータを取得する
    """
    client = node.create_client(
        GetParameters,
        '/robot_state_publisher/get_parameters'
    )

    node.get_logger().info("Waiting for robot_state_publisher service...")

    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError("robot_state_publisher service not available")

    from rcl_interfaces.msg import Parameter
    request = GetParameters.Request()
    request.names = ['robot_description']

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if future.result() is None:
        raise RuntimeError("Failed to get robot_description parameter")

    response = future.result()
    if not response.values:
        raise RuntimeError("robot_description parameter is empty")

    return response.values[0].string_value


def main():
    rclpy.init()
    node = Node('joint_gui')

    app = QApplication(sys.argv)

    # robot_descriptionを取得
    try:
        node.get_logger().info("Fetching robot_description from robot_state_publisher...")
        urdf_xml = get_robot_description(node)
        node.get_logger().info(f"Got URDF ({len(urdf_xml)} bytes)")
    except Exception as e:
        QMessageBox.critical(
            None,
            "Error",
            f"Failed to get robot_description:\n{e}\n\n"
            "Make sure robot_state_publisher is running."
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # URDFから関節可動範囲をパース
    joint_limits = parse_joint_limits_from_urdf(urdf_xml)

    if not joint_limits:
        QMessageBox.critical(
            None,
            "Error",
            "No revolute/prismatic joints found in URDF."
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(f"Found {len(joint_limits)} joints: {list(joint_limits.keys())}")

    # GUI起動
    window = JointGUI(node, joint_limits)
    window.show()

    # ROS2スピンとQtイベントループを統合
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
