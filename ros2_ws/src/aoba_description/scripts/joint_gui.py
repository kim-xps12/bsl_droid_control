#!/usr/bin/env python3
"""
Aoba カスタム関節操作GUI

robot_state_publisherからURDFを取得してバリデーションを行い、
ハードコードされた関節設定に基づいてスライダーGUIを表示する。
歩行待機姿勢へのスムーズ遷移ボタン付き。
"""

import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState

from aoba_description.joint_limits import JOINT_LIMITS

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    print("PyQt5 is required. Install with: pip install PyQt5")
    sys.exit(1)


# --- 関節設定 ---
# (表示名, グループ) — UI固有情報のみ保持
# 順序は JOINT_LIMITS (left 5 + right 5 + neck 1) に対応
_JOINT_UI: list[tuple[str, str, str]] = [
    ("rev11", "L: hip yaw", "left"),
    ("rev12", "L: hip roll", "left"),
    ("rev13", "L: hip pitch", "left"),
    ("rev14", "L: knee pitch", "left"),
    ("rev15", "L: ankle pitch", "left"),
    ("rev21", "R: hip yaw", "right"),
    ("rev22", "R: hip roll", "right"),
    ("rev23", "R: hip pitch", "right"),
    ("rev24", "R: knee pitch", "right"),
    ("rev25", "R: ankle pitch", "right"),
    ("rev31", "Neck", "neck"),
]

# (表示名, 下限[deg], 上限[deg], グループ) — 可動範囲は joint_limits.py から取得
JOINT_CONFIG: dict[str, tuple[str, float, float, str]] = {
    name: (
        display,
        math.degrees(JOINT_LIMITS[i][0]),
        math.degrees(JOINT_LIMITS[i][1]),
        group,
    )
    for i, (name, display, group) in enumerate(_JOINT_UI)
}

# 歩行待機姿勢 [deg]
WALKING_STANDBY_POSE: dict[str, float] = {
    "rev11": 0.0,
    "rev12": 0.0,
    "rev13": -60.0,
    "rev14": 90.0,
    "rev15": -30.0,
    "rev21": 0.0,
    "rev22": 0.0,
    "rev23": -60.0,
    "rev24": 90.0,
    "rev25": -30.0,
    "rev31": 0.0,
}

# アニメーション設定
ANIM_DURATION_MS = 2000
ANIM_TICK_MS = 20


def parse_joint_names_from_urdf(urdf_xml: str) -> list[str]:
    """URDFからmovable関節名を抽出する（continuous/revolute/prismatic）"""
    names: list[str] = []
    try:
        root = ET.fromstring(urdf_xml)
        for joint in root.findall(".//joint"):
            if joint.get("type") in ("revolute", "prismatic", "continuous"):
                name = joint.get("name")
                if name:
                    names.append(name)
    except ET.ParseError as e:
        print(f"URDF parse error: {e}")
    return names


class JointSlider(QWidget):
    """個別の関節スライダーウィジェット"""

    def __init__(self, joint_name: str, display_name: str, min_rad: float, max_rad: float, parent: QWidget | None = None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.min_rad = min_rad
        self.max_rad = max_rad

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        self.name_label = QLabel(display_name)
        self.name_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.name_label)

        min_deg = math.degrees(min_rad)
        max_deg = math.degrees(max_rad)
        range_label = QLabel(f"[{min_deg:.0f}\u00b0 ~ {max_deg:.0f}\u00b0]")
        range_label.setAlignment(Qt.AlignCenter)
        range_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(range_label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)
        self.slider.setValue(self._rad_to_slider(0.0))
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider)

        self.value_label = QLabel("0.0\u00b0")
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)

    def _rad_to_slider(self, rad: float) -> int:
        if self.max_rad == self.min_rad:
            return 500
        ratio = (rad - self.min_rad) / (self.max_rad - self.min_rad)
        return int(max(0, min(1000, ratio * 1000)))

    def _slider_to_rad(self, val: int) -> float:
        ratio = val / 1000.0
        return self.min_rad + ratio * (self.max_rad - self.min_rad)

    def _on_slider_changed(self, val: int) -> None:
        rad = self._slider_to_rad(val)
        deg = math.degrees(rad)
        self.value_label.setText(f"{deg:.1f}\u00b0")

    def get_position(self) -> float:
        """現在の関節角度をラジアンで取得"""
        return self._slider_to_rad(self.slider.value())

    def set_position(self, rad: float) -> None:
        """関節角度をラジアンで設定"""
        self.slider.setValue(self._rad_to_slider(rad))

    def reset(self) -> None:
        """スライダーを0にリセット"""
        self.slider.setValue(self._rad_to_slider(0.0))


class JointGUI(QMainWindow):
    """メインGUIウィンドウ"""

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.sliders: dict[str, JointSlider] = {}

        # アニメーション状態
        self._anim_timer: QTimer | None = None
        self._anim_start: dict[str, float] = {}
        self._anim_target: dict[str, float] = {}
        self._anim_elapsed_ms: int = 0

        self.setWindowTitle("Aoba Joint Control")
        self.setMinimumSize(600, 500)

        self.joint_pub = self.node.create_publisher(JointState, "/joint_states", 10)

        self._setup_ui()

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._publish_joint_states)
        self.publish_timer.start(20)  # 50Hz

    def _setup_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # 左脚・右脚・テールを横に並べる
        groups_layout = QHBoxLayout()

        for group_key, group_label in [("left", "Left Leg"), ("right", "Right Leg"), ("neck", "Neck")]:
            joints = [(name, cfg) for name, cfg in JOINT_CONFIG.items() if cfg[3] == group_key]
            group = QGroupBox(f"{group_label} ({len(joints)})")
            group_layout = QVBoxLayout(group)
            for name, (display_name, lower_deg, upper_deg, _) in joints:
                slider = JointSlider(name, display_name, math.radians(lower_deg), math.radians(upper_deg))
                self.sliders[name] = slider
                group_layout.addWidget(slider)
            groups_layout.addWidget(group)

        main_layout.addLayout(groups_layout)

        # ボタン行
        button_layout = QHBoxLayout()

        reset_btn = QPushButton("Reset All")
        reset_btn.clicked.connect(self._reset_all)
        button_layout.addWidget(reset_btn)

        standby_btn = QPushButton("Walking Standby")
        standby_btn.setStyleSheet("font-weight: bold;")
        standby_btn.clicked.connect(self._walking_standby)
        button_layout.addWidget(standby_btn)

        main_layout.addLayout(button_layout)

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for name, slider in self.sliders.items():
            msg.name.append(name)
            msg.position.append(slider.get_position())
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
        self.joint_pub.publish(msg)

    def _reset_all(self) -> None:
        self._stop_animation()
        for slider in self.sliders.values():
            slider.reset()

    def _walking_standby(self) -> None:
        self._start_smooth_transition(WALKING_STANDBY_POSE)

    # --- スムーズ遷移アニメーション ---

    def _start_smooth_transition(self, target_degrees: dict[str, float]) -> None:
        self._stop_animation()

        # 現在角度を取得
        self._anim_start = {name: slider.get_position() for name, slider in self.sliders.items()}
        # 目標角度をradに変換
        self._anim_target = {name: math.radians(deg) for name, deg in target_degrees.items()}
        self._anim_elapsed_ms = 0

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._anim_step)
        self._anim_timer.start(ANIM_TICK_MS)

    def _anim_step(self) -> None:
        self._anim_elapsed_ms += ANIM_TICK_MS
        t = min(1.0, self._anim_elapsed_ms / ANIM_DURATION_MS)
        # smoothstep: 滑らかな加減速
        t_smooth = t * t * (3.0 - 2.0 * t)

        for name, slider in self.sliders.items():
            start = self._anim_start.get(name, 0.0)
            target = self._anim_target.get(name, start)
            pos = start + (target - start) * t_smooth
            slider.set_position(pos)

        if t >= 1.0:
            self._stop_animation()

    def _stop_animation(self) -> None:
        if self._anim_timer is not None:
            self._anim_timer.stop()
            self._anim_timer.deleteLater()
            self._anim_timer = None


def get_robot_description(node: Node, timeout_sec: float = 10.0) -> str:
    """robot_state_publisherからrobot_descriptionパラメータを取得する"""
    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")

    node.get_logger().info("Waiting for robot_state_publisher service...")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError("robot_state_publisher service not available")

    request = GetParameters.Request()
    request.names = ["robot_description"]

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if future.result() is None:
        raise RuntimeError("Failed to get robot_description parameter")

    response = future.result()
    if not response.values:
        raise RuntimeError("robot_description parameter is empty")

    return response.values[0].string_value


def main() -> None:
    rclpy.init()
    node = Node("joint_gui")

    app = QApplication(sys.argv)

    # robot_descriptionを取得してバリデーション
    try:
        node.get_logger().info("Fetching robot_description from robot_state_publisher...")
        urdf_xml = get_robot_description(node)
        node.get_logger().info(f"Got URDF ({len(urdf_xml)} bytes)")
    except Exception as e:
        QMessageBox.critical(
            None,
            "Error",
            f"Failed to get robot_description:\n{e}\n\n" "Make sure robot_state_publisher is running.",
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # URDFの関節名と設定の整合性チェック
    urdf_joints = set(parse_joint_names_from_urdf(urdf_xml))
    config_joints = set(JOINT_CONFIG.keys())
    missing = config_joints - urdf_joints
    if missing:
        QMessageBox.warning(
            None,
            "Warning",
            f"Joints in config but not in URDF: {sorted(missing)}\n\n" "The GUI will still start, but these joints may not work.",
        )

    node.get_logger().info(f"URDF joints: {sorted(urdf_joints)}")
    node.get_logger().info(f"Config joints: {sorted(config_joints)}")

    # GUI起動
    window = JointGUI(node)
    window.show()

    # ROS2スピンとQtイベントループを統合
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    spin_timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
