#!/usr/bin/env python3
"""
カスタム関節操作GUI

左右の脚を2列に並べて表示し、全10関節をスライダーで操作可能。
joint_state_publisher_guiの代替として使用。
"""

import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QLabel, QSlider, QGroupBox, QGridLayout, QPushButton
    )
    from PyQt5.QtCore import Qt, QTimer
except ImportError:
    print("PyQt5 is required. Install with: pip install PyQt5")
    sys.exit(1)


class JointSlider(QWidget):
    """個別の関節スライダーウィジェット"""
    
    def __init__(self, joint_name: str, min_deg: float, max_deg: float, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.min_rad = math.radians(min_deg)
        self.max_rad = math.radians(max_deg)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # 関節名ラベル（短縮表示）
        short_name = joint_name.replace('_joint', '').replace('left_', 'L:').replace('right_', 'R:')
        self.name_label = QLabel(short_name)
        self.name_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.name_label)
        
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
        return int(ratio * 1000)
    
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
    
    # 関節定義: (名前, 最小角度[deg], 最大角度[deg])
    LEFT_JOINTS = [
        ('left_hip_yaw_joint', -30, 30),
        ('left_hip_roll_joint', -20, 20),
        ('left_hip_pitch_joint', -90, 90),
        ('left_knee_pitch_joint', -120, 0),
        ('left_ankle_pitch_joint', -45, 135),
    ]
    
    RIGHT_JOINTS = [
        ('right_hip_yaw_joint', -30, 30),
        ('right_hip_roll_joint', -20, 20),
        ('right_hip_pitch_joint', -90, 90),
        ('right_knee_pitch_joint', -120, 0),
        ('right_ankle_pitch_joint', -45, 135),
    ]
    
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.sliders: dict[str, JointSlider] = {}
        
        self.setWindowTitle("Biped Joint Control")
        self.setMinimumSize(500, 400)
        
        # JointState パブリッシャー
        self.joint_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        
        self._setup_ui()
        
        # 定期的にJointStateをパブリッシュ (50Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._publish_joint_states)
        self.timer.start(20)
    
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QVBoxLayout(central)
        
        # 左右の脚を横に並べる
        legs_layout = QHBoxLayout()
        
        # 左脚グループ
        left_group = QGroupBox("Left Leg (j1x)")
        left_layout = QVBoxLayout(left_group)
        for name, min_deg, max_deg in self.LEFT_JOINTS:
            slider = JointSlider(name, min_deg, max_deg)
            self.sliders[name] = slider
            left_layout.addWidget(slider)
        legs_layout.addWidget(left_group)
        
        # 右脚グループ
        right_group = QGroupBox("Right Leg (j2x)")
        right_layout = QVBoxLayout(right_group)
        for name, min_deg, max_deg in self.RIGHT_JOINTS:
            slider = JointSlider(name, min_deg, max_deg)
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
        # 逆関節の自然な立位: 膝を-30度程度曲げる
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


def main():
    rclpy.init()
    node = Node('joint_gui')
    
    app = QApplication(sys.argv)
    window = JointGUI(node)
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
