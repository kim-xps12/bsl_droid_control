#!/usr/bin/env python3
"""
Single Motor Test Commander

目標位置への軌道生成とテスト自動終了を行うノード．
- /joint_states から現在位置を取得
- 設定速度でリニア補間し目標位置へ移動
- 到達後、設定時間ホールドして自動終了

パラメータ:
  target_position: 目標位置 [rad] (default: 0.0)
  max_velocity:    最大移動速度 [rad/s] (default: 0.5)
  hold_duration:   到達後ホールド時間 [s] (default: 10.0)
  joint_name:      対象ジョイント名 (default: "test_joint")
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SingleMotorTestCommander(Node):
    def __init__(self) -> None:
        super().__init__("single_motor_test_commander")

        self.declare_parameter("target_position", 0.0)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("hold_duration", 10.0)
        self.declare_parameter("joint_name", "test_joint")

        self.target_pos: float = self.get_parameter("target_position").value  # type: ignore[assignment]
        self.max_vel: float = self.get_parameter("max_velocity").value  # type: ignore[assignment]
        self.hold_duration: float = self.get_parameter("hold_duration").value  # type: ignore[assignment]
        self.joint_name: str = self.get_parameter("joint_name").value  # type: ignore[assignment]

        self.start_pos: float | None = None
        self.trajectory_start_time: rclpy.time.Time | None = None
        self.trajectory_duration: float = 0.0
        self.reached_target: bool = False
        self.reached_target_time: rclpy.time.Time | None = None
        self.test_finished: bool = False

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        # 50 Hz command rate
        self.timer = self.create_timer(0.02, self._timer_cb)

        self.get_logger().info(
            f"Test commander: target={math.degrees(self.target_pos):.1f} deg "
            f"({self.target_pos:.4f} rad), max_vel={self.max_vel:.3f} rad/s, "
            f"hold={self.hold_duration:.1f}s"
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        if self.joint_name not in msg.name:
            return
        idx = list(msg.name).index(self.joint_name)
        current_pos = msg.position[idx]

        if self.start_pos is None:
            self.start_pos = current_pos
            distance = abs(self.target_pos - self.start_pos)
            self.trajectory_duration = distance / self.max_vel if self.max_vel > 0 else 0.0
            self.trajectory_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Trajectory: {math.degrees(self.start_pos):.2f} -> "
                f"{math.degrees(self.target_pos):.2f} deg "
                f"(distance={math.degrees(distance):.2f} deg, "
                f"duration={self.trajectory_duration:.2f}s)"
            )

    def _timer_cb(self) -> None:
        if self.start_pos is None or self.trajectory_start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.trajectory_start_time).nanoseconds * 1e-9

        if not self.reached_target:
            if self.trajectory_duration > 0 and elapsed < self.trajectory_duration:
                t = elapsed / self.trajectory_duration
                cmd_pos = self.start_pos + t * (self.target_pos - self.start_pos)
            else:
                cmd_pos = self.target_pos
                self.reached_target = True
                self.reached_target_time = now
                self.get_logger().info(
                    f"Reached target: {math.degrees(self.target_pos):.2f} deg. "
                    f"Holding for {self.hold_duration:.1f}s..."
                )
        else:
            cmd_pos = self.target_pos
            assert self.reached_target_time is not None
            hold_elapsed = (now - self.reached_target_time).nanoseconds * 1e-9
            if hold_elapsed >= self.hold_duration:
                self.get_logger().info(
                    f"Hold complete ({self.hold_duration:.1f}s). Test finished."
                )
                self.test_finished = True
                return

        msg = Float64MultiArray()
        msg.data = [cmd_pos]
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SingleMotorTestCommander()
    try:
        while rclpy.ok() and not node.test_finished:
            rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
