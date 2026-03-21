#!/usr/bin/env python3
"""
Multi Motor Zero Commander

複数ジョイントを同時にゼロ点（0 rad）へ移動するノード。
各ジョイントは独立に線形補間で移動し、近いジョイントは先に到着して0.0でホールドする。
全ジョイントがゼロ点に到達後、hold_duration秒ホールドして自動終了する。

パラメータ:
  joint_names:   カンマ区切りのジョイント名 (例: "motor_can1_11,motor_can1_12")
  max_velocity:  最大移動速度 [rad/s] (default: 0.5)
  hold_duration: 全到達後ホールド時間 [s] (default: 10.0)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MultiMotorZeroCommander(Node):
    def __init__(self) -> None:
        super().__init__('multi_motor_zero_commander')

        self.declare_parameter('joint_names', '')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('hold_duration', 10.0)

        joint_names_str: str = self.get_parameter('joint_names').value  # type: ignore[assignment]
        self.max_vel: float = self.get_parameter('max_velocity').value  # type: ignore[assignment]
        self.hold_duration: float = self.get_parameter('hold_duration').value  # type: ignore[assignment]

        if not joint_names_str:
            raise RuntimeError('joint_names parameter is required')
        self.joint_names: list[str] = [n.strip() for n in joint_names_str.split(',')]
        self.num_joints: int = len(self.joint_names)

        # Per-joint trajectory state
        self.start_positions: dict[str, float] = {}
        self.durations: dict[str, float] = {}
        self.reached: dict[str, bool] = {name: False for name in self.joint_names}

        self.trajectory_start_time: rclpy.time.Time | None = None
        self.all_reached: bool = False
        self.all_reached_time: rclpy.time.Time | None = None
        self.test_finished: bool = False

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10,
        )
        # 50 Hz command rate
        self.timer = self.create_timer(0.02, self._timer_cb)

        self.get_logger().info(
            f'Multi motor zero commander: {self.num_joints} joints, '
            f'max_vel={self.max_vel:.3f} rad/s, hold={self.hold_duration:.1f}s'
        )
        self.get_logger().info(f'Joints: {self.joint_names}')

    def _joint_state_cb(self, msg: JointState) -> None:
        """Collect initial positions from /joint_states."""
        if self.trajectory_start_time is not None:
            return  # Already initialized

        msg_names = list(msg.name)
        for name in self.joint_names:
            if name in self.start_positions:
                continue
            if name in msg_names:
                idx = msg_names.index(name)
                pos = msg.position[idx]
                self.start_positions[name] = pos
                distance = abs(pos)
                duration = distance / self.max_vel if self.max_vel > 0 else 0.0
                self.durations[name] = duration
                self.get_logger().info(
                    f'  {name}: start={math.degrees(pos):.2f} deg, '
                    f'distance={math.degrees(distance):.2f} deg, '
                    f'duration={duration:.2f}s'
                )

        # All joints initialized?
        if len(self.start_positions) == self.num_joints:
            self.trajectory_start_time = self.get_clock().now()
            self.get_logger().info('All joint positions acquired. Starting trajectory.')

    def _timer_cb(self) -> None:
        if self.trajectory_start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self.trajectory_start_time).nanoseconds * 1e-9

        cmd_positions: list[float] = []
        for name in self.joint_names:
            start_pos = self.start_positions[name]
            duration = self.durations[name]

            if not self.reached[name]:
                if duration > 0 and elapsed < duration:
                    t = elapsed / duration
                    cmd_pos = start_pos + t * (0.0 - start_pos)
                else:
                    cmd_pos = 0.0
                    self.reached[name] = True
                    self.get_logger().info(
                        f'  {name}: reached zero '
                        f'(from {math.degrees(start_pos):.2f} deg)'
                    )
            else:
                cmd_pos = 0.0

            cmd_positions.append(cmd_pos)

        # Check if all joints reached zero
        if not self.all_reached and all(self.reached.values()):
            self.all_reached = True
            self.all_reached_time = now
            self.get_logger().info(
                f'All {self.num_joints} joints reached zero. '
                f'Holding for {self.hold_duration:.1f}s...'
            )

        # Hold duration check
        if self.all_reached and self.all_reached_time is not None:
            hold_elapsed = (now - self.all_reached_time).nanoseconds * 1e-9
            if hold_elapsed >= self.hold_duration:
                self.get_logger().info(
                    f'Hold complete ({self.hold_duration:.1f}s). Test finished.'
                )
                self.test_finished = True
                return

        msg = Float64MultiArray()
        msg.data = cmd_positions
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MultiMotorZeroCommander()
    try:
        while rclpy.ok() and not node.test_finished:
            rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
