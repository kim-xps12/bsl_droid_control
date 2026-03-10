# Copyright 2024, All rights reserved.
#
# Licensed under the MIT License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class BipedJoySafetyNode(Node):
    """Emergency stop button monitoring and gamepad disconnect detection.

    Subscribes to /joy and publishes /emergency_stop (Bool).
    - L3+R3 simultaneous press → emergency_stop toggle
    - Gamepad disconnect (joy_timeout) → emergency_stop = True
    """

    def __init__(self) -> None:
        super().__init__('biped_joy_safety_node')

        # Parameters
        self.declare_parameter('emergency_stop_buttons', [9, 10])
        self.declare_parameter('joy_timeout', 0.5)

        self._estop_buttons: list[int] = self.get_parameter('emergency_stop_buttons').value
        self._joy_timeout: float = self.get_parameter('joy_timeout').value

        # State
        self._emergency_stop = False
        self._prev_all_pressed = False
        self._last_joy_time = self.get_clock().now()
        self._joy_received = False

        # Pub/Sub
        self._estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self._joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        # 20Hz timer for timeout monitoring
        self._timer = self.create_timer(0.05, self._timer_callback)

        self.get_logger().info(
            f'Joy safety node started (estop_buttons={self._estop_buttons}, '
            f'timeout={self._joy_timeout}s)'
        )

    def _joy_callback(self, msg: Joy) -> None:
        self._last_joy_time = self.get_clock().now()
        self._joy_received = True

        # Rising edge detection on simultaneous press of all emergency stop buttons
        all_pressed = all(
            len(msg.buttons) > btn and msg.buttons[btn] == 1 for btn in self._estop_buttons
        )
        if all_pressed and not self._prev_all_pressed:
            self._emergency_stop = not self._emergency_stop
            if self._emergency_stop:
                self.get_logger().warn('Emergency stop ACTIVATED (L3+R3)')
            else:
                self.get_logger().info('Emergency stop DEACTIVATED (L3+R3)')
            self._publish_estop()
        self._prev_all_pressed = all_pressed

    def _timer_callback(self) -> None:
        if not self._joy_received:
            return

        # Check for gamepad disconnect (timeout)
        elapsed = (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9
        if elapsed > self._joy_timeout:
            if not self._emergency_stop:
                self._emergency_stop = True
                self.get_logger().warn(
                    f'Emergency stop ACTIVATED (gamepad timeout: {elapsed:.2f}s)'
                )
            self._publish_estop()
        else:
            # Periodically publish current state
            self._publish_estop()

    def _publish_estop(self) -> None:
        msg = Bool()
        msg.data = self._emergency_stop
        self._estop_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BipedJoySafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
