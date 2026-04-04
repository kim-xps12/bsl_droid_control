#!/usr/bin/env python3
"""
Joint State Bridge Node.

Subscribes to /joint_states (sensor_msgs/JointState) and republishes as
/forward_position_controller/commands (std_msgs/Float64MultiArray).

Only processes 11-joint messages (GUI origin) and ignores 10-joint messages
(hardware broadcaster origin). Extracts the 10 actuated joints (rev11-rev25,
skipping rev31=neck), applies joint limit clamping, and publishes.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from aoba_description.joint_limits import JOINT_LIMITS, clamp_joint_angles

# 11-joint canonical order (matching TrajectorySource / joint_gui.py)
_ALL_JOINT_NAMES: list[str] = [
    "rev11", "rev12", "rev13", "rev14", "rev15",
    "rev21", "rev22", "rev23", "rev24", "rev25",
    "rev31",
]

# 10 actuated joints sent to hardware (neck excluded)
_HW_JOINT_NAMES: list[str] = [
    "rev11", "rev12", "rev13", "rev14", "rev15",
    "rev21", "rev22", "rev23", "rev24", "rev25",
]

_NUM_GUI_JOINTS = len(_ALL_JOINT_NAMES)  # 11
_NUM_HW_JOINTS = len(_HW_JOINT_NAMES)   # 10


class JointStateBridgeNode(Node):
    """Bridge /joint_states → /forward_position_controller/commands."""

    def __init__(self) -> None:
        super().__init__("joint_state_bridge")

        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        self._log_count = 0
        self.get_logger().info(
            "JointStateBridgeNode started: "
            "/joint_states (11-axis) -> /forward_position_controller/commands (10-axis)"
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        # Only process 11-joint messages (from GUI), ignore 10-joint (from hw broadcaster)
        if len(msg.name) != _NUM_GUI_JOINTS:
            return

        # Build name→position map
        name_to_pos: dict[str, float] = dict(zip(msg.name, msg.position))

        # Extract 11-joint positions in canonical order for clamping
        try:
            all_positions = [name_to_pos[name] for name in _ALL_JOINT_NAMES]
        except KeyError:
            return  # Unexpected joint names, skip

        # Clamp all 11 joints
        clamped = clamp_joint_angles(all_positions, log_warnings=False)

        # Extract 10 actuated joints (exclude neck = index 10)
        hw_positions = clamped[:_NUM_HW_JOINTS]

        # Publish
        cmd_msg = Float64MultiArray()
        cmd_msg.data = hw_positions
        self._cmd_pub.publish(cmd_msg)

        # Periodic logging
        self._log_count += 1
        if self._log_count % 100 == 0:
            self.get_logger().info(
                f"Bridge: {hw_positions[0]:+.3f} {hw_positions[1]:+.3f} ... "
                f"({_NUM_HW_JOINTS} joints)"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = JointStateBridgeNode()

    def _shutdown(sig: int, _frame: object) -> None:
        if rclpy.ok():
            node.get_logger().info(f"Received signal {sig}, shutting down...")
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
