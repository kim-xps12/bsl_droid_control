#!/usr/bin/env python3
"""
ROS 2 Trajectory Replay Node.

Generic trajectory replay node that supports multiple trajectory sources
via the Strategy pattern. Publishes joint commands for both rviz2
visualization and real hardware control.

Published Topics:
    /joint_states (sensor_msgs/JointState):
        Joint angle targets for all leg joints (always published).

    /forward_position_controller/commands (std_msgs/Float64MultiArray):
        Position commands for ros2_control hardware (control mode only).

Subscribed Topics:
    /emergency_stop (std_msgs/Bool):
        Emergency stop signal. When True, outputs default positions.

Parameters:
    source_type (string): Trajectory source type ("foot", "oscillation", "waypoint")
    mode (string): Operating mode ("viz" or "control")
    publish_rate (double): Publishing rate in Hz (default: 50.0)
    enabled (bool): Master enable (default: true)

    Additional parameters depend on the selected source_type.
    See each source's configure() method for details.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Header

from .joint_limits import clamp_joint_angles
from .sources.foot_trajectory_source import FootTrajectorySource
from .sources.single_joint_oscillation_source import SingleJointOscillationSource
from .sources.waypoint_playback_source import WaypointPlaybackSource
from .trajectory_source import TrajectorySource


class TrajectoryReplayNode(Node):
    """ROS 2 node for replaying pre-designed trajectories."""

    def __init__(self) -> None:
        super().__init__("trajectory_replay")

        # Core parameters
        self.declare_parameter("source_type", "foot")
        self.declare_parameter("mode", "viz")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("enabled", True)

        source_type: str = self.get_parameter("source_type").value
        self._mode: str = self.get_parameter("mode").value
        publish_rate: float = self.get_parameter("publish_rate").value
        self._enabled: bool = self.get_parameter("enabled").value

        # Create trajectory source
        self._source = self._create_source(source_type)
        self._source.configure(self)

        # E-stop state
        self._estop_active = False

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._joint_state_pub = self.create_publisher(
            JointState, "/joint_states", qos
        )

        self._hw_cmd_pub = None
        if self._mode == "control":
            self._hw_cmd_pub = self.create_publisher(
                Float64MultiArray,
                "/forward_position_controller/commands",
                10,
            )

        # E-stop subscriber
        self.create_subscription(
            Bool, "/emergency_stop", self._estop_callback, qos
        )

        # Timer
        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)
        self._start_time = self.get_clock().now()
        self._log_count = 0

        self.get_logger().info(
            f"TrajectoryReplayNode started:\n"
            f"  Source: {source_type}\n"
            f"  Mode: {self._mode}\n"
            f"  Rate: {publish_rate} Hz\n"
            f"  Enabled: {self._enabled}"
        )

    @staticmethod
    def _create_source(source_type: str) -> TrajectorySource:
        """Create a trajectory source by type name."""
        sources: dict[str, type[TrajectorySource]] = {
            "foot": FootTrajectorySource,
            "oscillation": SingleJointOscillationSource,
            "waypoint": WaypointPlaybackSource,
        }
        if source_type not in sources:
            raise ValueError(
                f"Unknown source_type '{source_type}'. "
                f"Valid types: {list(sources.keys())}"
            )
        return sources[source_type]()

    def _estop_callback(self, msg: Bool) -> None:
        if msg.data and not self._estop_active:
            self.get_logger().warn("Emergency stop ACTIVATED")
        elif not msg.data and self._estop_active:
            self.get_logger().info("Emergency stop released")
        self._estop_active = msg.data

    def _timer_callback(self) -> None:
        now = self.get_clock().now()
        elapsed_sec = (now - self._start_time).nanoseconds / 1e9

        if not self._enabled or self._estop_active:
            # Output zeros (default standing pose)
            positions = [0.0] * 10
        else:
            positions = self._source.compute(elapsed_sec)

        # Safety: clamp to joint limits
        positions = clamp_joint_angles(positions)

        # Publish /joint_states (always)
        self._publish_joint_state(positions, now)

        # Publish hardware commands (control mode only)
        if self._hw_cmd_pub is not None:
            self._publish_hw_command(positions)

        # Periodic logging (every 2 seconds)
        self._log_count += 1
        publish_rate = self.get_parameter("publish_rate").value
        if self._log_count % int(publish_rate * 2) == 0:
            self.get_logger().info(
                f"t={elapsed_sec:.2f}s | "
                f"pos[2]={positions[2]:+.3f} pos[3]={positions[3]:+.3f} "
                f"pos[7]={positions[7]:+.3f} pos[8]={positions[8]:+.3f}"
            )

    def _publish_joint_state(self, positions: list[float], stamp: object) -> None:
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(TrajectorySource.JOINT_NAMES)
        msg.position = positions
        msg.velocity = [0.0] * 10
        msg.effort = [0.0] * 10
        self._joint_state_pub.publish(msg)

    def _publish_hw_command(self, positions: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = positions
        self._hw_cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryReplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
