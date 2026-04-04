#!/usr/bin/env python3
"""
ROS 2 Trajectory Replay Node.

Generic trajectory replay node that supports multiple trajectory sources
via the Strategy pattern. Publishes joint commands for both rviz2
visualization and real hardware control.

In control mode, an initialization phase smoothly ramps from the robot's
current joint positions to the trajectory's initial pose before replay begins.

Published Topics:
    /cmd/joint_states (sensor_msgs/JointState):
        Joint angle targets for all joints including neck (always published).
        Used for RViz visualization and PlotJuggler comparison.

    /forward_position_controller/commands (std_msgs/Float64MultiArray):
        Position commands for ros2_control hardware (control mode only).
        Contains 10 actuated joints (excludes neck).

Subscribed Topics:
    /emergency_stop (std_msgs/Bool):
        Emergency stop signal. When True, outputs default positions.

    /joint_states (sensor_msgs/JointState):
        Current joint positions from hardware (control mode only).
        Used during initialization to read the robot's current pose.

Parameters:
    source_type (string): Trajectory source type ("foot", "oscillation", "waypoint")
    mode (string): Operating mode ("viz" or "control")
    publish_rate (double): Publishing rate in Hz (default: 50.0)
    enabled (bool): Master enable (default: true)
    init_duration (double): Ramp duration from current to initial pose [s] (default: 3.0)
    init_state_timeout (double): Timeout waiting for /joint_states [s] (default: 10.0)

    Additional parameters depend on the selected source_type.
    See each source's configure() method for details.
"""

import math
from enum import Enum, auto

import rclpy
from aoba_description.joint_limits import clamp_joint_angles
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Header

from .sources.foot_trajectory_source import FootTrajectorySource
from .sources.single_joint_oscillation_source import SingleJointOscillationSource
from .sources.waypoint_playback_source import WaypointPlaybackSource
from .trajectory_source import TrajectorySource


class ReplayPhase(Enum):
    """State machine phases for trajectory replay."""

    WAITING_FOR_STATE = auto()
    MOVING_TO_INITIAL = auto()
    REPLAYING = auto()


class TrajectoryReplayNode(Node):
    """ROS 2 node for replaying pre-designed trajectories."""

    def __init__(self) -> None:
        super().__init__("trajectory_replay")

        # Core parameters
        self.declare_parameter("source_type", "foot")
        self.declare_parameter("mode", "viz")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("init_duration", 1.5)
        self.declare_parameter("init_state_timeout", 10.0)

        source_type: str = self.get_parameter("source_type").value
        self._mode: str = self.get_parameter("mode").value
        publish_rate: float = self.get_parameter("publish_rate").value
        self._enabled: bool = self.get_parameter("enabled").value
        self._init_duration: float = self.get_parameter("init_duration").value
        self._init_state_timeout: float = self.get_parameter("init_state_timeout").value

        # Create trajectory source
        self._source = self._create_source(source_type)
        self._source.configure(self)

        # E-stop state
        self._estop_active = False

        # Initialization phase state
        if self._mode == "control":
            self._phase = ReplayPhase.WAITING_FOR_STATE
        else:
            self._phase = ReplayPhase.REPLAYING

        self._current_joint_positions: list[float] | None = None
        self._init_start_positions: list[float] | None = None
        self._init_target_positions: list[float] | None = None
        self._init_start_time: object | None = None

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._joint_state_pub = self.create_publisher(
            JointState, "/cmd/joint_states", qos
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

        # Joint state subscriber (control mode only, for initialization)
        if self._mode == "control":
            self.create_subscription(
                JointState, "/joint_states", self._joint_state_callback, 10
            )

        # Timer
        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)
        self._start_time = self.get_clock().now()
        self._phase_start_time = self.get_clock().now()
        self._log_count = 0

        self.get_logger().info(
            f"TrajectoryReplayNode started:\n"
            f"  Source: {source_type}\n"
            f"  Mode: {self._mode}\n"
            f"  Rate: {publish_rate} Hz\n"
            f"  Enabled: {self._enabled}\n"
            f"  Phase: {self._phase.name}\n"
            f"  Init duration: {self._init_duration:.1f}s\n"
            f"  Init state timeout: {self._init_state_timeout:.1f}s"
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

    def _joint_state_callback(self, msg: JointState) -> None:
        """Store current joint positions from hardware feedback."""
        name_to_pos = dict(zip(msg.name, msg.position, strict=False))
        positions = [name_to_pos.get(name, 0.0) for name in TrajectorySource.JOINT_NAMES]
        self._current_joint_positions = positions

    def _timer_callback(self) -> None:
        now = self.get_clock().now()

        # E-stop / disabled: output zeros regardless of phase
        if not self._enabled or self._estop_active:
            positions = [0.0] * TrajectorySource.NUM_JOINTS
            positions = clamp_joint_angles(positions)
            self._publish_joint_state(positions, now)
            if self._hw_cmd_pub is not None:
                self._publish_hw_command(positions)
            return

        # Phase dispatch
        if self._phase == ReplayPhase.WAITING_FOR_STATE:
            self._handle_waiting_phase(now)
            return

        if self._phase == ReplayPhase.MOVING_TO_INITIAL:
            positions = self._handle_init_phase(now)
        else:  # REPLAYING
            elapsed_sec = (now - self._start_time).nanoseconds / 1e9
            positions = self._source.compute(elapsed_sec)

        # Safety: clamp to joint limits
        positions = clamp_joint_angles(positions)

        # Publish
        self._publish_joint_state(positions, now)
        if self._hw_cmd_pub is not None:
            self._publish_hw_command(positions)

        # Periodic logging (every 2 seconds)
        self._log_count += 1
        publish_rate = self.get_parameter("publish_rate").value
        if self._log_count % int(publish_rate * 2) == 0:
            if self._phase == ReplayPhase.REPLAYING:
                elapsed_sec = (now - self._start_time).nanoseconds / 1e9
            else:
                elapsed_sec = 0.0
            self.get_logger().info(
                f"[{self._phase.name}] t={elapsed_sec:.2f}s | "
                f"pos[2]={positions[2]:+.3f} pos[3]={positions[3]:+.3f} "
                f"pos[7]={positions[7]:+.3f} pos[8]={positions[8]:+.3f}"
            )

    def _handle_waiting_phase(self, now: object) -> None:
        """Wait for /joint_states, then transition to MOVING_TO_INITIAL."""
        waiting_sec = (now - self._phase_start_time).nanoseconds / 1e9
        if waiting_sec > self._init_state_timeout:
            self.get_logger().error(
                f"Timed out waiting for /joint_states after "
                f"{self._init_state_timeout:.1f}s. "
                f"Aborting - is the hardware bringup running?"
            )
            self._enabled = False
            return

        if self._current_joint_positions is not None:
            self._init_start_positions = list(self._current_joint_positions)
            self._init_target_positions = self._source.compute(0.0)
            self._init_start_time = now
            self._phase = ReplayPhase.MOVING_TO_INITIAL

            self.get_logger().info(
                f"Received /joint_states. Starting initialization ramp "
                f"({self._init_duration:.1f}s) to trajectory initial pose."
            )
            for i, name in enumerate(TrajectorySource.JOINT_NAMES[:10]):
                delta = self._init_target_positions[i] - self._init_start_positions[i]
                self.get_logger().info(
                    f"  {name}: "
                    f"{math.degrees(self._init_start_positions[i]):+.1f} deg "
                    f"-> {math.degrees(self._init_target_positions[i]):+.1f} deg "
                    f"(delta={math.degrees(delta):+.1f} deg)"
                )

    def _handle_init_phase(self, now: object) -> list[float]:
        """Linearly interpolate from current positions to initial pose."""
        elapsed = (now - self._init_start_time).nanoseconds / 1e9
        t = min(elapsed / self._init_duration, 1.0)

        positions = [
            start + t * (target - start)
            for start, target in zip(
                self._init_start_positions, self._init_target_positions, strict=True
            )
        ]

        if t >= 1.0:
            self._phase = ReplayPhase.REPLAYING
            self._start_time = self.get_clock().now()
            self._source.reset()
            self.get_logger().info(
                "Initialization complete. Transitioning to trajectory replay."
            )

        return positions

    def _publish_joint_state(self, positions: list[float], stamp: object) -> None:
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(TrajectorySource.JOINT_NAMES)
        msg.position = positions
        msg.velocity = [0.0] * TrajectorySource.NUM_JOINTS
        msg.effort = [0.0] * TrajectorySource.NUM_JOINTS
        self._joint_state_pub.publish(msg)

    def _publish_hw_command(self, positions: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = positions[:10]  # 10 actuated joints (exclude neck)
        self._hw_cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    import signal

    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = TrajectoryReplayNode()

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
