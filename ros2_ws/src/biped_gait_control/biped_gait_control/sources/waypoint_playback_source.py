"""
Waypoint playback source with linear interpolation.

Plays back a sequence of multi-joint waypoints, interpolating
linearly between them. Supports looping.
"""

import numpy as np
from rclpy.node import Node

from ..trajectory_source import TrajectorySource


class WaypointPlaybackSource(TrajectorySource):
    """Multi-joint waypoint playback with linear interpolation."""

    def configure(self, node: Node) -> None:
        n_joints = self.NUM_JOINTS
        node.declare_parameter("waypoint.times", [0.0, 2.0])
        node.declare_parameter("waypoint.positions", [0.0] * (2 * n_joints))
        node.declare_parameter("waypoint.loop", True)

        times = list(node.get_parameter("waypoint.times").value)
        positions_flat = list(node.get_parameter("waypoint.positions").value)
        self._loop: bool = node.get_parameter("waypoint.loop").value

        n_waypoints = len(times)
        expected_len = n_waypoints * n_joints

        if len(positions_flat) != expected_len:
            raise ValueError(
                f"waypoint.positions must have {expected_len} elements "
                f"({n_waypoints} waypoints × {n_joints} joints), got {len(positions_flat)}"
            )

        if n_waypoints < 2:
            raise ValueError("At least 2 waypoints are required")

        self._times = np.array(times)

        if not np.all(np.diff(self._times) > 0):
            raise ValueError("waypoint.times must be strictly monotonically increasing")

        self._positions = np.array(positions_flat).reshape(n_waypoints, n_joints)
        self._duration = self._times[-1] - self._times[0]

        node.get_logger().info(
            f"WaypointPlaybackSource configured: "
            f"{n_waypoints} waypoints, duration={self._duration:.2f}s, "
            f"loop={self._loop}"
        )

    def compute(self, elapsed_sec: float) -> list[float]:
        if self._loop:
            t = self._times[0] + ((elapsed_sec - self._times[0]) % self._duration)
        else:
            t = max(self._times[0], min(elapsed_sec, self._times[-1]))

        # Find bracketing waypoint indices
        idx = int(np.searchsorted(self._times, t, side="right") - 1)
        idx = max(0, min(idx, len(self._times) - 2))

        t0 = self._times[idx]
        t1 = self._times[idx + 1]
        dt = t1 - t0

        if dt <= 0:
            alpha = 0.0
        else:
            alpha = (t - t0) / dt

        interpolated = (1.0 - alpha) * self._positions[idx] + alpha * self._positions[idx + 1]
        return interpolated.tolist()

    def reset(self) -> None:
        pass  # Stateless (position is computed from elapsed_sec)
