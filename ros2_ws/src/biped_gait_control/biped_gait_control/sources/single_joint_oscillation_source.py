"""
Single joint oscillation source for hardware verification.

Oscillates a single specified joint sinusoidally while holding
all other joints at their default positions.
"""

import math

from rclpy.node import Node

from ..trajectory_source import TrajectorySource


class SingleJointOscillationSource(TrajectorySource):
    """Oscillate a single joint for hardware testing."""

    def configure(self, node: Node) -> None:
        n_joints = self.NUM_JOINTS
        node.declare_parameter("oscillation.joint_name", "rev14")
        node.declare_parameter("oscillation.amplitude", 0.3)
        node.declare_parameter("oscillation.frequency", 0.5)
        node.declare_parameter("oscillation.offset", 0.0)
        node.declare_parameter("oscillation.default_positions", [0.0] * n_joints)

        self._joint_name: str = node.get_parameter("oscillation.joint_name").value
        self._amplitude: float = node.get_parameter("oscillation.amplitude").value
        self._frequency: float = node.get_parameter("oscillation.frequency").value
        self._offset: float = node.get_parameter("oscillation.offset").value
        self._default_positions: list[float] = list(
            node.get_parameter("oscillation.default_positions").value
        )

        if self._joint_name not in self.JOINT_NAMES:
            raise ValueError(
                f"Unknown joint '{self._joint_name}'. "
                f"Valid joints: {self.JOINT_NAMES}"
            )

        self._joint_index = self.JOINT_NAMES.index(self._joint_name)

        if len(self._default_positions) != n_joints:
            raise ValueError(
                f"default_positions must have {n_joints} elements, got {len(self._default_positions)}"
            )

        node.get_logger().info(
            f"SingleJointOscillationSource configured: "
            f"joint={self._joint_name} (index={self._joint_index}), "
            f"amplitude={self._amplitude}, frequency={self._frequency}, "
            f"offset={self._offset}"
        )

    def compute(self, elapsed_sec: float) -> list[float]:
        positions = list(self._default_positions)
        positions[self._joint_index] = (
            self._default_positions[self._joint_index]
            + self._offset
            + self._amplitude * math.sin(2.0 * math.pi * self._frequency * elapsed_sec)
        )
        return positions

    def reset(self) -> None:
        pass  # Stateless (phase is computed from elapsed_sec)
