"""
Abstract base class for trajectory sources.

All trajectory sources produce 10 joint angles (radians) in a canonical order
matching the forward_position_controller joint configuration.
"""

from abc import ABC, abstractmethod

from rclpy.node import Node


class TrajectorySource(ABC):
    """Abstract base class for trajectory sources.

    Each concrete source generates 10 joint angles in radians at the canonical order:
    [L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee_pitch, L_ankle_pitch,
     R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee_pitch, R_ankle_pitch]
    """

    JOINT_NAMES: list[str] = [
        "left_hip_yaw_joint",
        "left_hip_roll_joint",
        "left_hip_pitch_joint",
        "left_knee_pitch_joint",
        "left_ankle_pitch_joint",
        "right_hip_yaw_joint",
        "right_hip_roll_joint",
        "right_hip_pitch_joint",
        "right_knee_pitch_joint",
        "right_ankle_pitch_joint",
    ]

    @abstractmethod
    def configure(self, node: Node) -> None:
        """Read ROS 2 parameters from the owning node and initialize.

        Args:
            node: The ROS 2 node that owns this source.
                  Sources should declare their parameters on this node.
        """

    @abstractmethod
    def compute(self, elapsed_sec: float) -> list[float]:
        """Compute 10 joint angles in radians for the given time.

        Args:
            elapsed_sec: Elapsed time since start in seconds.

        Returns:
            List of 10 joint angles in radians, in canonical order.
        """

    @abstractmethod
    def reset(self) -> None:
        """Reset internal state (phase counter, waypoint index, etc.)."""
