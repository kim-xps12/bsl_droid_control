"""
Abstract base class for trajectory sources.

All trajectory sources produce joint angles (radians) in a canonical order
matching the aoba_description URDF joint configuration.
"""

from abc import ABC, abstractmethod

from rclpy.node import Node


class TrajectorySource(ABC):
    """Abstract base class for trajectory sources.

    Each concrete source generates joint angles in radians at the canonical order:
    [rev11, rev12, rev13, rev14, rev15,   (left leg)
     rev21, rev22, rev23, rev24, rev25,   (right leg)
     rev31]                                (neck)
    """

    JOINT_NAMES: list[str] = [
        "rev11",  # left hip yaw
        "rev12",  # left hip roll
        "rev13",  # left hip pitch
        "rev14",  # left knee pitch
        "rev15",  # left ankle pitch
        "rev21",  # right hip yaw
        "rev22",  # right hip roll
        "rev23",  # right hip pitch
        "rev24",  # right knee pitch
        "rev25",  # right ankle pitch
        "rev31",  # neck
    ]

    NUM_JOINTS: int = len(JOINT_NAMES)

    @abstractmethod
    def configure(self, node: Node) -> None:
        """Read ROS 2 parameters from the owning node and initialize.

        Args:
            node: The ROS 2 node that owns this source.
                  Sources should declare their parameters on this node.
        """

    @abstractmethod
    def compute(self, elapsed_sec: float) -> list[float]:
        """Compute joint angles in radians for the given time.

        Args:
            elapsed_sec: Elapsed time since start in seconds.

        Returns:
            List of NUM_JOINTS joint angles in radians, in canonical order.
        """

    @abstractmethod
    def reset(self) -> None:
        """Reset internal state (phase counter, waypoint index, etc.)."""

    @property
    def speed_scale(self) -> float:
        """Current speed scaling factor (-1.0 = full reverse, 0.0 = stopped, 1.0 = full speed).

        Default implementation always returns 1.0 (no speed control).
        Override in subclasses that support dynamic speed scaling.
        """
        return 1.0

    @speed_scale.setter
    def speed_scale(self, value: float) -> None:
        """Set speed scale. Default implementation is a no-op."""

    @property
    def turn_scale(self) -> float:
        """Current turn scaling factor (-1.0 = full right, 0.0 = straight, 1.0 = full left).

        Default implementation always returns 0.0 (no turning).
        Override in subclasses that support dynamic turn control.
        """
        return 0.0

    @turn_scale.setter
    def turn_scale(self, value: float) -> None:
        """Set turn scale. Default implementation is a no-op."""
