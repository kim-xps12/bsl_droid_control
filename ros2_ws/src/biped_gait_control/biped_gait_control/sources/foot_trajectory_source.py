"""
Foot trajectory source using CamberTrajectory + inverse kinematics.

Wraps the existing WalkingPatternGenerator to produce 10 joint angles
from an elliptical arc foot trajectory with 2-link IK.
"""

import numpy as np
from rclpy.node import Node

from ..trajectory import GaitParameters, WalkingPatternGenerator
from ..trajectory_source import TrajectorySource


class FootTrajectorySource(TrajectorySource):
    """Foot-level trajectory using CamberTrajectory + IK."""

    def configure(self, node: Node) -> None:
        node.declare_parameter("foot.step_height", 0.04)
        node.declare_parameter("foot.step_length", 0.08)
        node.declare_parameter("foot.step_frequency", 0.5)
        node.declare_parameter("foot.leg_extension_ratio", 0.90)
        node.declare_parameter("foot.thigh_length", 0.11)
        node.declare_parameter("foot.shank_length", 0.12)

        gait_params = GaitParameters(
            step_height=node.get_parameter("foot.step_height").value,
            step_length=node.get_parameter("foot.step_length").value,
            step_frequency=node.get_parameter("foot.step_frequency").value,
            leg_extension_ratio=node.get_parameter("foot.leg_extension_ratio").value,
        )

        thigh_length = node.get_parameter("foot.thigh_length").value
        shank_length = node.get_parameter("foot.shank_length").value

        self._generator = WalkingPatternGenerator(
            gait_params=gait_params,
            thigh_length=thigh_length,
            shank_length=shank_length,
        )

        node.get_logger().info(
            f"FootTrajectorySource configured: "
            f"step_height={gait_params.step_height}, "
            f"step_length={gait_params.step_length}, "
            f"step_frequency={gait_params.step_frequency}, "
            f"thigh={thigh_length}, shank={shank_length}"
        )

    def compute(self, elapsed_sec: float) -> list[float]:
        left_deg, right_deg = self._generator.generate(elapsed_sec)
        # Convert degrees to radians
        left_rad = [np.radians(a) for a in left_deg]
        right_rad = [np.radians(a) for a in right_deg]
        return left_rad + right_rad

    def reset(self) -> None:
        pass  # Stateless (phase is computed from elapsed_sec)
