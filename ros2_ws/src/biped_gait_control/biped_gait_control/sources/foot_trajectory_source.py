"""
Foot trajectory source using CamberTrajectory + inverse kinematics.

Wraps the existing WalkingPatternGenerator to produce joint angles
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
        node.declare_parameter("foot.thigh_length", 0.125)
        node.declare_parameter("foot.shank_length", 0.13)

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

        # Virtual time accumulator for speed scaling
        self._speed_scale: float = 1.0
        self._virtual_time: float = 0.0
        self._last_elapsed: float = 0.0

        node.get_logger().info(
            f"FootTrajectorySource configured: "
            f"step_height={gait_params.step_height}, "
            f"step_length={gait_params.step_length}, "
            f"step_frequency={gait_params.step_frequency}, "
            f"thigh={thigh_length}, shank={shank_length}"
        )

    @property
    def speed_scale(self) -> float:
        return self._speed_scale

    @speed_scale.setter
    def speed_scale(self, value: float) -> None:
        self._speed_scale = max(0.0, min(value, 1.0))

    @staticmethod
    def _apply_axis_sign(angles_rad: list[float]) -> list[float]:
        """Negate pitch angles for aoba's (0, -1, 0) axis convention.

        IK outputs assume positive-Y axis. aoba's hip_pitch, knee_pitch,
        ankle_pitch joints all use negative-Y axis, so indices 2,3,4 are negated.
        """
        signs = [1.0, 1.0, -1.0, -1.0, -1.0]
        return [a * s for a, s in zip(angles_rad, signs)]

    def compute(self, elapsed_sec: float) -> list[float]:
        # Advance virtual time proportionally to speed_scale
        dt = max(0.0, elapsed_sec - self._last_elapsed)
        self._last_elapsed = elapsed_sec
        self._virtual_time += dt * self._speed_scale

        left_deg, right_deg = self._generator.generate(self._virtual_time)
        # Convert degrees to radians and apply aoba axis sign convention
        left_rad = self._apply_axis_sign([np.radians(a) for a in left_deg])
        right_rad = self._apply_axis_sign([np.radians(a) for a in right_deg])
        # Neck (rev31) held at 0
        return left_rad + right_rad + [0.0]

    def reset(self) -> None:
        self._virtual_time = 0.0
        self._last_elapsed = 0.0
