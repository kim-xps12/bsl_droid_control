"""
Gait trajectory generation for biped walking.

This module provides trajectory generators for foot motion during walking.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class GaitParameters:
    """Walking gait parameters."""

    step_height: float = 0.04    # Foot lift height [m] (Z direction)
    step_length: float = 0.08    # Stride length [m] (X direction)
    step_frequency: float = 0.5  # Walking frequency [Hz]
    leg_extension_ratio: float = 0.90  # Leg extension ratio for standing pose


class CamberTrajectory:
    """
    Elliptical arc trajectory generator for foot motion (X-Z plane).

    Generates smooth walking trajectories with:
    - Stance phase (0.0-0.5): Linear ground contact, back to front
    - Swing phase (0.5-1.0): Semi-elliptical lift, front to back
    """

    def __init__(self, gait_params: GaitParameters = None):
        """
        Initialize trajectory generator.

        Args:
            gait_params: Gait parameters. If None, uses defaults.
        """
        self.params = gait_params or GaitParameters()

    def generate(self, phase: float) -> Tuple[float, float]:
        """
        Generate foot trajectory point for given phase.

        Args:
            phase: Walking cycle phase (0.0-1.0)
                - 0.0-0.5: Stance phase
                - 0.5-1.0: Swing phase

        Returns:
            (x, z) relative coordinates from hip reference.
        """
        half_step = self.params.step_length / 2

        if phase < 0.5:
            # Stance phase: linear ground contact (back to front)
            progress = phase / 0.5
            x = half_step * (2 * progress - 1)  # -half_step -> +half_step
            z = 0  # Ground contact
        else:
            # Swing phase: semi-elliptical trajectory (front to back)
            progress = (phase - 0.5) / 0.5
            theta = np.pi * progress  # 0 -> pi

            # Elliptical trajectory: x = a*cos(theta), z = b*sin(theta)
            x = half_step * np.cos(theta)  # +half_step -> -half_step
            z = self.params.step_height * np.sin(theta)  # 0 -> peak -> 0

        return x, z


class WalkingPatternGenerator:
    """
    Complete walking pattern generator for biped robot.

    Generates coordinated joint angles for both legs during walking.
    Left and right legs are phase-shifted by 0.5 (180 degrees).
    """

    def __init__(
        self,
        gait_params: GaitParameters = None,
        thigh_length: float = 0.18,
        shank_length: float = 0.20
    ):
        """
        Initialize walking pattern generator.

        Args:
            gait_params: Gait parameters.
            thigh_length: Thigh link length [m].
            shank_length: Shank link length [m].
        """
        self.params = gait_params or GaitParameters()
        self.trajectory = CamberTrajectory(self.params)
        self.thigh_length = thigh_length
        self.shank_length = shank_length

        # Import kinematics functions
        from .kinematics import BipedKinematics, LinkLengths
        link_lengths = LinkLengths(thigh=thigh_length, shank=shank_length)
        self.kinematics = BipedKinematics(
            link_lengths,
            self.params.leg_extension_ratio
        )

    def generate(self, t: float) -> Tuple[list, list]:
        """
        Generate walking pattern for given time.

        Args:
            t: Current time [seconds]

        Returns:
            (left_angles, right_angles) where each is:
            [yaw, roll, hip_pitch, knee_pitch, ankle_pitch] in degrees.
        """
        # Phase in walking cycle
        phase = (t * self.params.step_frequency) % 1.0

        # Phase-shifted for left/right legs
        left_phase = phase
        right_phase = (phase + 0.5) % 1.0

        l1 = self.thigh_length
        l2 = self.shank_length

        # Base ankle height (natural standing pose)
        base_leg_height = (l1 + l2) * self.params.leg_extension_ratio

        # Left leg
        left_angles = self._compute_leg_angles(left_phase, base_leg_height, l1, l2)

        # Right leg
        right_angles = self._compute_leg_angles(right_phase, base_leg_height, l1, l2)

        return left_angles, right_angles

    def _compute_leg_angles(
        self,
        phase: float,
        base_leg_height: float,
        l1: float,
        l2: float
    ) -> list:
        """
        Compute joint angles for single leg.

        Args:
            phase: Walking cycle phase for this leg.
            base_leg_height: Base leg vertical length.
            l1: Thigh length.
            l2: Shank length.

        Returns:
            [yaw, roll, hip_pitch, knee_pitch, ankle_pitch] in degrees.
        """
        # Get foot trajectory
        rel_x, rel_z = self.trajectory.generate(phase)

        # Target ankle position
        target_x = 0 + rel_x
        target_z = -base_leg_height + rel_z

        # Inverse kinematics
        hip_pitch, knee_pitch = self.kinematics.inverse_kinematics_2link(
            target_x, target_z, l1, l2
        )

        # Ankle angle for ground-parallel foot
        ankle_pitch = self.kinematics.calculate_foot_angle(hip_pitch, knee_pitch)

        return [
            0.0,           # Yaw
            0.0,           # Roll
            hip_pitch,     # Hip pitch
            knee_pitch,    # Knee pitch
            ankle_pitch    # Ankle pitch
        ]
