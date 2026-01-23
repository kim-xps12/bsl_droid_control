"""
Inverse and Forward Kinematics for Digitigrade (Bird-Leg) Biped Robot.

This module provides kinematics calculations for a biped robot with
reverse-joint (digitigrade) leg structure.

Leg structure (per leg, 5 DOF):
- j11/j21: Hip Yaw
- j12/j22: Hip Roll
- j13/j23: Hip Pitch
- j14/j24: Knee Pitch
- j15/j25: Ankle Pitch

Coordinate system:
- X-axis: Forward/backward (positive = forward)
- Y-axis: Left/right (positive = right, negative = left)
- Z-axis: Up/down (positive = up)
- Origin: Ground center (0, 0, 0)
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class LinkLengths:
    """Robot link length parameters."""

    hip_width: float = 0.16   # Distance between left/right hip joints (Y direction) [m]
    hip_yaw: float = 0.03     # Yaw axis link length (hip yaw -> hip roll) [m]
    hip_roll: float = 0.02    # Roll axis link length (hip roll -> hip pitch) [m]
    thigh: float = 0.18       # Thigh length (hip pitch -> knee pitch) [m]
    shank: float = 0.20       # Shank length (knee pitch -> ankle pitch) [m]
    foot: float = 0.10        # Foot length (ankle pitch -> foot tip) [m]


class BipedKinematics:
    """
    Kinematics calculator for digitigrade biped robot.

    Provides inverse kinematics (target position -> joint angles) and
    forward kinematics (joint angles -> joint positions) for the robot legs.
    """

    def __init__(self, link_lengths: LinkLengths = None, leg_extension_ratio: float = 0.90):
        """
        Initialize kinematics calculator.

        Args:
            link_lengths: Robot link length parameters. If None, uses defaults.
            leg_extension_ratio: Ratio of leg extension for standing pose (0.0-1.0).
        """
        self.link_lengths = link_lengths or LinkLengths()
        self.leg_extension_ratio = leg_extension_ratio
        self.body_height = self._calculate_body_height()

    def _calculate_body_height(self) -> float:
        """
        Calculate body height from leg structure.

        Sets the body height such that foot tips touch Z=0 (ground) in standing pose.

        Returns:
            Body height in meters.
        """
        L = self.link_lengths

        # Vertical component of thigh + shank (considering extension ratio)
        thigh_shank_vertical = (L.thigh + L.shank) * self.leg_extension_ratio

        # Calculate standing pose angles
        target_x = 0  # Directly below hip when standing
        target_z = -thigh_shank_vertical
        hip_pitch, knee_pitch = self.inverse_kinematics_2link(
            target_x, target_z, L.thigh, L.shank
        )

        # Ankle angle and foot Z component
        ankle_pitch = self.calculate_foot_angle(hip_pitch, knee_pitch)
        total_angle = np.radians(hip_pitch + knee_pitch + ankle_pitch)
        foot_z = L.foot * np.sin(total_angle)

        # Body height = roll link + thigh/shank vertical + foot Z component
        body_height = L.hip_roll + thigh_shank_vertical + foot_z

        return body_height

    def inverse_kinematics_2link(
        self,
        target_x: float,
        target_z: float,
        l1: float,
        l2: float
    ) -> Tuple[float, float]:
        """
        2-link inverse kinematics for digitigrade leg (X-Z plane).

        Initial pose (pitch angles = 0): leg extends straight down.
        - Thigh: (0, 0) -> (0, -l1)
        - Shank: (0, -l1) -> (0, -l1-l2)

        Digitigrade characteristics:
        - Knee protrudes forward (negative knee pitch angle)
        - Thigh tilts backward, shank tilts forward

        Args:
            target_x: Target ankle X coordinate relative to hip pitch axis [m]
            target_z: Target ankle Z coordinate relative to hip pitch axis [m]
            l1: Thigh length [m]
            l2: Shank length [m]

        Returns:
            (hip_pitch_angle, knee_pitch_angle) in degrees.
            - Hip pitch: positive = forward rotation
            - Knee pitch: negative = bend forward (digitigrade)
        """
        # Distance to target
        d = np.sqrt(target_x**2 + target_z**2)

        # Reachability check
        max_reach = l1 + l2 - 0.001
        min_reach = abs(l1 - l2) + 0.001

        if d > max_reach:
            scale = max_reach / d
            target_x *= scale
            target_z *= scale
            d = max_reach
        elif d < min_reach:
            d = min_reach

        # Knee angle using law of cosines
        cos_knee = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_knee = np.clip(cos_knee, -1.0, 1.0)

        # Knee inner angle (180 degrees = fully extended)
        knee_inner_angle = np.arccos(cos_knee)

        # Digitigrade: knee bends forward = negative pitch angle
        j14 = -(np.pi - knee_inner_angle)

        # Calculate hip pitch
        theta_target = np.arctan2(target_x, -target_z)
        theta_correction = np.arctan2(l2 * np.sin(-j14), l1 + l2 * np.cos(-j14))
        j13 = theta_target + theta_correction

        return np.degrees(j13), np.degrees(j14)

    def calculate_foot_angle(self, hip_pitch_deg: float, knee_pitch_deg: float) -> float:
        """
        Calculate ankle pitch angle to keep foot parallel to ground.

        For digitigrade legs, the foot extends backward (-X direction).

        URDF convention:
        - Ankle angle 0 deg: foot already extends backward (-X direction)
        - To keep foot horizontal, we only need to cancel the leg's pitch angle

        Args:
            hip_pitch_deg: Hip pitch angle in degrees
            knee_pitch_deg: Knee pitch angle in degrees

        Returns:
            Ankle pitch angle in degrees.
        """
        total_leg_angle = hip_pitch_deg + knee_pitch_deg
        ankle_pitch = -total_leg_angle  # Cancel leg pitch to keep foot horizontal
        return ankle_pitch

    def forward_kinematics_leg(
        self,
        angles: List[float],
        is_left: bool = True
    ) -> List[np.ndarray]:
        """
        Forward kinematics for visualization.

        Args:
            angles: [yaw, roll, hip_pitch, knee_pitch, ankle_pitch] in degrees
            is_left: True for left leg, False for right leg

        Returns:
            List of joint positions [[x, y, z], ...] for each joint.
        """
        yaw, roll, hip_pitch, knee_pitch, ankle_pitch = np.radians(angles)
        L = self.link_lengths
        side = 1 if is_left else -1

        positions = []

        # 0: Hip base (branch from body center)
        hip_base = np.array([0, side * L.hip_width / 2, self.body_height])
        positions.append(hip_base)

        # 1: After hip yaw
        yaw_offset = np.array([
            L.hip_yaw * np.sin(yaw),
            L.hip_yaw * np.cos(yaw) * side,
            0
        ])
        hip_yaw_end_pos = hip_base + yaw_offset
        positions.append(hip_yaw_end_pos)

        # 2: After hip roll
        roll_link_vec = np.array([0, 0, -L.hip_roll])
        hip_roll_end_pos = hip_yaw_end_pos + roll_link_vec
        positions.append(hip_roll_end_pos)

        # Roll rotation matrix (X-axis rotation)
        c_roll, s_roll = np.cos(roll), np.sin(roll)
        roll_matrix = np.array([
            [1, 0, 0],
            [0, c_roll, -s_roll],
            [0, s_roll, c_roll]
        ])

        # 3: Knee position (after hip pitch)
        thigh_vec = np.array([
            L.thigh * np.sin(hip_pitch),
            0,
            -L.thigh * np.cos(hip_pitch)
        ])
        thigh_vec = roll_matrix @ thigh_vec
        knee_pos = hip_roll_end_pos + thigh_vec
        positions.append(knee_pos)

        # 4: Ankle position (after knee pitch)
        total_pitch = hip_pitch + knee_pitch
        shank_vec = np.array([
            L.shank * np.sin(total_pitch),
            0,
            -L.shank * np.cos(total_pitch)
        ])
        shank_vec = roll_matrix @ shank_vec
        ankle_pos = knee_pos + shank_vec
        positions.append(ankle_pos)

        # 5: Foot tip position
        total_pitch_ankle = total_pitch + ankle_pitch
        foot_vec = np.array([
            -L.foot * np.cos(total_pitch_ankle),  # Backward (-X) component
            0,
            -L.foot * np.sin(total_pitch_ankle)   # Downward component
        ])
        foot_vec = roll_matrix @ foot_vec
        foot_pos = ankle_pos + foot_vec
        positions.append(foot_pos)

        return positions
