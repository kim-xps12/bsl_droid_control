"""
Joint limit constants and clamping for aoba_description.

Limits are derived from the URDF joint definitions.
All joints in aoba are continuous type, so limits are set conservatively.
"""

import logging
import math

logger = logging.getLogger(__name__)

# Joint limits per joint type (lower, upper) in radians.
# Both left and right legs share the same limits.
_LEG_LIMITS: list[tuple[float, float]] = [
    (-30 * math.pi / 180, 30 * math.pi / 180),     # hip_yaw: ±30°
    (-25 * math.pi / 180, 25 * math.pi / 180),     # hip_roll: ±25°
    (-90 * math.pi / 180, 120 * math.pi / 180),    # hip_pitch: -90° to 120° (axis negated)
    (0 * math.pi / 180, 150 * math.pi / 180),      # knee_pitch: 0° to 150° (axis negated)
    (-90 * math.pi / 180, 90 * math.pi / 180),     # ankle_pitch: ±90°
]

_NECK_LIMITS: list[tuple[float, float]] = [
    (-90 * math.pi / 180, 90 * math.pi / 180),     # neck: ±90°
]

# Full 11-joint limit table (left 5 + right 5 + neck 1)
JOINT_LIMITS: list[tuple[float, float]] = _LEG_LIMITS + _LEG_LIMITS + _NECK_LIMITS


def clamp_joint_angles(
    positions: list[float],
    log_warnings: bool = True,
) -> list[float]:
    """Clamp joint angles to URDF limits.

    Args:
        positions: Joint angles in radians (canonical order).
        log_warnings: If True, log a warning when clamping occurs.

    Returns:
        Clamped joint angles in radians.
    """
    if len(positions) != len(JOINT_LIMITS):
        raise ValueError(
            f"positions must have {len(JOINT_LIMITS)} elements, got {len(positions)}"
        )

    clamped = []
    for i, (pos, (lo, hi)) in enumerate(zip(positions, JOINT_LIMITS)):
        if pos < lo:
            if log_warnings:
                logger.warning(
                    "Joint %d clamped: %.4f rad < lower limit %.4f rad",
                    i, pos, lo,
                )
            clamped.append(lo)
        elif pos > hi:
            if log_warnings:
                logger.warning(
                    "Joint %d clamped: %.4f rad > upper limit %.4f rad",
                    i, pos, hi,
                )
            clamped.append(hi)
        else:
            clamped.append(pos)
    return clamped
