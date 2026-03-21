"""
Joint limit constants and clamping for bsl_droid_simplified_v2.

Limits are derived from the URDF joint definitions.
"""

import logging
import math

logger = logging.getLogger(__name__)

# Joint limits per joint type (lower, upper) in radians.
# Both left and right legs share the same limits.
_JOINT_LIMITS: list[tuple[float, float]] = [
    (-30 * math.pi / 180, 30 * math.pi / 180),     # hip_yaw: ±30°
    (-25 * math.pi / 180, 25 * math.pi / 180),     # hip_roll: ±25°
    (-120 * math.pi / 180, 90 * math.pi / 180),    # hip_pitch: -120° to 90°
    (-150 * math.pi / 180, 0 * math.pi / 180),     # knee_pitch: -150° to 0°
    (-90 * math.pi / 180, 90 * math.pi / 180),     # ankle_pitch: ±90°
]

# Full 10-joint limit table (left 5 + right 5, same limits)
JOINT_LIMITS_10: list[tuple[float, float]] = _JOINT_LIMITS + _JOINT_LIMITS


def clamp_joint_angles(
    positions: list[float],
    log_warnings: bool = True,
) -> list[float]:
    """Clamp joint angles to URDF limits.

    Args:
        positions: 10 joint angles in radians (canonical order).
        log_warnings: If True, log a warning when clamping occurs.

    Returns:
        Clamped 10 joint angles in radians.
    """
    if len(positions) != len(JOINT_LIMITS_10):
        raise ValueError(
            f"positions must have {len(JOINT_LIMITS_10)} elements, got {len(positions)}"
        )

    clamped = []
    for i, (pos, (lo, hi)) in enumerate(zip(positions, JOINT_LIMITS_10)):
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
