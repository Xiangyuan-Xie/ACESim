"""Small math helpers shared across simulation and robot-control code."""

import math
from typing import List, Sequence

import numpy as np
from scipy.spatial.transform import Rotation


def quat_mul(q1: Sequence[float], q2: Sequence[float]) -> List[float]:
    """Multiply two scalar-first quaternions and return a scalar-first result."""
    R1 = Rotation.from_quat(np.asarray(q1, dtype=np.float64).reshape(1, 4), scalar_first=True)
    R2 = Rotation.from_quat(np.asarray(q2, dtype=np.float64).reshape(1, 4), scalar_first=True)
    R = R1 * R2
    return list(R.as_quat(scalar_first=True)[0])


def quat_rotate(q: Sequence[float], v: Sequence[float]) -> List[float]:
    """Rotate a 3D vector by a scalar-first quaternion."""
    R = Rotation.from_quat(np.asarray(q, dtype=np.float64).reshape(1, 4), scalar_first=True)
    vec = np.asarray(v, dtype=np.float64).reshape(1, 3)
    res = R.apply(vec)[0]
    return [float(res[0]), float(res[1]), float(res[2])]


GRIPPER_JOINT5_CLOSED_RAD = -1.723


def calculate_slider_position(
    theta_rad: float, r: float = 0.02821, L: float = 0.0343, calibration_offset: float = 0.665
):
    """Compute the calibrated gripper slider position from the linkage angle."""
    adjusted_theta = theta_rad + calibration_offset
    x = r * math.cos(adjusted_theta) + math.sqrt(L * L - (r * math.sin(adjusted_theta)) ** 2)
    return np.clip(x - 0.00778, 0, 0.04225)


def calculate_coupled_gripper_positions(joint5_rad: float) -> tuple[float, float]:
    """Return MuJoCo left/right gripper slider qpos coupled to joint_5."""
    opening = 0.04225 - float(calculate_slider_position(abs(float(joint5_rad))))
    slider = -opening
    return slider, slider


def gripper_public_to_joint5(public_position: float) -> float:
    """Convert ACETele public gripper position to MuJoCo joint_5 radians."""
    open_fraction = float(np.clip(float(public_position), 0.0, 1.0))
    return GRIPPER_JOINT5_CLOSED_RAD * (1.0 - open_fraction)


def joint5_to_gripper_public(joint5_rad: float) -> float:
    """Convert MuJoCo joint_5 radians to ACETele public gripper position."""
    closed_fraction = float(np.clip(float(joint5_rad) / GRIPPER_JOINT5_CLOSED_RAD, 0.0, 1.0))
    return 1.0 - closed_fraction
