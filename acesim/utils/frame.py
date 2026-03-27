from __future__ import annotations

"""Coordinate-frame helpers shared by the PX4 HIL transport and sensor scheduler.

The simulator backends expose their world-frame quantities in NWU
(north-west-up). Body-frame IMU and magnetometer values are produced in FLU
(forward-left-up). PX4 expects body-frame sensor values in FRD
(forward-right-down) and GPS/world-frame velocities in NED
(north-east-down).
"""

import numpy as np
from scipy.spatial.transform import Rotation


def body_flu_to_frd(vec_flu: np.ndarray) -> np.ndarray:
    """Convert a body-frame vector from FLU to FRD."""

    vec_flu = np.asarray(vec_flu, dtype=float)
    return np.array([vec_flu[0], -vec_flu[1], -vec_flu[2]], dtype=float)


def world_nwu_to_ned(vec_world_nwu: np.ndarray) -> np.ndarray:
    """Convert a world-frame vector from simulator NWU to PX4 NED."""

    vec_world_nwu = np.asarray(vec_world_nwu, dtype=float)
    return np.array([vec_world_nwu[0], -vec_world_nwu[1], -vec_world_nwu[2]], dtype=float)


def rotation_world_nwu_body_flu_to_ned_frd(rotation_world_nwu_body_flu: Rotation) -> Rotation:
    """Convert a body-to-world rotation from NWU/FLU to NED/FRD."""

    transform = np.diag([1.0, -1.0, -1.0])
    matrix_nwu_flu = rotation_world_nwu_body_flu.as_matrix()
    matrix_ned_frd = transform @ matrix_nwu_flu @ transform
    return Rotation.from_matrix(matrix_ned_frd)


def quat_world_nwu_body_flu_to_ned_frd(quat_world_nwu_body_flu: np.ndarray) -> np.ndarray:
    """Convert a scalar-first body-to-world quaternion from NWU/FLU to NED/FRD."""

    quat = np.asarray(quat_world_nwu_body_flu, dtype=float).reshape(-1)
    if quat.size != 4:
        raise ValueError("quat_world_nwu_body_flu must contain four elements")
    rotation = Rotation.from_quat(quat, scalar_first=True)
    converted = rotation_world_nwu_body_flu_to_ned_frd(rotation)
    return converted.as_quat(scalar_first=True)
