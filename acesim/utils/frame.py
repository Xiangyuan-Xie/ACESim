from __future__ import annotations

"""Coordinate-frame helpers shared by the PX4 HIL transport and sensor scheduler.

The simulator backends expose their world-frame quantities in NWU
(north-west-up). Body-frame IMU and magnetometer values are produced in FLU
(forward-left-up). PX4 expects body-frame sensor values in FRD
(forward-right-down) and GPS/world-frame velocities in NED
(north-east-down).
"""

import numpy as np


def body_flu_to_frd(vec_flu: np.ndarray) -> np.ndarray:
    """Convert a body-frame vector from FLU to FRD."""

    vec_flu = np.asarray(vec_flu, dtype=float)
    return np.array([vec_flu[0], -vec_flu[1], -vec_flu[2]], dtype=float)


def world_nwu_to_ned(vec_world_nwu: np.ndarray) -> np.ndarray:
    """Convert a world-frame vector from simulator NWU to PX4 NED."""

    vec_world_nwu = np.asarray(vec_world_nwu, dtype=float)
    return np.array([vec_world_nwu[0], -vec_world_nwu[1], -vec_world_nwu[2]], dtype=float)
