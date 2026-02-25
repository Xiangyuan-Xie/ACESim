import math

import numpy as np


def calculate_slider_position(
    theta_rad: float, r: float = 0.02821, L: float = 0.0343, calibration_offset: float = 0.665
):
    """Compute gripper slider position from angle with calibration offset."""
    adjusted_theta = theta_rad + calibration_offset
    x = r * math.cos(adjusted_theta) + math.sqrt(L * L - (r * math.sin(adjusted_theta)) ** 2)
    return np.clip(x - 0.00778, 0, 0.04225)
