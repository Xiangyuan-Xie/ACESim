"""Small dynamics helpers shared by MuJoCo vehicle backends.

These helpers stay free of MuJoCo model/data types so the same update rules can
be reused across multiple environments without pulling backend-specific state
into the utility layer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeVar

import numpy as np

ArrayLikeFloat = TypeVar("ArrayLikeFloat", float, np.ndarray)


@dataclass(frozen=True)
class LumpedDragParams:
    """Mass-normalized diagonal linear body drag parameters."""

    enabled: bool
    d: np.ndarray

    @classmethod
    def from_config(cls, config: dict[str, object] | None) -> "LumpedDragParams":
        if not config:
            return cls(enabled=False, d=np.zeros(3, dtype=float))
        units = str(config.get("units", "mass_normalized"))
        if units != "mass_normalized":
            raise ValueError(f"Unsupported lumped_drag units '{units}'. Expected 'mass_normalized'.")
        d = np.asarray(config.get("D", [0.0, 0.0, 0.0]), dtype=float)
        if d.shape != (3,):
            raise ValueError("lumped_drag D must contain exactly three diagonal coefficients")
        return cls(enabled=bool(config.get("enabled", False)), d=d)


def first_order_response_step(
    current: ArrayLikeFloat,
    target: ArrayLikeFloat,
    dt_s: float,
    time_constant_up: float,
    time_constant_down: float,
) -> ArrayLikeFloat:
    """Advance a first-order response using separate rise and fall time constants.

    Args:
        current: Current state value. May be a scalar or an ``ndarray``.
        target: Target state value with the same broadcastable shape as
            ``current``.
        dt_s: Simulation step in seconds.
        time_constant_up: Time constant used when ``target > current``.
        time_constant_down: Time constant used when ``target <= current``.

    Returns:
        Updated state with the same scalar/array shape convention as
        ``current``. Zero or negative time constants snap directly to target,
        matching the "no lag" intent of the callers.
    """

    current_arr = np.asarray(current, dtype=float)
    target_arr = np.asarray(target, dtype=float)
    delta = target_arr - current_arr
    time_constants = np.where(delta > 0.0, time_constant_up, time_constant_down).astype(float, copy=False)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        blend = np.where(time_constants > 0.0, 1.0 - np.exp(-dt_s / time_constants), 1.0)
    updated = current_arr + delta * blend
    if np.isscalar(current) and np.isscalar(target):
        return float(updated)
    return updated


def idle_visual_speed_target(
    physical_speed: float,
    actuator_output: float,
    armed: bool,
    idle_speed: float,
    low_speed_blend_end: float,
) -> float:
    """Return the target visual rotor speed while preserving idle-spin semantics.

    The physics state may legitimately sit near zero while the vehicle is armed
    and visually expected to show a spinning propeller. Callers provide the
    already-normalized physical speed and actuator magnitude that make sense for
    their vehicle model.
    """

    physical_speed = float(physical_speed)
    actuator_output = float(actuator_output)
    if not armed:
        return physical_speed
    if actuator_output <= 0.0:
        return max(physical_speed, float(idle_speed))
    if low_speed_blend_end <= 0.0:
        return physical_speed
    blend_weight = float(np.clip(1.0 - physical_speed / low_speed_blend_end, 0.0, 1.0))
    low_speed_target = blend_weight * idle_speed + (1.0 - blend_weight) * physical_speed
    return max(physical_speed, float(low_speed_target))
