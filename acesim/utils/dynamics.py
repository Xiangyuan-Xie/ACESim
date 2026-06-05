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


@dataclass(frozen=True)
class RotorFlowParams:
    """Lightweight rotor-flow correction parameters."""

    enabled: bool
    advance_c_lambda: float
    advance_c_mu: float
    advance_scale_min: float
    advance_scale_max: float
    ground_effect_enabled: bool
    ground_effect_max_scale: float
    ground_effect_height_rotor_diameters: float
    ground_effect_normal_min_dot: float

    @classmethod
    def from_config(cls, config: dict[str, object] | None) -> "RotorFlowParams":
        if not config:
            return cls(
                enabled=False,
                advance_c_lambda=0.0,
                advance_c_mu=0.0,
                advance_scale_min=1.0,
                advance_scale_max=1.0,
                ground_effect_enabled=False,
                ground_effect_max_scale=1.0,
                ground_effect_height_rotor_diameters=0.0,
                ground_effect_normal_min_dot=0.5,
            )

        def parse_float(key: str, default: float) -> float:
            value = config.get(key, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be a finite number")
            value_float = float(value)
            if not np.isfinite(value_float):
                raise ValueError(f"{key} must be a finite number")
            return value_float

        advance_scale_min = parse_float("advance_scale_min", 0.85)
        advance_scale_max = parse_float("advance_scale_max", 1.10)
        if advance_scale_min < 0.0:
            raise ValueError("advance_scale_min must be >= 0.0")
        if advance_scale_max < advance_scale_min:
            raise ValueError("advance_scale_max must be >= advance_scale_min")
        ground_effect_max_scale = parse_float("ground_effect_max_scale", 1.25)
        if ground_effect_max_scale < 1.0:
            raise ValueError("ground_effect_max_scale must be >= 1.0")
        ground_effect_height_rotor_diameters = parse_float("ground_effect_height_rotor_diameters", 1.0)
        if ground_effect_height_rotor_diameters < 0.0:
            raise ValueError("ground_effect_height_rotor_diameters must be >= 0.0")
        ground_effect_normal_min_dot = parse_float("ground_effect_normal_min_dot", 0.5)
        if ground_effect_normal_min_dot < -1.0 or ground_effect_normal_min_dot > 1.0:
            raise ValueError("ground_effect_normal_min_dot must be in [-1.0, 1.0]")
        return cls(
            enabled=bool(config.get("enabled", False)),
            advance_c_lambda=parse_float("advance_c_lambda", 0.0),
            advance_c_mu=parse_float("advance_c_mu", 0.0),
            advance_scale_min=advance_scale_min,
            advance_scale_max=advance_scale_max,
            ground_effect_enabled=bool(config.get("ground_effect_enabled", True)),
            ground_effect_max_scale=ground_effect_max_scale,
            ground_effect_height_rotor_diameters=ground_effect_height_rotor_diameters,
            ground_effect_normal_min_dot=ground_effect_normal_min_dot,
        )


@dataclass(frozen=True)
class DownwashParams:
    """Configurable rotor downwash force-field parameters."""

    enabled: bool
    exclude_body_patterns: tuple[str, ...]
    drag_coefficient: float
    area_scale: float
    wake_speed_scale: float
    wake_spread_angle_rad: float
    axial_decay_m: float

    @classmethod
    def from_config(cls, config: dict[str, object] | None) -> "DownwashParams":
        if not config:
            return cls(
                enabled=False,
                exclude_body_patterns=(),
                drag_coefficient=0.0,
                area_scale=1.0,
                wake_speed_scale=1.0,
                wake_spread_angle_rad=0.0,
                axial_decay_m=0.0,
            )
        patterns = config.get("exclude_body_patterns", [])
        exclude_body_patterns: tuple[str, ...]
        if isinstance(patterns, str):
            exclude_body_patterns = (patterns,)
        elif isinstance(patterns, (list, tuple)):
            exclude_body_patterns = tuple(str(pattern) for pattern in patterns)
        else:
            raise ValueError("exclude_body_patterns must be a string or a list of body name patterns")

        def parse_float(key: str, default: float) -> float:
            value = config.get(key, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be a finite number")
            value_float = float(value)
            if not np.isfinite(value_float):
                raise ValueError(f"{key} must be a finite number")
            return value_float

        drag_coefficient = parse_float("drag_coefficient", 1.1)
        if drag_coefficient < 0.0:
            raise ValueError("drag_coefficient must be >= 0.0")
        area_scale = parse_float("area_scale", 1.0)
        if area_scale < 0.0:
            raise ValueError("area_scale must be >= 0.0")
        wake_speed_scale = parse_float("wake_speed_scale", 1.0)
        if wake_speed_scale < 0.0:
            raise ValueError("wake_speed_scale must be >= 0.0")
        wake_spread_angle_rad = parse_float("wake_spread_angle_rad", 0.20)
        if wake_spread_angle_rad < 0.0:
            raise ValueError("wake_spread_angle_rad must be >= 0.0")
        axial_decay_m = parse_float("axial_decay_m", 0.45)
        if axial_decay_m <= 0.0:
            raise ValueError("axial_decay_m must be > 0.0")
        return cls(
            enabled=bool(config.get("enabled", False)),
            exclude_body_patterns=exclude_body_patterns,
            drag_coefficient=drag_coefficient,
            area_scale=area_scale,
            wake_speed_scale=wake_speed_scale,
            wake_spread_angle_rad=wake_spread_angle_rad,
            axial_decay_m=axial_decay_m,
        )


@dataclass(frozen=True)
class RotorInertialTorqueParams:
    """Optional analytical rotor acceleration and gyroscopic torque terms."""

    enabled: bool
    inertia_kg_m2: float
    apply_acceleration_torque: bool
    apply_gyro_torque: bool
    randomize_enabled: bool
    enabled_probability: float

    @classmethod
    def from_config(cls, config: dict[str, object] | None) -> "RotorInertialTorqueParams":
        if not config:
            return cls(
                enabled=False,
                inertia_kg_m2=0.0,
                apply_acceleration_torque=True,
                apply_gyro_torque=True,
                randomize_enabled=False,
                enabled_probability=1.0,
            )

        def parse_float(key: str, default: float) -> float:
            value = config.get(key, default)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be a finite number")
            value_float = float(value)
            if not np.isfinite(value_float):
                raise ValueError(f"{key} must be a finite number")
            return value_float

        inertia_kg_m2 = parse_float("inertia_kg_m2", 0.0)
        if inertia_kg_m2 < 0.0:
            raise ValueError("inertia_kg_m2 must be >= 0.0")
        enabled_probability = parse_float("enabled_probability", 1.0)
        if enabled_probability < 0.0 or enabled_probability > 1.0:
            raise ValueError("enabled_probability must be in [0.0, 1.0]")
        return cls(
            enabled=bool(config.get("enabled", False)),
            inertia_kg_m2=inertia_kg_m2,
            apply_acceleration_torque=bool(config.get("apply_acceleration_torque", True)),
            apply_gyro_torque=bool(config.get("apply_gyro_torque", True)),
            randomize_enabled=bool(config.get("randomize_enabled", False)),
            enabled_probability=enabled_probability,
        )


@dataclass(frozen=True)
class AeroSurfaceSamples:
    """Body-local surface samples used for distributed aerodynamic wrenches."""

    points_b: np.ndarray
    normals_b: np.ndarray
    areas: np.ndarray


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
