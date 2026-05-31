"""Velocity-tracking primitives used by the x500_arm2x benchmark."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Sequence

import numpy as np

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class VelocityTrackingProfileConfig:
    segment_duration_s: float = 3.0
    rest_duration_s: float = 2.0
    cycles: int = 1
    forward_speed_mps: float = 0.4
    lateral_speed_mps: float = 0.3
    vertical_speed_mps: float = 0.2
    yaw_rate_radps: float = 0.3
    include_yaw_rate: bool = True


@dataclass(frozen=True)
class VelocityTrackingCommand:
    active: bool = False
    segment_name: str = "complete"
    segment_index: int = 0
    cycle_index: int = 0
    segment_elapsed_s: float = 0.0
    velocity_h: Vector3 = (0.0, 0.0, 0.0)
    yaw_rate: float | None = None


@dataclass(frozen=True)
class _ProfileSegment:
    name: str
    duration_s: float
    velocity_h: Vector3
    yaw_rate: float | None


@dataclass(frozen=True)
class VelocityTrackingSummary:
    sample_count: int = 0
    mean_forward_error_mps: float = 0.0
    rms_forward_error_mps: float = 0.0
    max_abs_forward_error_mps: float = 0.0
    mean_left_error_mps: float = 0.0
    rms_left_error_mps: float = 0.0
    max_abs_left_error_mps: float = 0.0
    mean_up_error_mps: float = 0.0
    rms_up_error_mps: float = 0.0
    max_abs_up_error_mps: float = 0.0
    mean_speed_error_norm_mps: float = 0.0
    rms_speed_error_norm_mps: float = 0.0
    max_speed_error_norm_mps: float = 0.0
    mean_lateral_velocity_bias_mps: float = 0.0
    rms_lateral_velocity_bias_mps: float = 0.0
    max_abs_lateral_velocity_bias_mps: float = 0.0
    mean_yaw_rate_error_radps: float = 0.0
    rms_yaw_rate_error_radps: float = 0.0
    max_abs_yaw_rate_error_radps: float = 0.0
    forward_segment_lateral_sample_count: int = 0
    mean_forward_segment_actual_left_mps: float = 0.0
    rms_forward_segment_actual_left_mps: float = 0.0
    max_abs_forward_segment_actual_left_mps: float = 0.0

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)


def _as_vec3(value: Sequence[float]) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,):
        raise ValueError("expected exactly three values")
    if not np.all(np.isfinite(array)):
        raise ValueError("vector values must be finite")
    return array


def heading_frame_velocity_to_world_enu(heading_w: float, velocity_h: Sequence[float]) -> Vector3:
    """Rotate forward-left-up heading-frame velocity into ENU world coordinates."""

    velocity = _as_vec3(velocity_h)
    c = math.cos(float(heading_w))
    s = math.sin(float(heading_w))
    world = np.array(
        [
            c * velocity[0] - s * velocity[1],
            s * velocity[0] + c * velocity[1],
            velocity[2],
        ],
        dtype=float,
    )
    return (float(world[0]), float(world[1]), float(world[2]))


def world_enu_velocity_to_heading_frame(heading_w: float, velocity_w: Sequence[float]) -> Vector3:
    """Rotate ENU world velocity into the vehicle heading frame."""

    velocity = _as_vec3(velocity_w)
    c = math.cos(float(heading_w))
    s = math.sin(float(heading_w))
    heading = np.array(
        [
            c * velocity[0] + s * velocity[1],
            -s * velocity[0] + c * velocity[1],
            velocity[2],
        ],
        dtype=float,
    )
    return (float(heading[0]), float(heading[1]), float(heading[2]))


def velocity_enu_to_ned(velocity_enu: Sequence[float]) -> Vector3:
    """Convert ENU velocity into PX4's NED frame."""

    velocity = _as_vec3(velocity_enu)
    return (float(velocity[1]), float(velocity[0]), float(-velocity[2]))


def yaw_rate_enu_to_ned(yaw_rate_radps: float) -> float:
    return -float(yaw_rate_radps)


class VelocityTrackingProfile:
    """ACEPliot-compatible heading-frame velocity tracking profile."""

    def __init__(self, config: VelocityTrackingProfileConfig) -> None:
        self._config = config
        segment_duration_s = max(0.01, float(config.segment_duration_s))
        rest_duration_s = max(0.0, float(config.rest_duration_s))
        self._segments: list[_ProfileSegment] = []

        def add_rest(name: str) -> None:
            if rest_duration_s > 0.0:
                self._segments.append(_ProfileSegment(name, rest_duration_s, (0.0, 0.0, 0.0), None))

        def add_velocity(name: str, velocity_h: Vector3) -> None:
            self._segments.append(_ProfileSegment(name, segment_duration_s, velocity_h, None))

        add_rest("rest_initial")
        add_velocity("forward", (float(config.forward_speed_mps), 0.0, 0.0))
        add_rest("rest_after_forward")
        add_velocity("backward", (-float(config.forward_speed_mps), 0.0, 0.0))
        add_rest("rest_after_backward")
        add_velocity("left", (0.0, float(config.lateral_speed_mps), 0.0))
        add_rest("rest_after_left")
        add_velocity("right", (0.0, -float(config.lateral_speed_mps), 0.0))
        add_rest("rest_after_right")
        add_velocity("up", (0.0, 0.0, float(config.vertical_speed_mps)))
        add_rest("rest_after_up")
        add_velocity("down", (0.0, 0.0, -float(config.vertical_speed_mps)))

        if config.include_yaw_rate:
            add_rest("rest_after_down")
            self._segments.append(
                _ProfileSegment("yaw_positive", segment_duration_s, (0.0, 0.0, 0.0), float(config.yaw_rate_radps))
            )
            add_rest("rest_after_yaw_positive")
            self._segments.append(
                _ProfileSegment("yaw_negative", segment_duration_s, (0.0, 0.0, 0.0), -float(config.yaw_rate_radps))
            )
        add_rest("rest_final")

        self._cycle_duration_s = sum(segment.duration_s for segment in self._segments)

    @property
    def duration_s(self) -> float:
        if self._config.cycles <= 0:
            return math.inf
        return self._cycle_duration_s * float(self._config.cycles)

    def sample(self, elapsed_s: float) -> VelocityTrackingCommand:
        if not self._segments or self._cycle_duration_s <= 0.0 or elapsed_s < 0.0:
            return VelocityTrackingCommand()

        if self._config.cycles > 0:
            total_duration_s = self.duration_s
            if elapsed_s >= total_duration_s:
                return VelocityTrackingCommand()
            cycle_index = int(elapsed_s / self._cycle_duration_s)
            cycle_elapsed_s = elapsed_s - float(cycle_index) * self._cycle_duration_s
        else:
            cycle_elapsed_s = math.fmod(elapsed_s, self._cycle_duration_s)
            cycle_index = int(elapsed_s / self._cycle_duration_s)

        segment_start_s = 0.0
        for segment_index, segment in enumerate(self._segments):
            segment_end_s = segment_start_s + segment.duration_s
            if cycle_elapsed_s < segment_end_s or segment_index == len(self._segments) - 1:
                return VelocityTrackingCommand(
                    active=True,
                    segment_name=segment.name,
                    segment_index=segment_index,
                    cycle_index=cycle_index,
                    segment_elapsed_s=cycle_elapsed_s - segment_start_s,
                    velocity_h=segment.velocity_h,
                    yaw_rate=segment.yaw_rate,
                )
            segment_start_s = segment_end_s

        return VelocityTrackingCommand()


class VelocityTrackingMetrics:
    """Accumulate heading-frame velocity and yaw-rate tracking metrics."""

    def __init__(self) -> None:
        self._sample_count = 0
        self._forward_errors: list[float] = []
        self._left_errors: list[float] = []
        self._up_errors: list[float] = []
        self._speed_error_norms: list[float] = []
        self._lateral_biases: list[float] = []
        self._yaw_rate_errors: list[float] = []
        self._forward_segment_actual_left: list[float] = []

    def record(
        self,
        *,
        heading_w: float,
        actual_velocity_world_enu: Sequence[float],
        actual_yaw_rate_flu_radps: float,
        command: VelocityTrackingCommand,
    ) -> None:
        if not command.active:
            return

        actual_h = np.asarray(world_enu_velocity_to_heading_frame(heading_w, actual_velocity_world_enu), dtype=float)
        desired_h = np.asarray(command.velocity_h, dtype=float)
        velocity_error_h = desired_h - actual_h
        speed_error_norm = float(np.linalg.norm(velocity_error_h))
        lateral_bias = float(actual_h[1] - desired_h[1])
        desired_yaw_rate = 0.0 if command.yaw_rate is None else float(command.yaw_rate)
        yaw_rate_error = desired_yaw_rate - float(actual_yaw_rate_flu_radps)

        self._sample_count += 1
        self._forward_errors.append(float(velocity_error_h[0]))
        self._left_errors.append(float(velocity_error_h[1]))
        self._up_errors.append(float(velocity_error_h[2]))
        self._speed_error_norms.append(speed_error_norm)
        self._lateral_biases.append(lateral_bias)
        if command.yaw_rate is not None:
            self._yaw_rate_errors.append(yaw_rate_error)
        if command.segment_name in {"forward", "backward"}:
            self._forward_segment_actual_left.append(float(actual_h[1]))

    @staticmethod
    def _mean(values: Sequence[float]) -> float:
        return float(sum(values) / len(values)) if values else 0.0

    @staticmethod
    def _rms(values: Sequence[float]) -> float:
        return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0

    @staticmethod
    def _max_abs(values: Sequence[float]) -> float:
        return max((abs(value) for value in values), default=0.0)

    @staticmethod
    def _max(values: Sequence[float]) -> float:
        return max(values, default=0.0)

    def summary(self) -> VelocityTrackingSummary:
        return VelocityTrackingSummary(
            sample_count=self._sample_count,
            mean_forward_error_mps=self._mean(self._forward_errors),
            rms_forward_error_mps=self._rms(self._forward_errors),
            max_abs_forward_error_mps=self._max_abs(self._forward_errors),
            mean_left_error_mps=self._mean(self._left_errors),
            rms_left_error_mps=self._rms(self._left_errors),
            max_abs_left_error_mps=self._max_abs(self._left_errors),
            mean_up_error_mps=self._mean(self._up_errors),
            rms_up_error_mps=self._rms(self._up_errors),
            max_abs_up_error_mps=self._max_abs(self._up_errors),
            mean_speed_error_norm_mps=self._mean(self._speed_error_norms),
            rms_speed_error_norm_mps=self._rms(self._speed_error_norms),
            max_speed_error_norm_mps=self._max(self._speed_error_norms),
            mean_lateral_velocity_bias_mps=self._mean(self._lateral_biases),
            rms_lateral_velocity_bias_mps=self._rms(self._lateral_biases),
            max_abs_lateral_velocity_bias_mps=self._max_abs(self._lateral_biases),
            mean_yaw_rate_error_radps=self._mean(self._yaw_rate_errors),
            rms_yaw_rate_error_radps=self._rms(self._yaw_rate_errors),
            max_abs_yaw_rate_error_radps=self._max_abs(self._yaw_rate_errors),
            forward_segment_lateral_sample_count=len(self._forward_segment_actual_left),
            mean_forward_segment_actual_left_mps=self._mean(self._forward_segment_actual_left),
            rms_forward_segment_actual_left_mps=self._rms(self._forward_segment_actual_left),
            max_abs_forward_segment_actual_left_mps=self._max_abs(self._forward_segment_actual_left),
        )
