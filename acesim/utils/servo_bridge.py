"""Servo bridge that schedules arm control and state publication from simulation time."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

from acesim.utils.arm_state_manager import ArmStateManager
from acesim.utils.simulation_clock_manager import SimulationClockManager


@dataclass(frozen=True)
class ServoControlSample:
    """One scheduled servo control sample."""

    joint_positions: Sequence[float]


@dataclass(frozen=True)
class ServoStateSample:
    """One scheduled servo state sample."""

    positions: Sequence[float]
    velocities: Sequence[float]
    efforts: Sequence[float]


class ServoBridge:
    """Schedule servo control updates and state publication from simulation time."""

    def __init__(
        self,
        clock: SimulationClockManager,
        manager: ArmStateManager,
        control_rate_hz: float,
        state_publish_rate_hz: float,
        read_control_target: Callable[[], ServoControlSample | None],
        apply_control: Callable[[ServoControlSample], None],
        read_state: Callable[[], ServoStateSample],
    ) -> None:
        self._clock = clock
        self._manager = manager
        self._read_control_target = read_control_target
        self._apply_control = apply_control
        self._read_state = read_state
        self._control_period_s = 1.0 / max(float(control_rate_hz), 1e-9)
        self._state_publish_period_s = 1.0 / max(float(state_publish_rate_hz), 1e-9)
        self.reset()

    def reset(self) -> None:
        """Reset internal timers to the current simulation time."""

        self._last_update_time_us = self._clock.current_time_us
        self._control_elapsed_s = 0.0
        self._state_publish_elapsed_s = 0.0

    @staticmethod
    def _step_period_elapsed(elapsed_s: float, dt_s: float, period_s: float) -> tuple[bool, float]:
        """Advance one periodic timer and report whether it fired."""

        elapsed_s += dt_s
        triggered = False
        while elapsed_s + 1e-12 >= period_s:
            elapsed_s -= period_s
            triggered = True
        return triggered, elapsed_s

    def _consume_period(self, elapsed_attr: str, dt_s: float, period_s: float) -> bool:
        tick, elapsed_s = self._step_period_elapsed(getattr(self, elapsed_attr), dt_s, period_s)
        setattr(self, elapsed_attr, elapsed_s)
        return tick

    def update(self) -> None:
        """Advance periodic timers and run any due servo tasks."""

        current_time_us = self._clock.current_time_us
        dt_s = max(0.0, (current_time_us - self._last_update_time_us) * 1e-6)
        self._last_update_time_us = current_time_us

        if self._consume_period("_control_elapsed_s", dt_s, self._control_period_s):
            control_sample = self._read_control_target()
            if control_sample is not None:
                self._apply_control(control_sample)

        if self._consume_period("_state_publish_elapsed_s", dt_s, self._state_publish_period_s):
            state_sample = self._read_state()
            self._manager.publish(
                current_time_us,
                list(state_sample.positions),
                list(state_sample.velocities),
                list(state_sample.efforts),
            )
