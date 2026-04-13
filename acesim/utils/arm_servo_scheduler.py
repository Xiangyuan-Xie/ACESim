"""Arm control and state scheduler for the manipulator stack used by ``MCArmEnv``.

The scheduler is intentionally narrow in scope: one periodic loop samples arm
control targets and another periodic loop publishes the current arm state.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

from acesim.utils.arm_state_publisher import ArmStatePublisher
from acesim.utils.simulation_clock import SimulationClock


@dataclass(frozen=True)
class ArmControlSample:
    """One arm control target produced by the robot controller."""

    joint_positions: Sequence[float]


@dataclass(frozen=True)
class ArmStateSample:
    """One arm state snapshot ready for publication."""

    positions: Sequence[float]
    velocities: Sequence[float]
    efforts: Sequence[float]


class ArmServoScheduler:
    """Schedule arm control and arm state publication from simulation time."""

    def __init__(
        self,
        clock: SimulationClock,
        publisher: ArmStatePublisher,
        control_rate_hz: float,
        state_publish_rate_hz: float,
        read_control_target: Callable[[], ArmControlSample | None],
        apply_control: Callable[[ArmControlSample], None],
        read_state: Callable[[], ArmStateSample],
        publish_control: Callable[[int, ArmControlSample], None] | None = None,
    ) -> None:
        if control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if state_publish_rate_hz <= 0.0:
            raise ValueError("state_publish_rate_hz must be positive")

        self._clock: SimulationClock = clock
        self._publisher: ArmStatePublisher = publisher
        self._read_control_target: Callable[[], ArmControlSample | None] = read_control_target
        self._apply_control: Callable[[ArmControlSample], None] = apply_control
        self._publish_control: Callable[[int, ArmControlSample], None] | None = publish_control
        self._read_state: Callable[[], ArmStateSample] = read_state
        self._control_period_s: float = 1.0 / float(control_rate_hz)
        self._state_publish_period_s: float = 1.0 / float(state_publish_rate_hz)
        self._last_update_time_us: int = 0
        self._control_elapsed_s: float = 0.0
        self._state_publish_elapsed_s: float = 0.0

        self.reset()

    def reset(self) -> None:
        """Reset both periodic timers to the current simulation timestamp."""

        self._last_update_time_us = self._clock.current_time_us
        self._control_elapsed_s = 0.0
        self._state_publish_elapsed_s = 0.0

    def update(self) -> None:
        """Run any arm control or state publication work due this tick."""

        current_time_us = self._clock.current_time_us
        dt_s = max(0.0, (current_time_us - self._last_update_time_us) * 1e-6)
        self._last_update_time_us = current_time_us

        self._control_elapsed_s += dt_s
        control_due = False
        while self._control_elapsed_s + 1e-12 >= self._control_period_s:
            self._control_elapsed_s -= self._control_period_s
            control_due = True
        if control_due:
            control_sample = self._read_control_target()
            if control_sample is not None:
                self._apply_control(control_sample)
                if self._publish_control is not None:
                    self._publish_control(current_time_us, control_sample)

        self._state_publish_elapsed_s += dt_s
        state_due = False
        while self._state_publish_elapsed_s + 1e-12 >= self._state_publish_period_s:
            self._state_publish_elapsed_s -= self._state_publish_period_s
            state_due = True
        if state_due:
            state_sample = self._read_state()
            self._publisher.publish(
                current_time_us,
                state_sample.positions,
                state_sample.velocities,
                state_sample.efforts,
            )
