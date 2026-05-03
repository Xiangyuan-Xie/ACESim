"""Simulation clock for backend-driven simulated time.

The clock stores simulation time in integer microseconds. It does not publish
transport messages by itself; callers own any external clock stream.
"""

from __future__ import annotations


class SimulationClock:
    """Maintain simulation time in integer microseconds."""

    def __init__(
        self,
        start_time_us: int = 0,
    ) -> None:
        if start_time_us < 0:
            raise ValueError("start_time_us must be non-negative")

        self._current_time_us: int = int(start_time_us)

    @property
    def current_time_us(self) -> int:
        """Return the current simulation timestamp in microseconds."""

        return self._current_time_us

    def advance_us(self, delta_us: int) -> int:
        """Advance the clock by a non-negative microsecond delta."""

        if delta_us < 0:
            raise ValueError("delta_us must be non-negative")
        self._current_time_us += int(delta_us)
        return self._current_time_us

    def advance_seconds(self, dt_s: float) -> int:
        """Advance the clock by a non-negative duration in seconds."""

        if dt_s < 0.0:
            raise ValueError("dt_s must be non-negative")
        return self.advance_us(int(float(dt_s) * 1e6))

    def reset(self, time_us: int = 0) -> None:
        """Reset the clock to a non-negative absolute timestamp."""

        if time_us < 0:
            raise ValueError("time_us must be non-negative")
        self._current_time_us = int(time_us)

    def close(self) -> None:
        """Keep the old lifecycle hook for environments that close owned resources."""
