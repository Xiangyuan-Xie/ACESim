"""Simulation clock with optional ZeroMQ publication.

The clock stores simulation time in integer microseconds. When ZeroMQ is
enabled it also publishes each update as one little-endian unsigned 64-bit
integer payload. The class does not schedule anything by itself; callers drive
the clock explicitly from their simulation loop.
"""

from __future__ import annotations

import struct

import zmq


class SimulationClock:
    """Maintain simulation time and optionally publish it over ZeroMQ."""

    _PAYLOAD_STRUCT = struct.Struct("<Q")

    def __init__(
        self,
        start_time_us: int = 0,
        zmq_endpoint: str = "tcp://0.0.0.0:5600",
        enable_zmq: bool = True,
    ) -> None:
        if start_time_us < 0:
            raise ValueError("start_time_us must be non-negative")

        self._current_time_us: int = int(start_time_us)
        self._endpoint: str = zmq_endpoint
        self._socket: zmq.Socket | None = None

        if enable_zmq:
            context = zmq.Context.instance()
            socket = context.socket(zmq.PUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SNDHWM, 1)
            socket.setsockopt(zmq.CONFLATE, 1)
            socket.bind(self._endpoint)
            self._socket = socket

    @property
    def current_time_us(self) -> int:
        """Return the current simulation timestamp in microseconds."""

        return self._current_time_us

    def advance_us(self, delta_us: int) -> int:
        """Advance the clock by a non-negative microsecond delta."""

        if delta_us < 0:
            raise ValueError("delta_us must be non-negative")
        self._current_time_us += int(delta_us)
        self.publish()
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
        self.publish()

    def publish(self) -> None:
        """Publish the current time when a PUB socket is configured."""

        if self._socket is None:
            return
        payload = self._PAYLOAD_STRUCT.pack(self._current_time_us)
        self._socket.send(payload, flags=zmq.NOBLOCK)

    def close(self) -> None:
        """Close the PUB socket and stop future publication."""

        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
