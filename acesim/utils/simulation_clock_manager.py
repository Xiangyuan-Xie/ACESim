from __future__ import annotations

"""Shared simulation clock manager with ZeroMQ publication."""

import struct

import zmq


class SimulationClockManager:
    """Manage simulation time and publish it over ZeroMQ when enabled."""

    def __init__(
        self,
        start_time_us: int = 0,
        zmq_endpoint: str = "tcp://0.0.0.0:5600",
        enable_zmq: bool = True,
    ) -> None:
        self._current_time_us = max(0, int(start_time_us))
        self._enabled = False
        self._endpoint = zmq_endpoint
        self._socket = None

        if not enable_zmq:
            return

        try:
            context = zmq.Context.instance()
            socket = context.socket(zmq.PUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SNDHWM, 1)
            try:
                socket.setsockopt(zmq.CONFLATE, 1)
            except (AttributeError, zmq.ZMQError):
                pass
            socket.bind(self._endpoint)
            self._socket = socket
            self._enabled = True
        except zmq.ZMQError as exc:
            print(f"[ACESim] ZMQ simulation clock manager disabled: {exc}")
            self._socket = None

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def current_time_us(self) -> int:
        """Return the current simulation timestamp in microseconds."""

        return self._current_time_us

    def advance_us(self, delta_us: int) -> int:
        """Advance the clock by a microsecond delta and publish the new time."""

        self._current_time_us = max(0, self._current_time_us + int(delta_us))
        self.publish()
        return self._current_time_us

    def advance_seconds(self, dt_s: float) -> int:
        """Advance the clock by seconds and publish the new time."""

        return self.advance_us(int(float(dt_s) * 1e6))

    def reset(self, time_us: int = 0) -> None:
        """Reset the clock to an absolute timestamp and publish it."""

        self._current_time_us = max(0, int(time_us))
        self.publish()

    def publish(self) -> None:
        """Publish the current time without blocking the simulation loop."""

        if not self._enabled or self._socket is None:
            return

        payload = struct.pack("<Q", self._current_time_us)
        try:
            self._socket.send(payload, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except zmq.ZMQError as exc:
            print(f"[ACESim] ZMQ simulation clock manager publish failed: {exc}")

    def close(self) -> None:
        """Close the PUB socket and disable future publication attempts."""

        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        self._enabled = False
