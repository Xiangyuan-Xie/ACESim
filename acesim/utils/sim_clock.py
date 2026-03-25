from __future__ import annotations

"""Shared simulation clock with optional ZeroMQ publication.

The clock is the single source of truth for simulated time used by both
backends and the PX4 bridge. When ZeroMQ is available it also publishes the
current timestamp for external consumers, but publication failures should never
block the simulation loop.
"""

import struct

try:
    import zmq

    _ZMQ_AVAILABLE = True
except ImportError:
    zmq = None
    _ZMQ_AVAILABLE = False


class SimulationClock:
    """Manage simulation time and optionally publish it over ZeroMQ."""

    def __init__(
        self,
        start_time_us: int = 0,
        zmq_endpoint: str = "tcp://0.0.0.0:5600",
        enable_zmq: bool = True,
    ) -> None:
        """Create a simulation clock and optionally bind a PUB socket."""

        self._current_time_us = max(0, int(start_time_us))
        self._enabled = False
        self._endpoint = zmq_endpoint
        self._socket = None

        if not enable_zmq:
            return

        if not _ZMQ_AVAILABLE:
            print("[ACESim] ZMQ clock publisher disabled: pyzmq not available.")
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
            print(f"[ACESim] ZMQ clock publisher disabled: {exc}")
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
            # Drop if receiver queue is full; keep sim loop real-time.
            pass
        except zmq.ZMQError as exc:
            print(f"[ACESim] ZMQ clock publish failed: {exc}")

    def close(self) -> None:
        """Close the PUB socket and disable future publication attempts."""

        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        self._enabled = False
