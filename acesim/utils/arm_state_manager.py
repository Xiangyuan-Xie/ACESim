from __future__ import annotations

"""Real arm state manager with ZeroMQ publication."""

import struct

import zmq


class ArmStateManager:
    """Publish the real arm state over ZeroMQ without blocking the sim loop."""

    _STRUCT = struct.Struct("<Q15d")

    def __init__(
        self,
        zmq_endpoint: str = "tcp://0.0.0.0:5601",
        enable_zmq: bool = True,
    ) -> None:
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
            print(f"[ACESim] ZMQ arm state manager disabled: {exc}")
            self._socket = None

    def publish(
        self,
        timestamp_us: int,
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
    ) -> None:
        if not self._enabled or self._socket is None:
            return

        payload = self._STRUCT.pack(
            int(timestamp_us),
            *positions,
            *velocities,
            *efforts,
        )
        try:
            self._socket.send(payload, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except zmq.ZMQError as exc:
            print(f"[ACESim] ZMQ arm state manager publish failed: {exc}")

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        self._enabled = False
