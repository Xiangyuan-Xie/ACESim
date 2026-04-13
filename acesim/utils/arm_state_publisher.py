"""ZeroMQ publisher for the manipulator state exported by ``MCArmEnv``.

The wire format is fixed: one timestamp in microseconds followed by the
positions, velocities, and efforts of the 5 exported arm joints.
"""

from __future__ import annotations

import struct
from typing import Sequence

import zmq


class ArmStatePublisher:
    """Publish a fixed-size arm state payload over ZeroMQ."""

    JOINT_COUNT = 5
    _STRUCT = struct.Struct("<Q15d")

    def __init__(
        self,
        zmq_endpoint: str = "tcp://0.0.0.0:5603",
        enable_zmq: bool = True,
    ) -> None:
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

    def publish(
        self,
        timestamp_us: int,
        positions: Sequence[float],
        velocities: Sequence[float],
        efforts: Sequence[float],
    ) -> None:
        """Publish one timestamped arm state sample."""

        if len(positions) != self.JOINT_COUNT:
            raise ValueError(f"positions must contain exactly {self.JOINT_COUNT} values")
        if len(velocities) != self.JOINT_COUNT:
            raise ValueError(f"velocities must contain exactly {self.JOINT_COUNT} values")
        if len(efforts) != self.JOINT_COUNT:
            raise ValueError(f"efforts must contain exactly {self.JOINT_COUNT} values")

        if self._socket is None:
            return

        payload = self._STRUCT.pack(
            int(timestamp_us),
            *[float(value) for value in positions],
            *[float(value) for value in velocities],
            *[float(value) for value in efforts],
        )
        self._socket.send(payload, flags=zmq.NOBLOCK)

    def close(self) -> None:
        """Close the PUB socket and stop future publication."""

        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
