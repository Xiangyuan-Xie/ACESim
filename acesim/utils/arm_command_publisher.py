"""ZeroMQ publisher for the manipulator command exported by ``MCArmEnv``."""

from __future__ import annotations

import struct
from typing import Sequence

import zmq


class ArmCommandPublisher:
    """Publish a fixed-size arm command payload over ZeroMQ."""

    JOINT_COUNT = 5
    _STRUCT = struct.Struct("<Q5d")

    def __init__(
        self,
        zmq_endpoint: str = "tcp://0.0.0.0:5602",
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

    @classmethod
    def pack_payload(cls, timestamp_us: int, joint_positions: Sequence[float]) -> bytes:
        if len(joint_positions) != cls.JOINT_COUNT:
            raise ValueError(f"joint_positions must contain exactly {cls.JOINT_COUNT} values")

        return cls._STRUCT.pack(
            int(timestamp_us),
            *[float(value) for value in joint_positions],
        )

    @classmethod
    def unpack(cls, payload: bytes) -> dict[str, object]:
        if len(payload) != cls._STRUCT.size:
            raise ValueError(f"Unexpected payload size={len(payload)}, expected {cls._STRUCT.size}")
        decoded = cls._STRUCT.unpack(payload)
        return {
            "timestamp_us": int(decoded[0]),
            "joint_positions": [float(value) for value in decoded[1:6]],
        }

    def publish(self, timestamp_us: int, joint_positions: Sequence[float]) -> None:
        if self._socket is None:
            return
        self._socket.send(self.pack_payload(timestamp_us, joint_positions), flags=zmq.NOBLOCK)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
