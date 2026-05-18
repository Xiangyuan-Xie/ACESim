"""Small codecs and latest-sample ZMQ publishers for simulator streams."""

from __future__ import annotations

import struct
from typing import Sequence, TypedDict

import zmq


class ArmStatePayload(TypedDict):
    timestamp_us: int
    joint_count: int
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


class ClockCodec:
    """Encode the simulation clock as one unsigned 64-bit microsecond value."""

    _STRUCT = struct.Struct("<Q")

    @classmethod
    def pack(cls, timestamp_us: int) -> bytes:
        return cls._STRUCT.pack(int(timestamp_us))

    @classmethod
    def unpack(cls, payload: bytes) -> int:
        if len(payload) != cls._STRUCT.size:
            raise ValueError(f"Unexpected clock payload size={len(payload)}, expected {cls._STRUCT.size}")
        return int(cls._STRUCT.unpack(payload)[0])


class ArmStateCodec:
    """Encode exported arm joints as timestamp, positions, velocities, efforts."""

    JOINT_COUNT = 7
    LEGACY_JOINT_COUNT = 5
    _STRUCT = struct.Struct("<Q21d")
    LEGACY_STRUCT = struct.Struct("<Q15d")

    @classmethod
    def pack(
        cls,
        timestamp_us: int,
        positions: Sequence[float],
        velocities: Sequence[float],
        efforts: Sequence[float],
    ) -> bytes:
        cls._require_joint_count("positions", positions)
        cls._require_joint_count("velocities", velocities)
        cls._require_joint_count("efforts", efforts)
        return cls._STRUCT.pack(
            int(timestamp_us),
            *[float(value) for value in positions],
            *[float(value) for value in velocities],
            *[float(value) for value in efforts],
        )

    @classmethod
    def unpack(cls, payload: bytes) -> ArmStatePayload:
        if len(payload) == cls._STRUCT.size:
            struct_obj = cls._STRUCT
            joint_count = cls.JOINT_COUNT
        elif len(payload) == cls.LEGACY_STRUCT.size:
            struct_obj = cls.LEGACY_STRUCT
            joint_count = cls.LEGACY_JOINT_COUNT
        else:
            raise ValueError(
                f"Unexpected arm-state payload size={len(payload)}, "
                f"expected {cls._STRUCT.size} or {cls.LEGACY_STRUCT.size}"
            )
        decoded = struct_obj.unpack(payload)
        positions_end = 1 + joint_count
        velocities_end = positions_end + joint_count
        return {
            "timestamp_us": int(decoded[0]),
            "joint_count": joint_count,
            "positions": [float(value) for value in decoded[1:positions_end]],
            "velocities": [float(value) for value in decoded[positions_end:velocities_end]],
            "efforts": [float(value) for value in decoded[velocities_end:]],
        }

    @classmethod
    def _require_joint_count(cls, field_name: str, values: Sequence[float]) -> None:
        if len(values) != cls.JOINT_COUNT:
            raise ValueError(f"{field_name} must contain exactly {cls.JOINT_COUNT} values")


class LatestZmqPublisher:
    """PUB socket configured for newest-sample-only simulator telemetry."""

    def __init__(self, endpoint: str, enabled: bool = True) -> None:
        self._endpoint = endpoint
        self._socket: zmq.Socket | None = None

        if enabled:
            context = zmq.Context.instance()
            socket = context.socket(zmq.PUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SNDHWM, 1)
            socket.setsockopt(zmq.CONFLATE, 1)
            socket.bind(self._endpoint)
            self._socket = socket

    @property
    def endpoint(self) -> str:
        return self._endpoint

    def publish(self, payload: bytes) -> None:
        if self._socket is None:
            return
        self._socket.send(payload, flags=zmq.NOBLOCK)

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None


class ClockPublisher:
    """Publish simulation clock samples over the standard clock stream."""

    def __init__(self, zmq_endpoint: str = "tcp://0.0.0.0:5600", enable_zmq: bool = True) -> None:
        self._publisher = LatestZmqPublisher(zmq_endpoint, enabled=enable_zmq)

    def publish(self, timestamp_us: int) -> None:
        self._publisher.publish(ClockCodec.pack(timestamp_us))

    def close(self) -> None:
        self._publisher.close()
