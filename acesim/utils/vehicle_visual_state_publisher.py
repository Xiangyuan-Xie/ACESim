"""ZeroMQ publisher for MuJoCo vehicle visual state samples.

The payload is fixed-size and intended for low-latency visualization clients.
World-frame values are published in ACESim's canonical NWU frame and the body
attitude quaternion is scalar-first.
"""

from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from typing import Mapping

import numpy as np
import zmq

from acesim.config.asset_params import get_optional_table


@dataclass(frozen=True)
class VehicleVisualStreamParams:
    """Configuration for the optional vehicle visual-state stream."""

    enabled: bool = False
    rate_hz: float = 120.0
    zmq_endpoint: str = "tcp://0.0.0.0:5601"

    def __post_init__(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")

    @classmethod
    def from_asset_params(cls, asset_params: Mapping[str, object]) -> "VehicleVisualStreamParams":
        config = get_optional_table(asset_params, "visual_stream")
        return cls(
            enabled=bool(config.get("enabled", False)),
            rate_hz=float(config.get("rate_hz", 120.0)),
            zmq_endpoint=os.environ.get(
                "ACESIM_VISUAL_ZMQ_ENDPOINT",
                str(config.get("zmq_endpoint", "tcp://0.0.0.0:5601")),
            ),
        )


@dataclass(frozen=True)
class VehicleVisualState:
    """One timestamped vehicle visual-state sample."""

    timestamp_us: int
    position_world_m_nwu: np.ndarray
    attitude_world_quat_scalar_first: np.ndarray
    rotor_angle_rad: np.ndarray
    rotor_visual_speed_radps: np.ndarray


class VehicleVisualStatePublisher:
    """Publish fixed-size vehicle visual-state payloads over ZeroMQ."""

    MAX_ROTORS = 8
    _STRUCT = struct.Struct("<Q3d4dI8d8d")

    def __init__(self, params: VehicleVisualStreamParams) -> None:
        self._params = params
        self._socket: zmq.Socket | None = None
        if not self._params.enabled:
            return

        context = zmq.Context.instance()
        socket = context.socket(zmq.PUB)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.SNDHWM, 1)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.bind(self._params.zmq_endpoint)
        self._socket = socket

    @property
    def is_enabled(self) -> bool:
        return self._params.enabled

    @property
    def rate_hz(self) -> float:
        return self._params.rate_hz

    @property
    def endpoint(self) -> str:
        return self._params.zmq_endpoint

    def publish(self, state: VehicleVisualState) -> None:
        """Publish one state sample if streaming is enabled."""

        if self._socket is None:
            return

        position = np.asarray(state.position_world_m_nwu, dtype=float).reshape(-1)
        if position.size != 3:
            raise ValueError("position_world_m_nwu must contain exactly three values")

        quat = np.asarray(state.attitude_world_quat_scalar_first, dtype=float).reshape(-1)
        if quat.size != 4:
            raise ValueError("attitude_world_quat_scalar_first must contain exactly four values")

        rotor_angle = np.asarray(state.rotor_angle_rad, dtype=float).reshape(-1)
        rotor_speed = np.asarray(state.rotor_visual_speed_radps, dtype=float).reshape(-1)
        if rotor_angle.size != rotor_speed.size:
            raise ValueError("rotor_angle_rad and rotor_visual_speed_radps must have the same length")
        if rotor_angle.size > self.MAX_ROTORS:
            raise ValueError(f"At most {self.MAX_ROTORS} rotors are supported per payload")

        rotor_angle_padded = np.zeros(self.MAX_ROTORS, dtype=float)
        rotor_speed_padded = np.zeros(self.MAX_ROTORS, dtype=float)
        rotor_angle_padded[: rotor_angle.size] = rotor_angle
        rotor_speed_padded[: rotor_speed.size] = rotor_speed

        payload = self._STRUCT.pack(
            int(state.timestamp_us),
            *position.tolist(),
            *quat.tolist(),
            int(rotor_angle.size),
            *rotor_angle_padded.tolist(),
            *rotor_speed_padded.tolist(),
        )
        self._socket.send(payload, flags=zmq.NOBLOCK)

    @classmethod
    def unpack(cls, payload: bytes) -> dict[str, object]:
        """Decode one wire payload for validation and debug tools."""

        if len(payload) != cls._STRUCT.size:
            raise ValueError(f"Unexpected payload size={len(payload)}, expected {cls._STRUCT.size}")
        decoded = cls._STRUCT.unpack(payload)
        rotor_count = int(decoded[8])
        return {
            "timestamp_us": int(decoded[0]),
            "position_world_m_nwu": np.array(decoded[1:4], dtype=float),
            "attitude_world_quat_scalar_first": np.array(decoded[4:8], dtype=float),
            "rotor_count": rotor_count,
            "rotor_angle_rad": np.array(decoded[9:17], dtype=float)[:rotor_count],
            "rotor_visual_speed_radps": np.array(decoded[17:25], dtype=float)[:rotor_count],
        }

    def close(self) -> None:
        """Close the PUB socket and stop future publication."""

        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
