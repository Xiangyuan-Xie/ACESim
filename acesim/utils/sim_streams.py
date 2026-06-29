"""Small codecs and latest-sample ZMQ publishers for simulator streams."""

from __future__ import annotations

import math
import os
import struct
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Sequence, TypedDict

import zmq

from acesim.config.asset_params import get_optional_table


class ArmStatePayload(TypedDict):
    timestamp_us: int
    joint_count: int
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


class ArmCommandPayload(TypedDict):
    timestamp_us: int
    command_id: str
    positions: list[float]


class ControlStreamPayload(TypedDict):
    timestamp_us: int
    channel_count: int
    controls: list[float]


class VehicleTruthPayload(TypedDict):
    timestamp_us: int
    position_world_m_nwu: list[float]
    attitude_world_quat_scalar_first: list[float]
    linear_velocity_world_mps_nwu: list[float]
    angular_velocity_body_radps_flu: list[float]


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


class ArmCommandCodec:
    """Encode leader arm commands as timestamp, id, and seven actuator targets."""

    JOINT_COUNT = 7
    COMMAND_ID_BYTES = 64
    _STRUCT = struct.Struct("<Q64s7d")

    @classmethod
    def pack(cls, timestamp_us: int, command_id: str, positions: Sequence[float]) -> bytes:
        if len(positions) != cls.JOINT_COUNT:
            raise ValueError(f"positions must contain exactly {cls.JOINT_COUNT} values")
        values = [float(value) for value in positions]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("positions must contain finite values")
        command_id_bytes = command_id.encode("utf-8")
        if len(command_id_bytes) > cls.COMMAND_ID_BYTES:
            raise ValueError(f"command_id must be at most {cls.COMMAND_ID_BYTES} UTF-8 bytes")
        return cls._STRUCT.pack(int(timestamp_us), command_id_bytes.ljust(cls.COMMAND_ID_BYTES, b"\0"), *values)

    @classmethod
    def unpack(cls, payload: bytes) -> ArmCommandPayload:
        if len(payload) != cls._STRUCT.size:
            raise ValueError(f"Unexpected arm-command payload size={len(payload)}, expected {cls._STRUCT.size}")
        decoded = cls._STRUCT.unpack(payload)
        command_id = decoded[1].split(b"\0", 1)[0].decode("utf-8")
        return {
            "timestamp_us": int(decoded[0]),
            "command_id": command_id,
            "positions": [float(value) for value in decoded[2:]],
        }


class ControlStreamCodec:
    """Encode released PX4 actuator controls for debug visualization."""

    MAX_CHANNELS = 8
    _STRUCT = struct.Struct("<QI8d")

    @classmethod
    def pack(cls, timestamp_us: int, controls: Sequence[float]) -> bytes:
        if len(controls) > cls.MAX_CHANNELS:
            raise ValueError(f"controls supports at most {cls.MAX_CHANNELS} channels")
        padded_controls = [0.0] * cls.MAX_CHANNELS
        padded_controls[: len(controls)] = [float(value) for value in controls]
        return cls._STRUCT.pack(int(timestamp_us), int(len(controls)), *padded_controls)

    @classmethod
    def unpack(cls, payload: bytes) -> ControlStreamPayload:
        if len(payload) != cls._STRUCT.size:
            raise ValueError(f"Unexpected control-stream payload size={len(payload)}, expected {cls._STRUCT.size}")
        decoded = cls._STRUCT.unpack(payload)
        channel_count = int(decoded[1])
        if channel_count < 0 or channel_count > cls.MAX_CHANNELS:
            raise ValueError(f"Invalid control channel count: {channel_count}")
        return {
            "timestamp_us": int(decoded[0]),
            "channel_count": channel_count,
            "controls": [float(value) for value in decoded[2 : 2 + channel_count]],
        }


class VehicleTruthCodec:
    """Encode MuJoCo truth odometry for ROS2 debug/analysis consumers."""

    _STRUCT = struct.Struct("<Q3d4d3d3d")

    @classmethod
    def pack(
        cls,
        timestamp_us: int,
        position_world_m_nwu: Sequence[float],
        attitude_world_quat_scalar_first: Sequence[float],
        linear_velocity_world_mps_nwu: Sequence[float],
        angular_velocity_body_radps_flu: Sequence[float],
    ) -> bytes:
        position = cls._require_finite_vector("position_world_m_nwu", position_world_m_nwu, 3)
        quat = cls._require_finite_vector("attitude_world_quat_scalar_first", attitude_world_quat_scalar_first, 4)
        linear_velocity = cls._require_finite_vector("linear_velocity_world_mps_nwu", linear_velocity_world_mps_nwu, 3)
        angular_velocity = cls._require_finite_vector(
            "angular_velocity_body_radps_flu", angular_velocity_body_radps_flu, 3
        )
        return cls._STRUCT.pack(int(timestamp_us), *position, *quat, *linear_velocity, *angular_velocity)

    @classmethod
    def unpack(cls, payload: bytes) -> VehicleTruthPayload:
        if len(payload) != cls._STRUCT.size:
            raise ValueError(f"Unexpected vehicle-truth payload size={len(payload)}, expected {cls._STRUCT.size}")
        decoded = cls._STRUCT.unpack(payload)
        return {
            "timestamp_us": int(decoded[0]),
            "position_world_m_nwu": [float(value) for value in decoded[1:4]],
            "attitude_world_quat_scalar_first": [float(value) for value in decoded[4:8]],
            "linear_velocity_world_mps_nwu": [float(value) for value in decoded[8:11]],
            "angular_velocity_body_radps_flu": [float(value) for value in decoded[11:14]],
        }

    @staticmethod
    def _require_finite_vector(field_name: str, values: Sequence[float], expected_count: int) -> list[float]:
        if len(values) != expected_count:
            raise ValueError(f"{field_name} must contain exactly {expected_count} values")
        vector = [float(value) for value in values]
        if not all(math.isfinite(value) for value in vector):
            raise ValueError(f"{field_name} must contain finite values")
        return vector


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


@dataclass(frozen=True)
class ArmCommandStreamParams:
    enabled: bool = False
    zmq_endpoint: str = "tcp://127.0.0.1:5604"
    joint_names: tuple[str, ...] = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5")
    gripper_joint_names: tuple[str, ...] = ("joint_gripper_left", "joint_gripper_right")

    def __post_init__(self) -> None:
        if len(self.joint_names) != 5:
            raise ValueError("arm_command_stream joint_names must contain exactly five joints")
        if len(self.gripper_joint_names) != 2:
            raise ValueError("arm_command_stream gripper_joint_names must contain exactly two joints")

    @classmethod
    def from_asset_params(cls, asset_params: Mapping[str, object]) -> "ArmCommandStreamParams":
        config = get_optional_table(asset_params, "arm_command_stream")
        enabled = bool(config.get("enabled", False))
        enabled_override = os.environ.get("ACESIM_ARM_COMMAND_STREAM_ENABLED")
        if enabled_override is not None:
            enabled = enabled_override.strip().lower() in ("1", "true", "yes", "on")
        joint_names = config.get("joint_names", list(cls().joint_names))
        gripper_joint_names = config.get("gripper_joint_names", list(cls().gripper_joint_names))
        if not isinstance(joint_names, list) or not all(isinstance(name, str) for name in joint_names):
            raise ValueError("arm_command_stream joint_names must be a string list")
        if not isinstance(gripper_joint_names, list) or not all(isinstance(name, str) for name in gripper_joint_names):
            raise ValueError("arm_command_stream gripper_joint_names must be a string list")
        return cls(
            enabled=enabled,
            zmq_endpoint=os.environ.get(
                "ACESIM_ARM_COMMAND_STREAM_ENDPOINT",
                str(config.get("zmq_endpoint", "tcp://127.0.0.1:5604")),
            ),
            joint_names=tuple(joint_names),
            gripper_joint_names=tuple(gripper_joint_names),
        )


class ArmCommandStreamPublisher:
    """Publish newest ACETele leader commands for the MuJoCo arm follower."""

    def __init__(self, params: ArmCommandStreamParams) -> None:
        self._params = params
        endpoint = os.environ.get("ACESIM_ACE_FOLLOWER_COMMAND_ENDPOINT", params.zmq_endpoint)
        self._publisher = LatestZmqPublisher(endpoint, enabled=params.enabled)

    def publish(self, timestamp_us: int, command_id: str, positions: Sequence[float]) -> None:
        if not self._params.enabled:
            return
        self._publisher.publish(ArmCommandCodec.pack(timestamp_us, command_id, positions))

    def close(self) -> None:
        self._publisher.close()


class ArmCommandStreamSubscriber:
    """Read the newest available arm command without replaying stale commands."""

    def __init__(self, params: ArmCommandStreamParams) -> None:
        self._socket: zmq.Socket | None = None
        if params.enabled:
            socket = zmq.Context.instance().socket(zmq.SUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 1)
            socket.setsockopt(zmq.CONFLATE, 1)
            socket.setsockopt(zmq.SUBSCRIBE, b"")
            socket.connect(params.zmq_endpoint)
            self._socket = socket

    def read_latest(self) -> ArmCommandPayload | None:
        if self._socket is None:
            return None
        latest: ArmCommandPayload | None = None
        while True:
            try:
                latest = ArmCommandCodec.unpack(self._socket.recv(flags=zmq.NOBLOCK))
            except zmq.Again:
                return latest

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None


class ArmStateStreamSubscriber:
    """Read newest arm-state samples from the existing ACESim arm state stream."""

    def __init__(self, endpoint: str = "tcp://127.0.0.1:5603") -> None:
        endpoint = os.environ.get("ACESIM_ARM_STATE_INPUT_ENDPOINT", endpoint)
        socket = zmq.Context.instance().socket(zmq.SUB)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        socket.connect(endpoint)
        self._socket: zmq.Socket | None = socket

    def read_latest(self) -> ArmStatePayload | None:
        if self._socket is None:
            return None
        latest: ArmStatePayload | None = None
        while True:
            try:
                latest = ArmStateCodec.unpack(self._socket.recv(flags=zmq.NOBLOCK))
            except zmq.Again:
                return latest

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None


class ClockPublisher:
    """Publish simulation clock samples over the standard clock stream."""

    def __init__(self, zmq_endpoint: str = "tcp://0.0.0.0:5600", enable_zmq: bool = True) -> None:
        zmq_endpoint = os.environ.get("ACESIM_CLOCK_ZMQ_ENDPOINT", zmq_endpoint)
        self._publisher = LatestZmqPublisher(zmq_endpoint, enabled=enable_zmq)

    def publish(self, timestamp_us: int) -> None:
        self._publisher.publish(ClockCodec.pack(timestamp_us))

    def close(self) -> None:
        self._publisher.close()


@dataclass(frozen=True)
class ControlStreamParams:
    enabled: bool = False
    max_channels: int = ControlStreamCodec.MAX_CHANNELS
    zmq_endpoint: str = "tcp://0.0.0.0:5602"

    def __post_init__(self) -> None:
        if self.max_channels <= 0 or self.max_channels > ControlStreamCodec.MAX_CHANNELS:
            raise ValueError(f"max_channels must be in [1, {ControlStreamCodec.MAX_CHANNELS}]")

    @classmethod
    def from_asset_params(cls, asset_params: Mapping[str, object]) -> "ControlStreamParams":
        config = get_optional_table(asset_params, "control_stream")
        max_channels = config.get("max_channels", ControlStreamCodec.MAX_CHANNELS)
        if isinstance(max_channels, bool) or not isinstance(max_channels, int):
            raise ValueError("control_stream max_channels must be an integer")
        return cls(
            enabled=bool(config.get("enabled", False)),
            max_channels=int(max_channels),
            zmq_endpoint=os.environ.get(
                "ACESIM_CONTROL_ZMQ_ENDPOINT",
                str(config.get("zmq_endpoint", "tcp://0.0.0.0:5602")),
            ),
        )


class ControlStreamPublisher:
    """Publish released PX4 actuator controls over the control debug stream."""

    def __init__(self, params: ControlStreamParams) -> None:
        self._params = params
        self._publisher = LatestZmqPublisher(params.zmq_endpoint, enabled=params.enabled)

    @property
    def is_enabled(self) -> bool:
        return self._params.enabled

    def publish(self, timestamp_us: int, controls: Sequence[float]) -> None:
        if not self._params.enabled:
            return
        controls_to_publish = list(controls)
        if len(controls_to_publish) > self._params.max_channels:
            raise ValueError(f"controls supports at most {self._params.max_channels} configured channels")
        self._publisher.publish(ControlStreamCodec.pack(timestamp_us, controls_to_publish))

    def close(self) -> None:
        self._publisher.close()


@dataclass(frozen=True)
class VehicleTruthStreamParams:
    enabled: bool = False
    rate_hz: float = 120.0
    zmq_endpoint: str = "tcp://0.0.0.0:5605"

    def __post_init__(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError("truth_stream rate_hz must be positive")

    @classmethod
    def from_asset_params(cls, asset_params: Mapping[str, object]) -> "VehicleTruthStreamParams":
        config = get_optional_table(asset_params, "truth_stream")
        return cls(
            enabled=bool(config.get("enabled", False)),
            rate_hz=float(config.get("rate_hz", 120.0)),
            zmq_endpoint=os.environ.get(
                "ACESIM_TRUTH_ZMQ_ENDPOINT",
                str(config.get("zmq_endpoint", "tcp://0.0.0.0:5605")),
            ),
        )


class VehicleTruthStatePublisher:
    """Publish MuJoCo truth odometry over the vehicle truth debug stream."""

    def __init__(self, params: VehicleTruthStreamParams) -> None:
        self._params = params
        self._publisher = LatestZmqPublisher(params.zmq_endpoint, enabled=params.enabled)

    @property
    def is_enabled(self) -> bool:
        return self._params.enabled

    @property
    def rate_hz(self) -> float:
        return self._params.rate_hz

    @property
    def endpoint(self) -> str:
        return self._params.zmq_endpoint

    def publish(
        self,
        timestamp_us: int,
        position_world_m_nwu: Sequence[float],
        attitude_world_quat_scalar_first: Sequence[float],
        linear_velocity_world_mps_nwu: Sequence[float],
        angular_velocity_body_radps_flu: Sequence[float],
    ) -> None:
        if not self._params.enabled:
            return
        self._publisher.publish(
            VehicleTruthCodec.pack(
                timestamp_us,
                position_world_m_nwu,
                attitude_world_quat_scalar_first,
                linear_velocity_world_mps_nwu,
                angular_velocity_body_radps_flu,
            )
        )

    def close(self) -> None:
        self._publisher.close()
