from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Any, Callable

from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

_DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
_ARM_COMMAND_STRUCT = struct.Struct("<Q5d")


@dataclass(frozen=True)
class DecodedArmCommand:
    timestamp_us: int
    joint_positions: list[float]


def normalize_joint_values(values: list[float] | tuple[float, ...], *, joint_dim: int) -> list[float]:
    trimmed = list(values[:joint_dim])
    if len(trimmed) < joint_dim:
        trimmed.extend([0.0] * (joint_dim - len(trimmed)))
    normalized: list[float] = []
    for value in trimmed:
        numeric = float(value)
        normalized.append(0.0 if not math.isfinite(numeric) else numeric)
    return normalized


def apply_defaults(raw_bridge: dict[str, object]) -> dict[str, object]:
    normalized = dict(raw_bridge)
    raw_joint_names = raw_bridge.get("joint_names")
    if isinstance(raw_joint_names, list):
        normalized["joint_names"] = [str(name) for name in raw_joint_names]
    else:
        normalized["joint_names"] = list(_DEFAULT_JOINT_NAMES)
    return normalized


def decode_payload(payload: bytes) -> DecodedArmCommand:
    if len(payload) != _ARM_COMMAND_STRUCT.size:
        raise ValueError(f"Unexpected arm-command payload size={len(payload)}, expected {_ARM_COMMAND_STRUCT.size}")
    decoded = _ARM_COMMAND_STRUCT.unpack(payload)
    return DecodedArmCommand(
        timestamp_us=int(decoded[0]),
        joint_positions=[float(value) for value in decoded[1:6]],
    )


def extract_timestamp_us(decoded: DecodedArmCommand) -> int:
    return decoded.timestamp_us


def build_sink(node: Any, bridge_config: BridgeConfig) -> Callable[[DecodedArmCommand], None]:
    publisher = node.create_publisher(
        JointState,
        bridge_config.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    joint_names = list(bridge_config.joint_names or _DEFAULT_JOINT_NAMES)

    def publish(decoded: DecodedArmCommand) -> None:
        timestamp_us = decoded.timestamp_us
        message = JointState()
        message.header.stamp.sec = timestamp_us // 1_000_000
        message.header.stamp.nanosec = (timestamp_us % 1_000_000) * 1_000
        message.name = list(joint_names)
        message.position = normalize_joint_values(decoded.joint_positions, joint_dim=len(joint_names))
        message.velocity = [0.0] * len(joint_names)
        message.effort = [0.0] * len(joint_names)
        publisher.publish(message)

    return publish


PLUGIN = BridgePluginSpec(
    bridge_name="arm_command_ros",
    apply_defaults=apply_defaults,
    decode_payload=decode_payload,
    extract_timestamp_us=extract_timestamp_us,
    build_sink=build_sink,
)
