from __future__ import annotations

import math
import struct
from typing import Any, Callable

from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec
from acesim_ros2.bridge.plugins.arm_command_ros import DecodedArmCommand
from px4_msgs.msg import ArmJointCommand
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_ARM_COMMAND_STRUCT = struct.Struct("<Q5d")


def _normalize_joint_values(values: list[float] | tuple[float, ...], *, joint_dim: int = 5) -> list[float]:
    trimmed = list(values[:joint_dim])
    if len(trimmed) < joint_dim:
        trimmed.extend([0.0] * (joint_dim - len(trimmed)))
    normalized: list[float] = []
    for value in trimmed:
        numeric = float(value)
        normalized.append(0.0 if not math.isfinite(numeric) else numeric)
    return normalized


def apply_defaults(raw_bridge: dict[str, object]) -> dict[str, object]:
    return dict(raw_bridge)


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
        ArmJointCommand,
        bridge_config.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )

    def publish(decoded: DecodedArmCommand) -> None:
        message = ArmJointCommand()
        message.timestamp = decoded.timestamp_us
        message.arm_command = _normalize_joint_values(decoded.joint_positions)
        publisher.publish(message)

    return publish


PLUGIN = BridgePluginSpec(
    bridge_name="arm_command_px4",
    apply_defaults=apply_defaults,
    decode_payload=decode_payload,
    extract_timestamp_us=extract_timestamp_us,
    build_sink=build_sink,
)
