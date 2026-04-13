from __future__ import annotations

import struct
from typing import Any, Callable

from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

_CLOCK_STRUCT = struct.Struct("<Q")


def apply_defaults(raw_bridge: dict[str, object]) -> dict[str, object]:
    return dict(raw_bridge)


def decode_payload(payload: bytes) -> int:
    if len(payload) != _CLOCK_STRUCT.size:
        raise ValueError(f"Unexpected clock payload size={len(payload)}, expected {_CLOCK_STRUCT.size}")
    return int(_CLOCK_STRUCT.unpack(payload)[0])


def extract_timestamp_us(decoded: int) -> int:
    return decoded


def build_sink(node: Any, bridge_config: BridgeConfig) -> Callable[[int], None]:
    publisher = node.create_publisher(
        Clock,
        bridge_config.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )

    def publish(decoded: int) -> None:
        timestamp_us = decoded
        message = Clock()
        message.clock.sec = timestamp_us // 1_000_000
        message.clock.nanosec = (timestamp_us % 1_000_000) * 1_000
        publisher.publish(message)

    return publish


PLUGIN = BridgePluginSpec(
    bridge_name="simulation_clock",
    apply_defaults=apply_defaults,
    decode_payload=decode_payload,
    extract_timestamp_us=extract_timestamp_us,
    build_sink=build_sink,
)
