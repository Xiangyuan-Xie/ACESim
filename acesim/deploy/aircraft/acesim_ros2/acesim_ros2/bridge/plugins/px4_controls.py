from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from acesim.utils.sim_streams import ControlStreamCodec


@dataclass(frozen=True)
class DecodedPX4Controls:
    timestamp_us: int
    controls: list[float]


def apply_defaults(raw_bridge: dict[str, object]) -> dict[str, object]:
    return dict(raw_bridge)


def decode_payload(payload: bytes) -> DecodedPX4Controls:
    decoded = ControlStreamCodec.unpack(payload)
    return DecodedPX4Controls(
        timestamp_us=int(decoded["timestamp_us"]),
        controls=list(decoded["controls"]),
    )


def extract_timestamp_us(decoded: DecodedPX4Controls) -> int:
    return decoded.timestamp_us


def build_sink(node: Any, bridge_config: BridgeConfig) -> Callable[[DecodedPX4Controls], None]:
    publisher = node.create_publisher(
        Float64MultiArray,
        bridge_config.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )

    def publish(decoded: DecodedPX4Controls) -> None:
        message = Float64MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = "channel"
        dimension.size = len(decoded.controls)
        dimension.stride = len(decoded.controls)
        message.layout.dim = [dimension]
        message.layout.data_offset = 0
        message.data = list(decoded.controls)
        publisher.publish(message)

    return publish


PLUGIN = BridgePluginSpec(
    bridge_name="px4_controls",
    apply_defaults=apply_defaults,
    decode_payload=decode_payload,
    extract_timestamp_us=extract_timestamp_us,
    build_sink=build_sink,
)
