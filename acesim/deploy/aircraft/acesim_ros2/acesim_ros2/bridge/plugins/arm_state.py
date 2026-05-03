from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec
from px4_msgs.msg import ArmJointState
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from acesim.utils.sim_streams import ArmStateCodec


@dataclass(frozen=True)
class DecodedArmState:
    timestamp_us: int
    position: list[float]
    velocity: list[float]


def apply_defaults(raw_bridge: dict[str, object]) -> dict[str, object]:
    return dict(raw_bridge)


def decode_payload(payload: bytes) -> DecodedArmState:
    decoded = ArmStateCodec.unpack(payload)
    return DecodedArmState(
        timestamp_us=int(decoded["timestamp_us"]),
        position=list(decoded["positions"]),
        velocity=list(decoded["velocities"]),
    )


def extract_timestamp_us(decoded: DecodedArmState) -> int:
    return decoded.timestamp_us


def build_sink(node: Any, bridge_config: BridgeConfig) -> Callable[[DecodedArmState], None]:
    publisher = node.create_publisher(
        ArmJointState,
        bridge_config.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )

    def publish(decoded: DecodedArmState) -> None:
        message = ArmJointState()
        message.timestamp = decoded.timestamp_us
        message.arm_position = list(decoded.position)
        message.arm_velocity = list(decoded.velocity)
        publisher.publish(message)

    return publish


PLUGIN = BridgePluginSpec(
    bridge_name="arm_state",
    apply_defaults=apply_defaults,
    decode_payload=decode_payload,
    extract_timestamp_us=extract_timestamp_us,
    build_sink=build_sink,
)
