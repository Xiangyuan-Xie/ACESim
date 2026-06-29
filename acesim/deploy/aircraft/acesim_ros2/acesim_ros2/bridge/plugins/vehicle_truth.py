from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from acesim.utils.sim_streams import VehicleTruthCodec


@dataclass(frozen=True)
class DecodedVehicleTruth:
    timestamp_us: int
    position_world_m_nwu: list[float]
    attitude_world_quat_scalar_first: list[float]
    linear_velocity_world_mps_nwu: list[float]
    angular_velocity_body_radps_flu: list[float]


def apply_defaults(raw_bridge: dict[str, object]) -> dict[str, object]:
    return dict(raw_bridge)


def decode_payload(payload: bytes) -> DecodedVehicleTruth:
    decoded = VehicleTruthCodec.unpack(payload)
    return DecodedVehicleTruth(
        timestamp_us=int(decoded["timestamp_us"]),
        position_world_m_nwu=list(decoded["position_world_m_nwu"]),
        attitude_world_quat_scalar_first=list(decoded["attitude_world_quat_scalar_first"]),
        linear_velocity_world_mps_nwu=list(decoded["linear_velocity_world_mps_nwu"]),
        angular_velocity_body_radps_flu=list(decoded["angular_velocity_body_radps_flu"]),
    )


def extract_timestamp_us(decoded: DecodedVehicleTruth) -> int:
    return decoded.timestamp_us


def build_sink(node: Any, bridge_config: BridgeConfig) -> Callable[[DecodedVehicleTruth], None]:
    publisher = node.create_publisher(
        Odometry,
        bridge_config.topic,
        QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )

    def publish(decoded: DecodedVehicleTruth) -> None:
        message = Odometry()
        message.header.stamp.sec = decoded.timestamp_us // 1_000_000
        message.header.stamp.nanosec = (decoded.timestamp_us % 1_000_000) * 1_000
        message.header.frame_id = "acesim_world_nwu"
        message.child_frame_id = "base_link_flu"

        px, py, pz = decoded.position_world_m_nwu
        message.pose.pose.position.x = px
        message.pose.pose.position.y = py
        message.pose.pose.position.z = pz

        qw, qx, qy, qz = decoded.attitude_world_quat_scalar_first
        message.pose.pose.orientation.w = qw
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz

        vx, vy, vz = decoded.linear_velocity_world_mps_nwu
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        r02 = 2.0 * (qx * qz + qw * qy)
        r10 = 2.0 * (qx * qy + qw * qz)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qw * qx)
        r20 = 2.0 * (qx * qz - qw * qy)
        r21 = 2.0 * (qy * qz + qw * qx)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        message.twist.twist.linear.x = r00 * vx + r10 * vy + r20 * vz
        message.twist.twist.linear.y = r01 * vx + r11 * vy + r21 * vz
        message.twist.twist.linear.z = r02 * vx + r12 * vy + r22 * vz

        wx, wy, wz = decoded.angular_velocity_body_radps_flu
        message.twist.twist.angular.x = wx
        message.twist.twist.angular.y = wy
        message.twist.twist.angular.z = wz
        message.pose.covariance = [0.0] * 36
        message.twist.covariance = [0.0] * 36
        publisher.publish(message)

    return publish


PLUGIN = BridgePluginSpec(
    bridge_name="vehicle_truth",
    apply_defaults=apply_defaults,
    decode_payload=decode_payload,
    extract_timestamp_us=extract_timestamp_us,
    build_sink=build_sink,
)
