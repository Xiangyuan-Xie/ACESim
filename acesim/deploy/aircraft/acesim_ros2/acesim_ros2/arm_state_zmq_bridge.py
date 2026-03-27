from __future__ import annotations

import argparse
import struct

import rclpy
import zmq
from acesim_ros2.zmq_endpoints import resolve_endpoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class ArmStateZmqBridge(Node):
    """Bridge ACESim arm state ZMQ samples to ROS2 JointState."""

    _STRUCT = struct.Struct("<Q15d")
    _JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]

    def __init__(self, endpoint: str) -> None:
        super().__init__("arm_state_zmq_bridge")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_pub = self.create_publisher(JointState, "/arm/state", qos)
        self._last_sim_time_us = -1

        self._zmq_context = zmq.Context.instance()
        self._socket = self._zmq_context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVHWM, 1)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.connect(endpoint)

        self.get_logger().info(f"Arm-state bridge connected to {endpoint}, publishing /arm/state")

        self._timer = self.create_timer(0.001, self._poll)

    def _poll(self) -> None:
        while True:
            try:
                payload = self._socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                return

            if len(payload) != self._STRUCT.size:
                raise RuntimeError(f"Unexpected arm-state payload size={len(payload)}, expected {self._STRUCT.size}")

            decoded = self._STRUCT.unpack(payload)
            sim_time_us = decoded[0]
            if sim_time_us < self._last_sim_time_us:
                raise RuntimeError(f"Dropped non-monotonic arm-state sample: {sim_time_us} < {self._last_sim_time_us}")

            self._last_sim_time_us = sim_time_us
            msg = JointState()
            msg.header.stamp.sec = sim_time_us // 1_000_000
            msg.header.stamp.nanosec = (sim_time_us % 1_000_000) * 1_000
            msg.name = list(self._JOINT_NAMES)
            msg.position = list(decoded[1:6])
            msg.velocity = list(decoded[6:11])
            msg.effort = list(decoded[11:16])
            self._state_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._socket.close(linger=0)
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="Bridge ACESim ZeroMQ arm state to ROS2 /arm/state")
    parser.add_argument("--mode", choices=["linux", "wsl"], default="linux")
    args, remaining_args = parser.parse_known_args()

    endpoint = resolve_endpoint(args.mode, 5601)

    rclpy.init(args=remaining_args)
    node = ArmStateZmqBridge(endpoint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
