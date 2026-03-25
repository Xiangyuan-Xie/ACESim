from __future__ import annotations

import argparse
import struct

import rclpy
import zmq
from acesim_ros2.zmq_endpoints import resolve_endpoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimulationClockZmqBridge(Node):
    def __init__(self, endpoint: str) -> None:
        super().__init__("simulation_clock_zmq_bridge")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._clock_pub = self.create_publisher(Clock, "/acesim/clock", qos)
        self._last_sim_time_us = -1

        # Do not shadow rclpy.Node._context (ROS context used by timers/executors).
        self._zmq_context = zmq.Context.instance()
        self._socket = self._zmq_context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVHWM, 1)
        try:
            self._socket.setsockopt(zmq.CONFLATE, 1)
        except (AttributeError, zmq.ZMQError):
            pass
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.connect(endpoint)

        self.get_logger().info(f"Clock bridge connected to {endpoint}, publishing /acesim/clock")

        self._timer = self.create_timer(0.001, self._poll)

    def _poll(self) -> None:
        while True:
            try:
                payload = self._socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            except zmq.ZMQError as exc:
                self.get_logger().error(f"ZMQ receive error: {exc}")
                return

            if len(payload) != 8:
                self.get_logger().warning(f"Unexpected clock payload size={len(payload)}, expected 8")
                continue

            sim_time_us = struct.unpack("<Q", payload)[0]
            if sim_time_us < self._last_sim_time_us:
                self.get_logger().warning(
                    f"Dropped non-monotonic clock sample: {sim_time_us} < {self._last_sim_time_us}"
                )
                continue

            self._last_sim_time_us = sim_time_us
            msg = Clock()
            msg.clock.sec = sim_time_us // 1_000_000
            msg.clock.nanosec = (sim_time_us % 1_000_000) * 1_000
            self._clock_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._socket.close(linger=0)
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="Bridge ACESim ZeroMQ clock to ROS2 /acesim/clock")
    parser.add_argument("--mode", choices=["linux", "wsl"], default="linux")
    args = parser.parse_args()

    endpoint = resolve_endpoint(args.mode, 5600)

    rclpy.init()
    node = SimulationClockZmqBridge(endpoint)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
