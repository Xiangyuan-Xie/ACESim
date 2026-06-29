"""ACEFollower-compatible ROS 2 shim for simulated aerial manipulators."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from acesim.utils.math import calculate_coupled_gripper_positions, gripper_public_to_joint5, joint5_to_gripper_public
from acesim.utils.sim_streams import ArmCommandStreamParams, ArmCommandStreamPublisher, ArmStateStreamSubscriber


class ACESimACEFollowerNode(Node):
    """Bridge ACETele leader topics to ACESim's simulated arm command stream."""

    ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4"]
    GRIPPER_NAME = "joint_5"

    def __init__(self) -> None:
        super().__init__("acesim_ace_follower")
        self.declare_parameter("heartbeat_timeout_sec", 1.0)
        self.declare_parameter("command_rate_hz", 100.0)
        self.declare_parameter("state_rate_hz", 100.0)

        self._sync_mode = "idle"
        self._sync_status = "idle"
        self._latest_gripper_command: float | None = None
        self._latest_gripper_position: float | None = None
        self._last_command_ns: int | None = None
        self._command_sequence = 0

        qos = QoSProfile(depth=10)
        self._arm_state_pub = self.create_publisher(JointState, "/ace_follower/arm/state", qos)
        self._gripper_state_pub = self.create_publisher(JointState, "/ace_follower/gripper/state", qos)
        self._sync_status_pub = self.create_publisher(String, "/ace_follower/arm/sync_status", qos)
        self.create_subscription(JointState, "/ace_leader/arm/command", self._on_arm_command, qos)
        self.create_subscription(JointState, "/ace_leader/gripper/command", self._on_gripper_command, qos)
        self.create_subscription(String, "/ace_leader/arm/sync_mode", self._on_sync_mode, qos)

        command_params = ArmCommandStreamParams(enabled=True, zmq_endpoint="tcp://0.0.0.0:5604")
        self._command_publisher = ArmCommandStreamPublisher(command_params)
        self._state_subscriber = ArmStateStreamSubscriber("tcp://127.0.0.1:5603")
        self.create_timer(1.0 / float(self.get_parameter("command_rate_hz").value), self._sync_timer_callback)
        self.create_timer(1.0 / float(self.get_parameter("state_rate_hz").value), self._state_timer_callback)
        self._publish_sync_status("idle")

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _publish_sync_status(self, status: str) -> None:
        self._sync_status = status
        message = String()
        message.data = status
        self._sync_status_pub.publish(message)

    def _on_sync_mode(self, message: String) -> None:
        mode = str(message.data).strip()
        self._sync_mode = mode
        if mode == "stop":
            self._publish_sync_status("lost")
        elif mode in ("sync_request", "ready"):
            self._publish_sync_status("ready")
        elif mode not in ("tracking",):
            self._publish_sync_status("idle")

    def _on_gripper_command(self, message: JointState) -> None:
        if self._sync_mode != "tracking":
            return
        if message.position:
            self._latest_gripper_command = float(message.position[0])

    def _on_arm_command(self, message: JointState) -> None:
        if self._sync_mode != "tracking" or len(message.position) < len(self.ARM_JOINT_NAMES):
            return
        self._last_command_ns = int(self.get_clock().now().nanoseconds)
        if self._latest_gripper_command is not None:
            gripper_source = gripper_public_to_joint5(self._latest_gripper_command)
        elif self._latest_gripper_position is not None:
            gripper_source = gripper_public_to_joint5(self._latest_gripper_position)
        elif len(message.position) > len(self.ARM_JOINT_NAMES):
            gripper_source = gripper_public_to_joint5(float(message.position[len(self.ARM_JOINT_NAMES)]))
        else:
            gripper_source = gripper_public_to_joint5(0.0)
        positions = [float(value) for value in message.position[: len(self.ARM_JOINT_NAMES)]]
        positions.append(gripper_source)
        positions.extend(calculate_coupled_gripper_positions(gripper_source))
        self._command_sequence += 1
        self._command_publisher.publish(self._now_us(), "ace_leader", positions)
        self._publish_sync_status("tracking")

    def _sync_timer_callback(self) -> None:
        if self._sync_mode == "tracking" and self._last_command_ns is not None:
            elapsed_s = (int(self.get_clock().now().nanoseconds) - self._last_command_ns) * 1e-9
            if elapsed_s > float(self.get_parameter("heartbeat_timeout_sec").value):
                self._publish_sync_status("lost")
        else:
            self._publish_sync_status(self._sync_status)

    def _state_timer_callback(self) -> None:
        sample = self._state_subscriber.read_latest()
        if sample is None:
            return
        now_msg = self.get_clock().now().to_msg()
        positions = [float(value) for value in sample["positions"]]
        velocities = [float(value) for value in sample["velocities"]]
        efforts = [float(value) for value in sample["efforts"]]

        arm_msg = JointState()
        arm_msg.header.stamp = now_msg
        arm_msg.name = self.ARM_JOINT_NAMES
        arm_msg.position = positions[: len(self.ARM_JOINT_NAMES)]
        arm_msg.velocity = velocities[: len(self.ARM_JOINT_NAMES)]
        arm_msg.effort = efforts[: len(self.ARM_JOINT_NAMES)]
        self._arm_state_pub.publish(arm_msg)

        gripper_msg = JointState()
        gripper_msg.header.stamp = now_msg
        gripper_msg.name = [self.GRIPPER_NAME]
        if len(positions) > len(self.ARM_JOINT_NAMES):
            gripper_index = len(self.ARM_JOINT_NAMES)
            gripper_public = joint5_to_gripper_public(positions[gripper_index])
            gripper_msg.position = [gripper_public]
            gripper_msg.velocity = [velocities[gripper_index] / 1.723]
            gripper_msg.effort = [abs(efforts[gripper_index])]
            self._latest_gripper_position = gripper_public
        else:
            gripper_msg.position = [0.0]
            gripper_msg.velocity = [0.0]
            gripper_msg.effort = [0.0]
            self._latest_gripper_position = 0.0
        self._gripper_state_pub.publish(gripper_msg)

    def destroy_node(self) -> None:
        self._command_publisher.close()
        self._state_subscriber.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ACESimACEFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
