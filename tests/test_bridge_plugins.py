from __future__ import annotations

import struct
import unittest
from types import ModuleType
from typing import Any

from ros2_bridge_testbed import load_bridge_package_module


class _FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: list[Any] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _FakeNode:
    def __init__(self) -> None:
        self.publishers: list[_FakePublisher] = []

    def create_publisher(self, _message_type: object, topic: str, _qos: object) -> _FakePublisher:
        publisher = _FakePublisher(topic)
        self.publishers.append(publisher)
        return publisher


class BridgePluginTests(unittest.TestCase):
    plugin_api: ModuleType
    simulation_clock: ModuleType
    arm_state: ModuleType

    @classmethod
    def setUpClass(cls) -> None:
        cls.plugin_api = load_bridge_package_module("_test_acesim_ros2_plugin_api_for_plugins", "bridge/plugin_api.py")
        cls.simulation_clock = load_bridge_package_module(
            "_test_acesim_ros2_simulation_clock_plugin", "bridge/plugins/simulation_clock.py"
        )
        cls.arm_state = load_bridge_package_module("_test_acesim_ros2_arm_state_plugin", "bridge/plugins/arm_state.py")

    def test_simulation_clock_plugin_publishes_clock_message(self) -> None:
        node = _FakeNode()
        bridge_config = self.plugin_api.BridgeConfig(
            name="simulation_clock",
            enabled=True,
            poll_period_sec=0.001,
            transport=self.plugin_api.TransportConfig(type="zmq_sub", endpoint="tcp://127.0.0.1:5600"),
            topic="/acesim/clock",
        )

        sink = self.simulation_clock.build_sink(node, bridge_config)
        sink(self.simulation_clock.decode_payload(struct.pack("<Q", 2_500_000)))

        message = node.publishers[0].messages[0]
        self.assertEqual(message.clock.sec, 2)
        self.assertEqual(message.clock.nanosec, 500_000_000)

    def test_arm_state_plugin_publishes_px4_message(self) -> None:
        node = _FakeNode()
        bridge_config = self.plugin_api.BridgeConfig(
            name="arm_state",
            enabled=True,
            poll_period_sec=0.001,
            transport=self.plugin_api.TransportConfig(type="zmq_sub", endpoint="tcp://127.0.0.1:5603"),
            topic="/fmu/in/arm_joint_state",
        )

        sink = self.arm_state.build_sink(node, bridge_config)
        sink(
            self.arm_state.decode_payload(
                struct.pack(
                    "<Q15d",
                    123456,
                    0.1,
                    0.2,
                    0.3,
                    0.4,
                    0.5,
                    1.1,
                    1.2,
                    1.3,
                    1.4,
                    1.5,
                    9.1,
                    9.2,
                    9.3,
                    9.4,
                    9.5,
                )
            )
        )

        message = node.publishers[0].messages[0]
        self.assertEqual(message.timestamp, 123456)
        self.assertEqual(message.arm_position, [0.1, 0.2, 0.3, 0.4, 0.5])
        self.assertEqual(message.arm_velocity, [1.1, 1.2, 1.3, 1.4, 1.5])


if __name__ == "__main__":
    unittest.main()
