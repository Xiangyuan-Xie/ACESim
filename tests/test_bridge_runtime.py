from __future__ import annotations

import struct
import unittest
from types import ModuleType
from typing import Any

from ros2_bridge_testbed import load_bridge_package_module


def _load_runtime_module() -> ModuleType:
    return load_bridge_package_module("_test_acesim_ros2_runtime", "bridge/runtime.py")


class _FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)


class _FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: list[Any] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _FakeTimer:
    def __init__(self, period: float, callback: Any) -> None:
        self.period = period
        self.callback = callback


class _FakeNode:
    def __init__(self) -> None:
        self.logger = _FakeLogger()
        self.publishers: list[_FakePublisher] = []
        self.timers: list[_FakeTimer] = []

    def create_publisher(self, _message_type: object, topic: str, _qos: object) -> _FakePublisher:
        publisher = _FakePublisher(topic)
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, period: float, callback: object) -> _FakeTimer:
        timer = _FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self) -> _FakeLogger:
        return self.logger


class BridgeRuntimeTests(unittest.TestCase):
    runtime: ModuleType
    config_loader: ModuleType
    plugin_registry: ModuleType

    @classmethod
    def setUpClass(cls) -> None:
        cls.runtime = _load_runtime_module()
        cls.config_loader = load_bridge_package_module(
            "_test_acesim_ros2_config_loader_for_runtime",
            "bridge/config.py",
        )
        cls.plugin_registry = load_bridge_package_module(
            "_test_acesim_ros2_plugin_registry_for_runtime",
            "bridge/registry.py",
        )

    def test_zmq_sub_transport_uses_latest_sample_socket_options(self) -> None:
        transport = self.runtime.ZmqSubTransport("tcp://127.0.0.1:5600")

        socket = self.runtime.zmq._last_socket
        self.assertIn((self.runtime.zmq.LINGER, 0), socket.sockopts)
        self.assertIn((self.runtime.zmq.RCVHWM, 1), socket.sockopts)
        self.assertIn((self.runtime.zmq.CONFLATE, 1), socket.sockopts)
        self.assertIn((self.runtime.zmq.SUBSCRIBE, b""), socket.sockopts)

        transport.close()
        self.assertTrue(socket.closed)

    def test_bridge_host_builds_runtime_and_publishes(self) -> None:
        node = _FakeNode()
        bridge_config = self.config_loader.BridgeConfig(
            name="simulation_clock",
            enabled=True,
            poll_period_sec=0.005,
            transport=self.config_loader.TransportConfig(type="zmq_sub", endpoint="tcp://127.0.0.1:5600"),
            topic="/acesim/clock",
        )

        host = self.runtime.BridgeHost(node, [bridge_config], self.plugin_registry.PLUGIN_REGISTRY)
        runtime = host._bridge_runtimes[0]
        runtime.process_payload(struct.pack("<Q", 2_500_000))

        publisher = next(publisher for publisher in node.publishers if publisher.topic == "/acesim/clock")
        message = publisher.messages[0]
        self.assertEqual(message.clock.sec, 2)
        self.assertEqual(message.clock.nanosec, 500_000_000)
        self.assertEqual(node.timers[0].period, 0.005)

        host.close()
        self.assertTrue(self.runtime.zmq._last_socket.closed)


if __name__ == "__main__":
    unittest.main()
