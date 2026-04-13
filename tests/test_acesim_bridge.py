from __future__ import annotations

import importlib.util
import math
import struct
import sys
import tempfile
import textwrap
import types
import unittest
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import patch


def _load_acesim_bridge_module() -> ModuleType:
    module_name = "_test_acesim_ros2_acesim_bridge"
    for name in [
        module_name,
        "ament_index_python",
        "ament_index_python.packages",
        "rclpy",
        "rclpy.node",
        "rclpy.qos",
        "zmq",
        "rosgraph_msgs",
        "rosgraph_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
        "px4_msgs",
        "px4_msgs.msg",
    ]:
        sys.modules.pop(name, None)

    ament_index_module = types.ModuleType("ament_index_python")
    ament_index_packages_module = types.ModuleType("ament_index_python.packages")
    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_qos_module = types.ModuleType("rclpy.qos")
    zmq_module = types.ModuleType("zmq")
    rosgraph_msgs_module = types.ModuleType("rosgraph_msgs")
    rosgraph_msgs_msg_module = types.ModuleType("rosgraph_msgs.msg")
    sensor_msgs_module = types.ModuleType("sensor_msgs")
    sensor_msgs_msg_module = types.ModuleType("sensor_msgs.msg")
    px4_msgs_module = types.ModuleType("px4_msgs")
    px4_msgs_msg_module = types.ModuleType("px4_msgs.msg")

    class _FakeStamp:
        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

    class _FakeHeader:
        def __init__(self) -> None:
            self.stamp = _FakeStamp()

    class Clock:
        def __init__(self) -> None:
            self.clock = _FakeStamp()

    class JointState:
        def __init__(self) -> None:
            self.header = _FakeHeader()
            self.name: list[str] = []
            self.position: list[float] = []
            self.velocity: list[float] = []
            self.effort: list[float] = []

    class ArmJointCommand:
        def __init__(self) -> None:
            self.timestamp = 0
            self.arm_command: list[float] = []

    class ArmJointState:
        def __init__(self) -> None:
            self.timestamp = 0
            self.arm_position: list[float] = []
            self.arm_velocity: list[float] = []

    class PackageNotFoundError(Exception):
        pass

    class _NodeLogger:
        def __init__(self) -> None:
            self.infos: list[str] = []

        def info(self, message: str) -> None:
            self.infos.append(message)

    class _NodePublisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class _NodeTimer:
        def __init__(self, period: float, callback: object) -> None:
            self.period = period
            self.callback = callback

    class Node:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            self.logger = _NodeLogger()
            self.publishers: list[_NodePublisher] = []
            self.timers: list[_NodeTimer] = []
            self._parameters: dict[str, object] = {}

        def declare_parameter(self, name: str, value: object) -> None:
            self._parameters[name] = value

        def get_parameter(self, name: str) -> types.SimpleNamespace:
            return types.SimpleNamespace(value=self._parameters[name])

        def create_publisher(self, _message_type: object, topic: str, _qos: object) -> _NodePublisher:
            publisher = _NodePublisher(topic)
            self.publishers.append(publisher)
            return publisher

        def create_timer(self, period: float, callback: object) -> _NodeTimer:
            timer = _NodeTimer(period, callback)
            self.timers.append(timer)
            return timer

        def get_logger(self) -> _NodeLogger:
            return self.logger

        def destroy_node(self) -> bool:
            return True

    class QoSProfile:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    class _Enum:
        KEEP_LAST = "keep_last"
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"
        VOLATILE = "volatile"
        TRANSIENT_LOCAL = "transient_local"

    class Again(Exception):
        pass

    class ZMQError(Exception):
        pass

    class _FakeSocket:
        def __init__(self) -> None:
            self.sockopts: list[tuple[object, object]] = []

        def setsockopt(self, *_args: object, **_kwargs: object) -> None:
            self.sockopts.append((_args[0], _args[1]))

        def connect(self, *_args: object, **_kwargs: object) -> None:
            return None

        def close(self, *_args: object, **_kwargs: object) -> None:
            return None

        def recv(self, *_args: object, **_kwargs: object) -> bytes:
            raise Again()

    class _FakeContext:
        @staticmethod
        def instance() -> "_FakeContext":
            return _FakeContext()

        def socket(self, *_args: object, **_kwargs: object) -> _FakeSocket:
            socket = _FakeSocket()
            setattr(zmq_module, "_last_socket", socket)
            return socket

    def get_package_share_directory(_: str) -> str:
        return "/tmp/install/share/acesim_ros2"

    def init(*_args: object, **_kwargs: object) -> None:
        return None

    def spin(*_args: object, **_kwargs: object) -> None:
        return None

    def shutdown(*_args: object, **_kwargs: object) -> None:
        return None

    def ok() -> bool:
        return True

    setattr(ament_index_packages_module, "get_package_share_directory", get_package_share_directory)
    setattr(ament_index_packages_module, "PackageNotFoundError", PackageNotFoundError)
    setattr(ament_index_module, "packages", ament_index_packages_module)

    setattr(rclpy_module, "init", init)
    setattr(rclpy_module, "spin", spin)
    setattr(rclpy_module, "shutdown", shutdown)
    setattr(rclpy_module, "ok", ok)
    setattr(rclpy_module, "node", rclpy_node_module)
    setattr(rclpy_module, "qos", rclpy_qos_module)
    setattr(rclpy_node_module, "Node", Node)
    setattr(rclpy_qos_module, "QoSProfile", QoSProfile)
    setattr(rclpy_qos_module, "HistoryPolicy", _Enum)
    setattr(rclpy_qos_module, "ReliabilityPolicy", _Enum)
    setattr(rclpy_qos_module, "DurabilityPolicy", _Enum)

    setattr(zmq_module, "Context", _FakeContext)
    setattr(zmq_module, "Again", Again)
    setattr(zmq_module, "ZMQError", ZMQError)
    setattr(zmq_module, "SUB", 1)
    setattr(zmq_module, "LINGER", 2)
    setattr(zmq_module, "RCVHWM", 3)
    setattr(zmq_module, "CONFLATE", 4)
    setattr(zmq_module, "SUBSCRIBE", 5)
    setattr(zmq_module, "NOBLOCK", 6)

    setattr(rosgraph_msgs_msg_module, "Clock", Clock)
    setattr(rosgraph_msgs_module, "msg", rosgraph_msgs_msg_module)
    setattr(sensor_msgs_msg_module, "JointState", JointState)
    setattr(sensor_msgs_module, "msg", sensor_msgs_msg_module)
    setattr(px4_msgs_msg_module, "ArmJointCommand", ArmJointCommand)
    setattr(px4_msgs_msg_module, "ArmJointState", ArmJointState)
    setattr(px4_msgs_module, "msg", px4_msgs_msg_module)

    sys.modules["ament_index_python"] = ament_index_module
    sys.modules["ament_index_python.packages"] = ament_index_packages_module
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = rclpy_node_module
    sys.modules["rclpy.qos"] = rclpy_qos_module
    sys.modules["zmq"] = zmq_module
    sys.modules["rosgraph_msgs"] = rosgraph_msgs_module
    sys.modules["rosgraph_msgs.msg"] = rosgraph_msgs_msg_module
    sys.modules["sensor_msgs"] = sensor_msgs_module
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg_module
    sys.modules["px4_msgs"] = px4_msgs_module
    sys.modules["px4_msgs.msg"] = px4_msgs_msg_module

    module_path = (
        Path(__file__).resolve().parents[1]
        / "acesim"
        / "deploy"
        / "aircraft"
        / "acesim_ros2"
        / "acesim_ros2"
        / "acesim_bridge.py"
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


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


class AcesimBridgeTests(unittest.TestCase):
    acesim_bridge: ModuleType

    @classmethod
    def setUpClass(cls) -> None:
        cls.acesim_bridge = _load_acesim_bridge_module()

    def _make_bridge_node(
        self,
        config_text: str,
        *,
        overrides_text: str | None = None,
        bridge_config_parameter: str | None = None,
        bridge_overrides_parameter: str | None = None,
    ) -> Any:
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        config_path = Path(temp_dir.name) / "bridges.yaml"
        config_path.write_text(textwrap.dedent(config_text).strip() + "\n", encoding="utf-8")

        overrides_value = ""
        if overrides_text is not None:
            overrides_path = Path(temp_dir.name) / "overrides.yaml"
            overrides_path.write_text(textwrap.dedent(overrides_text), encoding="utf-8")
            overrides_value = str(overrides_path)

        config_value = str(config_path) if bridge_config_parameter is None else bridge_config_parameter
        if bridge_overrides_parameter is not None:
            overrides_value = bridge_overrides_parameter

        original_get_parameter = self.acesim_bridge.AcesimBridgeNode.get_parameter

        def get_parameter(node: object, name: str) -> types.SimpleNamespace:
            if name == "bridge_config_file":
                return types.SimpleNamespace(value=config_value)
            if name == "bridge_overrides_file":
                return types.SimpleNamespace(value=overrides_value)
            return original_get_parameter(node, name)

        with (
            patch.object(self.acesim_bridge, "default_bridge_config_path", return_value=str(config_path)),
            patch.object(self.acesim_bridge.AcesimBridgeNode, "get_parameter", new=get_parameter),
        ):
            return self.acesim_bridge.AcesimBridgeNode()

    def test_default_bridge_config_path_prefers_share_directory(self) -> None:
        with patch.object(
            self.acesim_bridge, "get_package_share_directory", return_value="/tmp/install/share/acesim_ros2"
        ):
            path = self.acesim_bridge.default_bridge_config_path()

        self.assertEqual(path, "/tmp/install/share/acesim_ros2/config/bridges.yaml")

    def test_default_bridge_config_path_falls_back_to_source_tree(self) -> None:
        package_error = self.acesim_bridge.PackageNotFoundError("missing")
        with patch.object(self.acesim_bridge, "get_package_share_directory", side_effect=package_error):
            path = self.acesim_bridge.default_bridge_config_path()

        self.assertTrue(path.endswith("acesim_ros2/config/bridges.yaml"))

    def test_default_bridge_config_path_does_not_swallow_unexpected_errors(self) -> None:
        with patch.object(self.acesim_bridge, "get_package_share_directory", side_effect=RuntimeError("boom")):
            with self.assertRaisesRegex(RuntimeError, "boom"):
                self.acesim_bridge.default_bridge_config_path()

    def test_bridge_module_does_not_keep_inlined_single_use_helpers(self) -> None:
        for name in (
            "package_share_dir",
            "resolve_bridge_config_file",
            "_load_raw_bridge_map",
            "build_bridge_definition",
            "load_enabled_bridge_definitions",
            "load_bridge_overrides",
            "_build_qos_profile",
            "MonotonicFieldTracker",
        ):
            self.assertFalse(hasattr(self.acesim_bridge, name), name)
        self.assertFalse(hasattr(self.acesim_bridge, "HandlerRuntimeSpec"))
        self.assertFalse(hasattr(self.acesim_bridge, "CompiledBridgeRuntime"))
        self.assertFalse(hasattr(self.acesim_bridge, "normalize_joint_values"))
        self.assertFalse(hasattr(self.acesim_bridge, "decode_arm_command_payload"))
        self.assertFalse(hasattr(self.acesim_bridge, "decode_arm_state_payload"))
        self.assertFalse(hasattr(self.acesim_bridge, "BRIDGE_REGISTRY"))
        self.assertFalse(hasattr(self.acesim_bridge, "zmq"))

    def test_bridge_node_uses_default_bridge_config_path_when_parameter_empty(self) -> None:
        node = self._make_bridge_node(
            """
            bridges:
              simulation_clock:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
            """,
            bridge_config_parameter="",
        )

        self.assertEqual([runtime.name for runtime in node._bridge_runtimes], ["simulation_clock"])

    def test_bridge_node_applies_arm_command_defaults(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              arm_command_ros:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /arm/command
            """)

        bridge = node._bridge_runtimes[0]._bridge
        self.assertEqual(bridge.name, "arm_command_ros")
        self.assertEqual(bridge.poll_period_sec, 0.001)
        self.assertEqual(bridge.joint_names, ["joint1", "joint2", "joint3", "joint4", "joint5"])
        self.assertEqual(bridge.topic, "/arm/command")

    def test_bridge_node_loads_all_enabled_bridges_from_config(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              simulation_clock:
                enabled: true
                poll_period_sec: 0.001
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
              disabled_bridge:
                enabled: false
                poll_period_sec: 0.001
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5601
                topic: /ignored
              arm_command_ros:
                enabled: true
                poll_period_sec: 0.005
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /arm/command
              arm_command_px4:
                enabled: true
                poll_period_sec: 0.005
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /fmu/in/arm_joint_command
            """)

        self.assertEqual(
            [runtime.name for runtime in node._bridge_runtimes],
            ["simulation_clock", "arm_command_ros", "arm_command_px4"],
        )
        self.assertEqual([timer.period for timer in node.timers], [0.001, 0.005, 0.005])

    def test_bridge_node_creates_timer_per_bridge(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              simulation_clock:
                enabled: true
                poll_period_sec: 0.001
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
              arm_command_ros:
                enabled: true
                poll_period_sec: 0.005
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /arm/command
              arm_command_px4:
                enabled: true
                poll_period_sec: 0.005
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /fmu/in/arm_joint_command
            """)

        self.assertEqual(
            [runtime.name for runtime in node._bridge_runtimes],
            ["simulation_clock", "arm_command_ros", "arm_command_px4"],
        )
        self.assertEqual([timer.period for timer in node.timers], [0.001, 0.005, 0.005])
        self.assertEqual(len(node.publishers), 3)

    def test_bridge_node_init_transport_uses_fixed_socket_options(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              simulation_clock:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
            """)

        last_socket = node._bridge_runtimes[0]._transport._socket
        self.assertIn((2, 0), last_socket.sockopts)
        self.assertIn((3, 1), last_socket.sockopts)
        self.assertIn((4, 1), last_socket.sockopts)
        self.assertEqual(node._bridge_runtimes[0]._input_endpoint, "tcp://127.0.0.1:5600")

    def test_bridge_node_prefers_input_endpoint_override(self) -> None:
        node = self._make_bridge_node(
            """
            bridges:
              simulation_clock:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
            """,
            overrides_text="""
            overrides:
              simulation_clock:
                input_endpoint: tcp://172.20.32.1:5600
            """,
        )

        self.assertEqual(node._bridge_runtimes[0]._input_endpoint, "tcp://172.20.32.1:5600")

    def test_simulation_clock_handler_publishes_clock_message(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              simulation_clock:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
            """)
        runtime = node._bridge_runtimes[0]

        runtime.process_payload(struct.pack("<Q", 2_500_000))

        publisher = next(publisher for publisher in node.publishers if publisher.topic == "/acesim/clock")
        message = publisher.messages[0]
        self.assertEqual(message.clock.sec, 2)
        self.assertEqual(message.clock.nanosec, 500_000_000)

    def test_arm_command_ros_handler_publishes_joint_state_message(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              arm_command_ros:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /arm/command
            """)
        runtime = node._bridge_runtimes[0]

        runtime.process_payload(struct.pack("<Q5d", 1_234_567, 1.0, math.nan, 3.0, 4.0, 5.0))

        ros_publisher = next(publisher for publisher in node.publishers if publisher.topic == "/arm/command")
        ros_message = ros_publisher.messages[0]
        self.assertEqual(ros_message.header.stamp.sec, 1)
        self.assertEqual(ros_message.header.stamp.nanosec, 234_567_000)
        self.assertEqual(ros_message.position, [1.0, 0.0, 3.0, 4.0, 5.0])
        self.assertEqual(len(node.publishers), 1)

    def test_arm_command_px4_handler_publishes_px4_message(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              arm_command_px4:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /fmu/in/arm_joint_command
            """)
        runtime = node._bridge_runtimes[0]

        runtime.process_payload(struct.pack("<Q5d", 1_234_567, 1.0, math.nan, 3.0, 4.0, 5.0))

        publisher = next(publisher for publisher in node.publishers if publisher.topic == "/fmu/in/arm_joint_command")
        message = publisher.messages[0]
        self.assertEqual(message.timestamp, 1_234_567)
        self.assertEqual(message.arm_command, [1.0, 0.0, 3.0, 4.0, 5.0])

    def test_arm_state_handler_publishes_px4_message(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              arm_state:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5603
                topic: /fmu/in/arm_joint_state
            """)
        runtime = node._bridge_runtimes[0]

        runtime.process_payload(
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

        publisher = next(publisher for publisher in node.publishers if publisher.topic == "/fmu/in/arm_joint_state")
        message = publisher.messages[0]
        self.assertEqual(message.timestamp, 123456)
        self.assertEqual(message.arm_position, [0.1, 0.2, 0.3, 0.4, 0.5])
        self.assertEqual(message.arm_velocity, [1.1, 1.2, 1.3, 1.4, 1.5])

    def test_compiled_bridge_runtime_decode_failure_raises(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              simulation_clock:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5600
                topic: /acesim/clock
            """)
        runtime = node._bridge_runtimes[0]

        with self.assertRaisesRegex(ValueError, "Unexpected clock payload size=3"):
            runtime.process_payload(b"bad")

    def test_compiled_bridge_runtime_non_monotonic_timestamp_raises(self) -> None:
        node = self._make_bridge_node("""
            bridges:
              arm_command_px4:
                enabled: true
                transport:
                  type: zmq_sub
                  endpoint: tcp://127.0.0.1:5602
                topic: /fmu/in/arm_joint_command
            """)
        runtime = node._bridge_runtimes[0]

        runtime.process_payload(struct.pack("<Q5d", 2_000_000, 1.0, 2.0, 3.0, 4.0, 5.0))
        with self.assertRaisesRegex(ValueError, "Non-monotonic timestamp_us"):
            runtime.process_payload(struct.pack("<Q5d", 1_000_000, 1.0, 2.0, 3.0, 4.0, 5.0))

    def test_main_cleans_up_on_keyboard_interrupt(self) -> None:
        calls: list[str] = []

        class FakeBridgeNode:
            def destroy_node(self) -> None:
                calls.append("destroy_node")

        fake_node = FakeBridgeNode()

        with (
            patch.object(self.acesim_bridge.rclpy, "init", side_effect=lambda args=None: calls.append("init")),
            patch.object(self.acesim_bridge, "AcesimBridgeNode", side_effect=lambda: fake_node),
            patch.object(self.acesim_bridge.rclpy, "spin", side_effect=KeyboardInterrupt),
            patch.object(self.acesim_bridge.rclpy, "ok", side_effect=lambda: True),
            patch.object(self.acesim_bridge.rclpy, "shutdown", side_effect=lambda: calls.append("shutdown")),
        ):
            self.acesim_bridge.main(["--test"])

        self.assertEqual(calls, ["init", "destroy_node", "shutdown"])

    def test_main_propagates_unexpected_errors(self) -> None:
        calls: list[str] = []

        class FakeBridgeNode:
            def destroy_node(self) -> None:
                calls.append("destroy_node")

        fake_node = FakeBridgeNode()

        with (
            patch.object(self.acesim_bridge.rclpy, "init", side_effect=lambda args=None: calls.append("init")),
            patch.object(self.acesim_bridge, "AcesimBridgeNode", side_effect=lambda: fake_node),
            patch.object(self.acesim_bridge.rclpy, "spin", side_effect=RuntimeError("boom")),
            patch.object(self.acesim_bridge.rclpy, "ok", side_effect=lambda: True),
            patch.object(self.acesim_bridge.rclpy, "shutdown", side_effect=lambda: calls.append("shutdown")),
        ):
            with self.assertRaisesRegex(RuntimeError, "boom"):
                self.acesim_bridge.main(["--test"])

        self.assertEqual(calls, ["init", "destroy_node", "shutdown"])


if __name__ == "__main__":
    unittest.main()
