from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from types import ModuleType


def package_root() -> Path:
    return Path(__file__).resolve().parents[1] / "acesim" / "deploy" / "aircraft" / "acesim_ros2" / "acesim_ros2"


def install_fake_ros2_bridge_dependencies() -> None:
    for name in [
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
        "acesim_ros2",
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

    class ArmJointState:
        def __init__(self) -> None:
            self.timestamp = 0
            self.arm_position: list[float] = []

    class PackageNotFoundError(Exception):
        pass

    class Node:
        pass

    class QoSProfile:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    class _Enum:
        KEEP_LAST = "keep_last"
        BEST_EFFORT = "best_effort"
        VOLATILE = "volatile"

    class Again(Exception):
        pass

    class _FakeSocket:
        def __init__(self) -> None:
            self.sockopts: list[tuple[object, object]] = []
            self.recv_values: list[bytes] = []
            self.closed = False

        def setsockopt(self, option: object, value: object) -> None:
            self.sockopts.append((option, value))

        def connect(self, *_args: object, **_kwargs: object) -> None:
            return None

        def close(self, *_args: object, **_kwargs: object) -> None:
            self.closed = True

        def recv(self, *_args: object, **_kwargs: object) -> bytes:
            if self.recv_values:
                return self.recv_values.pop(0)
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

    setattr(ament_index_packages_module, "get_package_share_directory", get_package_share_directory)
    setattr(ament_index_packages_module, "PackageNotFoundError", PackageNotFoundError)
    setattr(ament_index_module, "packages", ament_index_packages_module)
    setattr(rclpy_module, "node", rclpy_node_module)
    setattr(rclpy_module, "qos", rclpy_qos_module)
    setattr(rclpy_node_module, "Node", Node)
    setattr(rclpy_qos_module, "QoSProfile", QoSProfile)
    setattr(rclpy_qos_module, "HistoryPolicy", _Enum)
    setattr(rclpy_qos_module, "ReliabilityPolicy", _Enum)
    setattr(rclpy_qos_module, "DurabilityPolicy", _Enum)
    setattr(zmq_module, "Context", _FakeContext)
    setattr(zmq_module, "Again", Again)
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

    package_module = types.ModuleType("acesim_ros2")
    setattr(package_module, "__path__", [str(package_root())])
    sys.modules["acesim_ros2"] = package_module


def load_bridge_package_module(module_name: str, relative_path: str) -> ModuleType:
    install_fake_ros2_bridge_dependencies()
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, package_root() / relative_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load {relative_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module
