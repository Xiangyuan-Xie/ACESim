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
        "std_msgs",
        "std_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
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
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    nav_msgs_module = types.ModuleType("nav_msgs")
    nav_msgs_msg_module = types.ModuleType("nav_msgs.msg")
    px4_msgs_module = types.ModuleType("px4_msgs")
    px4_msgs_msg_module = types.ModuleType("px4_msgs.msg")

    class _FakeStamp:
        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

    class _FakeHeader:
        def __init__(self) -> None:
            self.stamp = _FakeStamp()
            self.frame_id = ""

    class _Vector3:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Pose:
        def __init__(self) -> None:
            self.position = _Vector3()
            self.orientation = _Quaternion()

    class _PoseWithCovariance:
        def __init__(self) -> None:
            self.pose = _Pose()
            self.covariance = [0.0] * 36

    class _Twist:
        def __init__(self) -> None:
            self.linear = _Vector3()
            self.angular = _Vector3()

    class _TwistWithCovariance:
        def __init__(self) -> None:
            self.twist = _Twist()
            self.covariance = [0.0] * 36

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

    class MultiArrayDimension:
        def __init__(self) -> None:
            self.label = ""
            self.size = 0
            self.stride = 0

    class MultiArrayLayout:
        def __init__(self) -> None:
            self.dim: list[MultiArrayDimension] = []
            self.data_offset = 0

    class Float64MultiArray:
        def __init__(self) -> None:
            self.layout = MultiArrayLayout()
            self.data: list[float] = []

    class String:
        def __init__(self) -> None:
            self.data = ""

    class Odometry:
        def __init__(self) -> None:
            self.header = _FakeHeader()
            self.child_frame_id = ""
            self.pose = _PoseWithCovariance()
            self.twist = _TwistWithCovariance()

    class ArmJointState:
        def __init__(self) -> None:
            self.timestamp = 0
            self.arm_position: list[float] = []

    class PackageNotFoundError(Exception):
        pass

    class _FakePublisher:
        def __init__(self, topic: str, qos: object) -> None:
            self.topic = topic
            self.qos = qos
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class _FakeTimer:
        def __init__(self, period_sec: float, callback: object) -> None:
            self.period_sec = period_sec
            self.callback = callback

    class _FakeClockNow:
        def __init__(self, nanoseconds: int) -> None:
            self.nanoseconds = nanoseconds

        def to_msg(self) -> _FakeStamp:
            stamp = _FakeStamp()
            stamp.sec = self.nanoseconds // 1_000_000_000
            stamp.nanosec = self.nanoseconds % 1_000_000_000
            return stamp

    class _FakeClock:
        def __init__(self, node: "Node") -> None:
            self._node = node

        def now(self) -> _FakeClockNow:
            return _FakeClockNow(self._node._now_ns)

    class _FakeLogger:
        def __init__(self) -> None:
            self.messages: list[tuple[str, str]] = []

        def info(self, message: str) -> None:
            self.messages.append(("info", message))

        def warning(self, message: str) -> None:
            self.messages.append(("warning", message))

        def error(self, message: str) -> None:
            self.messages.append(("error", message))

    class Node:
        def __init__(self, name: str = "fake_node") -> None:
            self.name = name
            self.publishers: dict[str, _FakePublisher] = {}
            self.subscriptions: dict[str, object] = {}
            self.timers: list[_FakeTimer] = []
            self.parameters: dict[str, object] = {}
            self._now_ns = 0
            self._logger = _FakeLogger()

        def declare_parameter(self, name: str, value: object) -> object:
            self.parameters.setdefault(name, value)
            return types.SimpleNamespace(value=self.parameters[name])

        def get_parameter(self, name: str) -> object:
            return types.SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, _msg_type: object, topic: str, _qos: object) -> _FakePublisher:
            publisher = _FakePublisher(topic, _qos)
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, _msg_type: object, topic: str, callback: object, _qos: object) -> object:
            self.subscriptions[topic] = callback
            self.subscriptions[f"{topic}__qos"] = _qos
            return callback

        def create_timer(self, period_sec: float, callback: object) -> _FakeTimer:
            timer = _FakeTimer(period_sec, callback)
            self.timers.append(timer)
            return timer

        def get_clock(self) -> _FakeClock:
            return _FakeClock(self)

        def get_logger(self) -> _FakeLogger:
            return self._logger

        def destroy_node(self) -> None:
            return None

    class QoSProfile:
        def __init__(self, *args: object, **kwargs: object) -> None:
            self.args = args
            self.kwargs = kwargs
            self.depth = kwargs.get("depth", args[0] if args else None)

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
    setattr(zmq_module, "REQ", 2)
    setattr(zmq_module, "LINGER", 3)
    setattr(zmq_module, "RCVHWM", 4)
    setattr(zmq_module, "CONFLATE", 5)
    setattr(zmq_module, "SUBSCRIBE", 6)
    setattr(zmq_module, "NOBLOCK", 7)
    setattr(zmq_module, "RCVTIMEO", 8)
    setattr(zmq_module, "SNDTIMEO", 9)
    setattr(zmq_module, "EAGAIN", 11)
    setattr(rosgraph_msgs_msg_module, "Clock", Clock)
    setattr(rosgraph_msgs_module, "msg", rosgraph_msgs_msg_module)
    setattr(sensor_msgs_msg_module, "JointState", JointState)
    setattr(sensor_msgs_module, "msg", sensor_msgs_msg_module)
    setattr(std_msgs_msg_module, "Float64MultiArray", Float64MultiArray)
    setattr(std_msgs_msg_module, "MultiArrayDimension", MultiArrayDimension)
    setattr(std_msgs_msg_module, "String", String)
    setattr(std_msgs_module, "msg", std_msgs_msg_module)
    setattr(nav_msgs_msg_module, "Odometry", Odometry)
    setattr(nav_msgs_module, "msg", nav_msgs_msg_module)
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
    sys.modules["std_msgs"] = std_msgs_module
    sys.modules["std_msgs.msg"] = std_msgs_msg_module
    sys.modules["nav_msgs"] = nav_msgs_module
    sys.modules["nav_msgs.msg"] = nav_msgs_msg_module
    sys.modules["px4_msgs"] = px4_msgs_module
    sys.modules["px4_msgs.msg"] = px4_msgs_msg_module

    setattr(rclpy_module, "init", lambda *args, **kwargs: None)
    setattr(rclpy_module, "spin", lambda *args, **kwargs: None)
    setattr(rclpy_module, "shutdown", lambda *args, **kwargs: None)

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
