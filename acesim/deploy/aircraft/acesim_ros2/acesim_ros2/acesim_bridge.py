from __future__ import annotations

import signal
from pathlib import Path

import rclpy
from acesim_ros2.bridge.config import load_bridge_configs
from acesim_ros2.bridge.registry import PLUGIN_REGISTRY
from acesim_ros2.bridge.runtime import BridgeHost
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node


class ShutdownRequested(Exception):
    """Raised by the SIGTERM handler so launch shutdown exits quietly."""


def default_bridge_config_path() -> str:
    try:
        return str(Path(get_package_share_directory("acesim_ros2")).resolve() / "config" / "bridges.yaml")
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1] / "config" / "bridges.yaml")


class AcesimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("acesim_bridge")
        self.declare_parameter("bridge_config_file", default_bridge_config_path())
        self.declare_parameter("bridge_overrides_file", "")

        bridge_config_file = str(self.get_parameter("bridge_config_file").value).strip() or default_bridge_config_path()
        bridge_overrides_file = str(self.get_parameter("bridge_overrides_file").value).strip()

        bridge_configs = load_bridge_configs(
            bridge_config_file,
            bridge_overrides_file or None,
        )
        self._bridge_host = BridgeHost(self, bridge_configs, PLUGIN_REGISTRY)
        self._bridge_runtimes = self._bridge_host._bridge_runtimes

    def destroy_node(self) -> bool:
        self._bridge_host.close()
        return super().destroy_node()


def _request_shutdown(_signum: int, _frame: object) -> None:
    raise ShutdownRequested()


def _is_ros_external_shutdown(exc: BaseException) -> bool:
    return exc.__class__.__name__ == "ExternalShutdownException" and exc.__class__.__module__.startswith("rclpy")


def _is_ros_context_shutdown_error(exc: BaseException) -> bool:
    message = str(exc).lower()
    return (
        exc.__class__.__name__ == "RCLError"
        and exc.__class__.__module__.startswith("rclpy")
        and ("context is invalid" in message or "context is not valid" in message)
        and not rclpy.ok()
    )


def main(args: list[str] | None = None) -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    node: AcesimBridgeNode | None = None
    shutdown_requested = False
    try:
        rclpy.init(args=args)
        node = AcesimBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ShutdownRequested):
        shutdown_requested = True
    except Exception as exc:
        if not (_is_ros_external_shutdown(exc) or _is_ros_context_shutdown_error(exc)):
            raise
        shutdown_requested = True
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ShutdownRequested):
                shutdown_requested = True
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, ShutdownRequested):
                shutdown_requested = True
    if shutdown_requested:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
