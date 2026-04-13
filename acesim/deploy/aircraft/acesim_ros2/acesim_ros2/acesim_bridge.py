from __future__ import annotations

from pathlib import Path

import rclpy
from acesim_ros2.bridge.config import load_bridge_configs
from acesim_ros2.bridge.registry import PLUGIN_REGISTRY
from acesim_ros2.bridge.runtime import BridgeHost
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node


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


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AcesimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
