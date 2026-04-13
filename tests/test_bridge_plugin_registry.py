from __future__ import annotations

import unittest
from types import ModuleType

from ros2_bridge_testbed import load_bridge_package_module


def _load_plugin_registry_module() -> ModuleType:
    return load_bridge_package_module("_test_acesim_ros2_plugin_registry", "bridge/registry.py")


class BridgePluginRegistryTests(unittest.TestCase):
    plugin_registry: ModuleType

    @classmethod
    def setUpClass(cls) -> None:
        cls.plugin_registry = _load_plugin_registry_module()

    def test_registry_contains_all_supported_bridge_plugins(self) -> None:
        self.assertEqual(
            tuple(self.plugin_registry.PLUGIN_REGISTRY),
            ("simulation_clock", "arm_command_ros", "arm_command_px4", "arm_state"),
        )


if __name__ == "__main__":
    unittest.main()
