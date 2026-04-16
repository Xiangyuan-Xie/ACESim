from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path
from types import ModuleType

from ros2_bridge_testbed import load_bridge_package_module


def _load_config_loader_module() -> ModuleType:
    return load_bridge_package_module("_test_acesim_ros2_config_loader", "bridge/config.py")


class BridgeConfigLoaderTests(unittest.TestCase):
    config_loader: ModuleType

    @classmethod
    def setUpClass(cls) -> None:
        cls.config_loader = _load_config_loader_module()

    def test_load_bridge_configs_applies_defaults_and_overrides(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "bridges.yaml"
            overrides_path = Path(temp_dir) / "overrides.yaml"
            config_path.write_text(
                textwrap.dedent("""
                    bridges:
                      arm_state:
                        enabled: true
                        transport:
                          type: zmq_sub
                          endpoint: tcp://127.0.0.1:5603
                        topic: /fmu/in/arm_joint_state
                    """).strip() + "\n",
                encoding="utf-8",
            )
            overrides_path.write_text(
                textwrap.dedent("""
                    overrides:
                      arm_state:
                        input_endpoint: tcp://172.20.32.1:5603
                    """).strip() + "\n",
                encoding="utf-8",
            )

            bridge_configs = self.config_loader.load_bridge_configs(str(config_path), str(overrides_path))

        self.assertEqual(len(bridge_configs), 1)
        bridge_config = bridge_configs[0]
        self.assertEqual(bridge_config.name, "arm_state")
        self.assertEqual(bridge_config.poll_period_sec, 0.001)
        self.assertEqual(bridge_config.topic, "/fmu/in/arm_joint_state")
        self.assertIsNone(bridge_config.joint_names)
        self.assertEqual(bridge_config.override.input_endpoint, "tcp://172.20.32.1:5603")

    def test_load_bridge_configs_rejects_unknown_bridge_name(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "bridges.yaml"
            config_path.write_text(
                textwrap.dedent("""
                    bridges:
                      unknown_bridge:
                        enabled: true
                        transport:
                          type: zmq_sub
                          endpoint: tcp://127.0.0.1:5602
                        topic: /unknown
                    """).strip() + "\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "Unsupported bridge name: unknown_bridge"):
                self.config_loader.load_bridge_configs(str(config_path))


if __name__ == "__main__":
    unittest.main()
