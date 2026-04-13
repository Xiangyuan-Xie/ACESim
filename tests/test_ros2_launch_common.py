from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import ModuleType
from typing import Any, ClassVar
from unittest.mock import patch

import yaml


def _load_launch_common_module() -> ModuleType:
    module_name = "_test_acesim_ros2_launch_common"
    for name in [
        module_name,
        "ament_index_python",
        "ament_index_python.packages",
        "launch",
        "launch.actions",
        "launch.event_handlers",
        "launch_ros",
        "launch_ros.actions",
    ]:
        sys.modules.pop(name, None)

    launch_module: Any = types.ModuleType("launch")
    launch_actions_module: Any = types.ModuleType("launch.actions")
    launch_event_handlers_module: Any = types.ModuleType("launch.event_handlers")
    launch_ros_module: Any = types.ModuleType("launch_ros")
    launch_ros_actions_module: Any = types.ModuleType("launch_ros.actions")
    ament_index_module: Any = types.ModuleType("ament_index_python")
    ament_index_packages_module: Any = types.ModuleType("ament_index_python.packages")

    class ExecuteProcess:
        def __init__(self, *, cmd=None, cwd=None, additional_env=None, output=None, **kwargs):
            self.cmd = cmd
            self.cwd = cwd
            self.additional_env = additional_env
            self.output = output
            self.kwargs = kwargs

    class RegisterEventHandler:
        def __init__(self, event_handler):
            self.event_handler = event_handler

    class TimerAction:
        def __init__(self, *, period, actions):
            self.period = period
            self.actions = actions

    class OnProcessStart:
        def __init__(self, *, target_action, on_start):
            self.target_action = target_action
            self.on_start = on_start

    class OnProcessExit:
        def __init__(self, *, target_action, on_exit):
            self.target_action = target_action
            self.on_exit = on_exit

    class Node:
        def __init__(self, *, package, executable, name=None, parameters=None, output=None, **kwargs):
            self.package = package
            self.executable = executable
            self.name = name
            self.parameters = parameters or []
            self.output = output
            self.kwargs = kwargs

    def get_package_share_directory(_: str) -> str:
        return "/tmp/install/share/acesim_ros2"

    launch_actions_module.ExecuteProcess = ExecuteProcess
    launch_actions_module.RegisterEventHandler = RegisterEventHandler
    launch_actions_module.TimerAction = TimerAction
    launch_event_handlers_module.OnProcessExit = OnProcessExit
    launch_event_handlers_module.OnProcessStart = OnProcessStart
    launch_ros_actions_module.Node = Node
    ament_index_packages_module.get_package_share_directory = get_package_share_directory

    launch_module.actions = launch_actions_module
    launch_module.event_handlers = launch_event_handlers_module
    launch_ros_module.actions = launch_ros_actions_module
    ament_index_module.packages = ament_index_packages_module

    sys.modules["ament_index_python"] = ament_index_module
    sys.modules["ament_index_python.packages"] = ament_index_packages_module
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions_module
    sys.modules["launch.event_handlers"] = launch_event_handlers_module
    sys.modules["launch_ros"] = launch_ros_module
    sys.modules["launch_ros.actions"] = launch_ros_actions_module

    module_path = (
        Path(__file__).resolve().parents[1]
        / "acesim"
        / "deploy"
        / "aircraft"
        / "acesim_ros2"
        / "acesim_ros2"
        / "launch_common.py"
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class _FakeConfigLoader:
    def __init__(self, asset_name: str, env_type: str = "multirotor"):
        self._asset_name = asset_name
        self._env_type = env_type

    def get_asset_name(self) -> str:
        return self._asset_name

    def get_env_type(self) -> str:
        return self._env_type

    def get_asset_params(self) -> dict[str, object]:
        return {}


class _FakePX4SensorParams:
    def __init__(self) -> None:
        self.fusion_mode = "hil"
        self.ekf2_ev_ctrl = 0
        self.ekf2_hgt_ref = "GPS"
        self.ekf2_ev_delay_ms = 0
        self.ekf2_ev_pos_body_m = (0.0, 0.0, 0.0)
        self.ekf2_ev_noise_md = 0
        self.ekf2_evp_noise = 0.0
        self.ekf2_evv_noise = 0.0
        self.ekf2_eva_noise = 0.0
        self.ekf2_gps_ctrl = 7
        self.ekf2_mag_type = 0

    @classmethod
    def from_asset_params(cls, asset_params: dict[str, object], dynamic_hil_sensor_fields: bool = False):
        return cls()


class ROS2LaunchCommonTests(unittest.TestCase):
    launch_common: ClassVar[ModuleType]

    @classmethod
    def setUpClass(cls) -> None:
        cls.launch_common = _load_launch_common_module()

    def test_resolve_px4_startup_env_supports_new_assets(self) -> None:
        cases = {
            "iris": {"PX4_SYS_AUTOSTART": "10016", "PX4_SIM_MODEL": "none"},
            "x500": {"PX4_SYS_AUTOSTART": "10016", "PX4_SIM_MODEL": "none"},
            "x500_arm2x": {"PX4_SYS_AUTOSTART": "10016", "PX4_SIM_MODEL": "none"},
            "typhoon_h480": {"PX4_SYS_AUTOSTART": "6011", "PX4_SIM_MODEL": "none"},
            "advanced_plane": {
                "PX4_SYS_AUTOSTART": "1039",
                "PX4_SIM_MODEL": "none",
                "PX4_SIMULATOR": "none",
                "PX4_PARAM_SIM_GZ_EN": "0",
            },
            "standard_vtol": {
                "PX4_SYS_AUTOSTART": "1040",
                "PX4_SIM_MODEL": "none",
                "PX4_SIMULATOR": "none",
                "PX4_PARAM_SIM_GZ_EN": "0",
            },
            "uuv_bluerov2_heavy": {
                "PX4_SYS_AUTOSTART": "60002",
                "PX4_SIM_MODEL": "none",
                "PX4_SIMULATOR": "none",
                "PX4_PARAM_SIM_GZ_EN": "0",
            },
        }

        for asset_name, expected in cases.items():
            with self.subTest(asset=asset_name):
                with patch.object(self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader(asset_name)):
                    self.assertEqual(self.launch_common.resolve_px4_startup_env(), expected)

    def test_resolve_px4_startup_env_reports_supported_assets(self) -> None:
        with patch.object(self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("unknown_vehicle")):
            with self.assertRaisesRegex(
                ValueError,
                "Unsupported PX4 startup asset: unknown_vehicle. Supported assets: "
                "advanced_plane, iris, standard_vtol, typhoon_h480, uuv_bluerov2_heavy, x500, x500_arm2x",
            ):
                self.launch_common.resolve_px4_startup_env()

    def test_build_launch_entities_uses_single_bridge_node(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        bridge_nodes = [entity for entity in entities if getattr(entity, "executable", None) == "acesim_bridge"]
        self.assertEqual(len(bridge_nodes), 1)
        self.assertEqual(bridge_nodes[0].name, "acesim_bridge")

    def test_load_bridge_entries_returns_all_configured_bridge_names(self) -> None:
        config_text = """
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
    poll_period_sec: 0.001
    transport:
      type: zmq_sub
      endpoint: tcp://127.0.0.1:5602
    topic: /arm/command
  arm_command_px4:
    enabled: true
    poll_period_sec: 0.001
    transport:
      type: zmq_sub
      endpoint: tcp://127.0.0.1:5602
    topic: /fmu/in/arm_joint_command
"""
        config_path = Path(self.id().replace(".", "_") + ".yaml")
        try:
            config_path.write_text(config_text, encoding="utf-8")
            bridges = self.launch_common.load_bridge_entries(str(config_path))
        finally:
            config_path.unlink(missing_ok=True)

        self.assertEqual(
            [bridge["name"] for bridge in bridges], ["simulation_clock", "arm_command_ros", "arm_command_px4"]
        )
        self.assertEqual(
            [bridge["endpoint"] for bridge in bridges],
            ["tcp://127.0.0.1:5600", "tcp://127.0.0.1:5602", "tcp://127.0.0.1:5602"],
        )

    def test_load_bridge_entries_rejects_invalid_tcp_endpoint(self) -> None:
        config_text = """
bridges:
  simulation_clock:
    enabled: true
    poll_period_sec: 0.001
    transport:
      type: zmq_sub
      endpoint: ipc:///tmp/acesim-clock
    topic: /acesim/clock
"""
        config_path = Path(self.id().replace(".", "_") + ".yaml")
        try:
            config_path.write_text(config_text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "tcp://host:port"):
                self.launch_common.load_bridge_entries(str(config_path))
        finally:
            config_path.unlink(missing_ok=True)

    def test_load_bridge_entries_rejects_legacy_handler_key(self) -> None:
        config_text = """
bridges:
  simulation_clock:
    enabled: true
    handler: simulation_clock
    poll_period_sec: 0.001
    transport:
      type: zmq_sub
      endpoint: tcp://127.0.0.1:5600
    topic: /acesim/clock
"""
        config_path = Path(self.id().replace(".", "_") + ".yaml")
        try:
            config_path.write_text(config_text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must not define 'handler'"):
                self.launch_common.load_bridge_entries(str(config_path))
        finally:
            config_path.unlink(missing_ok=True)

    def test_build_launch_entities_uses_default_bridge_config_path(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("x500", env_type="mc_arm")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities(
                    "/tmp/px4",
                    bridge_mode="wsl",
                    enable_px4_post_start_setup=False,
                )

        bridge_node = next(entity for entity in entities if getattr(entity, "executable", None) == "acesim_bridge")
        parameter_dict = next(item for item in bridge_node.parameters if isinstance(item, dict))
        self.assertNotIn("bridge_config_file", parameter_dict)

    def test_bridge_config_path_prefers_share_directory(self) -> None:
        with patch.object(self.launch_common, "package_share_dir", return_value=Path("/tmp/install/share/acesim_ros2")):
            path = self.launch_common.bridge_config_path()

        self.assertEqual(path, "/tmp/install/share/acesim_ros2/config/bridges.yaml")

    def test_bridge_config_path_falls_back_to_source_tree(self) -> None:
        with patch.object(self.launch_common, "package_share_dir", return_value=None):
            path = self.launch_common.bridge_config_path()

        self.assertTrue(path.endswith("acesim_ros2/config/bridges.yaml"))

    def test_build_launch_entities_injects_linux_input_endpoints(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities(
                    "/tmp/px4",
                    bridge_mode="linux",
                    enable_px4_post_start_setup=False,
                )

        bridge_node = next(entity for entity in entities if getattr(entity, "executable", None) == "acesim_bridge")
        parameter_dict = next(item for item in bridge_node.parameters if isinstance(item, dict))
        override_path = Path(parameter_dict["bridge_overrides_file"])
        overrides = yaml.safe_load(override_path.read_text(encoding="utf-8"))
        self.assertEqual(
            overrides["overrides"],
            {
                "simulation_clock": {"input_endpoint": "tcp://127.0.0.1:5600"},
                "arm_command_ros": {"input_endpoint": "tcp://127.0.0.1:5602"},
                "arm_command_px4": {"input_endpoint": "tcp://127.0.0.1:5602"},
                "arm_state": {"input_endpoint": "tcp://127.0.0.1:5603"},
            },
        )

    def test_build_launch_entities_injects_wsl_input_endpoints(self) -> None:
        with patch.object(self.launch_common, "resolve_bridge_host", return_value="172.20.32.1"):
            with patch.object(
                self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
            ):
                with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                    entities = self.launch_common.build_launch_entities(
                        "/tmp/px4",
                        bridge_mode="wsl",
                        enable_px4_post_start_setup=False,
                    )

        bridge_node = next(entity for entity in entities if getattr(entity, "executable", None) == "acesim_bridge")
        parameter_dict = next(item for item in bridge_node.parameters if isinstance(item, dict))
        override_path = Path(parameter_dict["bridge_overrides_file"])
        overrides = yaml.safe_load(override_path.read_text(encoding="utf-8"))
        self.assertEqual(
            overrides["overrides"],
            {
                "simulation_clock": {"input_endpoint": "tcp://172.20.32.1:5600"},
                "arm_command_ros": {"input_endpoint": "tcp://172.20.32.1:5602"},
                "arm_command_px4": {"input_endpoint": "tcp://172.20.32.1:5602"},
                "arm_state": {"input_endpoint": "tcp://172.20.32.1:5603"},
            },
        )

    def test_build_launch_entities_passes_only_parameter_dict(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        bridge_node = next(entity for entity in entities if getattr(entity, "executable", None) == "acesim_bridge")
        self.assertTrue(all(isinstance(item, dict) for item in bridge_node.parameters))

    def test_build_launch_entities_injects_override_file_path(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        bridge_node = next(entity for entity in entities if getattr(entity, "executable", None) == "acesim_bridge")
        parameter_dict = next(item for item in bridge_node.parameters if isinstance(item, dict))
        self.assertIn("bridge_overrides_file", parameter_dict)
        self.assertTrue(parameter_dict["bridge_overrides_file"].endswith(".yaml"))

    def test_launch_common_uses_non_private_helper_names(self) -> None:
        self.assertTrue(hasattr(self.launch_common, "resolve_px4_startup_env"))
        self.assertTrue(hasattr(self.launch_common, "package_share_dir"))
        self.assertTrue(hasattr(self.launch_common, "bridge_config_path"))
        self.assertTrue(hasattr(self.launch_common, "build_px4_post_start_command"))
        self.assertFalse(hasattr(self.launch_common, "_resolve_px4_startup_env"))
        self.assertFalse(hasattr(self.launch_common, "_package_share_dir"))
        self.assertFalse(hasattr(self.launch_common, "_bridge_config_path"))
        self.assertFalse(hasattr(self.launch_common, "_bridge_node_params_path"))
        self.assertFalse(hasattr(self.launch_common, "_build_px4_post_start_command"))
        self.assertFalse(hasattr(self.launch_common, "write_bridge_overrides_file"))

    def test_build_launch_entities_force_export_px4_non_gz_overrides(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        px4_process = next(entity for entity in entities if getattr(entity, "cwd", None) == "/tmp/px4")
        command_text = " ".join(px4_process.cmd)
        self.assertIn("PX4_SIM_MODEL=none", command_text)
        self.assertIn("PX4_SIMULATOR=none", command_text)
        self.assertIn("PX4_PARAM_SIM_GZ_EN=0", command_text)
        self.assertIn("make px4_sitl none", command_text)


if __name__ == "__main__":
    unittest.main()
