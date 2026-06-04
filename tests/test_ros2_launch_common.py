from __future__ import annotations

import importlib.util
import os
import shlex
import subprocess
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
        "launch.events",
        "launch_ros",
        "launch_ros.actions",
        "rclpy",
        "rclpy.qos",
        "rosgraph_msgs",
        "rosgraph_msgs.msg",
        "px4_msgs",
        "px4_msgs.msg",
    ]:
        sys.modules.pop(name, None)

    launch_module: Any = types.ModuleType("launch")
    launch_actions_module: Any = types.ModuleType("launch.actions")
    launch_event_handlers_module: Any = types.ModuleType("launch.event_handlers")
    launch_events_module: Any = types.ModuleType("launch.events")
    launch_ros_module: Any = types.ModuleType("launch_ros")
    launch_ros_actions_module: Any = types.ModuleType("launch_ros.actions")
    ament_index_module: Any = types.ModuleType("ament_index_python")
    ament_index_packages_module: Any = types.ModuleType("ament_index_python.packages")
    rclpy_module: Any = types.ModuleType("rclpy")
    rclpy_qos_module: Any = types.ModuleType("rclpy.qos")
    rosgraph_msgs_module: Any = types.ModuleType("rosgraph_msgs")
    rosgraph_msgs_msg_module: Any = types.ModuleType("rosgraph_msgs.msg")
    px4_msgs_module: Any = types.ModuleType("px4_msgs")
    px4_msgs_msg_module: Any = types.ModuleType("px4_msgs.msg")

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

    class EmitEvent:
        def __init__(self, *, event):
            self.event = event

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

    class Shutdown:
        def __init__(self, *, reason=None):
            self.reason = reason

    class QoSProfile:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    class _Enum:
        KEEP_LAST = "keep_last"
        BEST_EFFORT = "best_effort"
        VOLATILE = "volatile"
        RELIABLE = "reliable"

    class _FakeStamp:
        def __init__(self) -> None:
            self.sec = 0
            self.nanosec = 0

    class Clock:
        def __init__(self) -> None:
            self.clock = _FakeStamp()

    class ArmJointState:
        def __init__(self) -> None:
            self.timestamp = 0
            self.arm_position: list[float] = []
            self.arm_velocity: list[float] = []

    def get_package_share_directory(_: str) -> str:
        return "/tmp/install/share/acesim_ros2"

    launch_actions_module.EmitEvent = EmitEvent
    launch_actions_module.ExecuteProcess = ExecuteProcess
    launch_actions_module.RegisterEventHandler = RegisterEventHandler
    launch_actions_module.TimerAction = TimerAction
    launch_events_module.Shutdown = Shutdown
    launch_event_handlers_module.OnProcessExit = OnProcessExit
    launch_event_handlers_module.OnProcessStart = OnProcessStart
    launch_ros_actions_module.Node = Node
    ament_index_packages_module.get_package_share_directory = get_package_share_directory
    rclpy_qos_module.QoSProfile = QoSProfile
    rclpy_qos_module.HistoryPolicy = _Enum
    rclpy_qos_module.ReliabilityPolicy = _Enum
    rclpy_qos_module.DurabilityPolicy = _Enum
    rosgraph_msgs_msg_module.Clock = Clock
    rosgraph_msgs_module.msg = rosgraph_msgs_msg_module
    px4_msgs_msg_module.ArmJointState = ArmJointState
    px4_msgs_module.msg = px4_msgs_msg_module

    launch_module.actions = launch_actions_module
    launch_module.event_handlers = launch_event_handlers_module
    launch_module.events = launch_events_module
    launch_ros_module.actions = launch_ros_actions_module
    ament_index_module.packages = ament_index_packages_module

    sys.modules["ament_index_python"] = ament_index_module
    sys.modules["ament_index_python.packages"] = ament_index_packages_module
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions_module
    sys.modules["launch.event_handlers"] = launch_event_handlers_module
    sys.modules["launch.events"] = launch_events_module
    sys.modules["launch_ros"] = launch_ros_module
    sys.modules["launch_ros.actions"] = launch_ros_actions_module
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.qos"] = rclpy_qos_module
    sys.modules["rosgraph_msgs"] = rosgraph_msgs_module
    sys.modules["rosgraph_msgs.msg"] = rosgraph_msgs_msg_module
    sys.modules["px4_msgs"] = px4_msgs_module
    sys.modules["px4_msgs.msg"] = px4_msgs_msg_module

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
        self.gps_home_lat_lon = (39.98329, 116.34745)
        self.gps_alt_start = 50.0
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

    def _play_action(self, entities: list[object], executable: str) -> Any:
        for entity in entities:
            if getattr(entity, "executable", None) == executable:
                return entity
            handler = getattr(entity, "event_handler", None)
            actions = getattr(handler, "on_start", []) or []
            for action in actions:
                nested_actions = getattr(action, "actions", [action])
                for nested in nested_actions:
                    if getattr(nested, "executable", None) == executable:
                        return nested
        raise AssertionError(f"play action not found: {executable}")

    def _play_process(self, entities: list[object], executable: str) -> Any:
        needle = f"ros2 run acesim_ros2 {executable}"
        for entity in entities:
            if needle in " ".join(getattr(entity, "cmd", [])):
                return entity
            handler = getattr(entity, "event_handler", None)
            actions = getattr(handler, "on_start", []) or []
            for action in actions:
                nested_actions = getattr(action, "actions", [action])
                for nested in nested_actions:
                    if needle in " ".join(getattr(nested, "cmd", [])):
                        return nested
        raise AssertionError(f"play process not found: {executable}")

    @classmethod
    def setUpClass(cls) -> None:
        cls.launch_common = _load_launch_common_module()

    def _bridge_process(self, entities: list[object]) -> Any:
        return next(
            entity
            for entity in entities
            if "ros2 run acesim_ros2 acesim_bridge" in " ".join(getattr(entity, "cmd", []))
        )

    def _bridge_overrides_file(self, entities: list[object]) -> Path:
        bridge_process = self._bridge_process(entities)
        for token in shlex.split(bridge_process.cmd[4]):
            if token.startswith("bridge_overrides_file:="):
                return Path(token.split(":=", 1)[1])
        self.fail("bridge_overrides_file ros arg was not found")

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

    def test_build_launch_entities_uses_single_bridge_process(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        bridge_processes = [
            entity
            for entity in entities
            if "ros2 run acesim_ros2 acesim_bridge" in " ".join(getattr(entity, "cmd", []))
        ]
        self.assertEqual(len(bridge_processes), 1)
        self.assertIn("__node:=acesim_bridge", bridge_processes[0].cmd[4])

    def test_build_px4_additional_env_keeps_required_launch_overrides(self) -> None:
        with patch.object(self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("x500_arm2x")):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                additional_env = self.launch_common.build_px4_additional_env()

        self.assertEqual(additional_env["PX4_PARAM_COM_MODE_ARM_CHK"], "1")
        self.assertEqual(additional_env["PX4_PARAM_SIM_BAT_ENABLE"], "1")
        self.assertEqual(additional_env["PX4_PARAM_CBRK_SUPPLY_CHK"], "894281")
        self.assertNotIn("PX4_PARAM_SENS_IMU_MODE", additional_env)
        self.assertNotIn("PX4_PARAM_EKF2_MULTI_IMU", additional_env)
        self.assertNotIn("PX4_PARAM_COM_ARM_WO_GPS", additional_env)
        self.assertNotIn("PX4_PARAM_AM_POS_MANL_CTRL", additional_env)

    def test_build_px4_additional_env_can_use_explicit_config_loader(self) -> None:
        with patch.object(self.launch_common, "ConfigLoader", side_effect=AssertionError("unexpected default config")):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                additional_env = self.launch_common.build_px4_additional_env(_FakeConfigLoader("x500_arm2x"))

        self.assertEqual(additional_env["PX4_SYS_AUTOSTART"], "10016")
        self.assertEqual(additional_env["PX4_PARAM_COM_MODE_ARM_CHK"], "1")

    def test_build_px4_additional_env_does_not_override_am_offboard_mode_in_mocap_mode(self) -> None:
        mocap_params = _FakePX4SensorParams()
        mocap_params.fusion_mode = "mocap"

        class _FakeMocapPX4SensorParams:
            @classmethod
            def from_asset_params(cls, asset_params: dict[str, object], dynamic_hil_sensor_fields: bool = False):
                return mocap_params

        with patch.object(self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("x500_arm2x")):
            with patch.object(self.launch_common, "PX4SensorParams", _FakeMocapPX4SensorParams):
                additional_env = self.launch_common.build_px4_additional_env()

        self.assertNotIn("PX4_PARAM_AMPC_OFFB_EN", additional_env)

    def test_build_px4_additional_env_uses_configured_mag_type_for_mocap_mode(self) -> None:
        mocap_params = _FakePX4SensorParams()
        mocap_params.fusion_mode = "mocap"
        mocap_params.ekf2_ev_ctrl = 11
        mocap_params.ekf2_hgt_ref = "Vision"
        mocap_params.ekf2_gps_ctrl = 0
        mocap_params.ekf2_mag_type = 0

        class _FakeMocapPX4SensorParams:
            @classmethod
            def from_asset_params(cls, asset_params: dict[str, object], dynamic_hil_sensor_fields: bool = False):
                return mocap_params

        with patch.object(self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("x500_arm2x")):
            with patch.object(self.launch_common, "PX4SensorParams", _FakeMocapPX4SensorParams):
                additional_env = self.launch_common.build_px4_additional_env()

        self.assertEqual(additional_env["PX4_PARAM_EKF2_EV_CTRL"], "11")
        self.assertEqual(additional_env["PX4_PARAM_EKF2_MAG_TYPE"], "0")
        self.assertEqual(additional_env["PX4_PARAM_SYS_HAS_MAG"], "0")

    def test_build_px4_post_start_command_uses_module_entrypoint(self) -> None:
        mocap_params = _FakePX4SensorParams()
        mocap_params.fusion_mode = "mocap"

        command = self.launch_common.build_px4_post_start_command(mocap_params)

        self.assertEqual(command[:3], [sys.executable, "-m", "acesim_ros2.px4_post_start_setup"])
        self.assertEqual(command[3:], ["mocap", "39.98329", "116.34745", "50.0"])

    def test_build_px4_post_start_command_does_not_embed_shell_listener_polling(self) -> None:
        mocap_params = _FakePX4SensorParams()
        mocap_params.fusion_mode = "mocap"

        command = self.launch_common.build_px4_post_start_command(mocap_params)
        command_text = " ".join(command)

        self.assertNotIn("-c", command)
        self.assertNotIn("listener estimator_status", command_text)
        self.assertNotIn("listener vehicle_local_position", command_text)
        self.assertNotIn("listener vehicle_global_position", command_text)
        self.assertNotIn("traceback.print_exc", command_text)

    def test_build_px4_post_start_command_uses_zero_origin_for_hil_mode(self) -> None:
        hil_params = _FakePX4SensorParams()
        hil_params.fusion_mode = "hil"

        command = self.launch_common.build_px4_post_start_command(hil_params)

        self.assertEqual(command[3:], ["hil", "0.0", "0.0", "0.0"])

    def test_graceful_shutdown_command_exits_zero_only_after_signal(self) -> None:
        command = self.launch_common.build_graceful_shutdown_command("MicroXRCEAgent udp4 -p 8888")
        script = command[2]

        self.assertEqual(command[:2], ["bash", "-lc"])
        self.assertEqual(command[4], "MicroXRCEAgent udp4 -p 8888")
        self.assertIn("trap _forward_sigint INT", script)
        self.assertIn("trap _forward_sigterm TERM", script)
        self.assertIn("exit 0", script)
        self.assertIn('exit "$_status"', script)

    def test_graceful_shutdown_command_cleans_child_process_group(self) -> None:
        command = self.launch_common.build_graceful_shutdown_command("make px4_sitl none")
        script = command[2]

        self.assertIn('setsid bash -lc "$1" &', script)
        self.assertIn('kill -INT -- "-$_child_pid"', script)
        self.assertIn('kill -TERM -- "-$_child_pid"', script)
        self.assertIn('kill -KILL -- "-$_child_pid"', script)
        self.assertIn('kill -0 -- "-$_child_pid"', script)
        self.assertIn("_cleanup_after_signal", script)

    def test_graceful_shutdown_command_can_filter_px4_prompt_spam(self) -> None:
        command = self.launch_common.build_graceful_shutdown_command("make px4_sitl none", filter_px4_prompt=True)
        script = command[2]

        self.assertEqual(command[:2], ["bash", "-lc"])
        self.assertEqual(command[4], "make px4_sitl none")
        self.assertIn("mkfifo", script)
        self.assertIn("pxh>", script)
        self.assertIn("signal.SIG_IGN", script)
        self.assertIn('> "$_filter_pipe" 2>&1', script)
        self.assertIn('exit "$_status"', script)

    def test_graceful_shutdown_command_filters_prompt_bytes_without_dropping_logs(self) -> None:
        script = (
            "import sys; "
            "prompt = b'\\x1b[2K\\rpxh> '; "
            "sys.stdout.buffer.write(prompt * 1000 + b'INFO real log\\n' + prompt + "
            "b'WARN  [health_and_arming_checks] Preflight Fail: heading estimate not stable\\n' + "
            "b'pxh> PX4 Exiting...\\n'); "
            "sys.stdout.flush()"
        )
        command = self.launch_common.build_graceful_shutdown_command(
            "python3 -c " + shlex.quote(script),
            filter_px4_prompt=True,
        )

        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5, check=False)

        self.assertEqual(result.returncode, 0)
        self.assertEqual(
            result.stdout,
            b"INFO real log\n"
            b"WARN  [health_and_arming_checks] Preflight Fail: heading estimate not stable\n"
            b"PX4 Exiting...\n",
        )

    def test_build_launch_entities_wraps_long_running_external_processes(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        micro_agent = entities[0]
        px4_process = next(entity for entity in entities if getattr(entity, "cwd", None) == "/tmp/px4")

        self.assertEqual(micro_agent.cmd[:2], ["bash", "-lc"])
        self.assertEqual(micro_agent.cmd[4], "MicroXRCEAgent udp4 -p 8888")
        self.assertEqual(px4_process.cmd[:2], ["bash", "-lc"])
        self.assertIn("setsid bash -lc", px4_process.cmd[2])
        self.assertIn("kill -TERM --", px4_process.cmd[2])
        self.assertIn("kill -KILL --", px4_process.cmd[2])
        self.assertIn("pxh>", px4_process.cmd[2])
        self.assertIn("make px4_sitl none", px4_process.cmd[4])
        self.assertIn('exit "$_status"', px4_process.cmd[2])

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
  arm_state:
    enabled: true
    poll_period_sec: 0.001
    transport:
      type: zmq_sub
      endpoint: tcp://127.0.0.1:5603
    topic: /fmu/in/arm_joint_state
"""
        config_path = Path(self.id().replace(".", "_") + ".yaml")
        try:
            config_path.write_text(config_text, encoding="utf-8")
            bridges = self.launch_common.load_bridge_entries(str(config_path))
        finally:
            config_path.unlink(missing_ok=True)

        self.assertEqual([bridge["name"] for bridge in bridges], ["simulation_clock", "arm_state"])
        self.assertEqual(
            [bridge["endpoint"] for bridge in bridges],
            ["tcp://127.0.0.1:5600", "tcp://127.0.0.1:5603"],
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
        with patch.object(self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("x500", env_type="am")):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities(
                    "/tmp/px4",
                    bridge_mode="wsl",
                    enable_px4_post_start_setup=False,
                )

        bridge_process = self._bridge_process(entities)
        self.assertNotIn("bridge_config_file", bridge_process.cmd[4])

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

        override_path = self._bridge_overrides_file(entities)
        overrides = yaml.safe_load(override_path.read_text(encoding="utf-8"))
        self.assertEqual(
            overrides["overrides"],
            {
                "simulation_clock": {"input_endpoint": "tcp://127.0.0.1:5600"},
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

        override_path = self._bridge_overrides_file(entities)
        overrides = yaml.safe_load(override_path.read_text(encoding="utf-8"))
        self.assertEqual(
            overrides["overrides"],
            {
                "simulation_clock": {"input_endpoint": "tcp://172.20.32.1:5600"},
                "arm_state": {"input_endpoint": "tcp://172.20.32.1:5603"},
            },
        )

    def test_build_launch_entities_passes_bridge_overrides_as_ros_arg(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        bridge_process = self._bridge_process(entities)
        self.assertIn("--ros-args", bridge_process.cmd[4])
        self.assertIn("-p", bridge_process.cmd[4])
        self.assertIn("bridge_overrides_file:=", bridge_process.cmd[4])

    def test_build_launch_entities_injects_override_file_path(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        override_path = self._bridge_overrides_file(entities)
        self.assertTrue(str(override_path).endswith(".yaml"))

    def test_build_launch_entities_wraps_bridge_shutdown(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", enable_px4_post_start_setup=False)

        bridge_process = self._bridge_process(entities)
        self.assertEqual(bridge_process.cmd[:2], ["bash", "-lc"])
        self.assertIn('exit "$_status"', bridge_process.cmd[2])
        self.assertTrue(bridge_process.kwargs["emulate_tty"])
        self.assertIn("PYTHONUNBUFFERED=1", bridge_process.cmd[4])

    def test_build_launch_entities_sets_unbuffered_python_output_for_play_node(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4")

        play_node = self._play_action(entities, "acesim_play")
        self.assertEqual(play_node.executable, "acesim_play")
        self.assertTrue(play_node.kwargs["emulate_tty"])
        self.assertEqual(play_node.kwargs["additional_env"]["PYTHONUNBUFFERED"], "1")

    def test_build_launch_entities_wraps_headless_play_shutdown(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4", play_executable="acesim_play_headless")

        play_process = self._play_process(entities, "acesim_play_headless")

        self.assertEqual(play_process.cmd[:2], ["bash", "-lc"])
        self.assertIn("ros2 run acesim_ros2 acesim_play_headless", play_process.cmd[4])
        self.assertIn('exit "$_status"', play_process.cmd[2])
        self.assertIn("PYTHONUNBUFFERED=1", play_process.cmd[4])

    def test_build_launch_entities_shutdowns_when_mujoco_frontend_exits(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4")

        play_node = self._play_action(entities, "acesim_play")
        exit_handler: Any = next(
            entity
            for entity in entities
            if getattr(getattr(entity, "event_handler", None), "on_exit", None) is not None
        )

        self.assertIs(exit_handler.event_handler.target_action, play_node)
        self.assertEqual(exit_handler.event_handler.on_exit[0].event.reason, "ACESim frontend exited")

    def test_build_launch_entities_starts_frontend_after_post_start_setup(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4")

        post_start_process: Any = next(
            entity for entity in entities if "acesim_ros2.px4_post_start_setup" in " ".join(getattr(entity, "cmd", []))
        )
        start_handler: Any = next(
            entity
            for entity in entities
            if getattr(getattr(entity, "event_handler", None), "on_start", None) is not None
            and entity.event_handler.target_action is post_start_process
        )
        timer = start_handler.event_handler.on_start[0]
        play_node = timer.actions[0]

        self.assertIn("acesim_ros2.px4_post_start_setup", post_start_process.cmd)
        self.assertEqual(post_start_process.output, "both")
        self.assertEqual(post_start_process.additional_env["ACESIM_PX4_VERIFY_ARMABLE"], "1")
        self.assertEqual(play_node.executable, "acesim_play")
        self.assertEqual(timer.period, 2.0)

    def test_build_launch_entities_does_not_add_frontend_shutdown_without_play_node(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities(
                    "/tmp/px4",
                    play_executable=None,
                    enable_px4_post_start_setup=False,
                )

        exit_handlers = [
            entity
            for entity in entities
            if getattr(getattr(entity, "event_handler", None), "on_exit", None) is not None
        ]
        self.assertEqual(exit_handlers, [])

    def test_build_launch_entities_merges_additional_play_env(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities(
                    "/tmp/px4",
                    play_executable="acesim_play",
                    additional_play_env={"ACESIM_EXTRA_ENV": "1"},
                )

        play_node = self._play_action(entities, "acesim_play")
        self.assertEqual(play_node.executable, "acesim_play")
        self.assertEqual(play_node.kwargs["additional_env"]["PYTHONUNBUFFERED"], "1")
        self.assertEqual(play_node.kwargs["additional_env"]["ACESIM_EXTRA_ENV"], "1")

    def test_build_px4_post_start_setup_process_sets_unbuffered_python_output(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                process = self.launch_common.build_px4_post_start_setup_process()

        self.assertTrue(process.kwargs["emulate_tty"])
        self.assertEqual(process.output, "both")
        self.assertEqual(process.additional_env["PYTHONUNBUFFERED"], "1")
        self.assertNotIn("ACESIM_PX4_VERIFY_ARMABLE", process.additional_env)

    def test_build_launch_entities_enables_default_post_start_armability_verification(self) -> None:
        with patch.object(
            self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
        ):
            with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                entities = self.launch_common.build_launch_entities("/tmp/px4")

        post_start_process = next(
            entity for entity in entities if "acesim_ros2.px4_post_start_setup" in " ".join(getattr(entity, "cmd", []))
        )

        self.assertEqual(post_start_process.additional_env["ACESIM_PX4_VERIFY_ARMABLE"], "1")

    def test_build_launch_entities_preserves_post_start_armability_opt_out(self) -> None:
        with patch.dict(os.environ, {"ACESIM_PX4_VERIFY_ARMABLE": "0"}):
            with patch.object(
                self.launch_common, "ConfigLoader", return_value=_FakeConfigLoader("advanced_plane", env_type="fw")
            ):
                with patch.object(self.launch_common, "PX4SensorParams", _FakePX4SensorParams):
                    entities = self.launch_common.build_launch_entities("/tmp/px4")

        post_start_process = next(
            entity for entity in entities if "acesim_ros2.px4_post_start_setup" in " ".join(getattr(entity, "cmd", []))
        )

        self.assertEqual(post_start_process.additional_env["ACESIM_PX4_VERIFY_ARMABLE"], "0")

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

    def test_detect_acesim_root_preserves_ros2_package_root_contract(self) -> None:
        root = self.launch_common.detect_acesim_root()

        self.assertEqual(root.name, "acesim")
        self.assertTrue((root / "config").is_dir())
        self.assertTrue((root / "deploy" / "aircraft" / "acesim_ros2").is_dir())
        self.assertFalse(hasattr(self.launch_common, "build_core_sitl_action"))


if __name__ == "__main__":
    unittest.main()
