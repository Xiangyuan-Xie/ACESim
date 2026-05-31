from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from typing import Any, cast

import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = ROOT / "acesim" / "deploy" / "aircraft" / "acesim_ros2" / "launch" / "x500_arm2x_benchmark.launch.py"


def _load_launch_module() -> Any:
    module_name = "_test_x500_arm2x_benchmark_launch"
    for name in [
        module_name,
        "ament_index_python",
        "ament_index_python.packages",
        "launch",
        "launch.actions",
        "launch.substitutions",
    ]:
        sys.modules.pop(name, None)

    ament_index_module: Any = types.ModuleType("ament_index_python")
    ament_index_packages_module: Any = types.ModuleType("ament_index_python.packages")
    launch_module: Any = types.ModuleType("launch")
    launch_actions_module: Any = types.ModuleType("launch.actions")
    launch_substitutions_module: Any = types.ModuleType("launch.substitutions")

    class LaunchDescription:
        def __init__(self, actions: list[object]) -> None:
            self.actions = actions

    class DeclareLaunchArgument:
        def __init__(self, name: str, *, default_value: str = "", description: str = "") -> None:
            self.name = name
            self.default_value = default_value
            self.description = description

    class ExecuteProcess:
        def __init__(self, *, cmd: list[str], output: str | None = None, **kwargs: object) -> None:
            self.cmd = cmd
            self.output = output
            self.kwargs = kwargs

    class OpaqueFunction:
        def __init__(self, *, function: object) -> None:
            self.function = function

    class LaunchConfiguration:
        def __init__(self, name: str) -> None:
            self.name = name

        def perform(self, context: dict[str, str]) -> str:
            return context.get(self.name, "")

    def get_package_share_directory(_package_name: str) -> str:
        return str(ROOT / "acesim" / "deploy" / "aircraft" / "acesim_ros2")

    ament_index_packages_module.get_package_share_directory = get_package_share_directory
    ament_index_module.packages = ament_index_packages_module
    launch_module.LaunchDescription = LaunchDescription
    launch_actions_module.DeclareLaunchArgument = DeclareLaunchArgument
    launch_actions_module.ExecuteProcess = ExecuteProcess
    launch_actions_module.OpaqueFunction = OpaqueFunction
    launch_substitutions_module.LaunchConfiguration = LaunchConfiguration

    sys.modules["ament_index_python"] = ament_index_module
    sys.modules["ament_index_python.packages"] = ament_index_packages_module
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions_module
    sys.modules["launch.substitutions"] = launch_substitutions_module

    spec = importlib.util.spec_from_file_location(module_name, LAUNCH_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load launch module from {LAUNCH_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _execute_process_from_setup(module: Any, context: dict[str, str]) -> Any:
    actions = module._launch_setup(context)
    assert len(actions) == 1
    return cast(Any, actions[0])


def test_benchmark_launch_declares_expected_arguments() -> None:
    module = _load_launch_module()

    description = module.generate_launch_description()
    declared = [action.name for action in description.actions if hasattr(action, "name")]

    assert declared == [
        "config",
        "px4_repo",
        "case",
        "jobs",
        "output",
        "json_output",
        "raw_output_dir",
        "log_dir",
        "bridge_mode",
        "profile_cycles",
        "verbose_process_logs",
        "strict_exit_code",
        "port_base",
        "xrce_port_base",
        "ros_domain_base",
        "px4_instance_base",
        "startup_timeout_s",
        "takeoff_timeout_s",
        "arm_motion_duration_s",
        "post_arm_motion_settle_s",
    ]


def test_benchmark_launch_uses_yaml_defaults() -> None:
    module = _load_launch_module()

    process = _execute_process_from_setup(module, {})
    command = process.cmd

    assert command[:4] == ["ros2", "run", "acesim_ros2", "x500_arm2x_benchmark"]
    assert "--jobs" in command
    assert command[command.index("--jobs") + 1] == "8"
    assert "--output" in command
    assert command[command.index("--output") + 1].endswith("log/x500_arm2x_benchmark/summary.png")
    assert not command[command.index("--output") + 1].startswith("~")
    assert "--json-output" in command
    assert command[command.index("--json-output") + 1].endswith("log/x500_arm2x_benchmark/summary.json")
    assert not command[command.index("--json-output") + 1].startswith("~")
    assert "--raw-output-dir" in command
    assert command[command.index("--raw-output-dir") + 1].endswith("log/x500_arm2x_benchmark/raw")
    assert not command[command.index("--raw-output-dir") + 1].startswith("~")
    assert "--arm-motion-duration-s" in command
    assert command[command.index("--arm-motion-duration-s") + 1] == "10.0"
    assert "--real-time-rate" not in command


def test_benchmark_launch_expands_yaml_cases(tmp_path: Path) -> None:
    module = _load_launch_module()
    config_path = tmp_path / "benchmark.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "cases": ["home_folded", "left_high"],
                "jobs": 1,
                "output": str(tmp_path / "summary.png"),
            }
        ),
        encoding="utf-8",
    )

    process = _execute_process_from_setup(module, {"config": str(config_path)})
    command = process.cmd

    assert command.count("--case") == 2
    assert command[command.index("--case") + 1] == "home_folded"
    second_case_index = command.index("--case", command.index("--case") + 1)
    assert command[second_case_index + 1] == "left_high"


def test_benchmark_launch_argument_case_overrides_yaml_cases(tmp_path: Path) -> None:
    module = _load_launch_module()
    config_path = tmp_path / "benchmark.yaml"
    config_path.write_text(
        yaml.safe_dump({"cases": ["home_folded", "left_high"], "jobs": 1}),
        encoding="utf-8",
    )

    process = _execute_process_from_setup(module, {"config": str(config_path), "case": "right_high"})
    command = process.cmd

    assert command.count("--case") == 1
    assert command[command.index("--case") + 1] == "right_high"


def test_benchmark_launch_arguments_override_yaml_values(tmp_path: Path) -> None:
    module = _load_launch_module()
    config_path = tmp_path / "benchmark.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "jobs": 1,
                "output": "/tmp/from_yaml.png",
                "json_output": "/tmp/from_yaml.json",
                "raw_output_dir": "/tmp/from_yaml_raw",
                "log_dir": "/tmp/from_yaml_logs",
                "px4_repo": "/tmp/from_yaml_px4",
                "port_base": 5600,
                "arm_motion_duration_s": 6.0,
                "verbose_process_logs": False,
                "strict_exit_code": False,
            }
        ),
        encoding="utf-8",
    )

    process = _execute_process_from_setup(
        module,
        {
            "config": str(config_path),
            "jobs": "2",
            "output": "/tmp/from_launch.png",
            "json_output": "/tmp/from_launch.json",
            "raw_output_dir": "/tmp/from_launch_raw",
            "log_dir": "/tmp/from_launch_logs",
            "px4_repo": "/tmp/from_launch_px4",
            "port_base": "5700",
            "arm_motion_duration_s": "12.5",
            "verbose_process_logs": "true",
            "strict_exit_code": "true",
        },
    )
    command = process.cmd

    assert command[command.index("--jobs") + 1] == "2"
    assert command[command.index("--output") + 1] == "/tmp/from_launch.png"
    assert command[command.index("--json-output") + 1] == "/tmp/from_launch.json"
    assert command[command.index("--raw-output-dir") + 1] == "/tmp/from_launch_raw"
    assert command[command.index("--log-dir") + 1] == "/tmp/from_launch_logs"
    assert command[command.index("--px4-repo") + 1] == "/tmp/from_launch_px4"
    assert command[command.index("--port-base") + 1] == "5700"
    assert command[command.index("--arm-motion-duration-s") + 1] == "12.5"
    assert "--verbose-process-logs" in command
    assert "--strict-exit-code" in command


def test_benchmark_launch_uses_yaml_arm_motion_duration_when_not_overridden(tmp_path: Path) -> None:
    module = _load_launch_module()
    config_path = tmp_path / "benchmark.yaml"
    config_path.write_text(
        yaml.safe_dump({"jobs": 1, "arm_motion_duration_s": 7.5}),
        encoding="utf-8",
    )

    process = _execute_process_from_setup(module, {"config": str(config_path)})
    command = process.cmd

    assert command[command.index("--arm-motion-duration-s") + 1] == "7.5"
