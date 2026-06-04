from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "acesim" / "deploy" / "aircraft" / "acesim_ros2" / "launch"


def _load_launch_module(filename: str) -> Any:
    module_name = f"_test_acesim_ros2_{filename.replace('.', '_')}"
    module_names = [
        module_name,
        "acesim_ros2",
        "acesim_ros2.launch_common",
        "launch",
        "launch.actions",
        "launch.substitutions",
    ]
    previous_modules = {name: sys.modules.get(name) for name in module_names}
    for name in module_names:
        sys.modules.pop(name, None)

    acesim_ros2_module: Any = types.ModuleType("acesim_ros2")
    launch_common_module: Any = types.ModuleType("acesim_ros2.launch_common")
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

    class OpaqueFunction:
        def __init__(self, *, function: object) -> None:
            self.function = function

    class LaunchConfiguration:
        def __init__(self, name: str) -> None:
            self.name = name

        def perform(self, context: dict[str, str]) -> str:
            return context.get(self.name, "")

    def load_px4_repo_path(override: str) -> str:
        return override or "/tmp/px4"

    def build_launch_entities(px4_repo_path: str, **kwargs: object) -> list[dict[str, object]]:
        return [{"kind": "legacy_launch", "px4_repo_path": px4_repo_path, **kwargs}]

    launch_common_module.load_px4_repo_path = load_px4_repo_path
    launch_common_module.build_launch_entities = build_launch_entities
    acesim_ros2_module.launch_common = launch_common_module
    launch_module.LaunchDescription = LaunchDescription
    launch_actions_module.DeclareLaunchArgument = DeclareLaunchArgument
    launch_actions_module.OpaqueFunction = OpaqueFunction
    launch_substitutions_module.LaunchConfiguration = LaunchConfiguration

    sys.modules["acesim_ros2"] = acesim_ros2_module
    sys.modules["acesim_ros2.launch_common"] = launch_common_module
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions_module
    sys.modules["launch.substitutions"] = launch_substitutions_module

    launch_path = LAUNCH_DIR / filename
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load launch module from {launch_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        for name, previous in previous_modules.items():
            if name == module_name:
                continue
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
    return module


def test_linux_launch_defaults_to_legacy_gui_stack() -> None:
    module = _load_launch_module("linux.launch.py")

    description = module.generate_launch_description()
    declared = {action.name: action.default_value for action in description.actions if hasattr(action, "name")}
    action = module._launch_setup({"px4_repo": "/tmp/px4"})[0]

    assert "gui" not in declared
    assert "readiness" not in declared
    assert action["kind"] == "legacy_launch"
    assert action["px4_repo_path"] == "/tmp/px4"
    assert action["bridge_mode"] == "linux"
    assert action["play_executable"] == "acesim_play"
    assert action["enable_px4_post_start_setup"] is True


def test_linux_headless_launch_defaults_to_legacy_headless_stack() -> None:
    module = _load_launch_module("linux_headless.launch.py")

    action = module._launch_setup({"px4_repo": "/tmp/px4"})[0]

    assert action["kind"] == "legacy_launch"
    assert action["bridge_mode"] == "linux"
    assert action["play_executable"] == "acesim_play_headless"
    assert action["enable_px4_post_start_setup"] is True


def test_wsl_launch_defaults_to_bridge_only_legacy_stack() -> None:
    module = _load_launch_module("wsl.launch.py")

    action = module._launch_setup({"px4_repo": "/tmp/px4"})[0]

    assert action["kind"] == "legacy_launch"
    assert action["bridge_mode"] == "wsl"
    assert action["play_executable"] is None
    assert action["enable_px4_post_start_setup"] is False
