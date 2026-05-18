from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import ModuleType
from typing import Any


def _load_acesim_play_headless_module() -> ModuleType:
    module_name = "_test_acesim_ros2_acesim_play_headless"
    for name in [
        module_name,
        "acesim",
        "acesim.core",
        "acesim.core.play",
        "acesim.env",
        "acesim.env.base_env",
    ]:
        sys.modules.pop(name, None)

    acesim_module = types.ModuleType("acesim")
    acesim_core_module = types.ModuleType("acesim.core")
    acesim_core_play_module = types.ModuleType("acesim.core.play")
    acesim_env_module = types.ModuleType("acesim.env")
    acesim_env_base_env_module = types.ModuleType("acesim.env.base_env")

    class BaseEnv:
        pass

    def make_env() -> object:
        raise RuntimeError("make_env should be patched in tests")

    setattr(acesim_core_play_module, "make_env", make_env)
    setattr(acesim_env_base_env_module, "BaseEnv", BaseEnv)
    sys.modules["acesim"] = acesim_module
    sys.modules["acesim.core"] = acesim_core_module
    sys.modules["acesim.core.play"] = acesim_core_play_module
    sys.modules["acesim.env"] = acesim_env_module
    sys.modules["acesim.env.base_env"] = acesim_env_base_env_module

    module_path = (
        Path(__file__).resolve().parents[1]
        / "acesim"
        / "deploy"
        / "aircraft"
        / "acesim_ros2"
        / "acesim_ros2"
        / "acesim_play_headless.py"
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class AcesimPlayHeadlessTests(unittest.TestCase):
    module: Any

    def setUp(self) -> None:
        self.module = _load_acesim_play_headless_module()

    def test_main_returns_zero_on_keyboard_interrupt_and_closes_env(self) -> None:
        calls: list[str] = []

        class FakeEnv:
            def step(self) -> None:
                calls.append("step")
                raise KeyboardInterrupt

            def close(self) -> None:
                calls.append("close")

        self.module.make_env = lambda: FakeEnv()

        exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])

    def test_main_returns_zero_if_shutdown_interrupts_close(self) -> None:
        calls: list[str] = []

        class FakeEnv:
            def step(self) -> None:
                calls.append("step")
                raise KeyboardInterrupt

            def close(self) -> None:
                calls.append("close")
                raise KeyboardInterrupt

        self.module.make_env = lambda: FakeEnv()

        exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])

    def test_main_returns_zero_on_sigterm_shutdown_request(self) -> None:
        calls: list[str] = []

        class FakeEnv:
            def step(self) -> None:
                calls.append("step")
                self_module._request_shutdown(15, None)

            def close(self) -> None:
                calls.append("close")

        self_module = self.module
        self.module.make_env = lambda: FakeEnv()

        exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])

    def test_main_returns_zero_on_sigint_shutdown_request(self) -> None:
        calls: list[str] = []

        class FakeEnv:
            def step(self) -> None:
                calls.append("step")
                self_module._request_shutdown(2, None)

            def close(self) -> None:
                calls.append("close")

        self_module = self.module
        self.module.make_env = lambda: FakeEnv()

        exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])


if __name__ == "__main__":
    unittest.main()
