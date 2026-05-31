from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from typing import Any, cast
from unittest import mock


def _load_acesim_play_headless_module() -> Any:
    module_name = "_test_acesim_ros2_acesim_play_headless"
    module_names = [
        module_name,
        "acesim",
        "acesim.core",
        "acesim.core.play",
        "acesim.env",
        "acesim.env.base_env",
        "acesim.config",
        "acesim.config.config_loader",
    ]
    saved_modules = {name: sys.modules[name] for name in module_names if name in sys.modules}
    for name in module_names:
        sys.modules.pop(name, None)

    acesim_module: Any = types.ModuleType("acesim")
    acesim_core_module: Any = types.ModuleType("acesim.core")
    acesim_core_play_module: Any = types.ModuleType("acesim.core.play")
    acesim_env_module: Any = types.ModuleType("acesim.env")
    acesim_env_base_env_module: Any = types.ModuleType("acesim.env.base_env")
    acesim_config_module: Any = types.ModuleType("acesim.config")
    acesim_config_loader_module: Any = types.ModuleType("acesim.config.config_loader")

    class BaseEnv:
        pass

    class ConfigLoader:
        def __init__(self, path: Path | None = None) -> None:
            self.path = path

    def make_env() -> object:
        raise RuntimeError("make_env should be patched in tests")

    setattr(acesim_core_play_module, "make_env", make_env)
    setattr(acesim_env_base_env_module, "BaseEnv", BaseEnv)
    setattr(acesim_config_loader_module, "ConfigLoader", ConfigLoader)
    sys.modules["acesim"] = acesim_module
    sys.modules["acesim.core"] = acesim_core_module
    sys.modules["acesim.core.play"] = acesim_core_play_module
    sys.modules["acesim.env"] = acesim_env_module
    sys.modules["acesim.env.base_env"] = acesim_env_base_env_module
    sys.modules["acesim.config"] = acesim_config_module
    sys.modules["acesim.config.config_loader"] = acesim_config_loader_module

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
    try:
        spec.loader.exec_module(module)
        return module
    finally:
        for name in module_names:
            sys.modules.pop(name, None)
        sys.modules.update(saved_modules)


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

        with mock.patch.object(sys, "argv", ["acesim_play_headless"]):
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

        with mock.patch.object(sys, "argv", ["acesim_play_headless"]):
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

        with mock.patch.object(sys, "argv", ["acesim_play_headless"]):
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

        with mock.patch.object(sys, "argv", ["acesim_play_headless"]):
            exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])

    def test_main_calls_make_env_without_args_by_default(self) -> None:
        calls: list[tuple[object, ...]] = []

        class FakeEnv:
            def step(self) -> None:
                self_module._request_shutdown(2, None)

            def close(self) -> None:
                pass

        def fake_make_env(*args: object) -> FakeEnv:
            calls.append(args)
            return FakeEnv()

        self_module = self.module
        self.module.make_env = fake_make_env
        with mock.patch.object(sys, "argv", ["acesim_play_headless"]):
            exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, [()])

    def test_main_passes_config_loader_for_config_argument(self) -> None:
        calls: list[tuple[object, ...]] = []
        config_path = Path("/tmp/custom_acesim.toml")

        class FakeEnv:
            def step(self) -> None:
                self_module._request_shutdown(15, None)

            def close(self) -> None:
                pass

        def fake_make_env(*args: object) -> FakeEnv:
            calls.append(args)
            return FakeEnv()

        self_module = self.module
        self.module.make_env = fake_make_env
        with mock.patch.object(sys, "argv", ["acesim_play_headless", "--config", str(config_path)]):
            exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(calls[0]), 1)
        config_loader = cast(Any, calls[0][0])
        self.assertIsInstance(config_loader, self.module.ConfigLoader)
        self.assertEqual(config_loader.path, config_path)

    def test_default_headless_run_does_not_sleep(self) -> None:
        calls: list[str] = []

        class FakeEnv:
            _simulation_time_us = 0

            def step(self) -> None:
                calls.append("step")
                self._simulation_time_us += 100_000
                self_module._request_shutdown(15, None)

            def close(self) -> None:
                calls.append("close")

        self_module = self.module
        self.module.make_env = lambda: FakeEnv()
        with mock.patch.object(sys, "argv", ["acesim_play_headless"]):
            exit_code = self.module.main()

        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])

    def test_help_only_documents_config_argument(self) -> None:
        parser = self.module._build_arg_parser()

        help_text = parser.format_help()

        self.assertIn("--config", help_text)
        self.assertNotIn("real-time-rate", help_text)

    def test_parse_args_rejects_real_time_rate_argument(self) -> None:
        with mock.patch.object(sys, "argv", ["acesim_play_headless", "--real-time-rate", "1"]):
            with self.assertRaises(SystemExit):
                self.module._parse_args()


if __name__ == "__main__":
    unittest.main()
