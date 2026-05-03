from __future__ import annotations

import shutil
import tempfile
import unittest
from importlib import import_module
from pathlib import Path
from typing import Protocol
from unittest.mock import patch

from acesim.config.config_loader import ConfigLoader


class _FakePX4Transport:
    def __init__(self, *args: object, **kwargs: object) -> None:
        self.is_connected = False

    def update_connection_state(self) -> bool:
        return False

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        return None

    def read_applied_actuator_controls(self, channel_count: int):
        return None

    def update_arming_state(self) -> bool:
        return False

    def close(self) -> None:
        return None


class _FakeVisualPublisher:
    def __init__(self, params: object) -> None:
        self.is_enabled = False

    def publish(self, state: object) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeArmStatePublisher:
    def __init__(self, *args: object, **kwargs: object) -> None:
        return None

    def publish(
        self,
        timestamp_us: int,
        positions: object,
        velocities: object,
        efforts: object,
    ) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeClockPublisher:
    def __init__(self, *args: object, **kwargs: object) -> None:
        return None

    def publish(self, timestamp_us: int) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeRobotAgent:
    def act(self) -> tuple[list[float], None, None]:
        return ([0.0] * 7, None, None)

    def close(self) -> None:
        return None


class _SupportsHeadlessEnv(Protocol):
    _config_loader: ConfigLoader
    _rotor_count: int
    _step_count: int

    def step(self) -> None: ...

    def close(self) -> None: ...


def _config_path(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "config" / f"{name}.toml").resolve()


@patch("acesim.env.mujoco.am_env.make_robot", lambda: _FakeRobotAgent())
@patch("acesim.env.mujoco.am_env.ArmStatePublisher", _FakeArmStatePublisher)
@patch("acesim.env.mujoco.px4_mj_env.VehicleVisualStatePublisher", _FakeVisualPublisher)
@patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _FakePX4Transport)
@patch("acesim.env.mujoco.mj_env.ClockPublisher", _FakeClockPublisher)
class MujocoHeadlessStartupTests(unittest.TestCase):
    def _instantiate_and_step(self, config_file: Path) -> _SupportsHeadlessEnv:
        loader = ConfigLoader(config_file)
        module_name, class_name = loader.get_sim_info()
        env_cls = getattr(import_module(module_name), class_name)
        env = env_cls(loader)
        try:
            for _ in range(3):
                env.step()
            return env
        except Exception:
            env.close()
            raise

    def _write_mc_config(self, root: Path, *, asset_name: str) -> Path:
        config_path = root / f"{asset_name}.toml"
        asset_src = Path(__file__).resolve().parents[1] / "acesim" / "config" / "mujoco" / f"{asset_name}.toml"
        asset_dst_dir = root / "mujoco"
        asset_dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(asset_src, asset_dst_dir / f"{asset_name}.toml")
        config_path.write_text(
            "\n".join(
                [
                    "[basic]",
                    'sim_type = "mujoco"',
                    'env_type = "mc"',
                    'scene_name = "default"',
                    f'asset_name = "{asset_name}"',
                    'benchmark = "multirotor"',
                    "",
                ]
            ),
            encoding="utf-8",
        )
        return config_path

    def test_default_config_starts_headless(self) -> None:
        expected_loader = ConfigLoader(_config_path("default"))
        env = self._instantiate_and_step(_config_path("default"))
        try:
            for _ in range(8):
                env.step()
            self.assertEqual(env._config_loader.get_asset_name(), expected_loader.get_asset_name())
            self.assertEqual(env._config_loader.get_env_type(), expected_loader.get_env_type())
            self.assertGreaterEqual(env._step_count, 1)
        finally:
            env.close()

    def test_all_mujoco_configs_start_headless(self) -> None:
        config_cases = [
            _config_path("default"),
            _config_path("advanced_plane"),
            _config_path("standard_vtol"),
            _config_path("uuv_bluerov2_heavy"),
        ]
        synthetic_assets = ["iris", "x500", "typhoon_h480"]

        with tempfile.TemporaryDirectory(prefix="acesim_mujoco_headless_") as tmpdir:
            temp_root = Path(tmpdir)
            for asset_name in synthetic_assets:
                config_cases.append(self._write_mc_config(temp_root, asset_name=asset_name))

            for config_file in config_cases:
                with self.subTest(config=config_file.name):
                    env = self._instantiate_and_step(config_file)
                    try:
                        self.assertGreater(env._step_count, 0)
                    finally:
                        env.close()

    def test_default_mc_env_handles_split_visual_rotor_offsets(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_mc_split_offsets_") as tmpdir:
            config_path = self._write_mc_config(Path(tmpdir), asset_name="x500")
            env = self._instantiate_and_step(config_path)
            try:
                self.assertEqual(env._rotor_count, 4)
            finally:
                env.close()
