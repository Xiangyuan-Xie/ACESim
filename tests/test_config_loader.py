from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python < 3.11 compatibility
    import tomli as tomllib

from acesim.config.config_loader import ConfigLoader

ROOT = Path(__file__).resolve().parents[1]


class ConfigLoaderTests(unittest.TestCase):
    def _write_config(self, root: Path, *, env_type: str) -> Path:
        config_path = root / "config.toml"
        asset_dst_dir = root / "mujoco"
        asset_dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "acesim" / "config" / "mujoco" / "x500_arm2x.toml", asset_dst_dir / "x500_arm2x.toml")
        config_path.write_text(
            "\n".join(
                [
                    "[basic]",
                    'sim_type = "mujoco"',
                    f'env_type = "{env_type}"',
                    'scene_name = "default"',
                    'asset_name = "x500_arm2x"',
                    'benchmark = "multirotor"',
                    "",
                ]
            ),
            encoding="utf-8",
        )
        return config_path

    def test_get_sim_info_supports_am_env_type(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_config_loader_") as tmpdir:
            loader = ConfigLoader(self._write_config(Path(tmpdir), env_type="am"))

        self.assertEqual(loader.get_sim_info(), ("acesim.env.mujoco.am_env", "AMEnv"))

    def test_get_sim_info_rejects_mc_arm_env_type(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_config_loader_") as tmpdir:
            loader = ConfigLoader(self._write_config(Path(tmpdir), env_type="mc_arm"))

        with self.assertRaisesRegex(ValueError, "Env type 'mc_arm' not supported"):
            loader.get_sim_info()

    def test_mujoco_air_density_is_not_configured_in_x500_arm_downwash_toml(self) -> None:
        config = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500_arm2x.toml").read_text())

        params = config["params"]
        self.assertNotIn("air_density", params["downwash"])
        self.assertNotIn("body_aero_drag", params)
