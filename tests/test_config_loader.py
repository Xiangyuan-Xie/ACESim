from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python < 3.11 compatibility
    import tomli as tomllib

from acesim.config.asset_params import get_optional_table, resolve_vehicle_params_table
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

    def test_only_x500_arm2x_enables_arm_command_stream_by_default(self) -> None:
        x500 = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500.toml").read_text())["params"]
        x500_arm = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500_arm2x.toml").read_text())["params"]

        self.assertNotIn("arm_command_stream", x500)
        self.assertTrue(x500_arm["arm_command_stream"]["enabled"])
        self.assertEqual(x500_arm["arm_command_stream"]["zmq_endpoint"], "tcp://0.0.0.0:5604")

    def test_x500_assets_enable_vehicle_truth_stream(self) -> None:
        x500 = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500.toml").read_text())["params"]
        x500_arm = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500_arm2x.toml").read_text())["params"]

        for params in (x500, x500_arm):
            self.assertTrue(params["truth_stream"]["enabled"])
            self.assertEqual(params["truth_stream"]["rate_hz"], 120.0)
            self.assertEqual(params["truth_stream"]["zmq_endpoint"], "tcp://0.0.0.0:5605")

    def test_x500_uses_air_gear_450ii_table_coefficients(self) -> None:
        x500 = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500.toml").read_text())["params"]

        self.assertAlmostEqual(x500["max_rot_velocity"], 1032.2226262144864)
        self.assertAlmostEqual(x500["motor_constant"], 1.2108824692304194e-05)
        self.assertAlmostEqual(x500["moment_constant"], 0.014363125015663197)
        np.testing.assert_allclose(
            x500["throttle_to_omega"]["coefficients"],
            [0.0, 1.3883712147882463, -0.3866198458183226],
        )
        self.assertNotIn("thrust_intercept", x500)
        self.assertNotIn("moment_intercept", x500)
        self.assertNotIn("throttle_curve", x500)
        self.assertNotIn("model", x500["throttle_to_omega"])

    def test_x500_arm2x_uses_f100_hq8045_rotor_coefficients(self) -> None:
        params = tomllib.loads((ROOT / "acesim" / "config" / "mujoco" / "x500_arm2x.toml").read_text())["params"]

        self.assertAlmostEqual(params["max_rot_velocity"], 1554.8789240167084)
        self.assertAlmostEqual(params["motor_constant"], 9.810964962061632e-06)
        self.assertAlmostEqual(params["moment_constant"], 0.027091347106075237)
        self.assertAlmostEqual(params["rotor_radius"], 0.1016)
        np.testing.assert_allclose(
            params["throttle_to_omega"]["coefficients"],
            [0.0, 1.755537447080606, -0.7549872697502744],
        )
        self.assertNotIn("thrust_intercept", params)
        self.assertNotIn("moment_intercept", params)
        self.assertNotIn("throttle_curve", params)
        self.assertNotIn("model", params["throttle_to_omega"])

    def test_shared_asset_params_helper_reads_optional_tables(self) -> None:
        params = {"visual_stream": {"enabled": True}}

        self.assertEqual(get_optional_table(params, "visual_stream"), {"enabled": True})
        self.assertEqual(get_optional_table(params, "missing"), {})

        with self.assertRaisesRegex(ValueError, "visual_stream must be a table"):
            get_optional_table({"visual_stream": 1}, "visual_stream")

    def test_vehicle_params_table_rejects_flat_and_nested_conflicts(self) -> None:
        with self.assertRaisesRegex(ValueError, "params.mc conflicts with legacy flat params"):
            resolve_vehicle_params_table(
                {"mc": {"motor_constant": 1.0}, "motor_constant": 1.0},
                "mc",
                legacy_keys=("motor_constant",),
            )
