from __future__ import annotations

import unittest
from pathlib import Path
from unittest.mock import patch

import mujoco

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv
from acesim.env.mujoco.uuv_env import UUVEnv
from acesim.env.mujoco.vtol_env import VTOLEnv


def _config_path(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "config" / f"{name}.toml").resolve()


def _mujoco_asset_root(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "env" / "mujoco" / "asset" / name).resolve()


class _FakePX4Transport:
    HIL_SENSOR_FIELDS_ACCEL = 0
    HIL_SENSOR_FIELDS_GYRO = 0
    HIL_SENSOR_FIELDS_MAG = 0
    HIL_SENSOR_FIELDS_DIFF_PRESS = 0
    HIL_SENSOR_FIELDS_BARO = 0

    def __init__(self, *args: object, **kwargs: object) -> None:
        self.is_connected = False

    def update_connection_state(self) -> bool:
        return False

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> bool:
        return False

    def read_applied_actuator_controls(self, channel_count: int) -> None:
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


class _FakeClockPublisher:
    def __init__(self, *args: object, **kwargs: object) -> None:
        return None

    def publish(self, timestamp_us: int) -> None:
        return None

    def close(self) -> None:
        return None


@patch("acesim.env.mujoco.px4_mj_env.VehicleVisualStatePublisher", _FakeVisualPublisher)
@patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _FakePX4Transport)
@patch("acesim.env.mujoco.mj_env.ClockPublisher", _FakeClockPublisher)
class PX4SDFAssetPipelineRuntimeTests(unittest.TestCase):
    def test_runtime_rotor_marker_bodies_are_zero_mass_and_visual_rotors_are_mocap_only(self) -> None:
        expectations = {
            "advanced_plane": [4],
            "standard_vtol": [0, 1, 2, 3, 4],
            "uuv_bluerov2_heavy": list(range(8)),
        }
        for asset_name, rotor_indices in expectations.items():
            with self.subTest(asset=asset_name):
                model = mujoco.MjModel.from_xml_path(str(_mujoco_asset_root(asset_name) / f"{asset_name}.xml"))
                for rotor_index in rotor_indices:
                    rotor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}")
                    vis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}_vis")
                    self.assertGreaterEqual(rotor_body_id, 0)
                    self.assertGreaterEqual(vis_body_id, 0)
                    self.assertLessEqual(float(model.body_mass[rotor_body_id]), 1e-9)
                    self.assertGreaterEqual(int(model.body_mocapid[vis_body_id]), 0)
                    for geom_id in range(
                        model.body_geomadr[vis_body_id],
                        model.body_geomadr[vis_body_id] + model.body_geomnum[vis_body_id],
                    ):
                        self.assertEqual(int(model.geom_contype[geom_id]), 0)
                        self.assertEqual(int(model.geom_conaffinity[geom_id]), 0)
                        self.assertEqual(int(model.geom_type[geom_id]), int(mujoco.mjtGeom.mjGEOM_MESH))

                    if asset_name == "uuv_bluerov2_heavy":
                        self.assertEqual(int(model.body_geomnum[rotor_body_id]), 0)
                        continue

                    for geom_id in range(
                        model.body_geomadr[rotor_body_id],
                        model.body_geomadr[rotor_body_id] + model.body_geomnum[rotor_body_id],
                    ):
                        self.assertEqual(int(model.geom_contype[geom_id]), 0)
                        self.assertEqual(int(model.geom_conaffinity[geom_id]), 0)

    def test_runtime_visual_rotors_stay_attached_to_their_bodies(self) -> None:
        plane = FWEnv(ConfigLoader(_config_path("advanced_plane")))
        vtol = VTOLEnv(ConfigLoader(_config_path("standard_vtol")))
        uuv = UUVEnv(ConfigLoader(_config_path("uuv_bluerov2_heavy")))
        try:
            plane._update_vehicle_visuals()
            vtol._update_vehicle_visuals()
            uuv._update_vehicle_visuals()

            plane_mocap = plane._mj_data.mocap_pos[plane._puller_mocap_id]
            self.assertGreater(float(plane_mocap[0]), 0.20)
            self.assertLess(float(plane_mocap[0]), 0.23)

            for mocap_id in vtol._lift_rotor_mocap_ids:
                self.assertGreaterEqual(mocap_id, 0)
            self.assertGreaterEqual(vtol._puller_mocap_id, 0)

            for mocap_id in uuv._rotor_mocap_ids:
                self.assertGreaterEqual(mocap_id, 0)
        finally:
            plane.close()
            vtol.close()
            uuv.close()


if __name__ == "__main__":
    unittest.main()
