from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TypeAlias
from unittest.mock import patch

import mujoco
import numpy as np
import trimesh

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv
from acesim.env.mujoco.uuv_env import UUVEnv
from acesim.env.mujoco.vtol_env import VTOLEnv
from acesim.tools.px4_sdf_to_urdf import IMPORT_SPECS, PX4SDFAssetGenerator
from acesim.utils.simulation_clock import SimulationClock


class _FakePX4Transport:
    def __init__(self, *args: object, **kwargs: object) -> None:
        self.is_connected = False

    def update_connection_state(self) -> bool:
        return False

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        return None

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


def _config_path(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "config" / f"{name}.toml").resolve()


def _asset_xml_path(name: str) -> Path:
    return (
        Path(__file__).resolve().parents[1] / "acesim" / "env" / "mujoco" / "asset" / name / f"{name}.xml"
    ).resolve()


def _name_set(root: ET.Element, path: str, attribute: str = "name") -> set[str]:
    return {element.get(attribute, "") for element in root.findall(path) if element.get(attribute)}


def _collect_base_geoms(root: ET.Element) -> list[ET.Element]:
    base = root.find(".//worldbody//body[@name='base_link']")
    return [] if base is None else list(base.findall("geom"))


def _urdf_link(root: ET.Element, name: str) -> ET.Element:
    link = next((link for link in root.findall("link") if link.get("name") == name), None)
    assert link is not None, f"Missing URDF link {name}"
    return link


def _parse_float_vec(text: str | None) -> np.ndarray:
    assert text is not None
    return np.asarray([float(part) for part in text.split()], dtype=float)


_ExpectationMap: TypeAlias = dict[str, tuple[list[float], list[float] | None]]


class PX4SDFAssetPipelineTests(unittest.TestCase):
    def test_collada_units_are_applied_when_exporting_meshes(self) -> None:
        generator = PX4SDFAssetGenerator("plane")
        with tempfile.TemporaryDirectory(prefix="acesim_plane_units_") as tmpdir:
            paths = generator.generate(output_dir=Path(tmpdir) / "plane", compile_mjcf=False)
            mesh = trimesh.load(paths.mesh_dir / "base_link_visual_0.stl", force="mesh")
            extents = mesh.extents
            self.assertLess(extents[0], 1.0)
            self.assertLess(extents[1], 1.5)
            self.assertLess(extents[2], 0.5)

    def test_importer_generates_expected_urdf_and_mjcf_outputs(self) -> None:
        expected = {
            "plane": {
                "bodies": {"base_link", "rotor_4"},
                "joints": {
                    "left_elevon_joint",
                    "right_elevon_joint",
                    "elevator_joint",
                    "rudder_joint",
                    "left_flap_joint",
                    "right_flap_joint",
                },
                "actuators": {
                    "left_elevon_ctrl",
                    "right_elevon_ctrl",
                    "elevator_ctrl",
                    "rudder_ctrl",
                    "left_flap_ctrl",
                    "right_flap_ctrl",
                },
                "sites": {"base_link_origin", "rotor_offset_4"},
            },
            "standard_vtol": {
                "bodies": {"base_link", "rotor_0", "rotor_1", "rotor_2", "rotor_3", "rotor_4"},
                "joints": {"left_elevon_joint", "right_elevon_joint", "elevator_joint"},
                "actuators": {"left_elevon_ctrl", "right_elevon_ctrl", "elevator_ctrl"},
                "sites": {
                    "base_link_origin",
                    "rotor_offset_0",
                    "rotor_offset_1",
                    "rotor_offset_2",
                    "rotor_offset_3",
                    "rotor_offset_4",
                },
            },
            "uuv_bluerov2_heavy": {
                "bodies": {
                    "base_link",
                    "rotor_0",
                    "rotor_1",
                    "rotor_2",
                    "rotor_3",
                    "rotor_4",
                    "rotor_5",
                    "rotor_6",
                    "rotor_7",
                },
                "joints": set(),
                "actuators": set(),
                "sites": {
                    "base_link_origin",
                    "rotor_offset_0",
                    "rotor_offset_1",
                    "rotor_offset_2",
                    "rotor_offset_3",
                    "rotor_offset_4",
                    "rotor_offset_5",
                    "rotor_offset_6",
                    "rotor_offset_7",
                },
            },
        }

        for target in IMPORT_SPECS:
            with self.subTest(target=target):
                generator = PX4SDFAssetGenerator(target)
                with tempfile.TemporaryDirectory(prefix=f"acesim_{target}_") as tmpdir:
                    paths = generator.generate(output_dir=Path(tmpdir) / target, compile_mjcf=True)

                    self.assertTrue(paths.urdf_path.exists())
                    self.assertTrue(paths.xml_path.exists())
                    self.assertGreater(len(list(paths.mesh_dir.glob("*.stl"))), 0)

                    urdf_text = paths.urdf_path.read_text(encoding="utf-8")
                    self.assertNotIn("model://", urdf_text)
                    self.assertNotIn(".dae", urdf_text.lower())
                    self.assertIn(f"package://{target}/meshes/", urdf_text)

                    model = mujoco.MjModel.from_xml_path(str(paths.xml_path))
                    self.assertGreater(model.nmesh, 0)

                    xml_root = ET.parse(paths.xml_path).getroot()
                    body_names = _name_set(xml_root, ".//worldbody//body")
                    joint_names = _name_set(xml_root, ".//worldbody//joint")
                    actuator_names = _name_set(xml_root, ".//actuator/*")
                    site_names = _name_set(xml_root, ".//worldbody//site")
                    sensor_names = _name_set(xml_root, ".//sensor/*")

                    self.assertTrue(expected[target]["bodies"].issubset(body_names))
                    self.assertTrue(expected[target]["joints"].issubset(joint_names))
                    self.assertTrue(expected[target]["actuators"].issubset(actuator_names))
                    self.assertTrue(expected[target]["sites"].issubset(site_names))
                    self.assertTrue(
                        {"framepos", "framequat", "framelinvel", "gyro", "accelerometer", "magnetometer"}.issubset(
                            sensor_names
                        )
                    )

    def test_generated_asset_mesh_sizes_stay_in_expected_ranges(self) -> None:
        expected_limits = {
            "plane": (1.0, 1.5, 0.5),
            "standard_vtol": (1.5, 3.0, 3.0),
            "uuv_bluerov2_heavy": (1.0, 1.0, 0.5),
        }
        for target, (max_x, max_y, max_z) in expected_limits.items():
            with self.subTest(target=target):
                mesh = trimesh.load(_asset_xml_path(target).parent / "meshes" / "base_link_visual_0.stl", force="mesh")
                extents = mesh.extents
                self.assertLess(extents[0], max_x)
                self.assertLess(extents[1], max_y)
                self.assertLess(extents[2], max_z)

    def test_control_surface_meshes_are_exported_in_local_link_coordinates(self) -> None:
        mesh_cases = (
            ("plane", "left_elevon", "left_elevon_visual_0.stl"),
            ("plane", "right_elevon", "right_elevon_visual_0.stl"),
            ("standard_vtol", "left_elevon", "left_elevon_visual_0.stl"),
            ("standard_vtol", "right_elevon", "right_elevon_visual_0.stl"),
        )
        for target, link_name, mesh_name in mesh_cases:
            with self.subTest(target=target, link=link_name):
                mesh = trimesh.load(_asset_xml_path(target).parent / "meshes" / mesh_name, force="mesh")
                centroid = np.asarray(mesh.centroid, dtype=float)
                self.assertLess(
                    np.linalg.norm(centroid),
                    0.25,
                    f"{target}:{link_name} mesh should be centered near its local link frame",
                )

        rotor_mesh = trimesh.load(_asset_xml_path("plane").parent / "meshes" / "rotor_4_vis.stl", force="mesh")
        rotor_cov = np.cov(np.asarray(rotor_mesh.vertices, dtype=float).T)
        rotor_vals, rotor_vecs = np.linalg.eigh(rotor_cov)
        rotor_axis = rotor_vecs[:, np.argmax(rotor_vals)]
        self.assertGreater(abs(float(rotor_axis[0])), 0.95)
        self.assertLess(np.linalg.norm(np.asarray(rotor_mesh.centroid, dtype=float)), 1e-3)

        vtol_base_mesh = trimesh.load(
            _asset_xml_path("standard_vtol").parent / "meshes" / "base_link_visual_0.stl", force="mesh"
        )
        self.assertLess(np.linalg.norm(np.asarray(vtol_base_mesh.centroid, dtype=float)), 1e-3)

        urdf_expectations = {
            ("plane", "left_elevon"): ([0.07, 0.0, -0.08], [0.0, 0.0, 0.0]),
            ("plane", "right_elevon"): ([0.07, 0.0, -0.08], [0.0, 0.0, 0.0]),
            ("plane", "rotor_4"): ([0.0, 0.0, -0.09], [0.0, 0.0, 0.0]),
            ("standard_vtol", "left_elevon"): ([-0.105, 0.004, -0.034], [1.57079633, 0.0, -3.14159265]),
            ("standard_vtol", "right_elevon"): ([-0.105, -0.004, -0.034], [1.57079633, 0.0, -3.14159265]),
        }
        for (target, link_name), (xyz_expected, rpy_expected) in urdf_expectations.items():
            with self.subTest(target=target, link=link_name, field="origin"):
                root = ET.parse(_asset_xml_path(target).with_suffix(".urdf")).getroot()
                link = _urdf_link(root, link_name)
                visual = link.find("visual")
                assert visual is not None
                origin = visual.find("origin")
                assert origin is not None
                np.testing.assert_allclose(
                    _parse_float_vec(origin.get("xyz")), np.asarray(xyz_expected, dtype=float), atol=1e-6
                )
                np.testing.assert_allclose(
                    _parse_float_vec(origin.get("rpy")), np.asarray(rpy_expected, dtype=float), atol=1e-6
                )

    def test_generated_assets_keep_hidden_collision_geoms_on_base_link(self) -> None:
        for target in ("plane", "standard_vtol", "uuv_bluerov2_heavy"):
            with self.subTest(target=target):
                root = ET.parse(_asset_xml_path(target)).getroot()
                base_geoms = _collect_base_geoms(root)
                collision_geoms = [
                    geom
                    for geom in base_geoms
                    if geom.get("type") != "mesh" and (geom.get("contype") == "1" or geom.get("conaffinity") == "1")
                ]
                self.assertTrue(collision_geoms, f"{target} should keep a collidable primitive on base_link")
                for geom in collision_geoms:
                    self.assertEqual(geom.get("rgba"), "0 0 0 0")

                mesh_geoms = [geom for geom in base_geoms if geom.get("type") == "mesh"]
                self.assertTrue(mesh_geoms)
                for geom in mesh_geoms:
                    self.assertEqual(geom.get("contype"), "0")
                    self.assertEqual(geom.get("conaffinity"), "0")

                rotor_vis_geoms = root.findall(".//worldbody//body[@mocap='true']/geom")
                self.assertTrue(rotor_vis_geoms)
                for geom in rotor_vis_geoms:
                    self.assertEqual(geom.get("contype"), "0")
                    self.assertEqual(geom.get("conaffinity"), "0")

    def test_rotor_visuals_keep_mesh_pose_while_rotor_bodies_stay_nonvisual(self) -> None:
        expectations: dict[str, _ExpectationMap] = {
            "plane": {"rotor_4_vis": ([0.3, 0.0, 0.175], [0.0, 0.0, -0.09])},
            "standard_vtol": {
                "rotor_0_vis": ([0.35, -0.35, 0.19], None),
                "rotor_4_vis": ([-0.22, 0.0, 0.12], None),
            },
        }
        for target, rotor_map in expectations.items():
            with self.subTest(target=target):
                root = ET.parse(_asset_xml_path(target)).getroot()
                for rotor_name, (body_pos_expected, geom_pos_expected) in rotor_map.items():
                    body = root.find(f".//worldbody//body[@name='{rotor_name}']")
                    assert body is not None
                    np.testing.assert_allclose(
                        _parse_float_vec(body.get("pos")),
                        np.asarray(body_pos_expected, dtype=float),
                        atol=1e-6,
                    )
                    geom = body.find("geom")
                    assert geom is not None
                    if geom_pos_expected is not None:
                        np.testing.assert_allclose(
                            _parse_float_vec(geom.get("pos")),
                            np.asarray(geom_pos_expected, dtype=float),
                            atol=1e-6,
                        )

                for rotor_name in (f"rotor_{idx}" for idx in range(8)):
                    rotor_body = root.find(f".//worldbody//body[@name='{rotor_name}']")
                    if rotor_body is None:
                        continue
                    mesh_geoms = [geom for geom in rotor_body.findall("geom") if geom.get("mesh")]
                    self.assertFalse(mesh_geoms, f"{target}:{rotor_name} should not carry visible rotor meshes")

    def test_generated_assets_start_without_ground_penetration(self) -> None:
        for target in ("plane", "standard_vtol", "uuv_bluerov2_heavy"):
            with self.subTest(target=target):
                model = mujoco.MjModel.from_xml_path(str(_asset_xml_path(target)))
                data = mujoco.MjData(model)
                if model.nkey > 0:
                    mujoco.mj_resetDataKeyframe(model, data, 0)
                else:
                    mujoco.mj_resetData(model, data)
                mujoco.mj_forward(model, data)

                min_z = float("inf")
                for geom_id in range(model.ngeom):
                    if model.geom_contype[geom_id] == 0 and model.geom_conaffinity[geom_id] == 0:
                        continue
                    geom_type = int(model.geom_type[geom_id])
                    size = model.geom_size[geom_id]
                    if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                        local_points = np.array(
                            [
                                [sx, sy, sz]
                                for sx in (-size[0], size[0])
                                for sy in (-size[1], size[1])
                                for sz in (-size[2], size[2])
                            ],
                            dtype=float,
                        )
                    elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                        radius, half = float(size[0]), float(size[1])
                        local_points = np.array(
                            [
                                [radius, 0.0, half],
                                [radius, 0.0, -half],
                                [-radius, 0.0, half],
                                [-radius, 0.0, -half],
                                [0.0, radius, half],
                                [0.0, radius, -half],
                                [0.0, -radius, half],
                                [0.0, -radius, -half],
                            ],
                            dtype=float,
                        )
                    else:
                        continue

                    rot = data.geom_xmat[geom_id].reshape(3, 3)
                    world_points = data.geom_xpos[geom_id] + local_points @ rot.T
                    min_z = min(min_z, float(world_points[:, 2].min()))

                self.assertGreater(min_z, -1e-4, f"{target} should not start below the floor")

    def test_plane_and_standard_vtol_visual_geoms_stay_attached_to_their_bodies(self) -> None:
        cases = {
            "plane": ("left_elevon", "right_elevon", "elevator", "rudder"),
            "standard_vtol": ("base_link", "left_elevon", "right_elevon"),
        }
        for target, body_names in cases.items():
            with self.subTest(target=target):
                model = mujoco.MjModel.from_xml_path(str(_asset_xml_path(target)))
                data = mujoco.MjData(model)
                if model.nkey > 0:
                    mujoco.mj_resetDataKeyframe(model, data, 0)
                else:
                    mujoco.mj_resetData(model, data)
                mujoco.mj_forward(model, data)

                for body_name in body_names:
                    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                    self.assertGreaterEqual(body_id, 0)
                    geom_candidates = [
                        gid
                        for gid in range(model.ngeom)
                        if model.geom_bodyid[gid] == body_id and model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH
                    ]
                    self.assertTrue(geom_candidates, f"{target}:{body_name} should retain a visual mesh geom")
                    geom_id = geom_candidates[0]
                    body_pos = data.xpos[body_id]
                    geom_pos = data.geom_xpos[geom_id]
                    self.assertLess(
                        np.linalg.norm(geom_pos - body_pos),
                        0.35,
                        f"{target}:{body_name} visual geom should stay close to its parent body",
                    )


@patch("acesim.env.mujoco.px4_mj_env.VehicleVisualStatePublisher", _FakeVisualPublisher)
@patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _FakePX4Transport)
@patch("acesim.env.mujoco.mj_env.SimulationClock", lambda: SimulationClock(enable_zmq=False))
class GeneratedPX4AssetInstantiationTests(unittest.TestCase):
    def test_runtime_envs_load_generated_mesh_assets(self) -> None:
        cases = (
            ("plane", FWEnv, "rotor_4"),
            ("standard_vtol", VTOLEnv, "rotor_4"),
            ("uuv_bluerov2_heavy", UUVEnv, "rotor_7"),
        )
        for config_name, env_cls, rotor_name in cases:
            with self.subTest(config=config_name):
                env = env_cls(ConfigLoader(_config_path(config_name)))
                try:
                    self.assertGreater(env._mj_model.nmesh, 0)
                    asset_path = _asset_xml_path(config_name)
                    self.assertTrue(asset_path.exists())
                    mesh_dir = asset_path.parent / "meshes"
                    self.assertTrue(mesh_dir.exists())
                    self.assertGreater(len(list(mesh_dir.glob("*.stl"))), 0)
                    self.assertGreaterEqual(mujoco.mj_name2id(env._mj_model, mujoco.mjtObj.mjOBJ_BODY, rotor_name), 0)
                finally:
                    env.close()

    def test_visual_rotors_include_static_mount_rotation(self) -> None:
        plane = FWEnv(ConfigLoader(_config_path("plane")))
        uuv = UUVEnv(ConfigLoader(_config_path("uuv_bluerov2_heavy")))
        vtol = VTOLEnv(ConfigLoader(_config_path("standard_vtol")))
        try:
            plane._update_vehicle_visuals()
            plane_quat = plane._mj_data.mocap_quat[plane._puller_mocap_id]
            expected_plane = plane._puller_mount_rot.as_quat(scalar_first=True)
            self.assertTrue(np.allclose(plane_quat, expected_plane, atol=1e-6))

            uuv._update_vehicle_visuals()
            for rotor_idx in (4, 5, 6, 7):
                array_idx = uuv._rotor_indices.index(rotor_idx)
                mocap_id = uuv._rotor_mocap_ids[array_idx]
                quat = uuv._mj_data.mocap_quat[mocap_id]
                expected = uuv._rotor_mount_rot[array_idx].as_quat(scalar_first=True)
                self.assertTrue(np.allclose(quat, expected, atol=1e-6))

            vtol._update_vehicle_visuals()
            for i, mocap_id in enumerate(vtol._lift_rotor_mocap_ids):
                quat = vtol._mj_data.mocap_quat[mocap_id]
                expected = vtol._lift_rotor_mount_rot[i].as_quat(scalar_first=True)
                self.assertTrue(np.allclose(quat, expected, atol=1e-6))
        finally:
            plane.close()
            uuv.close()
            vtol.close()

    def test_runtime_envs_initialize_rotor_visual_positions(self) -> None:
        plane = FWEnv(ConfigLoader(_config_path("plane")))
        vtol = VTOLEnv(ConfigLoader(_config_path("standard_vtol")))
        try:
            plane_pos = plane._mj_data.mocap_pos[plane._puller_mocap_id]
            np.testing.assert_allclose(plane_pos, np.array([0.3, 0.0, 0.175]), atol=1e-6)

            for rotor_index, mocap_id, offset in zip(
                vtol._lift_rotor_indices,
                vtol._lift_rotor_mocap_ids,
                vtol._lift_rotor_offsets,
            ):
                expected = vtol._get_sensor_raw("pos") + offset
                np.testing.assert_allclose(vtol._mj_data.mocap_pos[mocap_id], expected, atol=1e-6)
        finally:
            plane.close()
            vtol.close()


if __name__ == "__main__":
    unittest.main()
