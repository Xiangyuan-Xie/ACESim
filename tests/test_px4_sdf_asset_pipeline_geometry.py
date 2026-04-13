from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from acesim.tools.sdf2urdf.providers import PX4_PROVIDER
from acesim.tools.sdf2urdf.providers.px4 import ADVANCED_PLANE_SCALE

_REPO_ROOT = Path(__file__).resolve().parents[1]
_PX4_MODEL_ROOT = (
    _REPO_ROOT
    / "acesim"
    / "third_party"
    / "aircraft"
    / "PX4-Autopilot"
    / "Tools"
    / "simulation"
    / "gazebo-classic"
    / "sitl_gazebo-classic"
    / "models"
)


def _mujoco_asset_root(name: str) -> Path:
    return (_REPO_ROOT / "acesim" / "env" / "mujoco" / "asset" / name).resolve()


def _load_scene_geometry(path: Path) -> trimesh.Trimesh:
    geometry = trimesh.load(path, force="scene")
    if isinstance(geometry, trimesh.Scene):
        geometry = geometry.to_geometry()
    assert isinstance(geometry, trimesh.Trimesh)
    return geometry


def _body_mesh_vertices_world(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"Missing body {body_name}")
    vertices_world: list[np.ndarray] = []
    for geom_id in range(model.body_geomadr[body_id], model.body_geomadr[body_id] + model.body_geomnum[body_id]):
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        start = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = model.mesh_vert[start : start + count]
        world = (vertices @ data.geom_xmat[geom_id].reshape(3, 3).T) + data.geom_xpos[geom_id]
        vertices_world.append(world)
    if not vertices_world:
        raise ValueError(f"Body {body_name} has no mesh vertices")
    return np.vstack(vertices_world)


def _body_mesh_vertices_relative_to(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    reference_body_name: str,
) -> np.ndarray:
    reference_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, reference_body_name)
    if reference_body_id < 0:
        raise ValueError(f"Missing reference body {reference_body_name}")
    reference_rot = Rotation.from_quat(data.xquat[reference_body_id].copy(), scalar_first=True)
    reference_pos = data.xpos[reference_body_id].copy()
    return reference_rot.inv().apply(_body_mesh_vertices_world(model, data, body_name) - reference_pos)


def _principal_axes(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centered = points - points.mean(axis=0)
    eigvals, eigvecs = np.linalg.eigh(np.cov(centered.T))
    return eigvals, eigvecs


def _projected_bbox(points: np.ndarray, *, dims: tuple[int, int]) -> np.ndarray:
    projected = points[:, dims]
    return np.vstack([projected.min(axis=0), projected.max(axis=0)])


def _source_sdf_path(name: str) -> Path:
    return PX4_PROVIDER.sdf_path_for_target(name)


def _source_visual_pose(name: str, link_name: str) -> tuple[str, str, str]:
    root = ET.parse(_source_sdf_path(name)).getroot()
    link = next(link for link in root.findall(".//link") if link.get("name") == link_name)
    visual = next(visual for visual in link.findall("visual") if visual.find("geometry/mesh/uri") is not None)
    pose_text = visual.findtext("pose", default="0 0 0 0 0 0")
    xyz = " ".join(pose_text.split()[:3])
    rpy = " ".join(pose_text.split()[3:])
    uri = visual.findtext("geometry/mesh/uri", default="")
    return xyz, rpy, uri


def _source_joint_truth(name: str, joint_name: str) -> tuple[str | None, str | None]:
    root = ET.parse(_source_sdf_path(name)).getroot()
    joint = next(joint for joint in root.findall(".//joint") if joint.get("name") == joint_name)
    pose_text = joint.findtext("pose")
    pose_xyz = None if pose_text is None else " ".join(pose_text.split()[:3])
    axis_xyz = joint.findtext("axis/xyz")
    return pose_xyz, axis_xyz


def _parse_pose_text(pose_text: str | None) -> tuple[np.ndarray, Rotation]:
    values = [float(value) for value in (pose_text or "0 0 0 0 0 0").split()]
    return np.asarray(values[:3], dtype=float), Rotation.from_euler("xyz", values[3:])


def _expected_local_visual_pose_from_source(name: str, link_name: str) -> tuple[np.ndarray, np.ndarray]:
    visual_xyz, visual_rpy, _ = _source_visual_pose(name, link_name)
    joint_name = f"{link_name}_joint"
    root = ET.parse(_source_sdf_path(name)).getroot()
    joint = next((joint for joint in root.findall(".//joint") if joint.get("name") == joint_name), None)
    requires_local_conversion = name == "advanced_plane" or (
        name == "standard_vtol" and link_name in {"left_elevon", "right_elevon"}
    )
    if joint is None or joint.findtext("pose") is None or not requires_local_conversion:
        visual_xyz_vals = np.asarray([float(value) for value in visual_xyz.split()], dtype=float)
        if name == "advanced_plane":
            visual_xyz_vals = visual_xyz_vals * ADVANCED_PLANE_SCALE
        return (
            visual_xyz_vals,
            np.asarray([float(value) for value in visual_rpy.split()], dtype=float),
        )
    joint_xyz, joint_rot = _parse_pose_text(joint.findtext("pose"))
    if name == "advanced_plane":
        joint_xyz = joint_xyz * ADVANCED_PLANE_SCALE
    visual_pos = np.asarray([float(value) for value in visual_xyz.split()], dtype=float)
    if name == "advanced_plane":
        visual_pos = visual_pos * ADVANCED_PLANE_SCALE
    visual_rot = Rotation.from_euler("xyz", [float(value) for value in visual_rpy.split()])
    local_pos = joint_rot.inv().apply(visual_pos - joint_xyz)
    local_rot = (joint_rot.inv() * visual_rot).as_euler("xyz")
    return local_pos, local_rot


def _assert_pose_text_equal(test_case: unittest.TestCase, actual: str | None, expected: str | None) -> None:
    test_case.assertIsNotNone(actual)
    test_case.assertIsNotNone(expected)
    assert actual is not None
    assert expected is not None
    test_case.assertTrue(
        np.allclose([float(value) for value in actual.split()], [float(value) for value in expected.split()], atol=1e-6)
    )


def _load_model_and_data(asset_name: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(_mujoco_asset_root(asset_name) / f"{asset_name}.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


class PX4SDFAssetPipelineGeometryTests(unittest.TestCase):
    def test_px4_manual_mesh_exports_preserve_source_mesh_origins(self) -> None:
        advanced_prop = trimesh.load(_mujoco_asset_root("advanced_plane") / "meshes" / "rotor_4_vis.stl", force="mesh")
        source_prop = _load_scene_geometry(_PX4_MODEL_ROOT / "plane" / "meshes" / "iris_prop_ccw.dae")
        self.assertTrue(np.allclose(advanced_prop.extents, source_prop.extents, atol=1e-3))

        standard_left = trimesh.load(
            _mujoco_asset_root("standard_vtol") / "meshes" / "left_elevon_visual_0.stl",
            force="mesh",
        )
        standard_right = trimesh.load(
            _mujoco_asset_root("standard_vtol") / "meshes" / "right_elevon_visual_0.stl",
            force="mesh",
        )
        source_left = _load_scene_geometry(_PX4_MODEL_ROOT / "standard_vtol" / "meshes" / "x8_elevon_left.dae")
        source_right = _load_scene_geometry(_PX4_MODEL_ROOT / "standard_vtol" / "meshes" / "x8_elevon_right.dae")
        self.assertTrue(np.allclose(standard_left.centroid, source_left.centroid, atol=1e-3))
        self.assertTrue(np.allclose(standard_right.centroid, source_right.centroid, atol=1e-3))

        uuv_body = trimesh.load(
            _mujoco_asset_root("uuv_bluerov2_heavy") / "meshes" / "base_link_visual_0.stl", force="mesh"
        )
        self.assertGreater(abs(float(uuv_body.centroid[2])), 0.01)

    def test_manual_mesh_exports_keep_source_local_offsets_for_sdf_truth_targets(self) -> None:
        cases = [
            ("advanced_plane", "left_elevon_visual_0.stl", "plane/meshes/left_aileron.dae"),
            ("advanced_plane", "right_elevon_visual_0.stl", "plane/meshes/right_aileron.dae"),
            ("advanced_plane", "left_flap_visual_0.stl", "plane/meshes/left_flap.dae"),
            ("advanced_plane", "right_flap_visual_0.stl", "plane/meshes/right_flap.dae"),
            ("advanced_plane", "elevator_visual_0.stl", "plane/meshes/elevators.dae"),
            ("advanced_plane", "rudder_visual_0.stl", "plane/meshes/rudder.dae"),
            ("standard_vtol", "left_elevon_visual_0.stl", "standard_vtol/meshes/x8_elevon_left.dae"),
            ("standard_vtol", "right_elevon_visual_0.stl", "standard_vtol/meshes/x8_elevon_right.dae"),
        ]
        for target, mesh_name, source_rel in cases:
            with self.subTest(target=target, mesh=mesh_name):
                exported = trimesh.load(_mujoco_asset_root(target) / "meshes" / mesh_name, force="mesh")
                source = _load_scene_geometry(_PX4_MODEL_ROOT / source_rel)
                self.assertTrue(np.allclose(exported.centroid, source.centroid, atol=1e-3))

    def test_advanced_plane_puller_uses_the_same_source_mesh_as_px4_sdf(self) -> None:
        exported = trimesh.load(_mujoco_asset_root("advanced_plane") / "meshes" / "rotor_4_vis.stl", force="mesh")
        _, _, source_uri = _source_visual_pose("advanced_plane", "rotor_puller")
        source = _load_scene_geometry(_PX4_MODEL_ROOT / "plane" / "meshes" / Path(source_uri).name)
        self.assertTrue(np.allclose(exported.extents, source.extents, atol=1e-3))

    def test_generated_urdf_matches_source_sdf_visual_and_joint_truth(self) -> None:
        advanced_root = ET.parse(_mujoco_asset_root("advanced_plane") / "advanced_plane.urdf").getroot()
        standard_root = ET.parse(_mujoco_asset_root("standard_vtol") / "standard_vtol.urdf").getroot()

        for link_name, source_link_name in {
            "left_elevon": "left_elevon",
            "right_elevon": "right_elevon",
            "left_flap": "left_flap",
            "right_flap": "right_flap",
            "elevator": "elevator",
            "rudder": "rudder",
        }.items():
            with self.subTest(link=link_name):
                expected_xyz, expected_rpy = _expected_local_visual_pose_from_source("advanced_plane", source_link_name)
                link = next(link for link in advanced_root.findall("link") if link.get("name") == link_name)
                origin = link.find("visual/origin")
                assert origin is not None
                self.assertTrue(
                    np.allclose([float(value) for value in origin.get("xyz", "0 0 0").split()], expected_xyz, atol=1e-6)
                )
                self.assertTrue(
                    np.allclose([float(value) for value in origin.get("rpy", "0 0 0").split()], expected_rpy, atol=1e-6)
                )

        for link_name, source_link_name in {
            "left_elevon": "left_elevon",
            "right_elevon": "right_elevon",
            "rotor_4": "rotor_puller",
        }.items():
            with self.subTest(vtol_link=link_name):
                expected_xyz, expected_rpy = _expected_local_visual_pose_from_source("standard_vtol", source_link_name)
                link = next(link for link in standard_root.findall("link") if link.get("name") == link_name)
                origin = link.find("visual/origin")
                assert origin is not None
                self.assertTrue(
                    np.allclose([float(value) for value in origin.get("xyz", "0 0 0").split()], expected_xyz, atol=1e-6)
                )
                self.assertTrue(
                    np.allclose([float(value) for value in origin.get("rpy", "0 0 0").split()], expected_rpy, atol=1e-6)
                )

        for joint_name, source_joint_name in {
            "rotor_4_joint": "rotor_puller_joint",
            "left_elevon_joint": "left_elevon_joint",
            "right_elevon_joint": "right_elevon_joint",
            "elevator_joint": "elevator_joint",
            "rudder_joint": "rudder_joint",
        }.items():
            with self.subTest(joint=joint_name):
                xyz, axis = _source_joint_truth("standard_vtol", source_joint_name)
                joint = next(joint for joint in standard_root.findall("joint") if joint.get("name") == joint_name)
                origin = joint.find("origin")
                axis_elem = joint.find("axis")
                assert origin is not None and axis_elem is not None
                if xyz is not None:
                    _assert_pose_text_equal(self, origin.get("xyz"), xyz)
                _assert_pose_text_equal(self, axis_elem.get("xyz"), axis)

    def test_advanced_plane_puller_uses_wide_propeller_disc_geometry(self) -> None:
        model, data = _load_model_and_data("advanced_plane")

        base_points_body = _body_mesh_vertices_relative_to(model, data, "base_link", "base_link")
        base_size_body = base_points_body.max(axis=0) - base_points_body.min(axis=0)
        self.assertGreater(float(base_size_body[0]), 0.45)
        self.assertLess(float(base_size_body[0]), 0.50)

        rotor_points_body = _body_mesh_vertices_relative_to(model, data, "rotor_4_vis", "base_link")
        rotor_size_body = rotor_points_body.max(axis=0) - rotor_points_body.min(axis=0)
        eigvals, eigvecs = _principal_axes(rotor_points_body)
        thin_axis = eigvecs[:, int(np.argmin(eigvals))]

        self.assertLess(rotor_size_body[0], 0.03)
        rotor_disc_diameter = max(float(rotor_size_body[1]), float(rotor_size_body[2]))
        self.assertGreater(rotor_disc_diameter, 0.13)
        self.assertLess(rotor_disc_diameter, 0.18)
        self.assertGreater(abs(float(thin_axis[0])), 0.95)

    def test_advanced_plane_puller_aligns_with_thrust_marker_and_nose(self) -> None:
        model, data = _load_model_and_data("advanced_plane")

        base_points = _body_mesh_vertices_relative_to(model, data, "base_link", "base_link")
        prop_points = _body_mesh_vertices_relative_to(model, data, "rotor_4_vis", "base_link")
        prop_center = (prop_points.min(axis=0) + prop_points.max(axis=0)) / 2.0
        prop_size = prop_points.max(axis=0) - prop_points.min(axis=0)
        thrust_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "rotor_joint_thrust4")
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_rot = Rotation.from_quat(data.xquat[base_id].copy(), scalar_first=True)
        thrust_site_body = base_rot.inv().apply(data.site_xpos[thrust_site_id].copy() - data.xpos[base_id].copy())

        self.assertLess(np.linalg.norm(prop_center - thrust_site_body), 0.015)
        self.assertLess(float(prop_size[0]), 0.03)
        self.assertLess(abs(float(max(prop_size[1], prop_size[2]) - 0.15)), 0.02)

        fuselage_nose_x = float(base_points.max(axis=0)[0])
        prop_back_x = float(prop_points[:, 0].min())
        nose_to_prop_back_gap = prop_back_x - fuselage_nose_x
        self.assertGreaterEqual(nose_to_prop_back_gap, 0.001)
        self.assertLessEqual(nose_to_prop_back_gap, 0.004)

        nose_band = base_points[base_points[:, 0] > fuselage_nose_x - 0.02]
        nose_center_z = float(nose_band[:, 2].mean())
        self.assertLess(abs(float(prop_center[2] - nose_center_z)), 0.01)

    def test_advanced_plane_control_surfaces_stay_near_airframe(self) -> None:
        model, data = _load_model_and_data("advanced_plane")

        centers = {}
        for body_name in ("left_elevon", "right_elevon", "left_flap", "right_flap", "elevator", "rudder"):
            actual = _body_mesh_vertices_relative_to(model, data, body_name, "base_link")
            centers[body_name] = (actual.min(axis=0) + actual.max(axis=0)) / 2.0

        self.assertGreater(float(centers["left_elevon"][1]), 0.1)
        self.assertLess(float(centers["left_elevon"][1]), 0.25)
        self.assertLess(float(centers["right_elevon"][1]), -0.1)
        self.assertGreater(float(centers["right_elevon"][1]), -0.25)
        self.assertGreater(float(centers["left_flap"][1]), 0.03)
        self.assertLess(float(centers["left_flap"][1]), 0.15)
        self.assertLess(float(centers["right_flap"][1]), -0.03)
        self.assertGreater(float(centers["right_flap"][1]), -0.15)
        self.assertLess(float(centers["elevator"][0]), -0.1)
        self.assertLess(float(centers["rudder"][0]), -0.1)

    def test_advanced_plane_debug_markers_stay_on_semantic_mounts(self) -> None:
        model, data = _load_model_and_data("advanced_plane")

        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_rot = Rotation.from_quat(data.xquat[base_id].copy(), scalar_first=True)
        base_pos = data.xpos[base_id].copy()

        base_link_origin = base_rot.inv().apply(
            data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "base_link_origin")] - base_pos
        )
        rotor_offset = base_rot.inv().apply(
            data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "rotor_offset_4")] - base_pos
        )
        base_points = _body_mesh_vertices_relative_to(model, data, "base_link", "base_link")
        nose_x = float(base_points.max(axis=0)[0])
        nose_band = base_points[base_points[:, 0] > nose_x - 0.02]
        nose_center_z = float(nose_band[:, 2].mean())

        self.assertLess(float(base_link_origin[2]), -0.015)
        self.assertLess(abs(float(base_link_origin[1])), 0.03)
        self.assertLess(float(base_link_origin[0]), 0.02)
        self.assertGreater(float(base_link_origin[0]), -0.08)

        self.assertGreater(float(rotor_offset[0]), 0.20)
        self.assertLess(float(rotor_offset[0]), 0.205)
        self.assertLess(abs(float(rotor_offset[1])), 0.02)
        self.assertLess(abs(float(rotor_offset[2] - nose_center_z)), 0.01)

    def test_advanced_plane_collision_lowest_point_matches_landing_clearance(self) -> None:
        model, data = _load_model_and_data("advanced_plane")

        mins = []
        for geom_id in range(model.ngeom):
            geom_type = model.geom_type[geom_id]
            size = model.geom_size[geom_id].copy()
            center = data.geom_xpos[geom_id].copy()
            if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                mins.append(float(center[2] - size[2]))
            elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                mins.append(float(center[2] - size[0]))
        self.assertTrue(mins)
        self.assertLess(abs(min(mins)), 0.003)
        self.assertLess(abs(float(_body_mesh_vertices_world(model, data, "base_link")[:, 2].min())), 0.003)

    def test_standard_vtol_urdf_uses_source_visual_poses_for_elevons(self) -> None:
        root = ET.parse(_mujoco_asset_root("standard_vtol") / "standard_vtol.urdf").getroot()
        expected = {
            "left_elevon": ("-0.083716 -0.594838 -0.029", "1.5707963268 0 2.8765926536"),
            "right_elevon": ("0.558052 -0.29618 -0.029", "1.5707963268 0 -2.8765926536"),
        }
        for link_name, (xyz, rpy) in expected.items():
            link = next(link for link in root.findall("link") if link.get("name") == link_name)
            origin = link.find("visual/origin")
            assert origin is not None
            _assert_pose_text_equal(self, origin.get("xyz"), xyz)
            _assert_pose_text_equal(self, origin.get("rpy"), rpy)

    def test_standard_vtol_elevons_do_not_escape_the_wing_slots(self) -> None:
        model, data = _load_model_and_data("standard_vtol")

        expected_ranges = {
            "left_elevon": ((-0.35, -0.14), (0.48, 1.0)),
            "right_elevon": ((-0.35, -0.14), (-1.0, -0.48)),
        }
        for body_name in ("left_elevon", "right_elevon"):
            points = _body_mesh_vertices_relative_to(model, data, body_name, "base_link")
            bbox = _projected_bbox(points, dims=(0, 1))
            (x_min, x_max), (y_min, y_max) = expected_ranges[body_name]
            self.assertGreaterEqual(float(bbox[0][0]), x_min, body_name)
            self.assertLessEqual(float(bbox[1][0]), x_max, body_name)
            self.assertGreaterEqual(float(bbox[0][1]), y_min, body_name)
            self.assertLessEqual(float(bbox[1][1]), y_max, body_name)

    def test_standard_vtol_rear_prop_keeps_body_x_disc_semantics(self) -> None:
        root = ET.parse(_mujoco_asset_root("standard_vtol") / "standard_vtol.urdf").getroot()
        rotor_4_joint = next(joint for joint in root.findall("joint") if joint.get("name") == "rotor_4_joint")
        axis = rotor_4_joint.find("axis")
        assert axis is not None
        self.assertEqual(axis.get("xyz"), "1 0 0")

        model, data = _load_model_and_data("standard_vtol")
        rear_points = _body_mesh_vertices_relative_to(model, data, "rotor_4_vis", "base_link")
        lift_points = _body_mesh_vertices_relative_to(model, data, "rotor_0_vis", "base_link")
        rear_eigvals, rear_eigvecs = _principal_axes(rear_points)
        lift_eigvals, lift_eigvecs = _principal_axes(lift_points)
        rear_thin_axis = rear_eigvecs[:, int(np.argmin(rear_eigvals))]
        rear_long_axis = rear_eigvecs[:, int(np.argmax(rear_eigvals))]
        lift_thin_axis = lift_eigvecs[:, int(np.argmin(lift_eigvals))]

        self.assertGreater(abs(float(rear_thin_axis[0])), 0.95)
        self.assertLess(abs(float(rear_long_axis[0])), 0.2)
        self.assertGreater(abs(float(lift_thin_axis[2])), 0.95)
        self.assertLess(abs(float(np.dot(rear_thin_axis, lift_thin_axis))), 0.2)

        geom_id = int(model.body_geomadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rotor_4_vis")])
        geom_rot = Rotation.from_quat(model.geom_quat[geom_id].copy(), scalar_first=True)
        corrected_spin_axis = geom_rot.inv().apply(np.array([1.0, 0.0, 0.0], dtype=float))
        corrected_spin_axis /= np.linalg.norm(corrected_spin_axis)
        self.assertGreater(abs(float(corrected_spin_axis[2])), 0.95)

    def test_standard_vtol_rudder_motion_follows_body_roll_axis(self) -> None:
        model, data = _load_model_and_data("standard_vtol")
        rudder_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rudder_joint")
        qpos_adr = int(model.jnt_qposadr[rudder_joint_id])
        data.qpos[qpos_adr] = 0.005
        mujoco.mj_forward(model, data)

        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        rudder_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rudder")
        base_rot = Rotation.from_quat(data.xquat[base_id].copy(), scalar_first=True)
        rudder_rot = Rotation.from_quat(data.xquat[rudder_id].copy(), scalar_first=True)
        delta_rot = (base_rot.inv() * rudder_rot).as_rotvec()
        axis = delta_rot / np.linalg.norm(delta_rot)
        self.assertGreater(abs(float(axis[0])), 0.95)

    def test_uuv_home_height_and_thruster_mounts_follow_source_layout(self) -> None:
        root = ET.parse(_mujoco_asset_root("uuv_bluerov2_heavy") / "uuv_bluerov2_heavy.urdf").getroot()
        expected_joint_xyz = {
            "rotor_0_joint": "0.14 -0.1 0",
            "rotor_1_joint": "0.14 0.1 0",
            "rotor_2_joint": "-0.14 -0.1 0",
            "rotor_3_joint": "-0.14 0.1 0",
        }
        for joint_name, xyz in expected_joint_xyz.items():
            joint = next(joint for joint in root.findall("joint") if joint.get("name") == joint_name)
            origin = joint.find("origin")
            assert origin is not None
            self.assertEqual(origin.get("xyz"), xyz)

        mjcf_root = ET.parse(_mujoco_asset_root("uuv_bluerov2_heavy") / "uuv_bluerov2_heavy.xml").getroot()
        home = mjcf_root.find(".//keyframe/key[@name='home']")
        assert home is not None
        qpos = [float(value) for value in home.get("qpos", "").split()]
        self.assertLess(qpos[2], 0.2)

    def test_uuv_upper_thrusters_sit_slightly_above_the_frame(self) -> None:
        model, data = _load_model_and_data("uuv_bluerov2_heavy")
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_z = float(data.xpos[base_id][2])
        for rotor_index in range(4, 8):
            rotor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}_vis")
            self.assertGreater(float(data.xpos[rotor_id][2] - base_z), 0.07)

    def test_standard_vtol_uses_forward_puller_and_no_tilt_actuators(self) -> None:
        root = ET.parse(_mujoco_asset_root("standard_vtol") / "standard_vtol.xml").getroot()
        actuator_names = {elem.get("name", "") for elem in root.findall(".//actuator/*")}
        self.assertNotIn("motor_0_tilt_ctrl", actuator_names)
        self.assertNotIn("motor_2_tilt_ctrl", actuator_names)
        self.assertIn("elevator_ctrl", actuator_names)

        rotor_offsets = {
            site.get("name"): np.asarray([float(value) for value in site.get("pos", "0 0 0").split()], dtype=float)
            for site in root.findall(".//worldbody//site")
            if site.get("name", "").startswith("rotor_offset_")
        }
        self.assertGreater(rotor_offsets["rotor_offset_0"][0], 0.0)
        self.assertGreater(rotor_offsets["rotor_offset_2"][0], 0.0)
        self.assertLess(rotor_offsets["rotor_offset_4"][0], 0.0)

    def test_uuv_vertical_thrusters_match_px4_classic_layout(self) -> None:
        root = ET.parse(_mujoco_asset_root("uuv_bluerov2_heavy") / "uuv_bluerov2_heavy.xml").getroot()
        rotor_offsets = {
            site.get("name"): np.asarray([float(value) for value in site.get("pos", "0 0 0").split()], dtype=float)
            for site in root.findall(".//worldbody//site")
            if site.get("name", "").startswith("rotor_offset_")
        }
        for rotor_index in range(4, 8):
            self.assertGreater(rotor_offsets[f"rotor_offset_{rotor_index}"][2], 0.0)


if __name__ == "__main__":
    unittest.main()
