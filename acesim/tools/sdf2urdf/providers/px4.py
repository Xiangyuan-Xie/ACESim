from __future__ import annotations

"""PX4-backed SDF provider for the stage-1 SDF -> URDF synchronization step."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from acesim.tools.sdf2urdf.contracts import (
    SDFInertialTruth,
    SDFJointTruth,
    SDFModelTruth,
    SDFSourceProvider,
    SDFVisualTruth,
)

from ..asset_context import AssetPaths, AssetToolchainConfig
from ..mesh_transforms import export_transformed_mesh_from_path
from ..xml_formatting import indent_xml, sort_attributes

PX4_SDF_ROOT = (
    Path(__file__).resolve().parents[3]
    / "third_party"
    / "aircraft"
    / "PX4-Autopilot"
    / "Tools"
    / "simulation"
    / "gazebo-classic"
    / "sitl_gazebo-classic"
    / "models"
)

PX4_SDF_MODELS = {
    "advanced_plane": "plane",
    "standard_vtol": "standard_vtol",
    "uuv_bluerov2_heavy": "uuv_bluerov2_heavy",
}

ADVANCED_PLANE_SCALE = 1.6


@dataclass(frozen=True)
class PX4VisualBinding:
    urdf_link_name: str
    source_link_name: str
    dst_mesh_name: str
    mesh_scale: str


@dataclass(frozen=True)
class PX4MeshSpec:
    dst_name: str
    src_path: Path
    center_mesh: bool = False
    translation: tuple[float, float, float] | None = None
    rotation_rpy: tuple[float, float, float] | None = None
    scale_xyz: tuple[float, float, float] | None = None


PX4_MANUAL_VISUAL_BINDINGS: dict[str, tuple[PX4VisualBinding, ...]] = {
    "advanced_plane": (
        PX4VisualBinding("base_link", "base_link", "base_link_visual_0.stl", "0.0016 0.0016 0.0016"),
        PX4VisualBinding("rotor_4", "rotor_puller", "rotor_4_vis.stl", "0.52 0.52 0.52"),
        PX4VisualBinding("left_elevon", "left_elevon", "left_elevon_visual_0.stl", "0.0016 0.0016 0.0016"),
        PX4VisualBinding("right_elevon", "right_elevon", "right_elevon_visual_0.stl", "0.0016 0.0016 0.0016"),
        PX4VisualBinding("left_flap", "left_flap", "left_flap_visual_0.stl", "0.0016 0.0016 0.0016"),
        PX4VisualBinding("right_flap", "right_flap", "right_flap_visual_0.stl", "0.0016 0.0016 0.0016"),
        PX4VisualBinding("elevator", "elevator", "elevator_visual_0.stl", "0.0016 0.0016 0.0016"),
        PX4VisualBinding("rudder", "rudder", "rudder_visual_0.stl", "0.0016 0.0016 0.0016"),
    ),
    "standard_vtol": (
        PX4VisualBinding("base_link", "base_link", "base_link_visual_0.stl", "0.001 0.001 0.001"),
        PX4VisualBinding("rotor_0", "rotor_0", "rotor_0_vis.stl", "1 1 1"),
        PX4VisualBinding("rotor_1", "rotor_1", "rotor_1_vis.stl", "1 1 1"),
        PX4VisualBinding("rotor_2", "rotor_2", "rotor_2_vis.stl", "1 1 1"),
        PX4VisualBinding("rotor_3", "rotor_3", "rotor_3_vis.stl", "1 1 1"),
        PX4VisualBinding("rotor_4", "rotor_puller", "rotor_4_vis.stl", "0.8 0.8 0.8"),
        PX4VisualBinding("left_elevon", "left_elevon", "left_elevon_visual_0.stl", "0.001 0.001 0.001"),
        PX4VisualBinding("right_elevon", "right_elevon", "right_elevon_visual_0.stl", "0.001 0.001 0.001"),
    ),
}

PX4_MANUAL_JOINT_BINDINGS: dict[str, dict[str, str]] = {
    "advanced_plane": {
        "rotor_4_joint": "rotor_puller_joint",
        "left_elevon_joint": "left_elevon_joint",
        "right_elevon_joint": "right_elevon_joint",
        "left_flap_joint": "left_flap_joint",
        "right_flap_joint": "right_flap_joint",
        "elevator_joint": "elevator_joint",
        "rudder_joint": "rudder_joint",
    },
    "standard_vtol": {
        "rotor_0_joint": "rotor_0_joint",
        "rotor_1_joint": "rotor_1_joint",
        "rotor_2_joint": "rotor_2_joint",
        "rotor_3_joint": "rotor_3_joint",
        "rotor_4_joint": "rotor_puller_joint",
        "left_elevon_joint": "left_elevon_joint",
        "right_elevon_joint": "right_elevon_joint",
        "elevator_joint": "elevator_joint",
        "rudder_joint": "rudder_joint",
    },
}

PX4_MANUAL_MESHES: dict[str, tuple[PX4MeshSpec, ...]] = {
    "advanced_plane": (
        PX4MeshSpec("base_link_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "body.dae"),
        PX4MeshSpec("rotor_4_vis.stl", PX4_SDF_ROOT / "plane" / "meshes" / "iris_prop_ccw.dae"),
        PX4MeshSpec("left_elevon_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "left_aileron.dae"),
        PX4MeshSpec("right_elevon_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "right_aileron.dae"),
        PX4MeshSpec("left_flap_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "left_flap.dae"),
        PX4MeshSpec("right_flap_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "right_flap.dae"),
        PX4MeshSpec("elevator_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "elevators.dae"),
        PX4MeshSpec("rudder_visual_0.stl", PX4_SDF_ROOT / "plane" / "meshes" / "rudder.dae"),
    ),
    "standard_vtol": (
        PX4MeshSpec("base_link_visual_0.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "x8_wing.dae"),
        PX4MeshSpec("rotor_0_vis.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "iris_prop_ccw.dae"),
        PX4MeshSpec("rotor_1_vis.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "iris_prop_ccw.dae"),
        PX4MeshSpec("rotor_2_vis.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "iris_prop_cw.dae"),
        PX4MeshSpec("rotor_3_vis.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "iris_prop_cw.dae"),
        PX4MeshSpec("rotor_4_vis.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "iris_prop_ccw.dae"),
        PX4MeshSpec("left_elevon_visual_0.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "x8_elevon_left.dae"),
        PX4MeshSpec("right_elevon_visual_0.stl", PX4_SDF_ROOT / "standard_vtol" / "meshes" / "x8_elevon_right.dae"),
    ),
    "uuv_bluerov2_heavy": (
        PX4MeshSpec("base_link_visual_0.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "BlueROV2heavy.dae"),
        PX4MeshSpec("rotor_0_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_1_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_2_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_3_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_4_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_5_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_6_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
        PX4MeshSpec("rotor_7_vis.stl", PX4_SDF_ROOT / "uuv_bluerov2_heavy" / "meshes" / "prop.dae"),
    ),
}


def _split_pose(pose_text: str | None) -> tuple[str | None, str | None]:
    if pose_text is None:
        return None, None
    values = pose_text.split()
    if len(values) != 6:
        raise ValueError(f"Expected 6 pose values, got {pose_text!r}")
    return " ".join(values[:3]), " ".join(values[3:])


def _ensure_child(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def _pose_arrays(xyz: str | None, rpy: str | None) -> tuple[np.ndarray, Rotation]:
    xyz_values = np.asarray([float(v) for v in (xyz or "0 0 0").split()], dtype=float)
    rpy_values = [float(v) for v in (rpy or "0 0 0").split()]
    return xyz_values, Rotation.from_euler("xyz", rpy_values)


def _format_pose_vector(values: np.ndarray) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def _requires_local_visual_conversion(target: str, source_link_name: str) -> bool:
    return target == "advanced_plane" or (
        target == "standard_vtol" and source_link_name in {"left_elevon", "right_elevon"}
    )


def px4_sdf_path_for_target(target: str) -> Path:
    model_name = PX4_SDF_MODELS[target]
    suffix = f"{model_name}.sdf.jinja"
    return (PX4_SDF_ROOT / model_name / suffix).resolve()


def load_px4_sdf_truth(target: str) -> SDFModelTruth:
    root = ET.parse(px4_sdf_path_for_target(target)).getroot()

    visuals: dict[str, SDFVisualTruth] = {}
    inertials: dict[str, SDFInertialTruth] = {}
    for link in root.findall(".//link"):
        link_name = link.get("name")
        if not link_name:
            continue
        for visual in link.findall("visual"):
            uri = visual.findtext("geometry/mesh/uri")
            if uri is None:
                continue
            xyz, rpy = _split_pose(visual.findtext("pose", "0 0 0 0 0 0"))
            assert xyz is not None and rpy is not None
            visuals[link_name] = SDFVisualTruth(xyz=xyz, rpy=rpy, uri=uri)
            break
        inertial = link.find("inertial")
        inertia = None if inertial is None else inertial.find("inertia")
        if inertial is not None and inertia is not None:
            inertials[link_name] = SDFInertialTruth(
                mass=float(inertial.findtext("mass", "0")),
                ixx=float(inertia.findtext("ixx", "0")),
                ixy=float(inertia.findtext("ixy", "0")),
                ixz=float(inertia.findtext("ixz", "0")),
                iyy=float(inertia.findtext("iyy", "0")),
                iyz=float(inertia.findtext("iyz", "0")),
                izz=float(inertia.findtext("izz", "0")),
            )

    joints: dict[str, SDFJointTruth] = {}
    for joint in root.findall(".//joint"):
        joint_name = joint.get("name")
        if not joint_name:
            continue
        xyz, rpy = _split_pose(joint.findtext("pose"))
        joints[joint_name] = SDFJointTruth(xyz=xyz, rpy=rpy, axis_xyz=joint.findtext("axis/xyz"))

    return SDFModelTruth(visuals=visuals, joints=joints, inertials=inertials)


def generate_px4_manual_meshes(config: AssetToolchainConfig, paths: AssetPaths) -> None:
    specs = PX4_MANUAL_MESHES.get(config.target)
    if specs is None:
        return

    paths.mesh_dir.mkdir(parents=True, exist_ok=True)
    for spec in specs:
        export_transformed_mesh_from_path(
            spec.src_path,
            paths.mesh_dir / spec.dst_name,
            center_mesh=spec.center_mesh,
            translation=list(spec.translation) if spec.translation is not None else None,
            rotation_rpy=list(spec.rotation_rpy) if spec.rotation_rpy is not None else None,
            scale_xyz=list(spec.scale_xyz) if spec.scale_xyz is not None else None,
        )


def cleanup_px4_manual_meshes(config: AssetToolchainConfig, paths: AssetPaths) -> None:
    specs = PX4_MANUAL_MESHES.get(config.target)
    if specs is None or not paths.mesh_dir.exists():
        return

    keep = {spec.dst_name for spec in specs}
    for file_path in paths.mesh_dir.iterdir():
        if file_path.is_file() and file_path.name not in keep:
            file_path.unlink()


def sync_px4_manual_urdf(config: AssetToolchainConfig, paths: AssetPaths) -> None:
    """Synchronize PX4 SDF truth into the checked-in URDF for a manual asset.

    This stage intentionally stops at URDF. Any MuJoCo-specific runtime
    semantics, such as mocap rotor visuals or actuator wiring, are handled by
    the later URDF -> MJCF stage.
    """

    visual_bindings = PX4_MANUAL_VISUAL_BINDINGS.get(config.target)
    joint_bindings = PX4_MANUAL_JOINT_BINDINGS.get(config.target)
    if visual_bindings is None or joint_bindings is None:
        return

    truth = load_px4_sdf_truth(config.target)
    visuals = truth.visuals
    joints = truth.joints
    tree = ET.parse(paths.urdf_path)
    root = tree.getroot()
    advanced_plane_nose_x: float | None = None
    advanced_plane_nose_center_z: float | None = None
    advanced_plane_collision_center_z: float | None = None
    advanced_plane_collision_size: np.ndarray | None = None
    advanced_plane_prop_half_thickness_x: float | None = None
    advanced_plane_prop_center_z_offset: float | None = None
    advanced_plane_prop_collision_radius: float | None = None
    advanced_plane_source_inertial = truth.inertials.get("base_link")
    if config.target == "advanced_plane":
        source_root = ET.parse(px4_sdf_path_for_target(config.target)).getroot()
        source_base_link = next(link for link in source_root.findall(".//link") if link.get("name") == "base_link")
        source_collision = source_base_link.find("collision")
        if source_collision is None:
            raise ValueError("Missing advanced_plane source collision in SDF")
        source_collision_size = np.asarray(
            [float(v) for v in source_collision.findtext("geometry/box/size", "0 0 0").split()],
            dtype=float,
        )
        base_mesh = trimesh.load(paths.mesh_dir / "base_link_visual_0.stl", force="mesh")
        if isinstance(base_mesh, trimesh.Scene):
            base_mesh = base_mesh.to_geometry()
        base_scale = np.asarray([0.001, 0.001, 0.001], dtype=float) * ADVANCED_PLANE_SCALE
        base_bounds = base_mesh.bounds * base_scale
        base_visual_xyz, _ = _pose_arrays(visuals["base_link"].xyz, visuals["base_link"].rpy)
        base_visual_xyz = base_visual_xyz * ADVANCED_PLANE_SCALE
        base_bounds = base_bounds + base_visual_xyz
        advanced_plane_nose_x = float(base_bounds[1][0])
        scaled_base_vertices = base_mesh.vertices * base_scale + base_visual_xyz
        nose_band = scaled_base_vertices[scaled_base_vertices[:, 0] > advanced_plane_nose_x - 0.02]
        advanced_plane_nose_center_z = float(nose_band[:, 2].mean())
        base_link = next((elem for elem in root.findall("link") if elem.get("name") == "base_link"), None)
        if base_link is None:
            raise ValueError("Missing advanced_plane base_link in URDF")
        collision = base_link.find("collision")
        if collision is None:
            raise ValueError("Missing advanced_plane base_link collision in URDF")
        box = collision.find("geometry/box")
        if box is None:
            raise ValueError("Missing advanced_plane base_link collision box in URDF")
        advanced_plane_collision_size = source_collision_size * ADVANCED_PLANE_SCALE
        half_size_z = float(advanced_plane_collision_size[2]) * 0.5
        advanced_plane_collision_center_z = float(base_bounds[0][2] + half_size_z)
        prop_mesh = trimesh.load(paths.mesh_dir / "rotor_4_vis.stl", force="mesh")
        if isinstance(prop_mesh, trimesh.Scene):
            prop_mesh = prop_mesh.to_geometry()
        prop_scale = np.asarray(
            [
                float(v)
                for v in next(
                    binding for binding in visual_bindings if binding.urdf_link_name == "rotor_4"
                ).mesh_scale.split()
            ],
            dtype=float,
        )
        source_joint = joints["rotor_puller_joint"]
        if source_joint.rpy is None:
            rotor_joint = next((elem for elem in root.findall("joint") if elem.get("name") == "rotor_4_joint"), None)
            if rotor_joint is None:
                raise ValueError("Missing advanced_plane rotor_4_joint in URDF")
            rotor_origin = rotor_joint.find("origin")
            if rotor_origin is None:
                raise ValueError("Missing advanced_plane rotor_4_joint origin in URDF")
            _, prop_joint_rot = _pose_arrays(rotor_origin.get("xyz"), rotor_origin.get("rpy"))
        else:
            _, prop_joint_rot = _pose_arrays(source_joint.xyz, source_joint.rpy)
        prop_vertices_body = prop_joint_rot.apply(prop_mesh.vertices * prop_scale)
        advanced_plane_prop_half_thickness_x = float(abs(prop_vertices_body[:, 0].min()))
        advanced_plane_prop_center_z_offset = float(prop_vertices_body.mean(axis=0)[2])
        prop_yz_span = prop_vertices_body.max(axis=0) - prop_vertices_body.min(axis=0)
        advanced_plane_prop_collision_radius = 0.5 * max(float(prop_yz_span[1]), float(prop_yz_span[2]))

        rotor_link = next((elem for elem in root.findall("link") if elem.get("name") == "rotor_4"), None)
        if rotor_link is None:
            raise ValueError("Missing advanced_plane rotor_4 link in URDF")
        rotor_collision = rotor_link.find("collision")
        if rotor_collision is None:
            raise ValueError("Missing advanced_plane rotor_4 collision in URDF")
        rotor_cylinder = rotor_collision.find("geometry/cylinder")
        if rotor_cylinder is None:
            raise ValueError("Missing advanced_plane rotor_4 collision cylinder in URDF")
        rotor_cylinder.set("radius", f"{advanced_plane_prop_collision_radius:.9g}")

    for binding in visual_bindings:
        source_visual = visuals.get(binding.source_link_name)
        if source_visual is None:
            raise ValueError(f"Missing source visual for {config.target}:{binding.source_link_name}")
        link = next((elem for elem in root.findall("link") if elem.get("name") == binding.urdf_link_name), None)
        if link is None:
            raise ValueError(f"Missing URDF link {binding.urdf_link_name} in {paths.urdf_path}")
        visual = _ensure_child(link, "visual")
        origin = _ensure_child(visual, "origin")
        visual_xyz = source_visual.xyz
        visual_rpy = source_visual.rpy
        if _requires_local_visual_conversion(
            config.target, binding.source_link_name
        ) and binding.source_link_name not in {
            "base_link",
            "rotor_puller",
        }:
            maybe_source_joint = joints.get(f"{binding.source_link_name}_joint")
            if maybe_source_joint is None or maybe_source_joint.xyz is None or maybe_source_joint.rpy is None:
                raise ValueError(f"Missing source joint pose for {config.target}:{binding.source_link_name}")
            source_joint = maybe_source_joint
            joint_xyz, joint_rot = _pose_arrays(source_joint.xyz, source_joint.rpy)
            source_visual_xyz, source_visual_rot = _pose_arrays(source_visual.xyz, source_visual.rpy)
            local_xyz = joint_rot.inv().apply(source_visual_xyz - joint_xyz)
            local_rpy = (joint_rot.inv() * source_visual_rot).as_euler("xyz")
            if config.target == "advanced_plane":
                local_xyz = local_xyz * ADVANCED_PLANE_SCALE
            visual_xyz = _format_pose_vector(local_xyz)
            visual_rpy = _format_pose_vector(local_rpy)
        elif config.target == "advanced_plane" and binding.source_link_name == "rotor_puller":
            # The puller prop is mounted from the rotor joint/site in the MJCF
            # stage, so the URDF visual stays centered on the physical axis.
            visual_xyz = "0 0 0"
            visual_rpy = "0 0 0"
        elif config.target == "advanced_plane" and binding.source_link_name == "base_link":
            base_xyz, _ = _pose_arrays(source_visual.xyz, source_visual.rpy)
            visual_xyz = _format_pose_vector(base_xyz * ADVANCED_PLANE_SCALE)
        origin.set("xyz", visual_xyz)
        origin.set("rpy", visual_rpy)
        geometry = _ensure_child(visual, "geometry")
        mesh = _ensure_child(geometry, "mesh")
        mesh.set("filename", f"package://{config.target}/meshes/{binding.dst_mesh_name}")
        mesh.set("scale", binding.mesh_scale)

    for urdf_joint_name, source_joint_name in joint_bindings.items():
        maybe_source_joint = joints.get(source_joint_name)
        if maybe_source_joint is None:
            raise ValueError(f"Missing source joint for {config.target}:{source_joint_name}")
        source_joint = maybe_source_joint
        joint = next((elem for elem in root.findall("joint") if elem.get("name") == urdf_joint_name), None)
        if joint is None:
            raise ValueError(f"Missing URDF joint {urdf_joint_name} in {paths.urdf_path}")
        origin = _ensure_child(joint, "origin")
        if config.target == "advanced_plane" and urdf_joint_name == "rotor_4_joint":
            assert advanced_plane_nose_x is not None
            assert advanced_plane_prop_half_thickness_x is not None
            assert advanced_plane_nose_center_z is not None
            assert advanced_plane_prop_center_z_offset is not None
            target_gap = 0.003
            origin.set(
                "xyz",
                f"{advanced_plane_nose_x + target_gap + advanced_plane_prop_half_thickness_x:.9g} 0 "
                f"{advanced_plane_nose_center_z - advanced_plane_prop_center_z_offset:.9g}",
            )
        elif source_joint.xyz is not None:
            if config.target == "advanced_plane":
                joint_xyz, _ = _pose_arrays(source_joint.xyz, source_joint.rpy)
                origin.set("xyz", _format_pose_vector(joint_xyz * ADVANCED_PLANE_SCALE))
            else:
                origin.set("xyz", source_joint.xyz)
        if source_joint.rpy is not None:
            origin.set("rpy", source_joint.rpy)
        axis = _ensure_child(joint, "axis")
        if source_joint.axis_xyz is not None:
            axis.set("xyz", source_joint.axis_xyz)

    if config.target == "advanced_plane":
        base_link = next((elem for elem in root.findall("link") if elem.get("name") == "base_link"), None)
        if base_link is None:
            raise ValueError("Missing advanced_plane base_link in URDF")
        collision = base_link.find("collision")
        if collision is not None:
            origin = _ensure_child(collision, "origin")
            assert advanced_plane_collision_center_z is not None
            origin.set("xyz", f"0 0 {advanced_plane_collision_center_z:.9g}")
            origin.set("rpy", "0 0 0")
            box = collision.find("geometry/box")
            if box is not None:
                assert advanced_plane_collision_size is not None
                box.set("size", _format_pose_vector(advanced_plane_collision_size))
        inertial = base_link.find("inertial")
        if inertial is not None and advanced_plane_source_inertial is not None:
            mass = inertial.find("mass")
            inertia = inertial.find("inertia")
            if mass is not None:
                mass.set("value", f"{advanced_plane_source_inertial.mass:.9g}")
            if inertia is not None:
                inertia.set("ixx", f"{advanced_plane_source_inertial.ixx:.9g}")
                inertia.set("ixy", f"{advanced_plane_source_inertial.ixy:.9g}")
                inertia.set("ixz", f"{advanced_plane_source_inertial.ixz:.9g}")
                inertia.set("iyy", f"{advanced_plane_source_inertial.iyy:.9g}")
                inertia.set("iyz", f"{advanced_plane_source_inertial.iyz:.9g}")
                inertia.set("izz", f"{advanced_plane_source_inertial.izz:.9g}")

    sort_attributes(root)
    indent_xml(root)
    tree.write(paths.urdf_path, encoding="utf-8", xml_declaration=True)


class PX4Provider:
    name = "px4"

    def sdf_path_for_target(self, target: str) -> Path:
        return px4_sdf_path_for_target(target)

    def load_truth(self, target: str) -> SDFModelTruth:
        return load_px4_sdf_truth(target)

    def generate_manual_meshes(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        generate_px4_manual_meshes(config, paths)

    def cleanup_manual_meshes(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        cleanup_px4_manual_meshes(config, paths)

    def sync_manual_urdf(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        sync_px4_manual_urdf(config, paths)


PX4_PROVIDER: SDFSourceProvider = PX4Provider()
