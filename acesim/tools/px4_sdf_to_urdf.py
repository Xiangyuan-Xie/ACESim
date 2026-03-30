"""Import selected PX4 Gazebo SDF assets into ACESim URDF/MJCF assets."""

from __future__ import annotations

import argparse
import math
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, TypeAlias

import numpy as np
import sdformat14 as sdf
from scipy.spatial.transform import Rotation

from acesim.tools.urdf2mjcf.compiler import compile_urdf_to_xml, find_mujoco_compile_binary
from acesim.tools.urdf2mjcf.config import ConverterConfig, ConverterPaths
from acesim.tools.urdf2mjcf.mesh_ops import load_trimesh_geometry
from acesim.tools.urdf2mjcf.mjcf_ops import postprocess_xml
from acesim.tools.urdf2mjcf.urdf_ops import calculate_min_z, parse_q0, preprocess_urdf
from acesim.tools.urdf2mjcf.xml_utils import indent_xml

ACESIM_ROOT = Path(__file__).resolve().parents[1]
PX4_GZ_MODELS_ROOT = (
    ACESIM_ROOT / "third_party" / "aircraft" / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models"
)
DEFAULT_ASSET_ROOT = ACESIM_ROOT / "env" / "mujoco" / "asset"


class _Pose3Like(Protocol):
    def x(self) -> float: ...
    def y(self) -> float: ...
    def z(self) -> float: ...
    def roll(self) -> float: ...
    def pitch(self) -> float: ...
    def yaw(self) -> float: ...


class _Vector3Like(Protocol):
    def x(self) -> float: ...
    def y(self) -> float: ...
    def z(self) -> float: ...


class _ColorLike(Protocol):
    def r(self) -> float: ...
    def g(self) -> float: ...
    def b(self) -> float: ...
    def a(self) -> float: ...


class _MeshShapeLike(Protocol):
    def uri(self) -> str: ...
    def scale(self) -> _Vector3Like: ...


class _BoxShapeLike(Protocol):
    def size(self) -> _Vector3Like: ...


class _CylinderShapeLike(Protocol):
    def radius(self) -> float: ...
    def length(self) -> float: ...


class _GeometryLike(Protocol):
    def type(self) -> object: ...
    def mesh_shape(self) -> _MeshShapeLike: ...
    def box_shape(self) -> _BoxShapeLike: ...
    def cylinder_shape(self) -> _CylinderShapeLike: ...


class _MaterialLike(Protocol):
    def diffuse(self) -> _ColorLike: ...
    def ambient(self) -> _ColorLike: ...


class _VisualLike(Protocol):
    def raw_pose(self) -> _Pose3Like: ...
    def geometry(self) -> _GeometryLike: ...
    def material(self) -> _MaterialLike | None: ...


class _CollisionLike(Protocol):
    def raw_pose(self) -> _Pose3Like: ...
    def geometry(self) -> _GeometryLike: ...


class _MassMatrixLike(Protocol):
    def mass(self) -> float: ...
    def ixx(self) -> float: ...
    def ixy(self) -> float: ...
    def ixz(self) -> float: ...
    def iyy(self) -> float: ...
    def iyz(self) -> float: ...
    def izz(self) -> float: ...


class _InertialLike(Protocol):
    def mass_matrix(self) -> _MassMatrixLike: ...
    def pose(self) -> _Pose3Like: ...


class _SemanticPoseLike(Protocol):
    def resolve(self, frame_name: str) -> _Pose3Like: ...


class _JointAxisLike(Protocol):
    def xyz(self) -> _Vector3Like: ...
    def resolve_xyz(self, frame_name: str) -> _Vector3Like: ...
    def lower(self) -> float: ...
    def upper(self) -> float: ...
    def effort(self) -> float: ...
    def max_velocity(self) -> float: ...
    def damping(self) -> float: ...
    def friction(self) -> float: ...


class _LinkLike(Protocol):
    def name(self) -> str: ...
    def inertial(self) -> _InertialLike: ...
    def visual_count(self) -> int: ...
    def visual_by_index(self, index: int) -> _VisualLike: ...
    def collision_count(self) -> int: ...
    def collision_by_index(self, index: int) -> _CollisionLike: ...
    def semantic_pose(self) -> _SemanticPoseLike: ...


class _JointLike(Protocol):
    def type(self) -> object: ...
    def axis(self, index: int) -> _JointAxisLike: ...
    def child_name(self) -> str: ...
    def parent_name(self) -> str: ...
    def name(self) -> str: ...
    def semantic_pose(self) -> _SemanticPoseLike: ...


class _ModelLike(Protocol):
    def link_by_index(self, index: int) -> _LinkLike: ...
    def link_count(self) -> int: ...
    def joint_count(self) -> int: ...
    def joint_by_index(self, index: int) -> _JointLike: ...
    def link_by_name(self, name: str) -> _LinkLike | None: ...


ModelLike: TypeAlias = _ModelLike


@dataclass(frozen=True)
class PX4ImportSpec:
    target: str
    source_model: str
    link_renames: dict[str, str]
    joint_renames: dict[str, str]
    strip_link_names: tuple[str, ...] = ()
    strip_joint_names: tuple[str, ...] = ()
    strip_include_uris: tuple[str, ...] = ()
    preserve_joint_frame_names: tuple[str, ...] = ()

    def rename_link(self, name: str) -> str:
        return self.link_renames.get(name, name)

    def rename_joint(self, name: str) -> str:
        return self.joint_renames.get(name, name)


@dataclass(frozen=True)
class GeneratedAssetPaths:
    asset_dir: Path
    mesh_dir: Path
    urdf_path: Path
    xml_path: Path


@dataclass(frozen=True)
class MeshFrameRule:
    origin_xyz: list[float] | None = None
    origin_rpy: list[float] | None = None
    export_mode: str = "preserve_visual_pose"
    removal_mode: str = "full"
    center_mesh: bool = False
    removal_uses_output_pose: bool = False


IMPORT_SPECS: dict[str, PX4ImportSpec] = {
    "plane": PX4ImportSpec(
        target="plane",
        source_model="advanced_plane",
        link_renames={"rotor_puller": "rotor_4"},
        joint_renames={
            "servo_0": "left_elevon_joint",
            "servo_1": "right_elevon_joint",
            "servo_2": "elevator_joint",
            "servo_3": "rudder_joint",
            "servo_4": "left_flap_joint",
            "servo_5": "right_flap_joint",
            "rotor_puller_joint": "rotor_4_joint",
        },
        strip_link_names=("lidar_sensor_link",),
        strip_joint_names=("lidar_model_joint", "lidar_sensor_joint"),
        strip_include_uris=("model://LW20",),
    ),
    "standard_vtol": PX4ImportSpec(
        target="standard_vtol",
        source_model="standard_vtol",
        link_renames={"rotor_puller": "rotor_4"},
        joint_renames={
            "servo_0": "left_elevon_joint",
            "servo_1": "right_elevon_joint",
            "servo_2": "elevator_joint",
            "rotor_puller_joint": "rotor_4_joint",
        },
        preserve_joint_frame_names=("left_elevon_joint", "right_elevon_joint"),
    ),
    "uuv_bluerov2_heavy": PX4ImportSpec(
        target="uuv_bluerov2_heavy",
        source_model="uuv_bluerov2_heavy",
        link_renames={f"thruster{i}": f"rotor_{i}" for i in range(8)},
        joint_renames={f"thruster{i}_joint": f"rotor_{i}_joint" for i in range(8)},
        preserve_joint_frame_names=tuple(f"rotor_{i}_joint" for i in range(8)),
    ),
}

VISUAL_FRAME_RULES: dict[str, dict[str, MeshFrameRule]] = {
    "plane": {
        "base_link": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="none"),
        "rotor_4": MeshFrameRule(
            origin_xyz=[0.0, 0.0, -0.09],
            origin_rpy=[0.0, 1.57079633, 0.0],
            export_mode="body_aligned_visual",
            removal_mode="visual",
            center_mesh=True,
            removal_uses_output_pose=True,
        ),
        "left_elevon": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="link"),
        "right_elevon": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="link"),
        "left_flap": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="link"),
        "right_flap": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="link"),
        "elevator": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="link"),
        "rudder": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="link"),
    },
    "standard_vtol": {
        "base_link": MeshFrameRule(export_mode="preserve_visual_pose", removal_mode="none"),
        "rotor_0": MeshFrameRule(export_mode="body_aligned_visual", center_mesh=True, removal_mode="visual"),
        "rotor_1": MeshFrameRule(export_mode="body_aligned_visual", center_mesh=True, removal_mode="visual"),
        "rotor_2": MeshFrameRule(export_mode="body_aligned_visual", center_mesh=True, removal_mode="visual"),
        "rotor_3": MeshFrameRule(export_mode="body_aligned_visual", center_mesh=True, removal_mode="visual"),
        "rotor_4": MeshFrameRule(
            origin_xyz=[0.0, 0.0, -0.04],
            origin_rpy=[0.0, 1.57079633, 0.0],
            export_mode="body_aligned_visual",
            removal_mode="visual",
            center_mesh=True,
            removal_uses_output_pose=True,
        ),
        "left_elevon": MeshFrameRule(
            origin_xyz=[-0.105, 0.004, -0.034],
            origin_rpy=[0.0, 0.0, 0.0],
            export_mode="body_aligned_visual",
            removal_mode="visual_then_link_translation",
            center_mesh=True,
        ),
        "right_elevon": MeshFrameRule(
            origin_xyz=[-0.105, -0.004, -0.034],
            origin_rpy=[0.0, 0.0, 0.0],
            export_mode="body_aligned_visual",
            removal_mode="visual_then_link_translation",
            center_mesh=True,
        ),
    },
    "uuv_bluerov2_heavy": {
        "rotor_0": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_1": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_2": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_3": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_4": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_5": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_6": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
        "rotor_7": MeshFrameRule(
            origin_xyz=[0.0, 0.0, 0.006434], export_mode="preserve_visual_pose", removal_mode="none"
        ),
    },
}


def _format_float(value: float) -> str:
    return f"{float(value):.9g}"


def _format_vec(values: list[float] | tuple[float, ...]) -> str:
    return " ".join(_format_float(v) for v in values)


def _pose_xyz_rpy(pose: _Pose3Like) -> tuple[list[float], list[float]]:
    xyz = [float(pose.x()), float(pose.y()), float(pose.z())]
    rpy = [float(pose.roll()), float(pose.pitch()), float(pose.yaw())]
    return xyz, rpy


def _pose_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_euler("xyz", np.asarray(rpy, dtype=float)).as_matrix()
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def _vector3_to_list(vec: _Vector3Like) -> list[float]:
    return [float(vec.x()), float(vec.y()), float(vec.z())]


def _color_rgba(color: _ColorLike) -> list[float]:
    return [float(color.r()), float(color.g()), float(color.b()), float(color.a())]


def _nonzero(values: list[float], eps: float = 1e-12) -> bool:
    return any(abs(v) > eps for v in values)


def _mesh_output_name(link_name: str, geom_kind: str, index: int) -> str:
    if geom_kind == "visual" and link_name.startswith("rotor_"):
        return f"{link_name}_vis.stl"
    return f"{link_name}_{geom_kind}_{index}.stl"


def _source_model_dir(model_name: str) -> Path:
    model_dir = PX4_GZ_MODELS_ROOT / model_name
    if not model_dir.exists():
        raise FileNotFoundError(f"PX4 Gazebo model directory not found: {model_dir}")
    return model_dir


def _resolve_model_uri(uri: str, source_model_dir: Path) -> Path:
    if uri.startswith("model://"):
        relative = uri[len("model://") :]
        if "/" not in relative:
            raise ValueError(f"Unsupported model URI: {uri}")
        model_name, subpath = relative.split("/", 1)
        resolved = PX4_GZ_MODELS_ROOT / model_name / subpath
    else:
        resolved = (source_model_dir / uri).resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Referenced SDF mesh does not exist: {uri} -> {resolved}")
    return resolved


def _ensure_parent_map(root: ET.Element) -> dict[ET.Element, ET.Element]:
    return {child: parent for parent in root.iter() for child in parent}


def _strip_sdf_for_import(spec: PX4ImportSpec) -> Path:
    source_model_dir = _source_model_dir(spec.source_model)
    source_sdf = source_model_dir / "model.sdf"
    if not (spec.strip_link_names or spec.strip_joint_names or spec.strip_include_uris):
        return source_sdf

    tree = ET.parse(source_sdf)
    root = tree.getroot()
    parent_map = _ensure_parent_map(root)
    for include_elem in root.findall(".//include"):
        uri_elem = include_elem.find("uri")
        if uri_elem is None or uri_elem.text is None:
            continue
        if uri_elem.text.strip() in spec.strip_include_uris:
            parent = parent_map.get(include_elem)
            if parent is not None:
                parent.remove(include_elem)

    for link_elem in root.findall(".//link"):
        if link_elem.get("name") in spec.strip_link_names:
            parent = parent_map.get(link_elem)
            if parent is not None:
                parent.remove(link_elem)

    for joint_elem in root.findall(".//joint"):
        if joint_elem.get("name") in spec.strip_joint_names:
            parent = parent_map.get(joint_elem)
            if parent is not None:
                parent.remove(joint_elem)

    handle = tempfile.NamedTemporaryFile("w", suffix=f"_{spec.target}.sdf", delete=False, encoding="utf-8")
    with handle:
        tree.write(handle, encoding="unicode", xml_declaration=True)
    return Path(handle.name)


def _append_origin(parent: ET.Element, xyz: list[float], rpy: list[float]) -> None:
    ET.SubElement(
        parent,
        "origin",
        {
            "xyz": _format_vec(xyz),
            "rpy": _format_vec(rpy),
        },
    )


def _append_material(parent: ET.Element, name: str, rgba: list[float]) -> None:
    material = ET.SubElement(parent, "material", {"name": name})
    ET.SubElement(material, "color", {"rgba": _format_vec(rgba)})


def _read_collada_unit_scale(src_path: Path) -> float:
    if src_path.suffix.lower() != ".dae":
        return 1.0
    try:
        root = ET.parse(src_path).getroot()
    except ET.ParseError:
        return 1.0

    unit_elem = root.find(".//{*}asset/{*}unit")
    if unit_elem is None:
        unit_elem = root.find(".//unit")
    if unit_elem is None:
        return 1.0

    meter_attr = unit_elem.get("meter")
    if meter_attr is None:
        return 1.0
    try:
        meter_scale = float(meter_attr)
    except ValueError:
        return 1.0
    if not math.isfinite(meter_scale) or meter_scale <= 0.0:
        return 1.0
    return meter_scale


def _export_mesh(
    src_path: Path,
    dst_path: Path,
    scale: list[float],
    *,
    removal_transform: np.ndarray | None = None,
    center_mesh: bool = False,
) -> np.ndarray:
    mesh = load_trimesh_geometry(src_path).copy()
    unit_scale = _read_collada_unit_scale(src_path)
    final_scale = [unit_scale * component for component in scale]
    if _nonzero([final_scale[0] - 1.0, final_scale[1] - 1.0, final_scale[2] - 1.0]):
        mesh.apply_scale(final_scale)

    if removal_transform is not None:
        mesh.apply_transform(np.linalg.inv(removal_transform))
    centroid_before_center = np.asarray(mesh.centroid, dtype=float)
    if center_mesh:
        mesh.apply_translation(-centroid_before_center)
    mesh.export(dst_path)
    return centroid_before_center


def _append_mesh_geometry(
    parent: ET.Element,
    mesh_uri: str,
    mesh_scale: list[float],
    source_model_dir: Path,
    mesh_output_path: Path,
    target: str,
    *,
    removal_transform: np.ndarray | None = None,
    center_mesh: bool = False,
) -> None:
    src_path = _resolve_model_uri(mesh_uri, source_model_dir)
    mesh_output_path.parent.mkdir(parents=True, exist_ok=True)
    _export_mesh(
        src_path,
        mesh_output_path,
        mesh_scale,
        removal_transform=removal_transform,
        center_mesh=center_mesh,
    )
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": f"package://{target}/meshes/{mesh_output_path.name}"})


def _append_exported_mesh_geometry(parent: ET.Element, *, target: str, mesh_output_path: Path) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": f"package://{target}/meshes/{mesh_output_path.name}"})


def _append_box_geometry(parent: ET.Element, size_vec: _Vector3Like) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(geometry, "box", {"size": _format_vec(_vector3_to_list(size_vec))})


def _append_cylinder_geometry(parent: ET.Element, radius: float, length: float) -> None:
    geometry = ET.SubElement(parent, "geometry")
    ET.SubElement(
        geometry,
        "cylinder",
        {
            "radius": _format_float(radius),
            "length": _format_float(length),
        },
    )


def _append_geometry(
    parent: ET.Element,
    geometry: _GeometryLike,
    source_model_dir: Path,
    mesh_output_path: Path,
    target: str,
    *,
    removal_transform: np.ndarray | None = None,
    center_mesh: bool = False,
) -> None:
    geometry_type = str(geometry.type()).split(".")[-1]
    if geometry_type == "MESH":
        mesh_shape = geometry.mesh_shape()
        _append_mesh_geometry(
            parent,
            str(mesh_shape.uri()),
            _vector3_to_list(mesh_shape.scale()),
            source_model_dir,
            mesh_output_path,
            target,
            removal_transform=removal_transform,
            center_mesh=center_mesh,
        )
        return
    if geometry_type == "BOX":
        _append_box_geometry(parent, geometry.box_shape().size())
        return
    if geometry_type == "CYLINDER":
        cylinder = geometry.cylinder_shape()
        _append_cylinder_geometry(parent, float(cylinder.radius()), float(cylinder.length()))
        return
    raise ValueError(f"Unsupported SDF geometry type for URDF export: {geometry_type}")


def _append_inertial(link_elem: ET.Element, link: _LinkLike) -> None:
    inertial = link.inertial()
    mass_matrix = inertial.mass_matrix()
    inertial_elem = ET.SubElement(link_elem, "inertial")
    xyz, rpy = _pose_xyz_rpy(inertial.pose())
    _append_origin(inertial_elem, xyz, rpy)
    ET.SubElement(inertial_elem, "mass", {"value": _format_float(float(mass_matrix.mass()))})
    ET.SubElement(
        inertial_elem,
        "inertia",
        {
            "ixx": _format_float(float(mass_matrix.ixx())),
            "ixy": _format_float(float(mass_matrix.ixy())),
            "ixz": _format_float(float(mass_matrix.ixz())),
            "iyy": _format_float(float(mass_matrix.iyy())),
            "iyz": _format_float(float(mass_matrix.iyz())),
            "izz": _format_float(float(mass_matrix.izz())),
        },
    )


def _append_visuals(
    link_elem: ET.Element,
    link: _LinkLike,
    link_name: str,
    source_model_dir: Path,
    mesh_dir: Path,
    target: str,
    link_model_transform: np.ndarray,
) -> None:
    for idx in range(link.visual_count()):
        visual = link.visual_by_index(idx)
        visual_elem = ET.SubElement(link_elem, "visual", {"name": f"{link_name}_visual_{idx}"})
        source_xyz, source_rpy = _pose_xyz_rpy(visual.raw_pose())
        xyz = source_xyz.copy()
        rpy = source_rpy.copy()
        rule = VISUAL_FRAME_RULES.get(target, {}).get(link_name)
        if rule is not None:
            if rule.origin_xyz is not None:
                xyz = rule.origin_xyz.copy()
            if rule.origin_rpy is not None:
                rpy = rule.origin_rpy.copy()
        mesh_output_path = mesh_dir / _mesh_output_name(link_name, "visual", idx)
        source_visual_tf = _pose_matrix(source_xyz, source_rpy)
        removal_visual_tf = (
            _pose_matrix(xyz, rpy) if rule is not None and rule.removal_uses_output_pose else source_visual_tf
        )
        if rule is None or rule.removal_mode == "full":
            removal_transform = link_model_transform @ removal_visual_tf
        elif rule.removal_mode == "link":
            removal_transform = link_model_transform
        elif rule.removal_mode == "visual":
            removal_transform = removal_visual_tf
        elif rule.removal_mode == "none":
            removal_transform = None
        elif rule.removal_mode == "visual_then_link_translation":
            link_translation_only = np.eye(4, dtype=float)
            link_translation_only[:3, 3] = link_model_transform[:3, 3]
            removal_transform = removal_visual_tf @ link_translation_only
        else:
            raise ValueError(f"Unsupported visual mesh frame removal mode: {rule.removal_mode}")
        geometry = visual.geometry()
        geometry_type = str(geometry.type()).split(".")[-1]
        if geometry_type == "MESH":
            mesh_shape = geometry.mesh_shape()
            src_path = _resolve_model_uri(str(mesh_shape.uri()), source_model_dir)
            mesh_output_path.parent.mkdir(parents=True, exist_ok=True)
            centroid_before_center = _export_mesh(
                src_path,
                mesh_output_path,
                _vector3_to_list(mesh_shape.scale()),
                removal_transform=removal_transform,
                center_mesh=False if rule is None else rule.center_mesh,
            )
            if rule is not None and rule.export_mode == "recenter_and_zero_visual":
                centroid_shift = Rotation.from_euler("xyz", np.asarray(rpy, dtype=float)).apply(centroid_before_center)
                xyz = [
                    xyz[0] + float(centroid_shift[0]),
                    xyz[1] + float(centroid_shift[1]),
                    xyz[2] + float(centroid_shift[2]),
                ]
            _append_origin(visual_elem, xyz, rpy)
            _append_exported_mesh_geometry(visual_elem, target=target, mesh_output_path=mesh_output_path)
        else:
            _append_origin(visual_elem, xyz, rpy)
            _append_geometry(
                visual_elem,
                geometry,
                source_model_dir,
                mesh_output_path,
                target,
                removal_transform=removal_transform,
                center_mesh=False if rule is None else rule.center_mesh,
            )
        material = visual.material()
        if material is None:
            continue
        rgba = _color_rgba(material.diffuse())
        if not _nonzero(rgba[:3]):
            rgba = _color_rgba(material.ambient())
        if _nonzero(rgba):
            _append_material(visual_elem, f"{link_name}_visual_material_{idx}", rgba)


def _append_collisions(
    link_elem: ET.Element,
    link: _LinkLike,
    link_name: str,
    source_model_dir: Path,
    mesh_dir: Path,
    target: str,
    link_model_transform: np.ndarray,
) -> None:
    for idx in range(link.collision_count()):
        collision = link.collision_by_index(idx)
        collision_elem = ET.SubElement(link_elem, "collision", {"name": f"{link_name}_collision_{idx}"})
        xyz, rpy = _pose_xyz_rpy(collision.raw_pose())
        _append_origin(collision_elem, xyz, rpy)
        mesh_output_path = mesh_dir / _mesh_output_name(link_name, "collision", idx)
        collision_tf = _pose_matrix(xyz, rpy)
        _append_geometry(
            collision_elem,
            collision.geometry(),
            source_model_dir,
            mesh_output_path,
            target,
            removal_transform=link_model_transform @ collision_tf,
        )


def _build_link_model_transforms(model: _ModelLike, spec: PX4ImportSpec) -> dict[str, np.ndarray]:
    transforms: dict[str, np.ndarray] = {}
    children: dict[str, list[tuple[str, np.ndarray]]] = {}
    child_links: set[str] = set()
    source_link_names_by_target = {
        spec.rename_link(model.link_by_index(i).name()): model.link_by_index(i).name()
        for i in range(model.link_count())
    }
    for index in range(model.joint_count()):
        joint = model.joint_by_index(index)
        parent_name = spec.rename_link(joint.parent_name())
        child_name = spec.rename_link(joint.child_name())
        joint_xyz, joint_rpy = _pose_xyz_rpy(joint.semantic_pose().resolve(parent_name))
        children.setdefault(parent_name, []).append((child_name, _pose_matrix(joint_xyz, joint_rpy)))
        child_links.add(child_name)

    root_links = [spec.rename_link(model.link_by_index(i).name()) for i in range(model.link_count())]
    root_links = [name for name in root_links if name not in child_links]
    for root_name in root_links:
        link = model.link_by_name(source_link_names_by_target[root_name])
        if link is None:
            transforms[root_name] = np.eye(4, dtype=float)
            continue
        xyz, rpy = _pose_xyz_rpy(link.semantic_pose().resolve("__model__"))
        transforms[root_name] = _pose_matrix(xyz, rpy)

    queue = list(root_links)
    while queue:
        parent_name = queue.pop(0)
        parent_tf = transforms[parent_name]
        for child_name, joint_tf in children.get(parent_name, []):
            transforms[child_name] = parent_tf @ joint_tf
            queue.append(child_name)

    return transforms


def _joint_type_and_limits(joint: _JointLike) -> tuple[str, _JointAxisLike | None]:
    joint_type = str(joint.type()).split(".")[-1].lower()
    if joint_type == "revolute":
        axis = joint.axis(0)
        lower = float(axis.lower())
        upper = float(axis.upper())
        if abs(lower) > 1e10 and abs(upper) > 1e10:
            return "continuous", axis
        return "revolute", axis
    if joint_type == "fixed":
        return "fixed", None
    raise ValueError(f"Unsupported SDF joint type for URDF export: {joint_type}")


def _finite_or_default(value: float, default: float) -> float:
    if not math.isfinite(value) or abs(value) < 1e-12:
        return default
    return value


def _append_joint(
    robot_root: ET.Element,
    joint: _JointLike,
    spec: PX4ImportSpec,
    exported_links: set[str],
) -> None:
    child_name = spec.rename_link(joint.child_name())
    parent_name = spec.rename_link(joint.parent_name())
    joint_name = spec.rename_joint(joint.name())
    if child_name not in exported_links or parent_name not in exported_links:
        return

    urdf_type, axis = _joint_type_and_limits(joint)
    joint_elem = ET.SubElement(
        robot_root,
        "joint",
        {
            "name": joint_name,
            "type": urdf_type,
        },
    )
    xyz, rpy = _pose_xyz_rpy(joint.semantic_pose().resolve(parent_name))
    preserve_joint_frame = joint_name in spec.preserve_joint_frame_names
    # Most PX4 assets work best in URDF when we keep the hinge anchor position
    # but flatten the joint frame and emit the axis directly in the parent frame.
    # UUV thrusters are the main exception: their installation attitude is part
    # of the joint frame and must be preserved.
    if axis is not None and not preserve_joint_frame:
        rpy = [0.0, 0.0, 0.0]
    _append_origin(joint_elem, xyz, rpy)
    ET.SubElement(joint_elem, "parent", {"link": parent_name})
    ET.SubElement(joint_elem, "child", {"link": child_name})

    if axis is None:
        return

    axis_xyz = _vector3_to_list(axis.xyz()) if preserve_joint_frame else _vector3_to_list(axis.resolve_xyz(parent_name))
    ET.SubElement(joint_elem, "axis", {"xyz": _format_vec(axis_xyz)})
    if urdf_type == "revolute":
        ET.SubElement(
            joint_elem,
            "limit",
            {
                "lower": _format_float(float(axis.lower())),
                "upper": _format_float(float(axis.upper())),
                "effort": _format_float(_finite_or_default(float(axis.effort()), 1000.0)),
                "velocity": _format_float(_finite_or_default(float(axis.max_velocity()), 1000.0)),
            },
        )
    damping = float(axis.damping())
    friction = float(axis.friction())
    if abs(damping) > 1e-12 or abs(friction) > 1e-12:
        ET.SubElement(
            joint_elem,
            "dynamics",
            {
                "damping": _format_float(damping),
                "friction": _format_float(friction),
            },
        )


class PX4SDFAssetGenerator:
    """Generate ACESim URDF/MJCF assets from selected PX4 Gazebo SDF models."""

    def __init__(self, target: str):
        if target not in IMPORT_SPECS:
            raise ValueError(f"Unsupported import target: {target}")
        self._spec = IMPORT_SPECS[target]

    @property
    def target(self) -> str:
        return self._spec.target

    def default_paths(self) -> GeneratedAssetPaths:
        asset_dir = DEFAULT_ASSET_ROOT / self.target
        return GeneratedAssetPaths(
            asset_dir=asset_dir,
            mesh_dir=asset_dir / "meshes",
            urdf_path=asset_dir / f"{self.target}.urdf",
            xml_path=asset_dir / f"{self.target}.xml",
        )

    def _load_model(self) -> tuple[ModelLike, Path | None]:
        sdf_path = _strip_sdf_for_import(self._spec)
        cleanup_path = sdf_path if sdf_path.name != "model.sdf" else None
        root = sdf.Root()
        root.load(str(sdf_path.resolve()))
        model = root.model()
        if model is None:
            raise ValueError(f"No model found in SDF for target {self.target}")
        return model, cleanup_path

    def _generate_urdf_tree(self, model: ModelLike, output_paths: GeneratedAssetPaths) -> ET.ElementTree:
        robot_root = ET.Element("robot", {"name": self.target})
        source_model_dir = _source_model_dir(self._spec.source_model)
        exported_links: set[str] = set()
        link_model_transforms = _build_link_model_transforms(model, self._spec)

        for index in range(model.link_count()):
            link = model.link_by_index(index)
            link_name = self._spec.rename_link(link.name())
            link_elem = ET.SubElement(robot_root, "link", {"name": link_name})
            _append_inertial(link_elem, link)
            _append_visuals(
                link_elem,
                link,
                link_name,
                source_model_dir,
                output_paths.mesh_dir,
                self.target,
                link_model_transforms.get(link_name, np.eye(4, dtype=float)),
            )
            _append_collisions(
                link_elem,
                link,
                link_name,
                source_model_dir,
                output_paths.mesh_dir,
                self.target,
                link_model_transforms.get(link_name, np.eye(4, dtype=float)),
            )
            exported_links.add(link_name)

        for index in range(model.joint_count()):
            _append_joint(robot_root, model.joint_by_index(index), self._spec, exported_links)

        indent_xml(robot_root)
        return ET.ElementTree(robot_root)

    def _compile_mjcf(
        self,
        output_paths: GeneratedAssetPaths,
        *,
        floating: bool,
        safety_margin: float,
        q0: str,
        mujoco_bin: str | None,
    ) -> None:
        config = ConverterConfig(
            target=self.target,
            floating=floating,
            decompose=False,
            safety_margin=safety_margin,
            q0=q0,
            mujoco_bin=mujoco_bin,
        )
        paths = ConverterPaths(
            base_dir=output_paths.asset_dir.parent,
            urdf_path=output_paths.urdf_path,
            mesh_dir=output_paths.mesh_dir,
            xml_path=output_paths.xml_path,
        )
        initial_q = parse_q0(q0)
        min_z = calculate_min_z(output_paths.urdf_path, initial_q)
        height_offset = -min_z + safety_margin
        tmp_urdf = preprocess_urdf(output_paths.urdf_path, floating=floating, height_offset=height_offset)
        try:
            binary = find_mujoco_compile_binary(config)
            output_paths.xml_path.unlink(missing_ok=True)
            compile_urdf_to_xml(binary, tmp_urdf, output_paths.xml_path)
        finally:
            tmp_urdf.unlink(missing_ok=True)
        postprocess_xml(
            output_paths.xml_path,
            config=config,
            paths=paths,
            initial_q=initial_q,
            height_offset=height_offset,
        )

    def generate(
        self,
        *,
        output_dir: Path | None = None,
        compile_mjcf: bool = False,
        floating: bool = True,
        safety_margin: float = 0.05,
        q0: str = "",
        mujoco_bin: str | None = None,
    ) -> GeneratedAssetPaths:
        default_paths = self.default_paths()
        if output_dir is None:
            output_paths = default_paths
        else:
            asset_dir = Path(output_dir).expanduser().resolve()
            output_paths = GeneratedAssetPaths(
                asset_dir=asset_dir,
                mesh_dir=asset_dir / "meshes",
                urdf_path=asset_dir / f"{self.target}.urdf",
                xml_path=asset_dir / f"{self.target}.xml",
            )
        output_paths.asset_dir.mkdir(parents=True, exist_ok=True)
        output_paths.mesh_dir.mkdir(parents=True, exist_ok=True)

        model, cleanup_path = self._load_model()
        try:
            tree = self._generate_urdf_tree(model, output_paths)
        finally:
            if cleanup_path is not None:
                cleanup_path.unlink(missing_ok=True)

        tree.write(output_paths.urdf_path, encoding="utf-8", xml_declaration=True)
        if compile_mjcf:
            self._compile_mjcf(
                output_paths,
                floating=floating,
                safety_margin=safety_margin,
                q0=q0,
                mujoco_bin=mujoco_bin,
            )
        return output_paths


def main() -> int:
    parser = argparse.ArgumentParser(description="Import PX4 Gazebo SDF assets into ACESim URDF/MJCF assets.")
    parser.add_argument(
        "--target",
        choices=[*IMPORT_SPECS.keys(), "all"],
        default="all",
        help="Asset target to generate.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional asset directory override. Only valid with a single --target.",
    )
    parser.add_argument("--compile-mjcf", action="store_true", help="Also compile the generated URDF into MJCF.")
    parser.add_argument("--mujoco-bin", type=str, default=None, help="Optional MuJoCo compile binary path.")
    parser.add_argument("--q0", type=str, default="", help="Initial joint positions passed through to urdf2mjcf.")
    parser.add_argument("--safety-margin", type=float, default=0.05, help="Auto-height safety margin for MJCF output.")
    args = parser.parse_args()

    if args.target == "all" and args.output_dir is not None:
        parser.error("--output-dir can only be used with a single --target.")

    targets = list(IMPORT_SPECS) if args.target == "all" else [args.target]
    for target in targets:
        generator = PX4SDFAssetGenerator(target)
        output_dir = args.output_dir if args.target != "all" else None
        paths = generator.generate(
            output_dir=output_dir,
            compile_mjcf=args.compile_mjcf,
            floating=True,
            safety_margin=args.safety_margin,
            q0=args.q0,
            mujoco_bin=args.mujoco_bin,
        )
        print(f"[px4_sdf_to_urdf] Generated {paths.urdf_path}")
        if args.compile_mjcf:
            print(f"[px4_sdf_to_urdf] Generated {paths.xml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
