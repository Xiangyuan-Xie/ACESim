"""Multirotor-specific runtime asset preparation and MJCF rewriting."""

import copy
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import TypeAlias, TypedDict

from ..asset_context import AssetPaths, AssetToolchainConfig
from ..mesh_processing import export_translated_mesh, mesh_bounds_center

Vec3: TypeAlias = list[float]
Vec4: TypeAlias = list[float]


class RotorSpec(TypedDict):
    source_mesh: str
    mesh_translation_mm: Vec3
    pos: Vec3
    quat: Vec4
    rgba: Vec4


class LegSpec(TypedDict):
    mesh: str
    collision_pose: Vec3
    collision_rpy: Vec3
    bar_pose: Vec3
    bar_rpy: Vec3


X500_RUNTIME_FILES = {
    "NXP-HGD-CF.stl",
    "5010Base.stl",
    "5010Bell.stl",
    "1345_prop_ccw_centered.stl",
    "1345_prop_cw_centered.stl",
    "nxp.png",
    "rd.png",
}
X500_SOURCE_FILES = {
    "1345_prop_ccw.stl",
    "1345_prop_cw.stl",
}
IRIS_RUNTIME_FILES = {
    "iris.stl",
    "iris_prop_ccw.stl",
    "iris_prop_cw.stl",
}
TYPHOON_RUNTIME_FILES = {
    "main_body_remeshed_v3.stl",
    "leg1_remeshed_v3.stl",
    "leg2_remeshed_v3.stl",
    "rotor_1_vis.stl",
    "rotor_2_vis.stl",
    "rotor_3_vis.stl",
    "rotor_4_vis.stl",
    "rotor_5_vis.stl",
    "rotor_6_vis.stl",
}
TYPHOON_SOURCE_FILES = {
    "cgo3_mount_remeshed_v1.stl",
    "cgo3_vertical_arm_remeshed_v1.stl",
    "cgo3_horizontal_arm_remeshed_v1.stl",
    "cgo3_camera_remeshed_v1.stl",
    "prop_ccw_assembly_remeshed_v3.stl",
    "prop_cw_assembly_remeshed_v3.stl",
}
TYPHOON_ROTOR_SPECS: dict[int, RotorSpec] = {
    1: {
        "source_mesh": "prop_ccw_assembly_remeshed_v3.stl",
        "mesh_translation_mm": [-211.396, -119.762, -82.219],
        "pos": [-0.00187896, -0.242705, 0.0822169],
        "quat": [0.5, 0.0, 0.0, -0.8660254037844386],
        "rgba": [0.2, 0.35, 0.85, 1.0],
    },
    2: {
        "source_mesh": "prop_cw_assembly_remeshed_v3.stl",
        "mesh_translation_mm": [1.87896, -242.705, -82.2169],
        "pos": [-0.00187896, 0.242705, 0.0822169],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "rgba": [0.24, 0.24, 0.24, 1.0],
    },
    3: {
        "source_mesh": "prop_ccw_assembly_remeshed_v3.stl",
        "mesh_translation_mm": [-211.396, -119.762, -82.219],
        "pos": [0.211396, 0.119762, 0.082219],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "rgba": [0.2, 0.35, 0.85, 1.0],
    },
    4: {
        "source_mesh": "prop_cw_assembly_remeshed_v3.stl",
        "mesh_translation_mm": [1.87896, -242.705, -82.2169],
        "pos": [-0.209396, -0.122762, 0.082219],
        "quat": [0.5, 0.0, 0.0, 0.8660254037844386],
        "rgba": [0.24, 0.24, 0.24, 1.0],
    },
    5: {
        "source_mesh": "prop_cw_assembly_remeshed_v3.stl",
        "mesh_translation_mm": [1.87896, -242.705, -82.2169],
        "pos": [0.211396, -0.119762, 0.082219],
        "quat": [0.5, 0.0, 0.0, -0.8660254037844386],
        "rgba": [0.24, 0.24, 0.24, 1.0],
    },
    6: {
        "source_mesh": "prop_ccw_assembly_remeshed_v3.stl",
        "mesh_translation_mm": [-211.396, -119.762, -82.219],
        "pos": [-0.209396, 0.122762, 0.082219],
        "quat": [0.5, 0.0, 0.0, 0.8660254037844386],
        "rgba": [0.2, 0.35, 0.85, 1.0],
    },
}
TYPHOON_LEG_SPECS: dict[str, LegSpec] = {
    "left": {
        "mesh": "typhoon_leg_left_vis",
        "collision_pose": [-0.005, -0.14314, -0.207252],
        "collision_rpy": [0.0, 1.56893, 0.0],
        "bar_pose": [0.00052, -0.08503, -0.121187],
        "bar_rpy": [-0.501318, 0.0, 0.0],
    },
    "right": {
        "mesh": "typhoon_leg_right_vis",
        "collision_pose": [-0.005, 0.14314, -0.207252],
        "collision_rpy": [0.0, 1.56893, 0.0],
        "bar_pose": [0.00052, 0.08503, -0.121187],
        "bar_rpy": [0.501318, 0.0, 0.0],
    },
}


def generate_runtime_meshes(config: AssetToolchainConfig, paths: AssetPaths) -> None:
    if config.target == "x500":
        export_translated_mesh(
            paths.mesh_dir,
            "1345_prop_ccw.stl",
            "1345_prop_ccw_centered.stl",
            [-0.022, -0.14638461538461536, -0.016],
        )
        export_translated_mesh(
            paths.mesh_dir,
            "1345_prop_cw.stl",
            "1345_prop_cw_centered.stl",
            [-0.022, -0.14638461538461536, -0.016],
        )
    elif config.target == "typhoon_h480":
        for idx, spec in TYPHOON_ROTOR_SPECS.items():
            export_translated_mesh(
                paths.mesh_dir,
                spec["source_mesh"],
                f"rotor_{idx}_vis.stl",
                spec["mesh_translation_mm"],
            )


def cleanup_unused_meshes(config: AssetToolchainConfig, paths: AssetPaths) -> None:
    if not paths.mesh_dir.exists():
        return

    if config.target == "x500":
        keep = X500_RUNTIME_FILES | X500_SOURCE_FILES
    elif config.target == "iris":
        keep = IRIS_RUNTIME_FILES
    elif config.target == "typhoon_h480":
        keep = TYPHOON_RUNTIME_FILES | TYPHOON_SOURCE_FILES
    else:
        return

    for file_path in paths.mesh_dir.iterdir():
        if file_path.is_file() and file_path.name not in keep:
            file_path.unlink()


def _fmt_floats(values: list[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _pose_attrib(
    *,
    pos: list[float] | None = None,
    quat: list[float] | None = None,
    rpy: list[float] | None = None,
) -> dict[str, str]:
    attrib: dict[str, str] = {}
    if pos is not None:
        attrib["pos"] = _fmt_floats(pos)
    if quat is None and rpy is not None:
        quat = _euler_to_quat(rpy[0], rpy[1], rpy[2])
    if quat is not None:
        attrib["quat"] = _fmt_floats(quat)
    return attrib


def _find_body(parent: ET.Element, name: str) -> ET.Element | None:
    for body in parent.iter("body"):
        if body.get("name") == name:
            return body
    return None


def _clear_body(body: ET.Element, *, keep_sites: bool) -> None:
    for child in list(body):
        if keep_sites and child.tag == "site":
            continue
        body.remove(child)


def _remove_named_bodies(parent: ET.Element, names: list[str]) -> None:
    for body in list(parent.findall("body")):
        if body.get("name") in names:
            parent.remove(body)


def _upsert_asset_child(asset: ET.Element, tag: str, key: str, value: str, attrib: dict[str, str]) -> ET.Element:
    for child in asset.findall(tag):
        if child.get(key) == value:
            child.attrib.clear()
            child.attrib.update(attrib)
            return child
    return ET.SubElement(asset, tag, attrib)


def _ensure_mesh_asset(asset: ET.Element, name: str, file_name: str, scale: list[float] | None = None) -> None:
    attrib = {"name": name, "file": file_name}
    if scale is not None:
        attrib["scale"] = _fmt_floats(scale)
    _upsert_asset_child(asset, "mesh", "name", name, attrib)


def _ensure_texture_asset(asset: ET.Element, name: str, file_name: str) -> None:
    _upsert_asset_child(asset, "texture", "name", name, {"name": name, "type": "2d", "file": file_name})


def _ensure_material_asset(
    asset: ET.Element,
    name: str,
    rgba: list[float] | None = None,
    texture: str | None = None,
) -> None:
    attrib = {"name": name}
    if rgba is not None:
        attrib["rgba"] = _fmt_floats(rgba)
    if texture is not None:
        attrib["texture"] = texture
    _upsert_asset_child(asset, "material", "name", name, attrib)


def _append_geom(
    parent: ET.Element,
    *,
    geom_type: str,
    name: str | None = None,
    mesh: str | None = None,
    material: str | None = None,
    size: list[float] | None = None,
    pos: list[float] | None = None,
    quat: list[float] | None = None,
    rpy: list[float] | None = None,
    rgba: list[float] | None = None,
    group: int,
    contype: int,
    conaffinity: int,
) -> None:
    attrib = {"type": geom_type, "group": str(group), "contype": str(contype), "conaffinity": str(conaffinity)}
    if name:
        attrib["name"] = name
    if mesh:
        attrib["mesh"] = mesh
    if material:
        attrib["material"] = material
    if size is not None:
        attrib["size"] = _fmt_floats(size)
    if rgba is not None:
        attrib["rgba"] = _fmt_floats(rgba)
    attrib.update(_pose_attrib(pos=pos, quat=quat, rpy=rpy))
    ET.SubElement(parent, "geom", attrib)


def _ensure_rotor_visual_body(
    worldbody: ET.Element,
    idx: int,
    mesh_name: str,
    *,
    quat: list[float] | None = None,
    rgba: list[float] | None = None,
) -> None:
    body_name = f"rotor_{idx}_vis"
    body = _find_body(worldbody, body_name)
    if body is None:
        body = ET.SubElement(worldbody, "body", {"name": body_name, "mocap": "true"})
    else:
        _clear_body(body, keep_sites=False)
        body.set("mocap", "true")
    _append_geom(
        body,
        geom_type="mesh",
        mesh=mesh_name,
        quat=quat,
        rgba=rgba,
        group=1,
        contype=0,
        conaffinity=0,
    )


def rebuild_multirotor_runtime_model(
    root: ET.Element, worldbody: ET.Element, config: AssetToolchainConfig, paths: AssetPaths
) -> None:
    if config.target not in {"x500", "iris", "typhoon_h480"}:
        return

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(1, asset)

    base_body = _find_body(worldbody, "base_link")
    if base_body is None:
        return

    kept_sites = [copy.deepcopy(site) for site in base_body.findall("site")]
    floating_joint = None
    for joint in base_body.findall("joint"):
        if joint.get("name") == "floating_base_joint":
            floating_joint = copy.deepcopy(joint)
            break
    inertial = copy.deepcopy(base_body.find("inertial")) if base_body.find("inertial") is not None else None
    _clear_body(base_body, keep_sites=False)
    if inertial is not None:
        base_body.append(inertial)
    if floating_joint is not None:
        base_body.append(floating_joint)
    for site in kept_sites:
        base_body.append(site)

    if config.target == "x500":
        _ensure_mesh_asset(asset, "x500_frame", "NXP-HGD-CF.stl")
        _ensure_mesh_asset(asset, "x500_motor_base", "5010Base.stl")
        _ensure_mesh_asset(asset, "x500_motor_bell", "5010Bell.stl")
        _ensure_mesh_asset(asset, "x500_prop_ccw_vis", "1345_prop_ccw_centered.stl", [0.8461538461538461] * 3)
        _ensure_mesh_asset(asset, "x500_prop_cw_vis", "1345_prop_cw_centered.stl", [0.8461538461538461] * 3)
        _ensure_texture_asset(asset, "x500_nxp_tex", "nxp.png")
        _ensure_texture_asset(asset, "x500_rd_tex", "rd.png")
        _ensure_material_asset(asset, "x500_nxp_mat", texture="x500_nxp_tex", rgba=[1, 1, 1, 1])
        _ensure_material_asset(asset, "x500_rd_mat", texture="x500_rd_tex", rgba=[1, 1, 1, 1])

        _append_geom(
            base_body,
            geom_type="mesh",
            mesh="x500_frame",
            pos=[0, 0, 0.025],
            rpy=[0, 0, math.pi],
            rgba=[0.95, 0.95, 0.95, 1.0],
            group=1,
            contype=0,
            conaffinity=0,
        )
        motor_base_positions: tuple[Vec3, ...] = (
            [0.174, 0.174, 0.032],
            [-0.174, 0.174, 0.032],
            [0.174, -0.174, 0.032],
            [-0.174, -0.174, 0.032],
        )
        for pos in motor_base_positions:
            _append_geom(
                base_body,
                geom_type="mesh",
                mesh="x500_motor_base",
                pos=pos,
                rpy=[0, 0, -0.45],
                rgba=[0.25, 0.25, 0.25, 1.0],
                group=1,
                contype=0,
                conaffinity=0,
            )
        _append_geom(
            base_body,
            geom_type="box",
            size=[0.0065, 0.0035, 0.00025],
            pos=[0.047, 0.001, 0.043],
            rpy=[1.0, 0.0, 1.57],
            material="x500_nxp_mat",
            group=1,
            contype=0,
            conaffinity=0,
        )
        _append_geom(
            base_body,
            geom_type="box",
            size=[0.0065, 0.0035, 0.00025],
            pos=[-0.023, 0.0, 0.0515],
            rpy=[0.0, 0.0, -1.57],
            material="x500_nxp_mat",
            group=1,
            contype=0,
            conaffinity=0,
        )
        _append_geom(
            base_body,
            geom_type="box",
            size=[0.016, 0.0017, 0.00025],
            pos=[-0.03, 0.0, 0.0515],
            rpy=[0.0, 0.0, -1.57],
            material="x500_rd_mat",
            group=1,
            contype=0,
            conaffinity=0,
        )
        for bell_pos in (
            [0.174, -0.174, 0.028],
            [-0.174, 0.174, 0.028],
            [0.174, 0.174, 0.028],
            [-0.174, -0.174, 0.028],
        ):
            _append_geom(
                base_body,
                geom_type="mesh",
                mesh="x500_motor_bell",
                pos=bell_pos,
                rgba=[0.15, 0.15, 0.15, 1.0],
                group=1,
                contype=0,
                conaffinity=0,
            )
        base_collisions: tuple[tuple[Vec3, Vec3, Vec3], ...] = (
            (
                [0, 0, 0.007],
                [0.35355339059327373 / 2, 0.35355339059327373 / 2, 0.05 / 2],
                [0, 0, 0],
            ),
            (
                [0, -0.098, -0.123],
                [0.015 / 2, 0.015 / 2, 0.21 / 2],
                [-0.35, 0, 0],
            ),
            (
                [0, 0.098, -0.123],
                [0.015 / 2, 0.015 / 2, 0.21 / 2],
                [0.35, 0, 0],
            ),
            (
                [0, -0.132, -0.2195],
                [0.25 / 2, 0.015 / 2, 0.015 / 2],
                [0, 0, 0],
            ),
            (
                [0, 0.132, -0.2195],
                [0.25 / 2, 0.015 / 2, 0.015 / 2],
                [0, 0, 0],
            ),
        )
        for i, (pos, size, rpy) in enumerate(base_collisions):
            _append_geom(
                base_body,
                geom_type="box",
                name=f"base_collision_{i}",
                size=size,
                pos=pos,
                rpy=rpy,
                group=3,
                contype=1,
                conaffinity=1,
            )
        x500_rotor_specs: dict[int, tuple[str, Vec3]] = {
            1: ("x500_prop_ccw_vis", [0.174, -0.174, 0.06]),
            2: ("x500_prop_ccw_vis", [-0.174, 0.174, 0.06]),
            3: ("x500_prop_cw_vis", [0.174, 0.174, 0.06]),
            4: ("x500_prop_cw_vis", [-0.174, -0.174, 0.06]),
        }
        for idx, (mesh_name, expected_pos) in x500_rotor_specs.items():
            rotor_body = _find_body(base_body, f"rotor_{idx}")
            if rotor_body is None:
                rotor_body = ET.SubElement(
                    base_body, "body", {"name": f"rotor_{idx}", "pos": _fmt_floats(expected_pos)}
                )
            rotor_body.set("pos", _fmt_floats(expected_pos))
            _clear_body(rotor_body, keep_sites=True)
            _append_geom(
                rotor_body,
                geom_type="box",
                size=[0.2792307692307692 / 2, 0.016923076923076923 / 2, 0.0008461538461538462 / 2],
                group=3,
                contype=1,
                conaffinity=1,
            )
            _ensure_rotor_visual_body(worldbody, idx, mesh_name, rgba=[0.2, 0.2, 0.2, 1.0])
        return

    if config.target == "iris":
        _ensure_mesh_asset(asset, "iris_body_vis", "iris.stl")
        _ensure_mesh_asset(asset, "iris_prop_ccw_vis", "iris_prop_ccw.stl")
        _ensure_mesh_asset(asset, "iris_prop_cw_vis", "iris_prop_cw.stl")
        _ensure_material_asset(asset, "gazebo_darkgrey_mat", rgba=[0.24, 0.24, 0.24, 1.0])
        _ensure_material_asset(asset, "gazebo_blue_mat", rgba=[0.2, 0.35, 0.85, 1.0])
        _append_geom(
            base_body,
            geom_type="mesh",
            mesh="iris_body_vis",
            material="gazebo_darkgrey_mat",
            group=1,
            contype=0,
            conaffinity=0,
        )
        _append_geom(base_body, geom_type="box", size=[0.47 / 2, 0.47 / 2, 0.11 / 2], group=3, contype=1, conaffinity=1)
        iris_rotor_specs: dict[int, tuple[str, Vec3, Vec4]] = {
            1: ("iris_prop_ccw_vis", [0.13, -0.22, 0.023], [0.2, 0.35, 0.85, 1.0]),
            2: ("iris_prop_ccw_vis", [-0.13, 0.2, 0.023], [0.24, 0.24, 0.24, 1.0]),
            3: ("iris_prop_cw_vis", [0.13, 0.22, 0.023], [0.2, 0.35, 0.85, 1.0]),
            4: ("iris_prop_cw_vis", [-0.13, -0.2, 0.023], [0.24, 0.24, 0.24, 1.0]),
        }
        for idx, (mesh_name, expected_pos, rgba) in iris_rotor_specs.items():
            rotor_body = _find_body(base_body, f"rotor_{idx}")
            if rotor_body is None:
                rotor_body = ET.SubElement(
                    base_body, "body", {"name": f"rotor_{idx}", "pos": _fmt_floats(expected_pos)}
                )
            rotor_body.set("pos", _fmt_floats(expected_pos))
            _clear_body(rotor_body, keep_sites=True)
            _append_geom(rotor_body, geom_type="cylinder", size=[0.128, 0.005 / 2], group=3, contype=1, conaffinity=1)
            _ensure_rotor_visual_body(worldbody, idx, mesh_name, rgba=rgba)
        return

    _ensure_mesh_asset(asset, "typhoon_body_vis", "main_body_remeshed_v3.stl", [0.001, 0.001, 0.001])
    _ensure_mesh_asset(asset, "typhoon_leg_left_vis", "leg2_remeshed_v3.stl", [0.001, 0.001, 0.001])
    _ensure_mesh_asset(asset, "typhoon_leg_right_vis", "leg1_remeshed_v3.stl", [0.001, 0.001, 0.001])
    _ensure_mesh_asset(asset, "typhoon_mount_vis", "cgo3_mount_remeshed_v1.stl", [0.001, 0.001, 0.001])
    _ensure_mesh_asset(asset, "typhoon_varm_vis", "cgo3_vertical_arm_remeshed_v1.stl", [0.001, 0.001, 0.001])
    _ensure_mesh_asset(asset, "typhoon_harm_vis", "cgo3_horizontal_arm_remeshed_v1.stl", [0.001, 0.001, 0.001])
    _ensure_mesh_asset(asset, "typhoon_camera_vis", "cgo3_camera_remeshed_v1.stl", [0.001, 0.001, 0.001])
    for idx in range(1, 7):
        _ensure_mesh_asset(asset, f"typhoon_rotor_vis_{idx}", f"rotor_{idx}_vis.stl", [0.001, 0.001, 0.001])
    _ensure_material_asset(asset, "typhoon_darkgrey_mat", rgba=[0.24, 0.24, 0.24, 1.0])
    _ensure_material_asset(asset, "typhoon_blue_mat", rgba=[0.2, 0.35, 0.85, 1.0])
    _append_geom(
        base_body,
        geom_type="mesh",
        mesh="typhoon_body_vis",
        rpy=[0, 0, math.pi],
        material="typhoon_darkgrey_mat",
        group=1,
        contype=0,
        conaffinity=0,
    )
    _append_geom(
        base_body,
        geom_type="box",
        name="base_link_collision",
        size=[0.67 / 2, 0.67 / 2, 0.15 / 2],
        group=3,
        contype=1,
        conaffinity=1,
    )
    _remove_named_bodies(base_body, ["left_leg", "right_leg", "cgo3_mount_link"])
    for side, leg in TYPHOON_LEG_SPECS.items():
        _append_geom(
            base_body,
            geom_type="mesh",
            name=f"{side}_leg_visual",
            mesh=leg["mesh"],
            material="typhoon_darkgrey_mat",
            group=1,
            contype=0,
            conaffinity=0,
        )
        _append_geom(
            base_body,
            geom_type="cylinder",
            name=f"{side}_leg_collision",
            pos=leg["collision_pose"],
            rpy=leg["collision_rpy"],
            size=[0.012209, 0.3 / 2],
            group=3,
            contype=1,
            conaffinity=1,
        )
        _append_geom(
            base_body,
            geom_type="cylinder",
            name=f"{side}_leg_collision_bar",
            pos=leg["bar_pose"],
            rpy=leg["bar_rpy"],
            size=[0.00914984, 0.176893 / 2],
            group=3,
            contype=1,
            conaffinity=1,
        )
    _append_geom(
        base_body,
        geom_type="mesh",
        name="cgo3_mount_visual",
        mesh="typhoon_mount_vis",
        material="typhoon_darkgrey_mat",
        group=1,
        contype=0,
        conaffinity=0,
    )
    _append_geom(
        base_body,
        geom_type="mesh",
        name="cgo3_vertical_arm_visual",
        mesh="typhoon_varm_vis",
        material="typhoon_darkgrey_mat",
        group=1,
        contype=0,
        conaffinity=0,
    )
    _append_geom(
        base_body,
        geom_type="mesh",
        name="cgo3_horizontal_arm_visual",
        mesh="typhoon_harm_vis",
        material="typhoon_darkgrey_mat",
        group=1,
        contype=0,
        conaffinity=0,
    )
    _append_geom(
        base_body,
        geom_type="mesh",
        name="cgo3_camera_visual",
        mesh="typhoon_camera_vis",
        material="typhoon_darkgrey_mat",
        group=1,
        contype=0,
        conaffinity=0,
    )
    camera_col_pos = mesh_bounds_center(paths.mesh_dir, "cgo3_camera_remeshed_v1.stl", mesh_scale=[0.001, 0.001, 0.001])
    _append_geom(
        base_body,
        geom_type="sphere",
        name="cgo3_camera_collision",
        pos=camera_col_pos,
        size=[0.035],
        group=3,
        contype=1,
        conaffinity=1,
    )
    for idx, spec in TYPHOON_ROTOR_SPECS.items():
        rotor_body = _find_body(base_body, f"rotor_{idx}")
        expected_pos = spec["pos"]
        if rotor_body is None:
            rotor_body = ET.SubElement(base_body, "body", {"name": f"rotor_{idx}", "pos": _fmt_floats(expected_pos)})
        rotor_body.set("pos", _fmt_floats(expected_pos))
        _clear_body(rotor_body, keep_sites=True)
        _append_geom(
            rotor_body,
            geom_type="cylinder",
            name=f"rotor_{idx}_collision",
            size=[0.128, 0.005 / 2],
            group=3,
            contype=1,
            conaffinity=1,
        )
        _ensure_rotor_visual_body(worldbody, idx, f"typhoon_rotor_vis_{idx}", quat=spec["quat"], rgba=spec["rgba"])


@dataclass(frozen=True)
class MultirotorRuntimeModelHandler:
    """Runtime handler for multirotor assets that need mesh/material rewrites."""

    family: str = "multirotor"

    def prepare_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        generate_runtime_meshes(config, paths)

    def cleanup_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        cleanup_unused_meshes(config, paths)

    def rewrite_runtime_model(
        self, root: ET.Element, worldbody: ET.Element, config: AssetToolchainConfig, paths: AssetPaths
    ) -> None:
        rebuild_multirotor_runtime_model(root, worldbody, config, paths)
