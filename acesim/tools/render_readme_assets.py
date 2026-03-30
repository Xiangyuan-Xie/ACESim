"""Render README preview images for core MuJoCo assets."""

from __future__ import annotations

import argparse
import os
import re
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from scipy.spatial.transform import Rotation

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "env" / "mujoco" / "scene" / "default.xml"
ASSET_ROOT = ROOT / "env" / "mujoco" / "asset"
DEFAULT_OUTPUT_DIR = ROOT.parents[0] / "docs" / "images" / "assets"

ASSET_ORDER = [
    "iris",
    "x500",
    "x500_arm2x",
    "typhoon_h480",
    "plane",
    "standard_vtol",
    "uuv_bluerov2_heavy",
]

CAMERA_OVERRIDES: dict[str, dict[str, float]] = {
    "iris": {"azimuth": 132.0, "elevation": -22.0, "distance_scale": 2.65, "lookat_z_scale": 0.42},
    "x500": {"azimuth": 132.0, "elevation": -22.0, "distance_scale": 2.65, "lookat_z_scale": 0.42},
    "x500_arm2x": {"azimuth": 126.0, "elevation": -18.0, "distance_scale": 2.95, "lookat_z_scale": 0.48},
    "typhoon_h480": {"azimuth": 132.0, "elevation": -22.0, "distance_scale": 2.75, "lookat_z_scale": 0.44},
    "plane": {"azimuth": 144.0, "elevation": -18.0, "distance_scale": 2.55, "lookat_z_scale": 0.36},
    "standard_vtol": {"azimuth": 140.0, "elevation": -18.0, "distance_scale": 2.65, "lookat_z_scale": 0.4},
    "uuv_bluerov2_heavy": {"azimuth": 138.0, "elevation": -16.0, "distance_scale": 2.7, "lookat_z_scale": 0.34},
}


def _merge_scene_robot_xml(asset_name: str) -> str:
    scene_root = ET.parse(SCENE_PATH).getroot()
    robot_path = (ASSET_ROOT / asset_name / f"{asset_name}.xml").resolve()
    robot_root = ET.parse(robot_path).getroot()

    def merge_children(tag: str) -> None:
        robot_elem = robot_root.find(tag)
        if robot_elem is None:
            return
        scene_elem = scene_root.find(tag)
        if scene_elem is None:
            scene_root.append(deepcopy(robot_elem))
            return
        for child in list(robot_elem):
            scene_elem.append(deepcopy(child))

    def copy_if_missing(tag: str) -> None:
        if scene_root.find(tag) is not None:
            return
        robot_elem = robot_root.find(tag)
        if robot_elem is not None:
            scene_root.append(deepcopy(robot_elem))

    for tag in ["compiler", "option", "size", "default", "visual", "statistic", "extension"]:
        copy_if_missing(tag)

    compiler = scene_root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(scene_root, "compiler")
    mesh_dir = (robot_path.parent / "meshes").resolve().as_posix()
    compiler.set("meshdir", mesh_dir)
    compiler.set("texturedir", mesh_dir)

    for tag in ["asset", "worldbody", "actuator", "sensor", "keyframe", "contact", "equality", "tendon"]:
        merge_children(tag)

    visual = scene_root.find("visual")
    if visual is None:
        visual = ET.SubElement(scene_root, "visual")
    global_settings = visual.find("global")
    if global_settings is None:
        global_settings = ET.SubElement(visual, "global")
    global_settings.set("offwidth", "1600")
    global_settings.set("offheight", "1200")

    return ET.tostring(scene_root, encoding="unicode")


def _load_model(asset_name: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(_merge_scene_robot_xml(asset_name))
    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    _initialize_visual_mocaps(model, data)
    return model, data


def _initialize_visual_mocaps(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Place mocap-based rotor visuals onto their intended mounted positions."""

    for body_id in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not body_name:
            continue
        match = re.fullmatch(r"rotor_(\d+)_vis", body_name)
        if match is None:
            continue

        mocap_id = int(model.body_mocapid[body_id])
        if mocap_id < 0:
            continue

        rotor_idx = match.group(1)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"rotor_offset_{rotor_idx}")
        physical_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_idx}")

        if site_id >= 0:
            data.mocap_pos[mocap_id] = data.site_xpos[site_id].copy()
        elif physical_body_id >= 0:
            data.mocap_pos[mocap_id] = data.xpos[physical_body_id].copy()
        else:
            data.mocap_pos[mocap_id] = data.xpos[body_id].copy()

        if physical_body_id >= 0:
            data.mocap_quat[mocap_id] = data.xquat[physical_body_id].copy()
        else:
            data.mocap_quat[mocap_id] = data.xquat[body_id].copy()

    mujoco.mj_forward(model, data)


def _mesh_geom_ids(model: mujoco.MjModel) -> list[int]:
    return [geom_id for geom_id in range(model.ngeom) if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH]


def _mesh_world_bounds(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    geom_ids = _mesh_geom_ids(model)
    if not geom_ids:
        raise ValueError("Model has no mesh geoms to frame")

    bounds_min = np.full(3, np.inf, dtype=float)
    bounds_max = np.full(3, -np.inf, dtype=float)
    for geom_id in geom_ids:
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0:
            continue
        vert_start = int(model.mesh_vertadr[mesh_id])
        vert_count = int(model.mesh_vertnum[mesh_id])
        vertices = np.asarray(model.mesh_vert[vert_start : vert_start + vert_count], dtype=float)
        body_id = int(model.geom_bodyid[geom_id])
        body_pos = np.asarray(data.xpos[body_id], dtype=float)
        body_rot = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
        geom_pos = np.asarray(model.geom_pos[geom_id], dtype=float)
        geom_rot = Rotation.from_quat(np.asarray(model.geom_quat[geom_id], dtype=float), scalar_first=True).as_matrix()
        transformed = ((vertices @ geom_rot.T) + geom_pos) @ body_rot.T
        transformed = transformed + body_pos
        bounds_min = np.minimum(bounds_min, transformed.min(axis=0))
        bounds_max = np.maximum(bounds_max, transformed.max(axis=0))

    return bounds_min, bounds_max


def _render_asset(asset_name: str, output_path: Path, *, width: int, height: int) -> None:
    model, data = _load_model(asset_name)
    bounds_min, bounds_max = _mesh_world_bounds(model, data)
    center = (bounds_min + bounds_max) / 2.0
    span = np.maximum(bounds_max - bounds_min, 1e-6)
    radius = float(np.linalg.norm(span) / 2.0)
    params = CAMERA_OVERRIDES[asset_name]

    renderer = mujoco.Renderer(model, height, width)
    try:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat = center.astype(float)
        camera.lookat[2] = float(bounds_min[2] + span[2] * params["lookat_z_scale"])
        camera.distance = max(radius * params["distance_scale"], 0.6)
        camera.azimuth = params["azimuth"]
        camera.elevation = params["elevation"]
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
        imageio.imwrite(output_path, image)
    finally:
        renderer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", nargs="*", default=ASSET_ORDER, choices=ASSET_ORDER)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    for asset_name in args.assets:
        output_path = output_dir / f"{asset_name}.png"
        _render_asset(asset_name, output_path, width=args.width, height=args.height)
        print(output_path)


if __name__ == "__main__":
    main()
