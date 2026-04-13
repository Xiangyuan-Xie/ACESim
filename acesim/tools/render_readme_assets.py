"""Render README preview images for core MuJoCo assets."""

from __future__ import annotations

import argparse
import os
import re
import xml.etree.ElementTree as ET
from copy import deepcopy
from dataclasses import dataclass
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

HOME_KEYFRAME = "home_keyframe"
DYNAMIC_SETTLE = "dynamic_settle"


@dataclass(frozen=True)
class PreviewPreset:
    azimuth: float
    elevation: float
    distance_scale: float
    lookat_z_scale: float
    pose_mode: str
    settle_steps: int
    ground_snap: bool = True


PREVIEW_PRESETS: dict[str, PreviewPreset] = {
    "iris": PreviewPreset(
        azimuth=138.0,
        elevation=-30.0,
        distance_scale=2.35,
        lookat_z_scale=0.35,
        pose_mode=HOME_KEYFRAME,
        settle_steps=0,
    ),
    "x500": PreviewPreset(
        azimuth=138.0,
        elevation=-32.0,
        distance_scale=2.45,
        lookat_z_scale=0.36,
        pose_mode=HOME_KEYFRAME,
        settle_steps=0,
    ),
    "x500_arm2x": PreviewPreset(
        azimuth=136.0,
        elevation=-28.0,
        distance_scale=2.8,
        lookat_z_scale=0.38,
        pose_mode=HOME_KEYFRAME,
        settle_steps=0,
    ),
    "typhoon_h480": PreviewPreset(
        azimuth=138.0,
        elevation=-28.0,
        distance_scale=2.7,
        lookat_z_scale=0.34,
        pose_mode=HOME_KEYFRAME,
        settle_steps=0,
    ),
    "advanced_plane": PreviewPreset(
        azimuth=144.0,
        elevation=-18.0,
        distance_scale=2.55,
        lookat_z_scale=0.36,
        pose_mode=HOME_KEYFRAME,
        settle_steps=0,
    ),
    "standard_vtol": PreviewPreset(
        azimuth=140.0,
        elevation=-18.0,
        distance_scale=2.65,
        lookat_z_scale=0.4,
        pose_mode=HOME_KEYFRAME,
        settle_steps=0,
    ),
    "uuv_bluerov2_heavy": PreviewPreset(
        azimuth=138.0,
        elevation=-16.0,
        distance_scale=2.7,
        lookat_z_scale=0.34,
        pose_mode=DYNAMIC_SETTLE,
        settle_steps=300,
    ),
}

ASSET_ORDER = list(PREVIEW_PRESETS)


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

        if physical_body_id >= 0:
            data.mocap_pos[mocap_id] = data.xpos[physical_body_id].copy()
        elif site_id >= 0:
            data.mocap_pos[mocap_id] = data.site_xpos[site_id].copy()
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


def _snap_model_to_ground(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    bounds_min, bounds_max = _mesh_world_bounds(model, data)
    if abs(float(bounds_min[2])) <= 1e-9:
        return bounds_min, bounds_max

    free_joint_id = next(
        (joint_id for joint_id in range(model.njnt) if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE),
        -1,
    )
    if free_joint_id < 0:
        raise ValueError("Preview ground snap requires a free joint on the asset root")

    qpos_adr = int(model.jnt_qposadr[free_joint_id])
    data.qpos[qpos_adr + 2] -= float(bounds_min[2])
    mujoco.mj_forward(model, data)
    _initialize_visual_mocaps(model, data)
    return _mesh_world_bounds(model, data)


def _prepare_preview_state(
    asset_name: str,
    *,
    settle_steps_override: int | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData, np.ndarray, np.ndarray]:
    preset = PREVIEW_PRESETS[asset_name]
    model, data = _load_model(asset_name)
    settle_steps = preset.settle_steps if settle_steps_override is None else max(settle_steps_override, 0)

    if preset.pose_mode not in {HOME_KEYFRAME, DYNAMIC_SETTLE}:
        raise ValueError(f"Unsupported preview pose mode: {preset.pose_mode}")
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)
    if settle_steps > 0:
        _initialize_visual_mocaps(model, data)

    bounds_min, bounds_max = _mesh_world_bounds(model, data)
    if preset.ground_snap:
        bounds_min, bounds_max = _snap_model_to_ground(model, data)
    return model, data, bounds_min, bounds_max


def _render_asset(
    asset_name: str,
    output_path: Path,
    *,
    width: int,
    height: int,
    settle_steps_override: int | None,
) -> None:
    preset = PREVIEW_PRESETS[asset_name]
    model, data, bounds_min, bounds_max = _prepare_preview_state(
        asset_name,
        settle_steps_override=settle_steps_override,
    )
    center = (bounds_min + bounds_max) / 2.0
    span = np.maximum(bounds_max - bounds_min, 1e-6)
    radius = float(np.linalg.norm(span) / 2.0)

    renderer = mujoco.Renderer(model, height, width)
    try:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat = center.astype(float)
        camera.lookat[2] = float(bounds_min[2] + span[2] * preset.lookat_z_scale)
        camera.distance = max(radius * preset.distance_scale, 0.6)
        camera.azimuth = preset.azimuth
        camera.elevation = preset.elevation
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
    parser.add_argument("--settle-steps", type=int, default=None)
    args = parser.parse_args()

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    for asset_name in args.assets:
        output_path = output_dir / f"{asset_name}.png"
        _render_asset(
            asset_name,
            output_path,
            width=args.width,
            height=args.height,
            settle_steps_override=args.settle_steps,
        )
        print(output_path)


if __name__ == "__main__":
    main()
