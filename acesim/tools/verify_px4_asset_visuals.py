"""Validate and render PX4-derived MuJoCo vehicle assets."""

from __future__ import annotations

import argparse
import json
import os
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = ROOT / "config"
DEFAULT_OUTPUT_DIR = Path("/tmp/acesim_px4_visual_check")

CASES = {
    "plane": {
        "config": CONFIG_ROOT / "plane.toml",
        "camera_pos": np.array([1.15, -0.85, 0.42], dtype=float),
        "lookat": np.array([-0.12, 0.0, 0.18], dtype=float),
    },
    "standard_vtol": {
        "config": CONFIG_ROOT / "standard_vtol.toml",
        "camera_pos": np.array([1.35, -1.05, 0.48], dtype=float),
        "lookat": np.array([-0.02, 0.0, 0.12], dtype=float),
    },
    "uuv_bluerov2_heavy": {
        "config": CONFIG_ROOT / "uuv_bluerov2_heavy.toml",
        "camera_pos": np.array([0.7, -0.58, 0.32], dtype=float),
        "lookat": np.array([0.0, 0.0, 0.05], dtype=float),
    },
}


def _merge_scene_robot_xml(config_path: Path) -> str:
    loader = ConfigLoader(config_path)
    scene_path = (ROOT / "env" / "mujoco" / "scene" / f"{loader.get_scene_name()}.xml").resolve()
    asset_name = loader.get_asset_name()
    robot_path = (ROOT / "env" / "mujoco" / "asset" / asset_name / f"{asset_name}.xml").resolve()

    scene_root = ET.parse(scene_path).getroot()
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

    return ET.tostring(scene_root, encoding="unicode")


def _load_case(config_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(_merge_scene_robot_xml(config_path))
    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    return model, data


def _mesh_geom_ids(model: mujoco.MjModel, body_id: int) -> list[int]:
    geom_adr = int(model.body_geomadr[body_id])
    geom_num = int(model.body_geomnum[body_id])
    return [
        geom_id
        for geom_id in range(geom_adr, geom_adr + geom_num)
        if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH
    ]


def _geom_world_pose(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> tuple[np.ndarray, Rotation]:
    body_id = int(model.geom_bodyid[geom_id])
    body_pos = data.xpos[body_id].copy()
    body_rot = Rotation.from_quat(data.xquat[body_id].copy(), scalar_first=True)
    geom_pos = model.geom_pos[geom_id].copy()
    geom_rot = Rotation.from_quat(model.geom_quat[geom_id].copy(), scalar_first=True)
    return body_pos + body_rot.apply(geom_pos), body_rot * geom_rot


def _mesh_center_from_body(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"Missing body {body_name}")
    geom_ids = _mesh_geom_ids(model, body_id)
    if not geom_ids:
        raise ValueError(f"Body {body_name} has no mesh geoms")
    pos, _ = _geom_world_pose(model, data, geom_ids[0])
    return pos


def _body_mesh_world_bounds(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"Missing body {body_name}")
    bounds_min = np.array([np.inf, np.inf, np.inf], dtype=float)
    bounds_max = np.array([-np.inf, -np.inf, -np.inf], dtype=float)
    for geom_id in _mesh_geom_ids(model, body_id):
        pos, rot = _geom_world_pose(model, data, geom_id)
        mesh_id = int(model.geom_dataid[geom_id])
        start = int(model.mesh_vertadr[mesh_id])
        count = int(model.mesh_vertnum[mesh_id])
        vertices = model.mesh_vert[start : start + count]
        transformed = rot.apply(vertices) + pos
        bounds_min = np.minimum(bounds_min, transformed.min(axis=0))
        bounds_max = np.maximum(bounds_max, transformed.max(axis=0))
    return np.vstack([bounds_min, bounds_max])


def _assert_close(actual: np.ndarray, expected: np.ndarray, tol: float, message: str) -> None:
    if np.linalg.norm(actual - expected) > tol:
        raise AssertionError(f"{message}: actual={actual.tolist()} expected={expected.tolist()} tol={tol}")


def _require_surface_thickness(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    *,
    max_thickness_z: float,
    min_span_y: float | None = None,
    min_span_x: float | None = None,
) -> list[float]:
    bounds = _body_mesh_world_bounds(model, data, body_name)
    size = bounds[1] - bounds[0]
    if size[2] > max_thickness_z:
        raise AssertionError(f"{body_name} is still standing up or too thick in z: size={size.tolist()}")
    if min_span_y is not None and size[1] < min_span_y:
        raise AssertionError(f"{body_name} span in y is too small: size={size.tolist()}")
    if min_span_x is not None and size[0] < min_span_x:
        raise AssertionError(f"{body_name} span in x is too small: size={size.tolist()}")
    return size.tolist()


def _validate_plane(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    rotor_center = _mesh_center_from_body(model, data, "rotor_4_vis")
    base_bounds = _body_mesh_world_bounds(model, data, "base_link")
    if rotor_center[0] <= base_bounds[1, 0] + 0.01:
        raise AssertionError(f"plane puller prop is not ahead of fuselage nose: {rotor_center.tolist()}")

    thrust_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "rotor_joint_thrust4")
    thrust_site_world = data.site_xpos[thrust_site_id].copy()
    if np.linalg.norm(rotor_center - thrust_site_world) > 0.12:
        raise AssertionError(
            "plane puller thrust marker too far from prop center: "
            f"rotor={rotor_center.tolist()} site={thrust_site_world.tolist()}"
        )

    left_center = _mesh_center_from_body(model, data, "left_elevon")
    right_center = _mesh_center_from_body(model, data, "right_elevon")
    if left_center[1] < 0.2 or right_center[1] > -0.2:
        raise AssertionError(
            f"plane elevon centers look detached: left={left_center.tolist()} right={right_center.tolist()}"
        )
    surface_sizes = {
        "left_elevon": _require_surface_thickness(model, data, "left_elevon", max_thickness_z=0.05, min_span_y=0.18),
        "right_elevon": _require_surface_thickness(model, data, "right_elevon", max_thickness_z=0.05, min_span_y=0.18),
        "left_flap": _require_surface_thickness(model, data, "left_flap", max_thickness_z=0.05, min_span_y=0.16),
        "right_flap": _require_surface_thickness(model, data, "right_flap", max_thickness_z=0.05, min_span_y=0.16),
        "elevator": _require_surface_thickness(model, data, "elevator", max_thickness_z=0.05, min_span_y=0.22),
        "rudder": _require_surface_thickness(model, data, "rudder", max_thickness_z=0.22, min_span_x=0.08),
    }
    return {
        "base_bounds_world": base_bounds.tolist(),
        "rotor_center_world": rotor_center.tolist(),
        "rotor_thrust_site_world": thrust_site_world.tolist(),
        "left_elevon_center_world": left_center.tolist(),
        "right_elevon_center_world": right_center.tolist(),
        "surface_sizes_world": surface_sizes,
    }


def _validate_standard_vtol(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    base_bounds = _body_mesh_world_bounds(model, data, "base_link")
    actual_size = base_bounds[1] - base_bounds[0]
    expected_size = np.array([0.93, 2.15, 0.31], dtype=float)
    if np.max(np.abs(actual_size - expected_size)) > 0.2:
        raise AssertionError(f"standard_vtol base mesh bounds unexpected: {actual_size.tolist()}")

    left_center = _mesh_center_from_body(model, data, "left_elevon")
    right_center = _mesh_center_from_body(model, data, "right_elevon")
    if left_center[1] < 0.25 or right_center[1] > -0.25:
        raise AssertionError(
            f"standard_vtol elevon centers look detached: left={left_center.tolist()} right={right_center.tolist()}"
        )
    left_size = _require_surface_thickness(model, data, "left_elevon", max_thickness_z=0.08, min_span_y=0.18)
    right_size = _require_surface_thickness(model, data, "right_elevon", max_thickness_z=0.08, min_span_y=0.18)

    puller_center = _mesh_center_from_body(model, data, "rotor_4_vis")
    if puller_center[0] >= 0.0:
        raise AssertionError(f"standard_vtol puller prop should sit behind CG: {puller_center.tolist()}")
    return {
        "base_bounds_world": base_bounds.tolist(),
        "base_size_world": actual_size.tolist(),
        "left_elevon_center_world": left_center.tolist(),
        "right_elevon_center_world": right_center.tolist(),
        "left_elevon_size_world": left_size,
        "right_elevon_size_world": right_size,
        "puller_center_world": puller_center.tolist(),
    }


def _validate_uuv(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    data.xpos[base_id].copy()
    base_rot_inv = Rotation.from_quat(data.xquat[base_id].copy(), scalar_first=True).inv()
    summary: dict[str, object] = {}
    for idx in range(8):
        rotor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{idx}")
        vis_center = _mesh_center_from_body(model, data, f"rotor_{idx}_vis")
        rotor_body_pos = data.xpos[rotor_body_id].copy()
        delta_b = base_rot_inv.apply(vis_center - rotor_body_pos)
        if np.linalg.norm(delta_b) > 0.015:
            raise AssertionError(f"uuv rotor_{idx} prop center is not seated in slot: {delta_b.tolist()}")
        summary[f"rotor_{idx}_center_world"] = vis_center.tolist()
        summary[f"rotor_{idx}_delta_body"] = delta_b.tolist()
    return summary


def _validate_case(name: str, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    if name == "plane":
        return _validate_plane(model, data)
    if name == "standard_vtol":
        return _validate_standard_vtol(model, data)
    if name == "uuv_bluerov2_heavy":
        return _validate_uuv(model, data)
    raise ValueError(f"Unsupported case {name}")


def _render(
    model: mujoco.MjModel, data: mujoco.MjData, *, camera_pos: np.ndarray, lookat: np.ndarray, output_path: Path
) -> None:
    renderer = mujoco.Renderer(model, 480, 640)
    try:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        direction = lookat - camera_pos
        distance = float(np.linalg.norm(direction))
        forward = direction / max(distance, 1e-9)
        camera.distance = distance
        camera.lookat = lookat.astype(float)
        camera.azimuth = float(np.degrees(np.arctan2(forward[1], forward[0])))
        camera.elevation = float(np.degrees(np.arcsin(forward[2])))
        renderer.update_scene(data, camera=camera)
        imageio.imwrite(output_path, renderer.render())
    finally:
        renderer.close()


def run_case(name: str, output_dir: Path) -> dict[str, object]:
    case = CASES[name]
    model, data = _load_case(case["config"])
    metrics = _validate_case(name, model, data)
    image_path = output_dir / f"{name}.png"
    _render(model, data, camera_pos=case["camera_pos"], lookat=case["lookat"], output_path=image_path)
    return {"name": name, "image": str(image_path), "metrics": metrics}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="*", default=list(CASES), choices=sorted(CASES))
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    results = [run_case(name, output_dir) for name in args.cases]
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(summary_path)
    for result in results:
        print(result["image"])


if __name__ == "__main__":
    main()
