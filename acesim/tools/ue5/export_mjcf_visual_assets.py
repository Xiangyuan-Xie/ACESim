#!/usr/bin/env python3
"""Export MuJoCo visual meshes into UE-friendly source assets."""

from __future__ import annotations

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import trimesh

DEFAULT_ASSET_NAME = "x500_arm2x"
MUJOCO_METERS_TO_UE_CENTIMETERS = 100.0


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _asset_dir(asset_name: str) -> Path:
    return _repo_root() / "acesim" / "env" / "mujoco" / "asset" / asset_name


def _mesh_to_obj(src: Path, dst: Path) -> None:
    mesh = trimesh.load(src, force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    mesh.apply_scale(MUJOCO_METERS_TO_UE_CENTIMETERS)
    dst.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(dst)


def _find_visual_mesh_names(root: ET.Element) -> set[str]:
    names: set[str] = set()
    for geom in root.findall(".//geom"):
        if geom.get("type") != "mesh":
            continue
        mesh_name = geom.get("mesh")
        if not mesh_name or "_decomp_" in mesh_name:
            continue
        group = geom.get("group")
        contype = geom.get("contype")
        conaffinity = geom.get("conaffinity")
        if group == "1" or (contype == "0" and conaffinity == "0"):
            names.add(mesh_name)
    return names


def _parse_float_list(value: str | None, default: list[float]) -> list[float]:
    if value is None or not value.strip():
        return list(default)
    return [float(part) for part in value.split()]


def _joint_type(joint: ET.Element) -> str:
    return joint.get("type", "hinge")


def _find_arm_joints(root: ET.Element) -> list[dict[str, Any]]:
    joints: list[dict[str, Any]] = []
    for joint in root.findall(".//joint"):
        name = joint.get("name", "")
        if not name.startswith("joint_"):
            continue
        axis = [float(value) for value in joint.get("axis", "0 0 1").split()]
        joints.append(
            {
                "name": name,
                "type": _joint_type(joint),
                "axis_mjcf": axis,
                "range": _parse_float_list(joint.get("range"), []),
            }
        )
    return joints


def _visual_geoms_for_body(body: ET.Element, visual_names: set[str]) -> list[dict[str, Any]]:
    geoms: list[dict[str, Any]] = []
    for geom in body.findall("geom"):
        if geom.get("type") != "mesh":
            continue
        mesh_name = geom.get("mesh", "")
        if mesh_name not in visual_names:
            continue
        geoms.append(
            {
                "name": geom.get("name", mesh_name),
                "mesh": mesh_name,
                "pos": _parse_float_list(geom.get("pos"), [0.0, 0.0, 0.0]),
                "quat": _parse_float_list(geom.get("quat"), [1.0, 0.0, 0.0, 0.0]),
                "rgba": _parse_float_list(geom.get("rgba"), [1.0, 1.0, 1.0, 1.0]),
            }
        )
    return geoms


def _all_bodies(root: ET.Element) -> dict[str, ET.Element]:
    bodies: dict[str, ET.Element] = {}
    for body in root.findall(".//body"):
        name = body.get("name")
        if name:
            bodies[name] = body
    return bodies


def _body_joint(body: ET.Element) -> dict[str, Any] | None:
    joint = body.find("joint")
    if joint is None:
        return None
    name = joint.get("name", "")
    if not name.startswith("joint_"):
        return None
    return {
        "name": name,
        "type": _joint_type(joint),
        "axis": _parse_float_list(joint.get("axis"), [0.0, 0.0, 1.0]),
        "range": _parse_float_list(joint.get("range"), []),
    }


def _build_visual_body_manifest(
    root: ET.Element, visual_names: set[str]
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MuJoCo XML is missing <worldbody>")

    root_body_element = worldbody.find("body")
    if root_body_element is None or not root_body_element.get("name"):
        raise ValueError("MuJoCo XML is missing a named root body")

    all_bodies = _all_bodies(root)
    root_body_name = str(root_body_element.get("name"))
    visual_bodies: list[dict[str, Any]] = []

    def append_body(body: ET.Element, parent_name: str | None) -> bool:
        name = body.get("name")
        if not name:
            return False

        geoms = _visual_geoms_for_body(body, visual_names)
        child_entries: list[dict[str, Any]] = []
        for child in body.findall("body"):
            before = len(visual_bodies)
            if append_body(child, name):
                child_entries.extend(visual_bodies[before:])

        include_body = bool(geoms or child_entries or _body_joint(body))
        if not include_body:
            return False

        visual_bodies.insert(
            len(visual_bodies) - len(child_entries),
            {
                "name": name,
                "parent": parent_name,
                "pos": _parse_float_list(body.get("pos"), [0.0, 0.0, 0.0]),
                "quat": _parse_float_list(body.get("quat"), [1.0, 0.0, 0.0, 0.0]),
                "mocap": body.get("mocap") == "true",
                "joint": _body_joint(body),
                "geoms": geoms,
            },
        )
        return True

    append_body(root_body_element, None)

    # Rotor visual bodies are mocap-only in MuJoCo; their visual mesh is mounted
    # at runtime onto the physical rotor body pose. Mirror that mounted pose in
    # UE so the model starts in the same home layout as the XML.
    for rotor_index in range(1, 5):
        visual_body = all_bodies.get(f"rotor_{rotor_index}_vis")
        physical_body = all_bodies.get(f"rotor_{rotor_index}")
        if visual_body is None or physical_body is None:
            continue
        geoms = _visual_geoms_for_body(visual_body, visual_names)
        if not geoms:
            continue
        visual_bodies.append(
            {
                "name": f"rotor_{rotor_index}_vis",
                "parent": root_body_name,
                "pos": _parse_float_list(physical_body.get("pos"), [0.0, 0.0, 0.0]),
                "quat": _parse_float_list(physical_body.get("quat"), [1.0, 0.0, 0.0, 0.0]),
                "mocap": True,
                "joint": None,
                "geoms": geoms,
                "rotor_index": rotor_index,
            }
        )

    root_body = {
        "name": root_body_name,
        "pos": _parse_float_list(root_body_element.get("pos"), [0.0, 0.0, 0.0]),
        "quat": _parse_float_list(root_body_element.get("quat"), [1.0, 0.0, 0.0, 0.0]),
    }
    return root_body, visual_bodies


def export_asset(
    asset_name: str = DEFAULT_ASSET_NAME,
    output_root: Path | str = Path("/tmp/ACESim-unreal/projects/ACESimUE/Content/ACESim"),
) -> dict[str, Any]:
    asset_dir = _asset_dir(asset_name)
    mjcf_path = asset_dir / f"{asset_name}.xml"
    if not mjcf_path.is_file():
        raise FileNotFoundError(f"MuJoCo asset XML not found: {mjcf_path}")

    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "meshes/") if compiler is not None else "meshes/"
    mesh_dir = asset_dir / meshdir
    mesh_assets = {
        mesh.get("name"): mesh.get("file")
        for mesh in root.findall("./asset/mesh")
        if mesh.get("name") and mesh.get("file")
    }
    visual_names = sorted(_find_visual_mesh_names(root))
    root_body, visual_bodies = _build_visual_body_manifest(root, set(visual_names))
    asset_output_root = Path(output_root).expanduser().resolve() / asset_name
    source_mesh_root = asset_output_root / "SourceMeshes"
    source_mesh_root.mkdir(parents=True, exist_ok=True)

    exported_meshes: list[dict[str, str]] = []
    for mesh_name in visual_names:
        mesh_file = mesh_assets.get(mesh_name)
        if mesh_file is None or "_decomp_" in mesh_name:
            continue
        src = mesh_dir / mesh_file
        dst = source_mesh_root / f"{mesh_name}.obj"
        _mesh_to_obj(src, dst)
        exported_meshes.append(
            {
                "name": mesh_name,
                "source": str(dst),
                "ue_path": f"/Game/ACESim/{asset_name}/{mesh_name}.{mesh_name}",
            }
        )

    manifest: dict[str, Any] = {
        "asset_name": asset_name,
        "root_body": root_body,
        "visual_bodies": visual_bodies,
        "meshes": exported_meshes,
        "rotors": [name for name in [f"rotor_{index}" for index in range(1, 5)] if name in visual_names],
        "arm_joints": _find_arm_joints(root),
    }
    manifest_path = asset_output_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (asset_output_root / "visual_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    _write_unreal_import_script(asset_output_root, manifest)
    return manifest


def _write_unreal_import_script(asset_output_root: Path, manifest: dict[str, Any]) -> None:
    script_path = asset_output_root / "import_acesim_assets.py"
    lines = [
        "import unreal",
        "",
        "unreal.SystemLibrary.execute_console_command(",
        "    None,",
        '    "Interchange.FeatureFlags.Import.SyncToBrowser 0",',
        ")",
        "",
        "asset_tools = unreal.AssetToolsHelpers.get_asset_tools()",
        f'destination_path = "/Game/ACESim/{manifest["asset_name"]}"',
        "tasks = []",
    ]
    for mesh in manifest["meshes"]:
        lines.extend(
            [
                "task = unreal.AssetImportTask()",
                f'task.filename = r"{mesh["source"]}"',
                "task.destination_path = destination_path",
                "task.automated = True",
                "task.replace_existing = True",
                "task.save = True",
                "tasks.append(task)",
            ]
        )
    lines.extend(
        [
            "asset_tools.import_asset_tasks(tasks)",
            "unreal.EditorAssetLibrary.save_directory(destination_path)",
            "",
        ]
    )
    script_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Export MuJoCo visual meshes for the ACESim UE runtime.")
    parser.add_argument("--asset", default=DEFAULT_ASSET_NAME, help="MuJoCo asset name to export.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("/tmp/ACESim-unreal/projects/ACESimUE/Content/ACESim"),
        help="UE Content/ACESim output directory.",
    )
    args = parser.parse_args()

    output_root = args.output_root.expanduser().resolve()
    if output_root.exists():
        target = output_root / args.asset
        if target.exists():
            shutil.rmtree(target)
    manifest = export_asset(args.asset, output_root)
    print(f"Exported {len(manifest['meshes'])} visual meshes for {args.asset} to {output_root / args.asset}")


if __name__ == "__main__":
    main()
