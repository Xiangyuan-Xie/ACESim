#!/usr/bin/env python3
"""Prepare local StarterContent plus generate_acesim_testfield_meshes.py output for ACESim UE."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from acesim.tools.ue5.generate_acesim_testfield_meshes import generate_testfield_meshes

STARTER_CONTENT_FILES = [
    "Shapes/Shape_Plane.uasset",
    "Shapes/Shape_Cube.uasset",
    "Shapes/Shape_Cylinder.uasset",
    "Props/SM_Rock.uasset",
    "Props/SM_Bush.uasset",
    "Props/Materials/M_Rock.uasset",
    "Props/Materials/M_Bush.uasset",
    "Materials/M_Basic_Wall.uasset",
    "Materials/M_Concrete_Grime.uasset",
    "Materials/M_Concrete_Panels.uasset",
    "Materials/M_Metal_Rust.uasset",
    "Materials/M_Ground_Grass.uasset",
    "Materials/M_Ground_Gravel.uasset",
    "Materials/M_Concrete_Poured.uasset",
    "Textures/T_Ground_Grass_D.uasset",
    "Textures/T_Ground_Grass_N.uasset",
    "Textures/T_Ground_Gravel_D.uasset",
    "Textures/T_Ground_Gravel_N.uasset",
    "Textures/T_Concrete_Poured_D.uasset",
    "Textures/T_Concrete_Poured_N.uasset",
    "Textures/T_Concrete_Grime_D.uasset",
    "Textures/T_Concrete_Panels_D.uasset",
    "Textures/T_Concrete_Panels_N.uasset",
    "Textures/T_Detail_Rocky_N.uasset",
    "Textures/T_Rock_Basalt_D.uasset",
    "Textures/T_RockMesh_M.uasset",
    "Textures/T_RockMesh_N.uasset",
    "Textures/T_Bush_D.uasset",
    "Textures/T_Bush_N.uasset",
    "Textures/T_MacroVariation.uasset",
    "Textures/T_Metal_Rust_D.uasset",
    "Textures/T_Metal_Rust_N.uasset",
    "Textures/T_Perlin_Noise_M.uasset",
]

INSTANCED_MATERIAL_PATHS = [
    "/Game/ACESim/Environment/Materials/M_Concrete_Poured.M_Concrete_Poured",
    "/Game/ACESim/Environment/Props/Materials/M_Rock.M_Rock",
    "/Game/ACESim/Environment/Props/Materials/M_Bush.M_Bush",
]


def _write_material_usage_fix_script(project_content_dir: Path) -> Path:
    script_path = project_content_dir / "ACESim" / "Environment" / "fix_acesim_environment_materials.py"
    material_lines = ",\n".join(f'    "{path}"' for path in INSTANCED_MATERIAL_PATHS)
    script = f"""import unreal

material_paths = [
{material_lines},
]

for material_path in material_paths:
    material = unreal.EditorAssetLibrary.load_asset(material_path)
    if material is None:
        raise RuntimeError(f"ACESim environment material not found: {{material_path}}")
    material.set_editor_property("used_with_instanced_static_meshes", True)
    unreal.EditorAssetLibrary.save_loaded_asset(material)

unreal.EditorAssetLibrary.save_directory("/Game/ACESim/Environment")
unreal.log("ACESim environment material usage fixed for InstancedStaticMeshes")
"""
    script_path.parent.mkdir(parents=True, exist_ok=True)
    script_path.write_text(script, encoding="utf-8")
    return script_path


def _write_testfield_import_script(project_content_dir: Path) -> Path:
    script_path = project_content_dir / "ACESim" / "Environment" / "TestField" / "import_acesim_testfield_assets.py"
    source_dir = project_content_dir / "ACESim" / "Environment" / "TestField" / "SourceMeshes"
    destination_dir = project_content_dir / "ACESim" / "Environment" / "TestField" / "Meshes"
    script = f"""import pathlib
import unreal

unreal.SystemLibrary.execute_console_command(None, "Interchange.FeatureFlags.Import.SyncToBrowser 0")

source_dir = pathlib.Path({str(source_dir)!r})
destination_dir = pathlib.Path({str(destination_dir)!r})
destination_path = "/Game/ACESim/Environment/TestField/Meshes"
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
mesh_files = [
    "SM_TestField_Ground.obj",
    "SM_TestField_Runway.obj",
    "SM_TestField_Taxiway.obj",
    "SM_TestField_GravelSafety.obj",
    "SM_TestField_LandingPad.obj",
    "SM_TestField_RunwayCenterline.obj",
    "SM_TestField_RunwayEdgeLines.obj",
    "SM_TestField_RunwayThresholdBars.obj",
    "SM_TestField_LandingPadMarkings.obj",
    "SM_TestField_BoundaryMarker.obj",
]

if not source_dir.is_dir():
    raise RuntimeError(f"ACESim test-field source mesh directory not found: {{source_dir}}")

if destination_dir.is_dir():
    for stale_asset in destination_dir.glob("SM_TestField_*.uasset"):
        stale_asset.unlink()

tasks = []
for mesh_file in mesh_files:
    source_file = source_dir / mesh_file
    if not source_file.is_file():
        raise RuntimeError(f"ACESim test-field mesh missing: {{source_file}}")
    task = unreal.AssetImportTask()
    task.filename = str(source_file)
    task.destination_path = destination_path
    task.automated = True
    task.save = True
    task.replace_existing = True
    tasks.append(task)

asset_tools.import_asset_tasks(tasks)
unreal.EditorAssetLibrary.save_directory(destination_path)
unreal.log("ACESim offline test field meshes imported")
"""
    script_path.parent.mkdir(parents=True, exist_ok=True)
    script_path.write_text(script, encoding="utf-8")
    return script_path


def prepare_environment_assets(
    *,
    ue_src_dir: Path,
    project_content_dir: Path,
) -> Path:
    starter_root = ue_src_dir / "Samples" / "StarterContent" / "Content" / "StarterContent"
    if not starter_root.is_dir():
        raise FileNotFoundError(f"StarterContent source directory not found: {starter_root}")

    # Preserve the original StarterContent package paths so copied materials keep
    # their texture references valid inside the generated project.
    starter_dest = project_content_dir / "StarterContent"
    environment_dest = project_content_dir / "ACESim" / "Environment"
    for relative_name in STARTER_CONTENT_FILES:
        source = starter_root / relative_name
        if not source.is_file():
            raise FileNotFoundError(f"StarterContent asset not found: {source}")
        destination = starter_dest / relative_name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)

        environment_copy = environment_dest / relative_name
        environment_copy.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, environment_copy)

    generate_testfield_meshes(environment_dest / "TestField" / "SourceMeshes")
    _write_testfield_import_script(project_content_dir)
    _write_material_usage_fix_script(project_content_dir)
    return environment_dest


def main() -> None:
    parser = argparse.ArgumentParser(description="Copy the local StarterContent subset used by ACESim UE.")
    parser.add_argument(
        "--ue-src-dir",
        type=Path,
        default=Path("/tmp/ACESim-unreal/UnrealEngine"),
        help="Unreal Engine source/install root containing Samples/StarterContent.",
    )
    parser.add_argument(
        "--project-content-dir",
        type=Path,
        default=Path("/tmp/ACESim-unreal/projects/ACESimUE/Content"),
        help="Generated ACESimUE Content directory.",
    )
    args = parser.parse_args()

    destination = prepare_environment_assets(
        ue_src_dir=args.ue_src_dir.expanduser().resolve(),
        project_content_dir=args.project_content_dir.expanduser().resolve(),
    )
    print(f"Prepared ACESim UE environment assets: {destination}")


if __name__ == "__main__":
    main()
