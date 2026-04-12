import copy
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh

from .asset_context import AssetPaths


def _load_trimesh_geometry(mesh_path: Path) -> trimesh.Trimesh:
    scene_or_mesh = trimesh.load(mesh_path, force="scene")
    if isinstance(scene_or_mesh, trimesh.Scene):
        return scene_or_mesh.to_geometry()
    return scene_or_mesh


def clean_artifacts(paths: AssetPaths) -> None:
    if paths.xml_path.exists():
        paths.xml_path.unlink()

    if not paths.mesh_dir.exists():
        return

    for mesh_path in paths.mesh_dir.glob("*_decomp_*.stl"):
        mesh_path.unlink()


def export_mesh_copy(src: Path, dst: Path) -> None:
    if not src.exists():
        raise FileNotFoundError(f"Required source mesh not found: {src}")
    mesh = _load_trimesh_geometry(src).copy()
    mesh.export(dst)


def export_translated_mesh(
    mesh_dir: Path,
    src_name: str,
    dst_name: str,
    translation: list[float],
    *,
    source_units_to_output: float = 1.0,
) -> None:
    src = mesh_dir / src_name
    dst = mesh_dir / dst_name
    if not src.exists():
        raise FileNotFoundError(f"Required source mesh not found: {src}")
    mesh = _load_trimesh_geometry(src).copy()
    mesh.apply_translation([t * source_units_to_output for t in translation])
    mesh.export(dst)


def export_centered_mesh(mesh_dir: Path, src_name: str, dst_name: str) -> None:
    src = mesh_dir / src_name
    dst = mesh_dir / dst_name
    if not src.exists():
        raise FileNotFoundError(f"Required source mesh not found: {src}")
    mesh = _load_trimesh_geometry(src).copy()
    centroid = mesh.centroid.tolist()
    mesh.apply_translation([-centroid[0], -centroid[1], -centroid[2]])
    mesh.export(dst)


def export_centered_mesh_from_path(src: Path, dst: Path) -> None:
    if not src.exists():
        raise FileNotFoundError(f"Required source mesh not found: {src}")
    mesh = _load_trimesh_geometry(src).copy()
    centroid = mesh.centroid.tolist()
    mesh.apply_translation([-centroid[0], -centroid[1], -centroid[2]])
    mesh.export(dst)


def mesh_bounds_center(mesh_dir: Path, mesh_name: str, *, mesh_scale: list[float] | None = None) -> list[float]:
    mesh = _load_trimesh_geometry(mesh_dir / mesh_name)
    center = ((mesh.bounds[0] + mesh.bounds[1]) / 2.0).tolist()
    if mesh_scale is not None:
        center = [center[i] * mesh_scale[i] for i in range(3)]
    return center


def decompose_mesh(mesh_path: Path, output_dir: Path, threshold: float = 0.2, resolution: int = 50) -> list[Path]:
    try:
        import coacd
    except ImportError as exc:
        raise ImportError("coacd is required when --decompose is enabled") from exc

    mesh = trimesh.load(mesh_path, force="mesh")
    coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    parts = coacd.run_coacd(coacd_mesh, threshold=threshold, preprocess_resolution=resolution)

    mesh_name = mesh_path.stem
    output_paths: list[Path] = []
    for idx, (vertices, faces) in enumerate(parts):
        part_mesh = trimesh.Trimesh(vertices, faces)
        export_path = output_dir / f"{mesh_name}_decomp_{idx}.stl"
        part_mesh.export(export_path)
        output_paths.append(export_path)
    return output_paths


def process_urdf_collisions(
    urdf_path: Path,
    mesh_dir: Path,
    threshold: float = 0.2,
    resolution: int = 50,
) -> Path:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for link in root.findall("link"):
        for collision in list(link.findall("collision")):
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            mesh_elem = geometry.find("mesh")
            if mesh_elem is None:
                continue

            filename = mesh_elem.get("filename")
            if not filename:
                continue

            mesh_path = mesh_dir / os.path.basename(filename)
            if not mesh_path.exists():
                raise FileNotFoundError(f"Collision mesh not found: {mesh_path}")

            parts_paths = decompose_mesh(mesh_path, mesh_dir, threshold, resolution)
            if len(parts_paths) <= 1 and parts_paths[0] == mesh_path:
                continue

            link.remove(collision)
            for part_path in parts_paths:
                new_collision = copy.deepcopy(collision)
                new_geom = new_collision.find("geometry")
                if new_geom is None:
                    raise ValueError(f"Collision geometry missing in link {link.get('name')}")
                new_mesh = new_geom.find("mesh")
                if new_mesh is None:
                    raise ValueError(f"Collision mesh missing in link {link.get('name')}")

                if "/" in filename or "\\" in filename:
                    prefix = os.path.dirname(filename)
                    new_filename = os.path.join(prefix, part_path.name).replace("\\", "/")
                else:
                    new_filename = part_path.name

                new_mesh.set("filename", new_filename)
                link.append(new_collision)

    new_urdf_path = urdf_path.with_name(f"{urdf_path.stem}_decomposed_tmp.urdf")
    tree.write(new_urdf_path, encoding="utf-8", xml_declaration=True)
    return new_urdf_path
