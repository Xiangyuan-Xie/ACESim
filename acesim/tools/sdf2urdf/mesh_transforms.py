from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh


def load_trimesh_geometry(mesh_path: Path) -> trimesh.Trimesh:
    scene_or_mesh = trimesh.load(mesh_path, force="scene")
    if isinstance(scene_or_mesh, trimesh.Scene):
        return scene_or_mesh.to_geometry()
    return scene_or_mesh


def export_transformed_mesh_from_path(
    src: Path,
    dst: Path,
    *,
    center_mesh: bool = False,
    translation: list[float] | None = None,
    rotation_rpy: list[float] | None = None,
    scale_xyz: list[float] | None = None,
) -> None:
    if not src.exists():
        raise FileNotFoundError(f"Required source mesh not found: {src}")

    mesh = load_trimesh_geometry(src).copy()
    if center_mesh:
        centroid = mesh.centroid.tolist()
        mesh.apply_translation([-centroid[0], -centroid[1], -centroid[2]])

    if scale_xyz is not None:
        scale = np.eye(4)
        scale[0, 0], scale[1, 1], scale[2, 2] = scale_xyz
        mesh.apply_transform(scale)

    if rotation_rpy is not None:
        mesh.apply_transform(trimesh.transformations.euler_matrix(*rotation_rpy, axes="sxyz"))

    if translation is not None:
        mesh.apply_translation(translation)

    mesh.export(dst)
