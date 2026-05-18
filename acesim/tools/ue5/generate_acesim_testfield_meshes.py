#!/usr/bin/env python3
"""Generate lightweight OBJ meshes for the offline ACESim UE test field."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

MeshData = tuple[list[tuple[float, float, float]], list[tuple[float, float]], list[tuple[int, ...]]]
OBJ_MATERIAL_NAME = "ACESimTestFieldSurface"


def _uv_for_vertex(x: float, y: float, uv_scale_cm: float) -> tuple[float, float]:
    return (x / uv_scale_cm, y / uv_scale_cm)


def _with_planar_uv(
    vertices: list[tuple[float, float, float]],
    faces: list[tuple[int, ...]],
    *,
    uv_scale_cm: float,
) -> MeshData:
    return vertices, [_uv_for_vertex(x, y, uv_scale_cm) for x, y, _ in vertices], faces


def _write_obj(
    path: Path,
    vertices: list[tuple[float, float, float]],
    uvs: list[tuple[float, float]],
    faces: list[tuple[int, ...]],
    *,
    material_name: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    material_file = path.with_suffix(".mtl").name
    lines = [f"mtllib {material_file}", f"o {path.stem}", f"usemtl {material_name}"]
    lines.extend(f"v {x:.4f} {y:.4f} {z:.4f}" for x, y, z in vertices)
    lines.extend(f"vt {u:.6f} {v:.6f}" for u, v in uvs)
    lines.extend(
        "f " + " ".join(f"{vertex_index}/{uv_index}" for vertex_index, uv_index in zip(face, face, strict=True))
        for face in faces
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    path.with_suffix(".mtl").write_text(
        "\n".join(
            [
                f"newmtl {material_name}",
                "Ka 0.2500 0.2500 0.2500",
                "Kd 0.8200 0.8000 0.7400",
                "Ks 0.0500 0.0500 0.0500",
                "Ns 8.0",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _grid_mesh(
    width_cm: float, depth_cm: float, cells_x: int, cells_y: int, z_scale: float, uv_scale_cm: float
) -> MeshData:
    vertices: list[tuple[float, float, float]] = []
    for y_index in range(cells_y + 1):
        y = -depth_cm * 0.5 + depth_cm * y_index / cells_y
        for x_index in range(cells_x + 1):
            x = -width_cm * 0.5 + width_cm * x_index / cells_x
            z = ((x_index % 3) - 1) * z_scale + ((y_index % 4) - 1.5) * z_scale * 0.55
            vertices.append((x, y, z))

    faces: list[tuple[int, ...]] = []
    stride = cells_x + 1
    for y_index in range(cells_y):
        for x_index in range(cells_x):
            a = y_index * stride + x_index + 1
            b = a + 1
            c = a + stride + 1
            d = a + stride
            faces.append((a, b, c, d))
    return _with_planar_uv(vertices, faces, uv_scale_cm=uv_scale_cm)


def _rect_mesh(width_cm: float, depth_cm: float, z_cm: float = 0.0, uv_scale_cm: float = 240.0) -> MeshData:
    half_w = width_cm * 0.5
    half_d = depth_cm * 0.5
    vertices = [
        (-half_w, -half_d, z_cm),
        (half_w, -half_d, z_cm),
        (half_w, half_d, z_cm),
        (-half_w, half_d, z_cm),
    ]
    return _with_planar_uv(vertices, [(1, 2, 3, 4)], uv_scale_cm=uv_scale_cm)


def _rect_at_mesh(
    width_cm: float,
    depth_cm: float,
    center_x_cm: float,
    center_y_cm: float,
    z_cm: float,
    uv_scale_cm: float,
) -> MeshData:
    return _offset_mesh(_rect_mesh(width_cm, depth_cm, z_cm, uv_scale_cm), center_x_cm, center_y_cm, 0.0)


def _merge_meshes(meshes: Iterable[MeshData]) -> MeshData:
    vertices: list[tuple[float, float, float]] = []
    uvs: list[tuple[float, float]] = []
    faces: list[tuple[int, ...]] = []
    for mesh_vertices, mesh_uvs, mesh_faces in meshes:
        index_offset = len(vertices)
        vertices.extend(mesh_vertices)
        uvs.extend(mesh_uvs)
        faces.extend(tuple(index + index_offset for index in face) for face in mesh_faces)
    return vertices, uvs, faces


def _disc_mesh(radius_cm: float, segments: int, z_cm: float = 0.0, uv_scale_cm: float = 180.0) -> MeshData:
    import math

    vertices = [(0.0, 0.0, z_cm)]
    for index in range(segments):
        angle = math.tau * index / segments
        vertices.append((math.cos(angle) * radius_cm, math.sin(angle) * radius_cm, z_cm))
    faces = [(1, *range(2, segments + 2))]
    return _with_planar_uv(vertices, faces, uv_scale_cm=uv_scale_cm)


def _ring_mesh(
    outer_radius_cm: float,
    inner_radius_cm: float,
    segments: int,
    z_cm: float,
    uv_scale_cm: float,
) -> MeshData:
    import math

    vertices: list[tuple[float, float, float]] = []
    for index in range(segments):
        angle = math.tau * index / segments
        vertices.append((math.cos(angle) * outer_radius_cm, math.sin(angle) * outer_radius_cm, z_cm))
        vertices.append((math.cos(angle) * inner_radius_cm, math.sin(angle) * inner_radius_cm, z_cm))
    faces: list[tuple[int, ...]] = []
    for index in range(segments):
        outer_a = index * 2 + 1
        inner_a = outer_a + 1
        outer_b = ((index + 1) % segments) * 2 + 1
        inner_b = outer_b + 1
        faces.append((outer_a, outer_b, inner_b, inner_a))
    return _with_planar_uv(vertices, faces, uv_scale_cm=uv_scale_cm)


def _box_mesh(width_cm: float, depth_cm: float, height_cm: float, uv_scale_cm: float = 120.0) -> MeshData:
    hw = width_cm * 0.5
    hd = depth_cm * 0.5
    vertices = [
        (-hw, -hd, 0.0),
        (hw, -hd, 0.0),
        (hw, hd, 0.0),
        (-hw, hd, 0.0),
        (-hw, -hd, height_cm),
        (hw, -hd, height_cm),
        (hw, hd, height_cm),
        (-hw, hd, height_cm),
    ]
    faces: list[tuple[int, ...]] = [
        (1, 2, 3, 4),
        (5, 8, 7, 6),
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (3, 7, 8, 4),
        (4, 8, 5, 1),
    ]
    return _with_planar_uv(vertices, faces, uv_scale_cm=uv_scale_cm)


def _runway_centerline_mesh() -> MeshData:
    return _merge_meshes(
        _rect_at_mesh(18.0, 150.0, -720.0, y_cm, 3.0, 80.0)
        for y_cm in (-1120.0, -840.0, -560.0, -280.0, 0.0, 280.0, 560.0, 840.0, 1120.0)
    )


def _runway_edge_lines_mesh() -> MeshData:
    return _merge_meshes(
        [
            _rect_at_mesh(14.0, 2620.0, -920.0, 0.0, 3.1, 120.0),
            _rect_at_mesh(14.0, 2620.0, -520.0, 0.0, 3.1, 120.0),
        ]
    )


def _runway_threshold_bars_mesh() -> MeshData:
    bars: list[MeshData] = []
    for y_cm in (-1275.0, 1275.0):
        for x_cm in (-845.0, -785.0, -725.0, -665.0, -605.0):
            bars.append(_rect_at_mesh(28.0, 135.0, x_cm, y_cm, 3.2, 64.0))
    return _merge_meshes(bars)


def _landing_pad_markings_mesh() -> MeshData:
    return _merge_meshes(
        [
            _ring_mesh(300.0, 278.0, 96, 3.4, 80.0),
            _rect_at_mesh(38.0, 260.0, -82.0, 0.0, 3.4, 70.0),
            _rect_at_mesh(38.0, 260.0, 82.0, 0.0, 3.4, 70.0),
            _rect_at_mesh(206.0, 38.0, 0.0, 0.0, 3.4, 70.0),
        ]
    )


def _offset(
    vertices: Iterable[tuple[float, float, float]], dx: float, dy: float, dz: float
) -> list[tuple[float, float, float]]:
    return [(x + dx, y + dy, z + dz) for x, y, z in vertices]


def _offset_mesh(mesh: MeshData, dx: float, dy: float, dz: float) -> MeshData:
    vertices, uvs, faces = mesh
    return _offset(vertices, dx, dy, dz), uvs, faces


def generate_testfield_meshes(output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    meshes = {
        "SM_TestField_Ground.obj": _grid_mesh(9000.0, 6800.0, 18, 14, 1.2, 520.0),
        "SM_TestField_Runway.obj": _offset_mesh(_rect_mesh(440.0, 3000.0, 1.2, 220.0), -780.0, 0.0, 0.0),
        "SM_TestField_Taxiway.obj": _offset_mesh(_rect_mesh(280.0, 1060.0, 1.6, 220.0), -330.0, -130.0, 0.0),
        "SM_TestField_GravelSafety.obj": _rect_mesh(1500.0, 1660.0, 0.8, 320.0),
        "SM_TestField_LandingPad.obj": _disc_mesh(340.0, 96, 2.4, 220.0),
        "SM_TestField_RunwayCenterline.obj": _runway_centerline_mesh(),
        "SM_TestField_RunwayEdgeLines.obj": _runway_edge_lines_mesh(),
        "SM_TestField_RunwayThresholdBars.obj": _runway_threshold_bars_mesh(),
        "SM_TestField_LandingPadMarkings.obj": _landing_pad_markings_mesh(),
        "SM_TestField_BoundaryMarker.obj": _box_mesh(120.0, 18.0, 28.0, 90.0),
    }
    for name, (vertices, uvs, faces) in meshes.items():
        _write_obj(output_dir / name, vertices, uvs, faces, material_name=OBJ_MATERIAL_NAME)
    return output_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ACESim offline test-field OBJ meshes.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/tmp/ACESim-unreal/projects/ACESimUE/Content/ACESim/Environment/TestField/SourceMeshes"),
    )
    args = parser.parse_args()
    destination = generate_testfield_meshes(args.output_dir.expanduser().resolve())
    print(f"Generated ACESim test field meshes: {destination}")


if __name__ == "__main__":
    main()
