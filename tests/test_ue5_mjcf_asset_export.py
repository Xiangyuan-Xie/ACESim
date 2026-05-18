from pathlib import Path

import pytest
import trimesh

from acesim.tools.ue5 import export_mjcf_visual_assets

ROOT = Path(__file__).resolve().parents[1]


def test_export_mjcf_visual_assets_writes_real_x500_arm_mesh_manifest(tmp_path: Path) -> None:
    output_root = tmp_path / "Content" / "ACESim"

    manifest = export_mjcf_visual_assets.export_asset("x500_arm2x", output_root)

    asset_root = output_root / "x500_arm2x"
    source_meshes = asset_root / "SourceMeshes"
    manifest_path = asset_root / "manifest.json"
    assert manifest_path.is_file()
    assert (source_meshes / "base_link.obj").is_file()
    assert (source_meshes / "rotor_1.obj").is_file()
    assert (source_meshes / "link_5.obj").is_file()
    assert not any("_decomp_" in path.name for path in source_meshes.iterdir())

    names = {mesh["name"] for mesh in manifest["meshes"]}
    assert {"base_link", "rotor_1", "rotor_2", "rotor_3", "rotor_4", "link_0", "link_5"} <= names
    assert all("_decomp_" not in name for name in names)
    assert [joint["name"] for joint in manifest["arm_joints"]] == [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_gripper_left",
        "joint_gripper_right",
    ]
    assert manifest["rotors"] == ["rotor_1", "rotor_2", "rotor_3", "rotor_4"]
    assert manifest["root_body"]["name"] == "base_link"
    visual_body_names = {body["name"] for body in manifest["visual_bodies"]}
    assert {"base_link", "link_1", "link_5", "gripper_left", "gripper_right", "rotor_1_vis"} <= visual_body_names
    base_body = next(body for body in manifest["visual_bodies"] if body["name"] == "base_link")
    assert base_body["pos"] == pytest.approx([0.0, 0.0, 0.269138])
    assert any(geom["mesh"] == "link_0" for geom in base_body["geoms"])
    link_1_body = next(body for body in manifest["visual_bodies"] if body["name"] == "link_1")
    assert link_1_body["parent"] == "base_link"
    assert link_1_body["joint"]["name"] == "joint_1"
    assert link_1_body["joint"]["type"] == "hinge"
    gripper_left_body = next(body for body in manifest["visual_bodies"] if body["name"] == "gripper_left")
    assert gripper_left_body["joint"]["type"] == "slide"
    assert gripper_left_body["joint"]["axis"] == pytest.approx([0.0, 0.0, 1.0])
    rotor_1_body = next(body for body in manifest["visual_bodies"] if body["name"] == "rotor_1_vis")
    assert rotor_1_body["mocap"] is True

    import_script = (asset_root / "import_acesim_assets.py").read_text(encoding="utf-8")
    assert "Interchange.FeatureFlags.Import.SyncToBrowser 0" in import_script
    assert import_script.index("Interchange.FeatureFlags.Import.SyncToBrowser 0") < import_script.index(
        "import_asset_tasks"
    )


def test_export_mjcf_visual_assets_scales_obj_vertices_to_ue_centimeters(tmp_path: Path) -> None:
    output_root = tmp_path / "Content" / "ACESim"

    export_mjcf_visual_assets.export_asset("x500_arm2x", output_root)

    source_stl = ROOT / "acesim" / "env" / "mujoco" / "asset" / "x500_arm2x" / "meshes" / "base_link.STL"
    exported_obj = output_root / "x500_arm2x" / "SourceMeshes" / "base_link.obj"
    source_mesh = trimesh.load(source_stl, force="mesh")
    exported_mesh = trimesh.load(exported_obj, force="mesh")

    assert exported_mesh.bounding_box.extents.tolist() == pytest.approx(
        (source_mesh.bounding_box.extents * 100.0).tolist(),
        rel=1e-5,
        abs=1e-5,
    )
