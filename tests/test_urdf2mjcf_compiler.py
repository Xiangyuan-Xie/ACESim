from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from acesim.tools.urdf2mjcf.asset_context import AssetPaths, AssetToolchainConfig
from acesim.tools.urdf2mjcf.mjcf_ops import postprocess_xml
from acesim.tools.urdf2mjcf.mujoco_compiler import (
    compile_urdf_to_xml_with_available_backend,
    write_python_mujoco_compatible_urdf,
)


class URDF2MJCFCompilerTests(unittest.TestCase):
    def test_available_backend_uses_python_mujoco_when_compile_binary_is_missing(self) -> None:
        config = AssetToolchainConfig(target="x500_arm2x")
        urdf_path = Path("robot.urdf")
        xml_path = Path("robot.xml")

        with (
            patch(
                "acesim.tools.urdf2mjcf.mujoco_compiler.find_mujoco_compile_binary",
                side_effect=FileNotFoundError("missing"),
            ),
            patch("acesim.tools.urdf2mjcf.mujoco_compiler.compile_urdf_to_xml") as binary_compile,
            patch("acesim.tools.urdf2mjcf.mujoco_compiler.compile_urdf_to_xml_with_python") as python_compile,
        ):
            compile_urdf_to_xml_with_available_backend(config, urdf_path, xml_path)

        binary_compile.assert_not_called()
        python_compile.assert_called_once_with(urdf_path, xml_path, preserve_static_root=True)

    def test_available_backend_allows_static_fusion_for_floating_urdfs(self) -> None:
        config = AssetToolchainConfig(target="x500_arm2x", floating=True)

        with (
            patch(
                "acesim.tools.urdf2mjcf.mujoco_compiler.find_mujoco_compile_binary",
                side_effect=FileNotFoundError("missing"),
            ),
            patch("acesim.tools.urdf2mjcf.mujoco_compiler.compile_urdf_to_xml_with_python") as python_compile,
        ):
            compile_urdf_to_xml_with_available_backend(config, Path("robot.urdf"), Path("robot.xml"))

        python_compile.assert_called_once_with(Path("robot.urdf"), Path("robot.xml"), preserve_static_root=False)

    def test_available_backend_rejects_missing_explicit_compile_binary(self) -> None:
        config = AssetToolchainConfig(target="x500_arm2x", mujoco_bin="/missing/compile")

        with (
            patch(
                "acesim.tools.urdf2mjcf.mujoco_compiler.find_mujoco_compile_binary",
                side_effect=FileNotFoundError("missing"),
            ),
            patch("acesim.tools.urdf2mjcf.mujoco_compiler.compile_urdf_to_xml_with_python") as python_compile,
        ):
            with self.assertRaises(FileNotFoundError):
                compile_urdf_to_xml_with_available_backend(config, Path("robot.urdf"), Path("robot.xml"))

        python_compile.assert_not_called()

    def test_python_mujoco_compatible_urdf_matches_legacy_compile_assumptions(self) -> None:
        urdf_path = Path("robot_tmp.urdf")
        compatible_urdf_path = Path("robot_tmp_python_mujoco_tmp.urdf")
        urdf_path.write_text(
            """
<robot name="robot">
  <mujoco>
    <compiler meshdir="meshes/" texturedir="meshes/" />
  </mujoco>
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="meshes/base_link.STL" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="meshes/base_link_decomp_0.stl" />
      </geometry>
    </collision>
  </link>
</robot>
""".strip(),
            encoding="utf-8",
        )

        try:
            result = write_python_mujoco_compatible_urdf(urdf_path, preserve_static_root=True)
            text = result.read_text(encoding="utf-8")
        finally:
            urdf_path.unlink(missing_ok=True)
            compatible_urdf_path.unlink(missing_ok=True)

        self.assertEqual(result, compatible_urdf_path)
        self.assertIn('meshdir="meshes/"', text)
        self.assertIn('fusestatic="false"', text)
        self.assertIn('filename="base_link.STL"', text)
        self.assertIn('filename="base_link_decomp_0.stl"', text)
        self.assertNotIn('filename="meshes/', text)

    def test_python_mujoco_compatible_urdf_can_keep_static_fusion_for_floating_roots(self) -> None:
        urdf_path = Path("robot_tmp.urdf")
        compatible_urdf_path = Path("robot_tmp_python_mujoco_tmp.urdf")
        urdf_path.write_text(
            """
<robot name="robot">
  <mujoco>
    <compiler meshdir="meshes/" texturedir="meshes/" />
  </mujoco>
  <link name="world_root_dummy_link" />
  <link name="base_link" />
  <joint name="floating_base_joint" type="floating">
    <parent link="world_root_dummy_link" />
    <child link="base_link" />
  </joint>
</robot>
""".strip(),
            encoding="utf-8",
        )

        try:
            result = write_python_mujoco_compatible_urdf(urdf_path, preserve_static_root=False)
            text = result.read_text(encoding="utf-8")
        finally:
            urdf_path.unlink(missing_ok=True)
            compatible_urdf_path.unlink(missing_ok=True)

        self.assertEqual(result, compatible_urdf_path)
        self.assertNotIn("fusestatic", text)

    def test_postprocess_rotor_visual_mocaps_ignore_decomposed_collision_meshes(self) -> None:
        with TemporaryDirectory(prefix="acesim_urdf2mjcf_postprocess_") as tmpdir:
            root_dir = Path(tmpdir)
            mesh_dir = root_dir / "meshes"
            mesh_dir.mkdir()
            xml_path = root_dir / "robot.xml"
            urdf_path = root_dir / "robot.urdf"
            xml_path.write_text(
                """
<mujoco model="x500_arm2x">
  <compiler angle="radian" meshdir="meshes/" texturedir="meshes/" />
  <asset>
    <mesh name="rotor_1" file="rotor_1.STL" />
    <mesh name="rotor_1_decomp_0" file="rotor_1_decomp_0.stl" />
  </asset>
  <worldbody>
    <body name="base_link" pos="0 0 0.3">
      <joint name="floating_base_joint" type="free" />
      <body name="rotor_1" pos="0.1 -0.2 0.03">
        <joint name="joint_rotor_1" axis="0 0 1" />
        <geom type="mesh" mesh="rotor_1" rgba="0.25 0.25 0.25 1" />
        <geom type="mesh" mesh="rotor_1_decomp_0" rgba="0.25 0.25 0.25 1" />
      </body>
    </body>
  </worldbody>
</mujoco>
""".strip(),
                encoding="utf-8",
            )
            paths = AssetPaths(base_dir=root_dir, urdf_path=urdf_path, mesh_dir=mesh_dir, xml_path=xml_path)

            postprocess_xml(
                xml_path,
                config=AssetToolchainConfig(target="x500_arm2x", floating=True, decompose=True),
                paths=paths,
                initial_q={},
                height_offset=0.3,
            )

            root = ET.parse(xml_path).getroot()
            visual_body = root.find("./worldbody/body[@name='rotor_1_vis']")
            self.assertIsNotNone(visual_body)
            assert visual_body is not None
            meshes = [geom.get("mesh") for geom in visual_body.findall("geom")]
            self.assertEqual(meshes, ["rotor_1"])
            visual_geom = visual_body.find("geom")
            self.assertIsNotNone(visual_geom)
            assert visual_geom is not None
            self.assertEqual(visual_geom.get("group"), "1")
            self.assertEqual(visual_geom.get("contype"), "0")
            self.assertEqual(visual_geom.get("conaffinity"), "0")

    def test_postprocess_injects_x500_arm2x_actuator_gains(self) -> None:
        with TemporaryDirectory(prefix="acesim_urdf2mjcf_actuators_") as tmpdir:
            root_dir = Path(tmpdir)
            mesh_dir = root_dir / "meshes"
            mesh_dir.mkdir()
            xml_path = root_dir / "robot.xml"
            urdf_path = root_dir / "robot.urdf"
            xml_path.write_text(
                """
<mujoco model="x500_arm2x">
  <compiler angle="radian" meshdir="meshes/" texturedir="meshes/" />
  <worldbody>
    <body name="base_link" pos="0 0 0.3">
      <joint name="floating_base_joint" type="free" />
      <site name="base_link_origin" />
      <body name="link_1">
        <joint name="joint_1" range="-2.6485 2.6485" />
        <body name="link_2">
          <joint name="joint_2" range="0 3.4907" />
          <body name="link_3">
            <joint name="joint_3" range="-2.6485 2.6485" />
            <body name="link_4">
              <joint name="joint_4" range="-3.1416 3.1416" />
              <body name="link_5">
                <joint name="joint_5" range="-1.723 0" />
                <body name="gripper_left">
                  <joint name="joint_gripper_left" type="slide" range="-0.04225 0" />
                </body>
                <body name="gripper_right">
                  <joint name="joint_gripper_right" type="slide" range="-0.04225 0" />
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
""".strip(),
                encoding="utf-8",
            )
            paths = AssetPaths(base_dir=root_dir, urdf_path=urdf_path, mesh_dir=mesh_dir, xml_path=xml_path)

            postprocess_xml(
                xml_path,
                config=AssetToolchainConfig(target="x500_arm2x", floating=True, decompose=True),
                paths=paths,
                initial_q={},
                height_offset=0.3,
            )

            root = ET.parse(xml_path).getroot()
            actuators = {actuator.get("name"): actuator.attrib for actuator in root.findall("./actuator/position")}
            expected = {
                "joint_1": ("60.05", "3.54", "-5.24 5.24", "-2.6485 2.6485"),
                "joint_2": ("44.95", "1.75", "-3.92 3.92", "0 3.4907"),
                "joint_3": ("31.47", "0.58", "-2.75 2.75", "-2.6485 2.6485"),
                "joint_4": ("12.77", "0.106", "-1.11 1.11", "-3.1416 3.1416"),
                "joint_5": ("12.77", "0.106", "-1.11 1.11", "-1.723 0"),
                "joint_gripper_left": ("10600.0", "88.6", "-22.7 22.7", "-0.04225 0"),
                "joint_gripper_right": ("10600.0", "88.6", "-22.7 22.7", "-0.04225 0"),
            }
            for name, (kp, kv, forcerange, ctrlrange) in expected.items():
                self.assertIn(name, actuators)
                self.assertEqual(actuators[name]["kp"], kp)
                self.assertEqual(actuators[name]["kv"], kv)
                self.assertEqual(actuators[name]["forcerange"], forcerange)
                self.assertEqual(actuators[name]["ctrlrange"], ctrlrange)


if __name__ == "__main__":
    unittest.main()
