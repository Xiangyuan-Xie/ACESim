from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


def _mujoco_asset_root(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "env" / "mujoco" / "asset" / name).resolve()


_MANUAL_ASSET_TARGETS = ("advanced_plane", "standard_vtol", "uuv_bluerov2_heavy")
_MANUAL_ASSET_EXPECTATIONS = {
    "advanced_plane": {
        "bodies": {"base_link", "rotor_4"},
        "joints": {
            "floating_base_joint",
            "left_elevon_joint",
            "right_elevon_joint",
            "left_flap_joint",
            "right_flap_joint",
            "elevator_joint",
            "rudder_joint",
        },
        "actuators": {
            "rudder_ctrl",
            "left_flap_ctrl",
            "right_flap_ctrl",
            "left_elevon_ctrl",
            "right_elevon_ctrl",
            "elevator_ctrl",
        },
        "sites": {"base_link_origin", "rotor_offset_4"},
    },
    "standard_vtol": {
        "bodies": {"base_link", "rotor_0", "rotor_1", "rotor_2", "rotor_3", "rotor_4"},
        "joints": {"floating_base_joint", "left_elevon_joint", "right_elevon_joint", "elevator_joint"},
        "actuators": {"left_elevon_ctrl", "right_elevon_ctrl", "elevator_ctrl"},
        "sites": {
            "base_link_origin",
            "rotor_offset_0",
            "rotor_offset_1",
            "rotor_offset_2",
            "rotor_offset_3",
            "rotor_offset_4",
        },
    },
    "uuv_bluerov2_heavy": {
        "bodies": {
            "base_link",
            "rotor_0",
            "rotor_1",
            "rotor_2",
            "rotor_3",
            "rotor_4",
            "rotor_5",
            "rotor_6",
            "rotor_7",
        },
        "joints": {"floating_base_joint"},
        "actuators": set(),
        "sites": {
            "base_link_origin",
            "rotor_offset_0",
            "rotor_offset_1",
            "rotor_offset_2",
            "rotor_offset_3",
            "rotor_offset_4",
            "rotor_offset_5",
            "rotor_offset_6",
            "rotor_offset_7",
        },
    },
}


class PX4SDFAssetPipelineStructureTests(unittest.TestCase):
    def test_handmaintained_urdf_and_mjcf_outputs_exist_and_load(self) -> None:
        for target in _MANUAL_ASSET_TARGETS:
            with self.subTest(target=target):
                asset_root = _mujoco_asset_root(target)
                urdf_path = asset_root / f"{target}.urdf"
                xml_path = asset_root / f"{target}.xml"
                self.assertTrue(urdf_path.exists())
                self.assertTrue(xml_path.exists())
                self.assertGreater(len(list((asset_root / "meshes").glob("*.stl"))), 0)

                urdf_text = urdf_path.read_text(encoding="utf-8")
                self.assertIn(f"package://{target}/meshes/", urdf_text)
                self.assertNotIn("model://", urdf_text)
                self.assertNotIn(".dae", urdf_text.lower())

                model = mujoco.MjModel.from_xml_path(str(xml_path))
                self.assertGreater(model.nbody, 1)
                root = ET.parse(xml_path).getroot()
                body_names = {elem.get("name", "") for elem in root.findall(".//worldbody//body")}
                joint_names = {elem.get("name", "") for elem in root.findall(".//worldbody//joint")}
                actuator_names = {elem.get("name", "") for elem in root.findall(".//actuator/*")}
                site_names = {elem.get("name", "") for elem in root.findall(".//worldbody//site")}
                sensor_names = {elem.get("name", "") for elem in root.findall(".//sensor/*")}

                self.assertTrue(_MANUAL_ASSET_EXPECTATIONS[target]["bodies"].issubset(body_names))
                self.assertTrue(_MANUAL_ASSET_EXPECTATIONS[target]["joints"].issubset(joint_names))
                self.assertTrue(_MANUAL_ASSET_EXPECTATIONS[target]["actuators"].issubset(actuator_names))
                self.assertTrue(_MANUAL_ASSET_EXPECTATIONS[target]["sites"].issubset(site_names))
                self.assertTrue(
                    {"framepos", "framequat", "framelinvel", "gyro", "accelerometer", "magnetometer"}.issubset(
                        sensor_names
                    )
                )


if __name__ == "__main__":
    unittest.main()
