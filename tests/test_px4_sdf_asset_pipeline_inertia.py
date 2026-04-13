from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from acesim.tools.sdf2urdf.providers import PX4_PROVIDER


def _mujoco_asset_root(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "env" / "mujoco" / "asset" / name).resolve()


def _source_link_inertial_truth(name: str, link_name: str) -> dict[str, float]:
    truth = PX4_PROVIDER.load_truth(name).inertials.get(link_name)
    if truth is None:
        raise ValueError(f"Missing source inertial for {name}:{link_name}")
    return {
        "mass": truth.mass,
        "ixx": truth.ixx,
        "ixy": truth.ixy,
        "ixz": truth.ixz,
        "iyy": truth.iyy,
        "iyz": truth.iyz,
        "izz": truth.izz,
    }


def _urdf_link_inertial_truth(name: str, link_name: str) -> dict[str, float]:
    root = ET.parse(_mujoco_asset_root(name) / f"{name}.urdf").getroot()
    link = next(link for link in root.findall("link") if link.get("name") == link_name)
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"Missing URDF inertial for {name}:{link_name}")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError(f"Missing URDF mass or inertia tensor for {name}:{link_name}")
    return {
        "mass": float(mass.get("value", "0")),
        "ixx": float(inertia.get("ixx", "0")),
        "ixy": float(inertia.get("ixy", "0")),
        "ixz": float(inertia.get("ixz", "0")),
        "iyy": float(inertia.get("iyy", "0")),
        "iyz": float(inertia.get("iyz", "0")),
        "izz": float(inertia.get("izz", "0")),
    }


class PX4SDFAssetPipelineInertiaTests(unittest.TestCase):
    def test_manual_asset_urdf_inertias_match_px4_source_truth(self) -> None:
        cases = {
            "advanced_plane": {
                "base_link": "base_link",
                "rotor_4": "rotor_puller",
            },
            "standard_vtol": {
                "base_link": "base_link",
                "rotor_0": "rotor_0",
                "rotor_1": "rotor_1",
                "rotor_2": "rotor_2",
                "rotor_3": "rotor_3",
                "rotor_4": "rotor_puller",
            },
            "uuv_bluerov2_heavy": {
                "base_link": "base_link",
                "rotor_0": "thruster1",
                "rotor_1": "thruster2",
                "rotor_2": "thruster3",
                "rotor_3": "thruster4",
                "rotor_4": "thruster5",
                "rotor_5": "thruster6",
                "rotor_6": "thruster7",
                "rotor_7": "thruster8",
            },
        }
        for asset_name, bindings in cases.items():
            for urdf_link_name, source_link_name in bindings.items():
                with self.subTest(asset=asset_name, link=urdf_link_name):
                    expected = _source_link_inertial_truth(asset_name, source_link_name)
                    actual = _urdf_link_inertial_truth(asset_name, urdf_link_name)
                    for key, expected_value in expected.items():
                        self.assertAlmostEqual(
                            actual[key], expected_value, places=9, msg=f"{asset_name}:{urdf_link_name}:{key}"
                        )


if __name__ == "__main__":
    unittest.main()
