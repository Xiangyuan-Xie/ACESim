from __future__ import annotations

import subprocess
import sys
import unittest

from acesim.tools.sdf2urdf import (
    AssetPaths,
    AssetToolchainConfig,
    SDFModelTruth,
    generate_manual_meshes_from_sdf,
)
from acesim.tools.sdf2urdf import xml_formatting as sdf_xml_formatting
from acesim.tools.sdf2urdf.providers import PX4_PROVIDER
from acesim.tools.urdf2mjcf import xml_ops as urdf_xml_ops
from acesim.tools.urdf2mjcf.asset_context import AssetPaths as URDF2MJCFAssetPaths
from acesim.tools.urdf2mjcf.asset_context import AssetToolchainConfig as URDF2MJCFAssetToolchainConfig


class SDF2URDFTests(unittest.TestCase):
    def test_sdf2urdf_exports_public_types(self) -> None:
        paths = AssetPaths.for_target("advanced_plane")
        config = AssetToolchainConfig(target="advanced_plane")

        self.assertTrue(paths.urdf_path.name.endswith(".urdf"))
        self.assertEqual(config.target, "advanced_plane")

    def test_asset_tool_context_is_shared_with_urdf2mjcf(self) -> None:
        self.assertIs(AssetPaths, URDF2MJCFAssetPaths)
        self.assertIs(AssetToolchainConfig, URDF2MJCFAssetToolchainConfig)

    def test_xml_formatting_helpers_are_shared_with_urdf2mjcf(self) -> None:
        self.assertIs(sdf_xml_formatting.ATTRIB_ORDER, urdf_xml_ops.ATTRIB_ORDER)
        self.assertIs(sdf_xml_formatting.indent_xml, urdf_xml_ops.indent_xml)
        self.assertIs(sdf_xml_formatting.sort_attributes, urdf_xml_ops.sort_attributes)

    def test_import_does_not_pull_in_urdf2mjcf_package(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                (
                    "import sys; "
                    "import acesim.tools.sdf2urdf; "
                    "bad=[name for name in sys.modules if name.startswith('acesim.tools.urdf2mjcf')]; "
                    "print('\\n'.join(sorted(bad)))"
                ),
            ],
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertEqual(result.stdout.strip(), "")

    def test_px4_provider_loads_truth_into_generic_model(self) -> None:
        truth = PX4_PROVIDER.load_truth("advanced_plane")

        self.assertIsInstance(truth, SDFModelTruth)
        self.assertIn("base_link", truth.visuals)
        self.assertIn("rotor_puller_joint", truth.joints)
        self.assertIn("base_link", truth.inertials)
        self.assertTrue(truth.visuals["base_link"].uri.endswith("body.dae"))
        self.assertGreater(truth.inertials["base_link"].mass, 0.0)

    def test_px4_provider_resolves_target_path(self) -> None:
        path = PX4_PROVIDER.sdf_path_for_target("advanced_plane")
        self.assertTrue(path.name.endswith(".sdf.jinja"))
        self.assertIn("plane", path.as_posix())

    def test_unknown_source_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unsupported SDF source"):
            generate_manual_meshes_from_sdf(
                AssetToolchainConfig(target="advanced_plane"),
                AssetPaths.for_target("advanced_plane"),
                source="missing",
            )


if __name__ == "__main__":
    unittest.main()
