from __future__ import annotations

import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from acesim.tools import render_readme_assets

ROOT = Path(__file__).resolve().parents[1]
README_PATH = ROOT / "README.md"


def _readme_gallery_assets() -> list[str]:
    text = README_PATH.read_text(encoding="utf-8")
    return re.findall(r"docs/images/assets/([a-z0-9_]+)\.png", text)


class RenderReadmeAssetsTests(unittest.TestCase):
    def test_readme_gallery_matches_render_script_assets(self) -> None:
        self.assertEqual(_readme_gallery_assets(), render_readme_assets.ASSET_ORDER)

    def test_preview_presets_use_expected_default_pose_modes(self) -> None:
        expected_pose_modes = {
            "x500_arm2x": ("home_keyframe", 0),
            "iris": ("home_keyframe", None),
            "x500": ("home_keyframe", None),
            "typhoon_h480": ("home_keyframe", None),
            "advanced_plane": ("home_keyframe", None),
            "standard_vtol": ("home_keyframe", None),
            "uuv_bluerov2_heavy": ("dynamic_settle", None),
        }
        for asset_name, (pose_mode, settle_steps) in expected_pose_modes.items():
            with self.subTest(asset=asset_name):
                preset = render_readme_assets.PREVIEW_PRESETS[asset_name]
                self.assertEqual(preset.pose_mode, pose_mode)
                if settle_steps is not None:
                    self.assertEqual(preset.settle_steps, settle_steps)

    def test_settle_override_uses_cli_value_when_provided(self) -> None:
        preset = render_readme_assets.PREVIEW_PRESETS["x500_arm2x"]
        self.assertEqual(render_readme_assets._resolve_settle_steps(preset, None), 0)
        self.assertEqual(render_readme_assets._resolve_settle_steps(preset, 12), 12)

    def test_home_pose_ground_snap_places_vehicle_on_ground(self) -> None:
        model, data, bounds_min, _ = render_readme_assets._prepare_preview_state("x500_arm2x")
        self.assertGreater(model.nkey, 0)
        self.assertAlmostEqual(float(bounds_min[2]), 0.0, places=6)

    def test_readme_does_not_document_internal_gallery_runtime_split(self) -> None:
        text = README_PATH.read_text(encoding="utf-8")
        self.assertNotIn("运行环境仍然是 `mc_arm`", text)
        self.assertNotIn("README 画廊图片直接从资产 MJCF 渲染", text)

    def test_readme_documents_sdf2urdf_and_urdf2mjcf_workflow(self) -> None:
        text = README_PATH.read_text(encoding="utf-8")
        self.assertIn("acesim.tools.sdf2urdf", text)
        self.assertIn("acesim.tools.urdf2mjcf", text)
        self.assertIn("python -m acesim.tools.sdf2urdf", text)
        self.assertIn("python -m acesim.tools.urdf2mjcf", text)
        self.assertNotIn("acesim/tools/px4_sdf_to_urdf.py", text)
        self.assertNotIn("acesim/tools/verify_px4_asset_visuals.py", text)

    def test_cli_can_render_single_asset_with_settle_steps(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "acesim.tools.render_readme_assets",
                    "--assets",
                    "advanced_plane",
                    "--output-dir",
                    str(output_dir),
                    "--width",
                    "320",
                    "--height",
                    "240",
                    "--settle-steps",
                    "1",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue((output_dir / "advanced_plane.png").exists())


if __name__ == "__main__":
    unittest.main()
