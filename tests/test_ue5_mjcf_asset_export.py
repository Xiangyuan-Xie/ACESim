from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_mjcf_visual_asset_exporter_was_removed_from_ue_tools() -> None:
    removed_script = ROOT / "acesim" / "tools" / "ue5" / "export_mjcf_visual_assets.py"

    assert not removed_script.exists()
