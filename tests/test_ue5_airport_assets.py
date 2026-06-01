from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_sketchfab_airport_asset_preparer_was_removed_from_ue_tools() -> None:
    removed_script = ROOT / "acesim" / "tools" / "ue5" / "prepare_ue_airport_assets.py"

    assert not removed_script.exists()
