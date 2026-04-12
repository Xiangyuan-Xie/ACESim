from __future__ import annotations

TARGET_FAMILIES = {
    "iris": "multirotor",
    "x500": "multirotor",
    "typhoon_h480": "multirotor",
    "advanced_plane": "fixedwing",
    "standard_vtol": "vtol",
    "uuv_bluerov2_heavy": "uuv",
}


def asset_family_for_target(target: str) -> str:
    """Return the runtime handler family for a target."""

    return TARGET_FAMILIES.get(target, "generic")
