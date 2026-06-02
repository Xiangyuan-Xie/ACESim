from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from acesim.tools.utils.tui_app import run_bios_form, run_bios_subform
from acesim.tools.utils.tui_models import (
    FIELD_ACTION,
    FIELD_BOOL,
    FIELD_CHOICE,
    FIELD_FLOAT,
    FIELD_TEXT,
    BIOSField,
    BIOSFormState,
)

from .asset_context import AssetPaths
from .converter import URDF2MJCFConverter


@dataclass(frozen=True)
class URDF2MJCFTUIConfig:
    target: str
    floating: bool = False
    decompose: bool = False
    safety_margin: float = 0.05
    q0: str = ""
    mujoco_bin: str | None = None
    overwrite: bool = False


def _asset_base_dir() -> Path:
    return AssetPaths.for_target("__dummy__").base_dir


def available_targets() -> tuple[str, ...]:
    base_dir = _asset_base_dir()
    if not base_dir.exists():
        return ()
    targets = [path.name for path in base_dir.iterdir() if path.is_dir() and (path / f"{path.name}.urdf").exists()]
    return tuple(sorted(targets))


def available_q0_joints(target: str) -> tuple[str, ...]:
    if not target:
        return ()
    urdf_path = AssetPaths.for_target(target).urdf_path
    if not urdf_path.exists():
        return ()

    root = ET.parse(urdf_path).getroot()
    joint_names: list[str] = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        joint_type = joint.get("type", "")
        if not name or joint_type == "fixed":
            continue
        if "rotor" in name or "propeller" in name or "gripper" in name:
            continue
        joint_names.append(name)
    return tuple(joint_names)


def collect_q0_values(values: dict[str, object]) -> str:
    q0_pairs = []
    for joint_name, value in values.items():
        raw_value = str(value).strip()
        if raw_value:
            q0_pairs.append(f"{joint_name}={raw_value}")
    return ",".join(q0_pairs)


def _q0_fields(joint_names: tuple[str, ...]) -> list[BIOSField]:
    return [
        BIOSField(
            key=f"q0.{joint_name}",
            label=f"q0: {joint_name}",
            value="",
            kind=FIELD_TEXT,
            help=f"Initial position for {joint_name}. Blank keeps zero/default.",
        )
        for joint_name in joint_names
    ]


def make_q0_editor():
    def edit_q0(state: BIOSFormState, stdscr, curses_module) -> str | None:
        target = str(state.values.get("target", "")).strip()
        joint_names = available_q0_joints(target)
        if not joint_names:
            values = run_bios_subform(
                "ACESim q0 Setup",
                [
                    BIOSField(
                        key="q0",
                        label="Initial q0",
                        value=str(state.values.get("q0", "")),
                        kind=FIELD_TEXT,
                        help="Raw q0 string, e.g. joint_1=-1.5708,joint_2=3.1416.",
                    )
                ],
                stdscr,
                curses_module,
            )
            if values is None:
                return None
            return str(values["q0"])

        existing_values = _parse_q0_values(str(state.values.get("q0", "")))
        values = run_bios_subform(
            "ACESim q0 Setup",
            [
                BIOSField(
                    key=joint_name,
                    label=joint_name,
                    value=existing_values.get(joint_name, ""),
                    kind=FIELD_TEXT,
                    help=f"Initial position for {joint_name}. Blank keeps zero/default.",
                )
                for joint_name in joint_names
            ],
            stdscr,
            curses_module,
        )
        if values is None:
            return None
        return collect_q0_values(values)

    return edit_q0


def _parse_q0_values(q0: str) -> dict[str, str]:
    values: dict[str, str] = {}
    if not q0:
        return values
    for pair in q0.split(","):
        if "=" not in pair:
            continue
        key, value = pair.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def prompt_config() -> URDF2MJCFTUIConfig:
    targets = available_targets()
    default_target = targets[0] if targets else ""
    overwrite_default = False
    if default_target:
        overwrite_default = AssetPaths.for_target(default_target).xml_path.exists()

    values = run_bios_form(
        "ACESim URDF -> MJCF Setup Utility",
        fields=[
            BIOSField(
                key="target",
                label="Target Asset",
                value=default_target,
                kind=FIELD_CHOICE if targets else FIELD_TEXT,
                choices=targets,
                help="Asset directory under acesim/env/mujoco/asset.",
            ),
            BIOSField(
                key="floating",
                label="Floating Root",
                value=False,
                kind=FIELD_BOOL,
                help="Add a free root joint for vehicles that move in 6 DoF.",
            ),
            BIOSField(
                key="decompose",
                label="Convex Decompose",
                value=False,
                kind=FIELD_BOOL,
                help="Run CoACD convex decomposition for collision meshes.",
            ),
            BIOSField(
                key="safety_margin",
                label="Safety Margin",
                value=0.05,
                kind=FIELD_FLOAT,
                help="Extra auto-height clearance in meters.",
            ),
            BIOSField(
                key="q0",
                label="Initial q0",
                value="",
                kind=FIELD_ACTION,
                help="Open per-joint q0 editor for the currently selected target.",
                editor=make_q0_editor(),
            ),
            BIOSField(
                key="mujoco_bin",
                label="MuJoCo Compile",
                value="",
                kind=FIELD_TEXT,
                help="Optional path to MuJoCo compile binary. Blank means auto.",
            ),
            BIOSField(
                key="overwrite",
                label="Overwrite XML",
                value=overwrite_default,
                kind=FIELD_BOOL,
                help="Replace an existing MJCF XML for this target.",
            ),
        ],
    )
    if values is None:
        raise KeyboardInterrupt
    mujoco_bin = str(values["mujoco_bin"]).strip()
    return URDF2MJCFTUIConfig(
        target=str(values["target"]),
        floating=bool(values["floating"]),
        decompose=bool(values["decompose"]),
        safety_margin=float(str(values["safety_margin"])),
        q0=str(values["q0"]),
        mujoco_bin=mujoco_bin or None,
        overwrite=bool(values["overwrite"]),
    )


def print_summary(config: URDF2MJCFTUIConfig) -> None:
    paths = AssetPaths.for_target(config.target)
    print("\nPlan")
    print("----")
    print(f"Target       : {config.target}")
    print(f"URDF         : {paths.urdf_path}")
    print(f"Output XML   : {paths.xml_path}")
    print(f"Floating     : {config.floating}")
    print(f"Decompose    : {config.decompose}")
    print(f"Safety margin: {config.safety_margin}")
    print(f"q0           : {config.q0 or '(default zeros)'}")
    print(f"MuJoCo binary: {config.mujoco_bin or '(auto)'}")
    print(f"Overwrite    : {config.overwrite}")


def run_urdf2mjcf_pipeline(config: URDF2MJCFTUIConfig) -> Path:
    converter = URDF2MJCFConverter(
        target=config.target,
        floating=config.floating,
        decompose=config.decompose,
        safety_margin=config.safety_margin,
        q0=config.q0,
        mujoco_bin=config.mujoco_bin,
        overwrite=config.overwrite,
    )
    converter.run()
    return converter.xml_path


def main() -> int:
    try:
        config = prompt_config()
        result_path = run_urdf2mjcf_pipeline(config)
    except (EOFError, KeyboardInterrupt):
        print("\nAborted.", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"\nGenerated MJCF: {result_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
