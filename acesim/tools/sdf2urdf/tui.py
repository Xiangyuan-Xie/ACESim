from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

from acesim.tools.sdf2urdf import (
    AssetPaths,
    AssetToolchainConfig,
    available_sources,
    cleanup_manual_meshes_from_sdf,
    generate_manual_meshes_from_sdf,
    sync_manual_urdf_from_sdf,
)
from acesim.tools.utils.tui_app import run_bios_form
from acesim.tools.utils.tui_models import FIELD_BOOL, FIELD_CHOICE, BIOSField


@dataclass(frozen=True)
class SDF2URDFTUIConfig:
    source: str
    target: str
    cleanup: bool = False


def _asset_base_dir() -> Path:
    return AssetPaths.for_target("__dummy__").base_dir


def available_targets() -> tuple[str, ...]:
    base_dir = _asset_base_dir()
    if not base_dir.exists():
        return ()
    targets = [path.name for path in base_dir.iterdir() if path.is_dir() and (path / f"{path.name}.urdf").exists()]
    return tuple(sorted(targets))


def prompt_config() -> SDF2URDFTUIConfig:
    sources = available_sources()
    default_source = sources[0] if sources else "px4"
    targets = available_targets()
    default_target = targets[0] if targets else ""

    values = run_bios_form(
        "ACESim SDF -> URDF Setup Utility",
        fields=[
            BIOSField(
                key="source",
                label="SDF Source",
                value=default_source,
                kind=FIELD_CHOICE,
                choices=sources or (default_source,),
                help="Source provider used as upstream SDF truth.",
            ),
            BIOSField(
                key="target",
                label="Target Asset",
                value=default_target,
                kind=FIELD_CHOICE if targets else "text",
                choices=targets,
                help="Asset directory under acesim/env/mujoco/asset.",
            ),
            BIOSField(
                key="cleanup",
                label="Cleanup Meshes",
                value=False,
                kind=FIELD_BOOL,
                help="Delete stale generated meshes after synchronizing URDF.",
            ),
        ],
    )
    if values is None:
        raise KeyboardInterrupt
    return SDF2URDFTUIConfig(
        source=str(values["source"]),
        target=str(values["target"]),
        cleanup=bool(values["cleanup"]),
    )


def print_summary(config: SDF2URDFTUIConfig) -> None:
    paths = AssetPaths.for_target(config.target)
    print("\nPlan")
    print("----")
    print(f"Source : {config.source}")
    print(f"Target : {config.target}")
    print(f"URDF   : {paths.urdf_path}")
    print(f"Meshes : {paths.mesh_dir}")
    print(f"Cleanup: {config.cleanup}")


def run_sdf2urdf_pipeline(config: SDF2URDFTUIConfig) -> Path:
    paths = AssetPaths.for_target(config.target)
    if not paths.urdf_path.exists():
        raise FileNotFoundError(f"URDF not found at {paths.urdf_path}")

    toolchain_config = AssetToolchainConfig(target=config.target)
    generate_manual_meshes_from_sdf(toolchain_config, paths, source=config.source)
    sync_manual_urdf_from_sdf(toolchain_config, paths, source=config.source)
    if config.cleanup:
        cleanup_manual_meshes_from_sdf(toolchain_config, paths, source=config.source)
    return paths.urdf_path


def main() -> int:
    try:
        config = prompt_config()
        result_path = run_sdf2urdf_pipeline(config)
    except (EOFError, KeyboardInterrupt):
        print("\nAborted.", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"\nUpdated URDF: {result_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
