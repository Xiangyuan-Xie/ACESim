from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AssetToolchainConfig:
    """Stage-1 options for syncing ACESim URDF assets from source truth."""

    target: str
    floating: bool = False
    decompose: bool = False
    safety_margin: float = 0.05
    q0: str = ""
    mujoco_bin: str | None = None


@dataclass(frozen=True)
class AssetPaths:
    """Resolved on-disk paths for a single ACESim asset target."""

    base_dir: Path
    urdf_path: Path
    mesh_dir: Path
    xml_path: Path

    @classmethod
    def for_target(cls, target: str) -> "AssetPaths":
        base_dir = Path(__file__).resolve().parents[2] / "env" / "mujoco" / "asset"
        urdf_path = base_dir / target / f"{target}.urdf"
        return cls(
            base_dir=base_dir,
            urdf_path=urdf_path,
            mesh_dir=urdf_path.parent / "meshes",
            xml_path=urdf_path.parent / f"{target}.xml",
        )
