from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from .asset_context import AssetPaths, AssetToolchainConfig


@dataclass(frozen=True)
class SDFVisualTruth:
    xyz: str
    rpy: str
    uri: str


@dataclass(frozen=True)
class SDFJointTruth:
    xyz: str | None
    rpy: str | None
    axis_xyz: str | None


@dataclass(frozen=True)
class SDFInertialTruth:
    mass: float
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float


@dataclass(frozen=True)
class SDFModelTruth:
    visuals: dict[str, SDFVisualTruth]
    joints: dict[str, SDFJointTruth]
    inertials: dict[str, SDFInertialTruth]


class SDFSourceProvider(Protocol):
    name: str

    def sdf_path_for_target(self, target: str) -> Path: ...

    def load_truth(self, target: str) -> SDFModelTruth: ...

    def generate_manual_meshes(self, config: AssetToolchainConfig, paths: AssetPaths) -> None: ...

    def cleanup_manual_meshes(self, config: AssetToolchainConfig, paths: AssetPaths) -> None: ...

    def sync_manual_urdf(self, config: AssetToolchainConfig, paths: AssetPaths) -> None: ...
