from __future__ import annotations

"""Stage 1 of the asset pipeline: synchronize ACESim URDF assets from SDF."""

from acesim.tools.sdf2urdf.contracts import (
    SDFInertialTruth,
    SDFJointTruth,
    SDFModelTruth,
    SDFSourceProvider,
    SDFVisualTruth,
)
from acesim.tools.sdf2urdf.providers import PX4_PROVIDER

from .asset_context import AssetPaths, AssetToolchainConfig

_PROVIDERS: dict[str, SDFSourceProvider] = {
    PX4_PROVIDER.name: PX4_PROVIDER,
}


def _provider_for_source(source: str) -> SDFSourceProvider:
    provider = _PROVIDERS.get(source)
    if provider is None:
        raise ValueError(f"Unsupported SDF source {source!r}. Available sources: {', '.join(sorted(_PROVIDERS))}")
    return provider


def available_sources() -> tuple[str, ...]:
    return tuple(sorted(_PROVIDERS))


def generate_manual_meshes_from_sdf(config: AssetToolchainConfig, paths: AssetPaths, *, source: str = "px4") -> None:
    """Materialize any source-owned meshes needed before URDF synchronization."""

    _provider_for_source(source).generate_manual_meshes(config, paths)


def cleanup_manual_meshes_from_sdf(config: AssetToolchainConfig, paths: AssetPaths, *, source: str = "px4") -> None:
    """Drop stale generated meshes left behind by the selected SDF provider."""

    _provider_for_source(source).cleanup_manual_meshes(config, paths)


def sync_manual_urdf_from_sdf(config: AssetToolchainConfig, paths: AssetPaths, *, source: str = "px4") -> None:
    """Apply provider truth to the hand-maintained URDF for a target asset."""

    _provider_for_source(source).sync_manual_urdf(config, paths)


__all__ = [
    "AssetPaths",
    "AssetToolchainConfig",
    "SDFInertialTruth",
    "SDFJointTruth",
    "SDFModelTruth",
    "SDFVisualTruth",
    "available_sources",
    "cleanup_manual_meshes_from_sdf",
    "generate_manual_meshes_from_sdf",
    "sync_manual_urdf_from_sdf",
]
