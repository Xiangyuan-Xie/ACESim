from __future__ import annotations

"""Registry for runtime model handlers used by the URDF -> MJCF stage."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Protocol, cast

from .asset_context import AssetPaths, AssetToolchainConfig
from .asset_families import asset_family_for_target
from .runtime_handlers.fixedwing import FixedwingRuntimeModelHandler
from .runtime_handlers.multirotor import MultirotorRuntimeModelHandler
from .runtime_handlers.uuv import UUVRuntimeModelHandler
from .runtime_handlers.vtol import VTOLRuntimeModelHandler


class RuntimeModelHandler(Protocol):
    family: str

    def prepare_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None: ...

    def cleanup_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None: ...

    def rewrite_runtime_model(
        self, root: ET.Element, worldbody: ET.Element, config: AssetToolchainConfig, paths: AssetPaths
    ) -> None: ...


@dataclass(frozen=True)
class NoOpRuntimeModelHandler:
    """Default handler for asset families without special runtime rewriting."""

    family: str

    def prepare_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        return None

    def cleanup_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        return None

    def rewrite_runtime_model(
        self, root: ET.Element, worldbody: ET.Element, config: AssetToolchainConfig, paths: AssetPaths
    ) -> None:
        return None


RUNTIME_MODEL_HANDLERS: dict[str, RuntimeModelHandler] = cast(
    dict[str, RuntimeModelHandler],
    {
        "generic": NoOpRuntimeModelHandler("generic"),
        "multirotor": MultirotorRuntimeModelHandler(),
        "fixedwing": FixedwingRuntimeModelHandler(),
        "vtol": VTOLRuntimeModelHandler(),
        "uuv": UUVRuntimeModelHandler(),
    },
)


def runtime_handler_for_target(target: str) -> RuntimeModelHandler:
    """Resolve the runtime handler used by the MJCF stage for a target."""

    family = asset_family_for_target(target)
    return RUNTIME_MODEL_HANDLERS[family]
