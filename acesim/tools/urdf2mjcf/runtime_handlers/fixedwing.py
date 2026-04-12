from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ..asset_context import AssetPaths, AssetToolchainConfig


@dataclass(frozen=True)
class FixedwingRuntimeModelHandler:
    """Placeholder hook point for fixed-wing runtime rewriting."""

    family: str = "fixedwing"

    def prepare_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        return None

    def cleanup_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        return None

    def rewrite_runtime_model(
        self, root: ET.Element, worldbody: ET.Element, config: AssetToolchainConfig, paths: AssetPaths
    ) -> None:
        return None
