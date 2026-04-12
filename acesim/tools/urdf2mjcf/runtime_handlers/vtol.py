from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ..asset_context import AssetPaths, AssetToolchainConfig


@dataclass(frozen=True)
class VTOLRuntimeModelHandler:
    """Placeholder hook point for VTOL runtime rewriting."""

    family: str = "vtol"

    def prepare_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        return None

    def cleanup_runtime_assets(self, config: AssetToolchainConfig, paths: AssetPaths) -> None:
        return None

    def rewrite_runtime_model(
        self, root: ET.Element, worldbody: ET.Element, config: AssetToolchainConfig, paths: AssetPaths
    ) -> None:
        return None
