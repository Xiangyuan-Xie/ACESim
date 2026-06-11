from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgeOverrideConfig, TransportConfig
from acesim_ros2.bridge.registry import PLUGIN_REGISTRY
from ament_index_python.packages import get_package_share_directory


def default_bridge_config_path() -> str:
    try:
        return str(Path(get_package_share_directory("acesim_ros2")).resolve() / "config" / "bridges.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[2] / "config" / "bridges.yaml")


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected YAML mapping: {path}")
    return data


def load_bridge_configs(config_file: str, overrides_file: str | None = None) -> list[BridgeConfig]:
    config_path = Path(config_file)
    raw_config = _load_yaml_mapping(config_path)
    raw_bridges = raw_config.get("bridges")
    if not isinstance(raw_bridges, dict):
        raise ValueError(f"Bridge config must define a mapping under 'bridges': {config_path}")

    raw_overrides: dict[str, dict[str, object]] = {}
    if overrides_file:
        overrides_path = Path(overrides_file)
        overrides_data = _load_yaml_mapping(overrides_path)
        candidate_overrides = overrides_data.get("overrides", {})
        if not isinstance(candidate_overrides, dict):
            raise ValueError(f"Bridge overrides must define a mapping under 'overrides': {overrides_path}")
        for bridge_name, override in candidate_overrides.items():
            if isinstance(bridge_name, str) and isinstance(override, dict):
                raw_overrides[bridge_name] = dict(override)

    bridge_configs: list[BridgeConfig] = []
    for bridge_name, raw_bridge in raw_bridges.items():
        if not isinstance(raw_bridge, dict):
            raise ValueError(f"Bridge '{bridge_name}' must be a mapping: {config_path}")
        if "handler" in raw_bridge:
            raise ValueError(f"Bridge '{bridge_name}' must not define 'handler'; the bridge name is the type")
        if not bool(raw_bridge.get("enabled", True)):
            continue
        if not isinstance(bridge_name, str) or bridge_name not in PLUGIN_REGISTRY:
            raise ValueError(f"Unsupported bridge name: {bridge_name}")

        normalized = PLUGIN_REGISTRY[bridge_name].apply_defaults(dict(raw_bridge))
        raw_transport = normalized.get("transport")
        if not isinstance(raw_transport, dict):
            raise ValueError(f"Bridge '{bridge_name}' must define a transport mapping")
        endpoint = raw_transport.get("endpoint")
        if not isinstance(endpoint, str) or not endpoint:
            raise ValueError(f"Bridge '{bridge_name}' transport must define a non-empty endpoint")
        topic = normalized.get("topic")
        if not isinstance(topic, str) or not topic:
            raise ValueError(f"Bridge '{bridge_name}' must define a non-empty topic")

        override = raw_overrides.get(bridge_name, {})
        bridge_configs.append(
            BridgeConfig(
                name=bridge_name,
                enabled=True,
                poll_period_sec=float(normalized.get("poll_period_sec", 0.001)),
                transport=TransportConfig(type=str(raw_transport.get("type", "")), endpoint=endpoint),
                topic=topic,
                joint_names=list(normalized["joint_names"]) if "joint_names" in normalized else None,
                override=BridgeOverrideConfig(
                    input_endpoint=str(override.get("input_endpoint")) if "input_endpoint" in override else None
                ),
            )
        )
    return bridge_configs
