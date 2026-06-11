from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def get_optional_table(params: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = params.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a table")
    return value


def resolve_vehicle_params_table(
    params: Mapping[str, Any],
    vehicle_key: str,
    *,
    legacy_keys: tuple[str, ...],
) -> Mapping[str, Any]:
    nested = params.get(vehicle_key)
    has_legacy = any(key in params for key in legacy_keys)
    if nested is None:
        return params
    if not isinstance(nested, Mapping):
        raise ValueError(f"params.{vehicle_key} must be a table")
    if has_legacy:
        raise ValueError(f"params.{vehicle_key} conflicts with legacy flat params")
    return nested
