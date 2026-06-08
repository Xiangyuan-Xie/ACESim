"""Delay configuration helpers shared by simulator schedulers."""

from __future__ import annotations

import math
from typing import Sequence


def parse_delay_range_ms(value: object, field_name: str) -> tuple[float, float]:
    """Parse a non-negative ``[min_ms, max_ms]`` delay range."""

    if value is None:
        return (0.0, 0.0)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field_name} must be a [min_ms, max_ms] array")
    values: list[float] = []
    for item in value:
        if isinstance(item, bool):
            raise ValueError(f"{field_name} must contain finite numbers")
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name} must contain finite numbers") from exc
        if not math.isfinite(number):
            raise ValueError(f"{field_name} must contain finite numbers")
        values.append(number)
    delay_range = tuple(values)
    if len(delay_range) != 2:
        raise ValueError(f"{field_name} must be a [min_ms, max_ms] array")
    min_ms, max_ms = delay_range
    if min_ms < 0.0 or max_ms < 0.0 or min_ms > max_ms:
        raise ValueError(f"{field_name} must satisfy 0 <= min_ms <= max_ms")
    return (min_ms, max_ms)
