from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Generic, TypeVar


@dataclass(frozen=True)
class TransportConfig:
    type: str
    endpoint: str


@dataclass(frozen=True)
class BridgeOverrideConfig:
    input_endpoint: str | None = None


@dataclass(frozen=True)
class BridgeConfig:
    name: str
    enabled: bool
    poll_period_sec: float
    transport: TransportConfig
    topic: str
    joint_names: list[str] | None = None
    override: BridgeOverrideConfig = BridgeOverrideConfig()


DecodedPayloadT = TypeVar("DecodedPayloadT")


@dataclass(frozen=True)
class BridgePluginSpec(Generic[DecodedPayloadT]):
    bridge_name: str
    apply_defaults: Callable[[dict[str, object]], dict[str, object]]
    decode_payload: Callable[[bytes], DecodedPayloadT]
    extract_timestamp_us: Callable[[DecodedPayloadT], int]
    build_sink: Callable[[Any, BridgeConfig], Callable[[DecodedPayloadT], None]]
