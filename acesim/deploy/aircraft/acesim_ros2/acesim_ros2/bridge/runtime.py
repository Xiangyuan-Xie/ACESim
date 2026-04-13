from __future__ import annotations

from typing import Any

import zmq
from acesim_ros2.bridge.plugin_api import BridgeConfig, BridgePluginSpec


class TimestampTracker:
    def __init__(self) -> None:
        self._last_timestamp_us: int | None = None

    def check(self, timestamp_us: int) -> None:
        if self._last_timestamp_us is not None and timestamp_us < self._last_timestamp_us:
            raise ValueError(f"Non-monotonic timestamp_us: {timestamp_us} < {self._last_timestamp_us}")
        self._last_timestamp_us = timestamp_us


class ZmqSubTransport:
    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        context = zmq.Context.instance()
        self._socket = context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVHWM, 1)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.connect(endpoint)

    def recv(self) -> bytes:
        return self._socket.recv(flags=zmq.NOBLOCK)

    def close(self) -> None:
        self._socket.close(linger=0)


class BridgeRuntime:
    def __init__(
        self,
        *,
        node: Any,
        bridge_config: BridgeConfig,
        plugin: BridgePluginSpec[Any],
        transport: ZmqSubTransport,
    ) -> None:
        self.name = bridge_config.name
        self._bridge = bridge_config
        self._plugin = plugin
        self._transport = transport
        self._sink = plugin.build_sink(node, bridge_config)
        self._timestamp_tracker = TimestampTracker()
        self._input_endpoint = bridge_config.override.input_endpoint or bridge_config.transport.endpoint
        node.get_logger().info(f"Bridge '{bridge_config.name}' connected to {self._input_endpoint}")
        self._timer = node.create_timer(bridge_config.poll_period_sec, self.poll_once)

    def process_payload(self, payload: bytes) -> None:
        decoded = self._plugin.decode_payload(payload)
        timestamp_us = self._plugin.extract_timestamp_us(decoded)
        self._timestamp_tracker.check(timestamp_us)
        self._sink(decoded)

    def poll_once(self) -> None:
        while True:
            try:
                payload = self._transport.recv()
            except zmq.Again:
                return
            self.process_payload(payload)

    def close(self) -> None:
        self._transport.close()


class BridgeHost:
    def __init__(
        self, node: Any, bridge_configs: list[BridgeConfig], plugins: dict[str, BridgePluginSpec[Any]]
    ) -> None:
        self._bridge_runtimes: list[BridgeRuntime] = []
        for bridge_config in bridge_configs:
            plugin = plugins[bridge_config.name]
            transport = ZmqSubTransport(bridge_config.override.input_endpoint or bridge_config.transport.endpoint)
            runtime = BridgeRuntime(
                node=node,
                bridge_config=bridge_config,
                plugin=plugin,
                transport=transport,
            )
            self._bridge_runtimes.append(runtime)

    def close(self) -> None:
        for runtime in self._bridge_runtimes:
            runtime.close()
