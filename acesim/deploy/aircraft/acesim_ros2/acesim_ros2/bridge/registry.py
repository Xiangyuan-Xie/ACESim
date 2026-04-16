from __future__ import annotations

from typing import Any

from acesim_ros2.bridge.plugin_api import BridgePluginSpec
from acesim_ros2.bridge.plugins.arm_state import PLUGIN as ARM_STATE_PLUGIN
from acesim_ros2.bridge.plugins.simulation_clock import PLUGIN as SIMULATION_CLOCK_PLUGIN

PLUGIN_REGISTRY: dict[str, BridgePluginSpec[Any]] = {
    SIMULATION_CLOCK_PLUGIN.bridge_name: SIMULATION_CLOCK_PLUGIN,
    ARM_STATE_PLUGIN.bridge_name: ARM_STATE_PLUGIN,
}
