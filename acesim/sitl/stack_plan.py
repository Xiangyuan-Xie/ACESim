from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal

ReadinessMode = Literal["background", "wait", "off"]


class StackComponent(str, Enum):
    ACESIM_FRONTEND = "acesim_frontend"
    PX4 = "px4"
    MICROXRCE = "microxrce"
    ROS2_BRIDGE = "ros2_bridge"
    POST_START_SETUP = "post_start_setup"


@dataclass(frozen=True)
class StackPlan:
    """Execution-agnostic description of one ACESim/PX4 stack."""

    components: tuple[StackComponent, ...]
    readiness_mode: ReadinessMode

    @property
    def uses_ros2(self) -> bool:
        return StackComponent.MICROXRCE in self.components or StackComponent.ROS2_BRIDGE in self.components


def build_core_sitl_stack_plan(*, headless: bool, readiness_mode: ReadinessMode) -> StackPlan:
    _ = headless
    return StackPlan(
        components=(StackComponent.ACESIM_FRONTEND, StackComponent.PX4),
        readiness_mode=readiness_mode,
    )


def build_ros_launch_stack_plan(
    *,
    play_executable: str | None,
    enable_px4_post_start_setup: bool,
    readiness_mode: ReadinessMode,
) -> StackPlan:
    components = [StackComponent.MICROXRCE, StackComponent.PX4, StackComponent.ROS2_BRIDGE]
    if enable_px4_post_start_setup:
        components.append(StackComponent.POST_START_SETUP)
    if play_executable:
        components.append(StackComponent.ACESIM_FRONTEND)
    return StackPlan(components=tuple(components), readiness_mode=readiness_mode)
