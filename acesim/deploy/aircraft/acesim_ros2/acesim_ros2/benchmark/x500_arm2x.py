from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import warnings
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import IO, Any, Callable, Sequence, cast

import zmq

from acesim.benchmark.x500_arm2x_velocity import (
    VelocityTrackingCommand,
    VelocityTrackingMetrics,
    VelocityTrackingProfile,
    VelocityTrackingProfileConfig,
    VelocityTrackingSummary,
    heading_frame_velocity_to_world_enu,
    velocity_enu_to_ned,
    yaw_rate_enu_to_ned,
)
from acesim.utils.math import calculate_coupled_gripper_positions

PX4_MAVLINK_URL = "udpin:0.0.0.0:14540"
DEFAULT_CLOCK_ZMQ_PORT = 5600
DEFAULT_VISUAL_ZMQ_PORT = 5601
DEFAULT_ARM_STATE_ZMQ_PORT = 5603
DEFAULT_ARM_COMMAND_ZMQ_PORT = 5604
DEFAULT_XRCE_PORT = 8888
DEFAULT_ROS_DOMAIN_ID = 80
CONTROLLER_VARIANTS = ("am", "px4_position")
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_SUB_MODE_AM_OFFBOARD = 1
PX4_CUSTOM_SUB_MODE_OFFBOARD_DEFAULT = 0
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_RESULT_ACCEPTED = 0
MAVLINK_SOURCE_SYSTEM = 250
MAVLINK_SOURCE_COMPONENT = 190
PX4_LOCAL_POSITION_TOPICS = (
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_local_position",
)
PX4_LAND_DETECTED_TOPIC = "/fmu/out/vehicle_land_detected"
ARM_JOINT_LIMITS: tuple[tuple[float, float], ...] = (
    (-2.6485, 2.6485),
    (0.0, 3.4907),
    (-2.6485, 2.6485),
    (-3.1416, 3.1416),
    (-1.723, 0.0),
    (-0.04225, 0.0),
    (0.0, 0.04225),
)


@dataclass(frozen=True)
class ArmPoseCase:
    name: str
    pose: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class ControllerCase:
    pose_case: ArmPoseCase
    controller: str

    @property
    def name(self) -> str:
        return self.pose_case.name

    @property
    def result_name(self) -> str:
        return f"{self.pose_case.name}_{self.controller}"

    @property
    def pose(self) -> tuple[float, float, float, float, float, float, float]:
        return self.pose_case.pose


@dataclass(frozen=True)
class BenchmarkThresholds:
    max_rms_speed_error_mps: float = 0.65
    max_abs_lateral_bias_mps: float = 0.35
    max_rms_yaw_rate_error_radps: float = 0.45


@dataclass(frozen=True)
class BenchmarkRuntimeConfig:
    setpoint_rate_hz: float = 50.0
    takeoff_altitude_m: float = 1.5
    takeoff_altitude_tolerance_m: float = 0.02
    takeoff_speed_mps: float = 0.45
    takeoff_timeout_s: float = 25.0
    land_timeout_s: float = 18.0
    startup_timeout_s: float = 180.0
    case_start_attempts: int = 1
    play_start_delay_s: float = 2.0
    post_takeoff_settle_s: float = 3.0
    arm_motion_duration_s: float = 10.0
    post_arm_motion_settle_s: float = 2.0
    am_offboard_settle_s: float = 2.0
    max_profile_altitude_m: float = 8.0
    jobs: int = 1
    port_base: int = DEFAULT_CLOCK_ZMQ_PORT
    xrce_port_base: int = DEFAULT_XRCE_PORT
    ros_domain_base: int = DEFAULT_ROS_DOMAIN_ID
    px4_instance_base: int = 0
    bridge_mode: str = "linux"
    px4_repo: str | None = None
    process_log_dir: str | None = None
    verbose_process_logs: bool = False
    profile_config: VelocityTrackingProfileConfig = VelocityTrackingProfileConfig()
    thresholds: BenchmarkThresholds = BenchmarkThresholds()


@dataclass(frozen=True)
class BenchmarkIsolationConfig:
    slot: int
    px4_instance: int
    ros_domain_id: int
    px4_sim_tcp_port: int
    mavlink_url: str
    xrce_port: int
    clock_zmq_endpoint: str
    visual_zmq_endpoint: str
    arm_state_zmq_endpoint: str
    arm_command_zmq_endpoint: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class ArmMotionSample:
    elapsed_s: float
    x_m: float
    y_m: float
    z_m: float
    dx_m: float
    dy_m: float
    dz_m: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    droll_rad: float
    dpitch_rad: float
    dyaw_rad: float
    progress: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class ArmMotionSummary:
    sample_count: int = 0
    duration_s: float = 0.0
    max_horizontal_offset_m: float = 0.0
    rms_horizontal_offset_m: float = 0.0
    max_vertical_offset_m: float = 0.0
    final_horizontal_offset_m: float = 0.0
    final_vertical_offset_m: float = 0.0
    max_abs_roll_rad: float = 0.0
    max_abs_pitch_rad: float = 0.0
    max_abs_yaw_rad: float = 0.0

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)


@dataclass(frozen=True)
class VelocityTrackingSample:
    elapsed_s: float
    segment_name: str
    desired_forward_mps: float
    desired_left_mps: float
    desired_up_mps: float
    actual_forward_mps: float
    actual_left_mps: float
    actual_up_mps: float
    desired_yaw_rate_radps: float
    actual_yaw_rate_radps: float
    speed_error_norm_mps: float

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class BenchmarkCaseResult:
    name: str
    pose: tuple[float, ...]
    takeoff_success: bool
    max_altitude_m: float
    tracking_duration_s: float
    passed: bool
    summary: VelocityTrackingSummary
    controller: str = "am"
    arm_motion_summary: ArmMotionSummary = ArmMotionSummary()
    isolation: BenchmarkIsolationConfig | None = None
    arm_motion_samples: tuple[ArmMotionSample, ...] = ()
    velocity_tracking_samples: tuple[VelocityTrackingSample, ...] = ()
    error: str | None = None

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["summary"] = self.summary.to_dict()
        payload["arm_motion_summary"] = self.arm_motion_summary.to_dict()
        payload["isolation"] = None if self.isolation is None else self.isolation.to_dict()
        payload["arm_motion_samples"] = [sample.to_dict() for sample in self.arm_motion_samples]
        payload["velocity_tracking_samples"] = [sample.to_dict() for sample in self.velocity_tracking_samples]
        return payload


def default_arm_pose_cases() -> list[ArmPoseCase]:
    """Baseline x500_arm2x arm configurations with gripper sliders inferred from joint 5."""

    return [
        _arm_pose_case("home_folded", (-1.57, 3.14, 0.0, 0.0, 0.0)),
        _arm_pose_case("center_carry", (0.0, 2.65, 0.0, -0.25, -0.35)),
        _arm_pose_case("forward_mid", (0.0, 2.05, 0.0, -0.70, -0.55)),
        _arm_pose_case("left_high", (0.55, 2.45, -0.30, -0.30, -0.45)),
        _arm_pose_case("right_high", (-0.55, 2.45, 0.30, 0.25, -0.45)),
        _arm_pose_case("left_low", (0.95, 1.85, -0.45, -0.65, -0.85)),
        _arm_pose_case("right_low", (-0.75, 1.95, 0.18, 0.05, -0.55)),
        _arm_pose_case("forward_grasp_low", (0.25, 1.60, 0.0, -0.85, -1.20)),
        _arm_pose_case("left_reach_high", (1.20, 2.75, -0.65, -0.15, -0.70)),
        _arm_pose_case("right_reach_forward", (-0.50, 2.10, 0.18, -0.25, -0.60)),
    ]


def _arm_pose_case(name: str, first_five_joints: Sequence[float]) -> ArmPoseCase:
    if len(first_five_joints) != 5:
        raise ValueError("x500_arm2x benchmark pose seed must contain exactly five arm joints")
    arm_pose = tuple(float(value) for value in first_five_joints)
    pose = arm_pose + calculate_coupled_gripper_positions(arm_pose[4])
    return ArmPoseCase(name, validate_arm_pose(pose))


def validate_arm_pose(pose: Sequence[float]) -> tuple[float, float, float, float, float, float, float]:
    if len(pose) != len(ARM_JOINT_LIMITS):
        raise ValueError(f"x500_arm2x arm pose must contain {len(ARM_JOINT_LIMITS)} values")

    values = tuple(float(value) for value in pose)
    for index, (value, (lower, upper)) in enumerate(zip(values, ARM_JOINT_LIMITS)):
        if not math.isfinite(value):
            raise ValueError(f"arm pose value at index {index} must be finite")
        if value < lower or value > upper:
            raise ValueError(f"arm pose value at index {index}={value:g} is outside [{lower:g}, {upper:g}]")
    return cast(tuple[float, float, float, float, float, float, float], values)


def expand_controller_cases(cases: Sequence[ArmPoseCase]) -> list[ControllerCase]:
    return [ControllerCase(case, controller) for case in cases for controller in CONTROLLER_VARIANTS]


def make_velocity_setpoint_payload(
    command: VelocityTrackingCommand,
    *,
    heading_w: float,
    timestamp_us: int,
) -> dict[str, object]:
    velocity_h = command.velocity_h if command.active else (0.0, 0.0, 0.0)
    velocity_enu = heading_frame_velocity_to_world_enu(heading_w, velocity_h)
    velocity_ned = velocity_enu_to_ned(velocity_enu)
    yaw_rate = 0.0 if command.yaw_rate is None else float(command.yaw_rate)
    nan3 = (math.nan, math.nan, math.nan)
    return {
        "timestamp": int(timestamp_us),
        "position": nan3,
        "velocity": velocity_ned,
        "acceleration": nan3,
        "jerk": nan3,
        "yaw": math.nan,
        "yawspeed": yaw_rate_enu_to_ned(yaw_rate),
    }


def make_position_setpoint_payload(
    *,
    position_ned: Sequence[float],
    timestamp_us: int,
) -> dict[str, object]:
    if len(position_ned) != 3:
        raise ValueError("position_ned must contain exactly three values")
    nan3 = (math.nan, math.nan, math.nan)
    return {
        "timestamp": int(timestamp_us),
        "position": tuple(float(value) for value in position_ned),
        "velocity": nan3,
        "acceleration": nan3,
        "jerk": nan3,
        "yaw": math.nan,
        "yawspeed": math.nan,
    }


def make_offboard_control_mode_payload(
    *,
    timestamp_us: int,
    position: bool = False,
    velocity: bool = True,
) -> dict[str, object]:
    return {
        "timestamp": int(timestamp_us),
        "position": bool(position),
        "velocity": bool(velocity),
        "acceleration": False,
        "attitude": False,
        "body_rate": False,
        "thrust_and_torque": False,
        "direct_actuator": False,
    }


def send_am_offboard_mode_command(mav: Any, *, mavlink: Any | None = None) -> None:
    if mavlink is None:
        from pymavlink import mavutil

        mavlink = mavutil.mavlink

    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        int(getattr(mavlink, "MAV_CMD_DO_SET_MODE", MAV_CMD_DO_SET_MODE)),
        0,
        float(getattr(mavlink, "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED", 1)),
        float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
        float(PX4_CUSTOM_SUB_MODE_AM_OFFBOARD),
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_offboard_mode_command(mav: Any, *, mavlink: Any | None = None) -> None:
    if mavlink is None:
        from pymavlink import mavutil

        mavlink = mavutil.mavlink

    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        int(getattr(mavlink, "MAV_CMD_DO_SET_MODE", MAV_CMD_DO_SET_MODE)),
        0,
        float(getattr(mavlink, "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED", 1)),
        float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
        float(PX4_CUSTOM_SUB_MODE_OFFBOARD_DEFAULT),
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_land_command(mav: Any, *, mavlink: Any | None = None) -> None:
    if mavlink is None:
        from pymavlink import mavutil

        mavlink = mavutil.mavlink

    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        int(getattr(mavlink, "MAV_CMD_NAV_LAND", MAV_CMD_NAV_LAND)),
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_takeoff_command(
    mav: Any,
    *,
    lat_deg: float,
    lon_deg: float,
    alt_amsl_m: float,
    mavlink: Any | None = None,
) -> None:
    if mavlink is None:
        from pymavlink import mavutil

        mavlink = mavutil.mavlink

    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        int(getattr(mavlink, "MAV_CMD_NAV_TAKEOFF", MAV_CMD_NAV_TAKEOFF)),
        0,
        0.0,
        0.0,
        0.0,
        math.nan,
        float(lat_deg),
        float(lon_deg),
        float(alt_amsl_m),
    )


def case_passed(
    summary: VelocityTrackingSummary,
    thresholds: BenchmarkThresholds,
    *,
    takeoff_success: bool,
) -> bool:
    return (
        takeoff_success
        and summary.sample_count > 0
        and summary.rms_speed_error_norm_mps <= thresholds.max_rms_speed_error_mps
        and summary.max_abs_lateral_velocity_bias_mps <= thresholds.max_abs_lateral_bias_mps
        and summary.rms_yaw_rate_error_radps <= thresholds.max_rms_yaw_rate_error_radps
    )


def _default_isolation(slot: int, runtime_config: BenchmarkRuntimeConfig) -> BenchmarkIsolationConfig:
    px4_instance = int(runtime_config.px4_instance_base) + int(slot)
    zmq_base = int(runtime_config.port_base) + int(slot) * 10
    return BenchmarkIsolationConfig(
        slot=int(slot),
        px4_instance=px4_instance,
        ros_domain_id=int(runtime_config.ros_domain_base) + int(slot),
        px4_sim_tcp_port=4560 + px4_instance,
        mavlink_url=f"udpin:0.0.0.0:{14540 + px4_instance}",
        xrce_port=int(runtime_config.xrce_port_base) + int(slot),
        clock_zmq_endpoint=f"tcp://0.0.0.0:{zmq_base}",
        visual_zmq_endpoint=f"tcp://0.0.0.0:{zmq_base + (DEFAULT_VISUAL_ZMQ_PORT - DEFAULT_CLOCK_ZMQ_PORT)}",
        arm_state_zmq_endpoint=f"tcp://0.0.0.0:{zmq_base + (DEFAULT_ARM_STATE_ZMQ_PORT - DEFAULT_CLOCK_ZMQ_PORT)}",
        arm_command_zmq_endpoint=(
            f"tcp://127.0.0.1:{zmq_base + (DEFAULT_ARM_COMMAND_ZMQ_PORT - DEFAULT_CLOCK_ZMQ_PORT)}"
        ),
    )


def _connect_endpoint(bind_endpoint: str, bridge_mode: str = "linux") -> str:
    if bind_endpoint.startswith("tcp://0.0.0.0:"):
        return "tcp://127.0.0.1:" + bind_endpoint.rsplit(":", 1)[1]
    return bind_endpoint


def _compute_arm_motion_summary(samples: Sequence[ArmMotionSample]) -> ArmMotionSummary:
    if not samples:
        return ArmMotionSummary()
    horizontal_offsets = [math.hypot(sample.dx_m, sample.dy_m) for sample in samples]
    vertical_offsets = [abs(sample.dz_m) for sample in samples]
    roll_offsets = [abs(sample.droll_rad) for sample in samples]
    pitch_offsets = [abs(sample.dpitch_rad) for sample in samples]
    yaw_offsets = [abs(sample.dyaw_rad) for sample in samples]
    final_sample = samples[-1]
    return ArmMotionSummary(
        sample_count=len(samples),
        duration_s=max(0.0, samples[-1].elapsed_s - samples[0].elapsed_s),
        max_horizontal_offset_m=max(horizontal_offsets),
        rms_horizontal_offset_m=math.sqrt(sum(value * value for value in horizontal_offsets) / len(horizontal_offsets)),
        max_vertical_offset_m=max(vertical_offsets),
        final_horizontal_offset_m=math.hypot(final_sample.dx_m, final_sample.dy_m),
        final_vertical_offset_m=abs(final_sample.dz_m),
        max_abs_roll_rad=max(roll_offsets),
        max_abs_pitch_rad=max(pitch_offsets),
        max_abs_yaw_rad=max(yaw_offsets),
    )


def build_px4_direct_command(px4_repo: str, px4_env: dict[str, str], isolation: BenchmarkIsolationConfig) -> str:
    binary = Path(px4_repo) / "build" / "px4_sitl_default" / "bin" / "px4"
    data_path = Path(px4_repo) / "build" / "px4_sitl_default" / "etc"
    rootfs_path = Path(px4_repo) / "build" / "px4_sitl_default" / "rootfs" / str(isolation.px4_instance)
    if not binary.exists():
        raise RuntimeError(f"PX4 SITL binary not found: {binary}. Build PX4 SITL before running parallel benchmark.")
    exports = " ".join(f"{name}={shlex_quote(value)}" for name, value in sorted(px4_env.items()))
    return (
        f"mkdir -p {shlex_quote(str(rootfs_path))} && "
        f"env {exports} {shlex_quote(str(binary))} "
        f"-d -i {isolation.px4_instance} -w {shlex_quote(str(rootfs_path))} {shlex_quote(str(data_path))}"
    )


def shlex_quote(value: object) -> str:
    import shlex

    return shlex.quote(str(value))


def _target_system(mav: Any) -> int:
    return int(getattr(mav, "target_system", 0) or 1)


def _target_component(mav: Any) -> int:
    return int(getattr(mav, "target_component", 0) or 1)


def _set_message_fields(message: Any, payload: dict[str, object]) -> Any:
    for name, value in payload.items():
        if isinstance(value, tuple):
            setattr(message, name, list(value))
        else:
            setattr(message, name, value)
    return message


def _send_arm_command(mav: Any, arm: bool, *, mavlink: Any) -> None:
    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        int(getattr(mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", MAV_CMD_COMPONENT_ARM_DISARM)),
        0,
        1.0 if arm else 0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _send_heartbeat(mav: Any, *, mavlink: Any) -> None:
    mav.mav.heartbeat_send(
        int(getattr(mavlink, "MAV_TYPE_GENERIC", 0)),
        int(getattr(mavlink, "MAV_AUTOPILOT_INVALID", 8)),
        0,
        0,
        0,
    )


def _wait_command_ack(
    mav: Any,
    command: int,
    *,
    timeout_s: float,
    tick: Any | None = None,
    resend: Any | None = None,
    resend_interval_s: float = 0.4,
) -> None:
    deadline = time.monotonic() + timeout_s
    next_resend = time.monotonic()
    while time.monotonic() < deadline:
        if tick is not None:
            tick()
        if resend is not None and time.monotonic() >= next_resend:
            resend()
            next_resend = time.monotonic() + resend_interval_s
        message = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.05)
        if message is None:
            continue
        if int(getattr(message, "command", -1)) != int(command):
            continue
        result = int(getattr(message, "result", -1))
        if result != MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 command {command} rejected with MAV_RESULT {result}")
        return
    raise RuntimeError(f"Timed out waiting for PX4 command {command} ACK")


class X500Arm2xBenchmarkController:
    def __init__(
        self,
        runtime_config: BenchmarkRuntimeConfig,
        isolation: BenchmarkIsolationConfig | None = None,
    ) -> None:
        import rclpy
        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleAngularVelocity,
            VehicleAttitude,
            VehicleGlobalPosition,
            VehicleLandDetected,
            VehicleLocalPosition,
        )
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._rclpy = rclpy
        self._offboard_cls = OffboardControlMode
        self._setpoint_cls = TrajectorySetpoint
        self._node = rclpy.create_node("x500_arm2x_benchmark")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._offboard_pub = self._node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self._setpoint_pub = self._node.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self._local_position: Any | None = None
        self._global_position: Any | None = None
        self._angular_velocity: Any | None = None
        self._attitude: Any | None = None
        self._land_detected: Any | None = None
        self._altitude_reference_z: float | None = None
        self._last_publish_time_us: int | None = None
        for topic in PX4_LOCAL_POSITION_TOPICS:
            self._node.create_subscription(
                VehicleLocalPosition,
                topic,
                lambda message: setattr(self, "_local_position", message),
                qos,
            )
        self._node.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            lambda message: setattr(self, "_global_position", message),
            qos,
        )
        self._node.create_subscription(
            VehicleAngularVelocity,
            "/fmu/out/vehicle_angular_velocity",
            lambda message: setattr(self, "_angular_velocity", message),
            qos,
        )
        self._node.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            lambda message: setattr(self, "_attitude", message),
            qos,
        )
        self._node.create_subscription(
            VehicleLandDetected,
            PX4_LAND_DETECTED_TOPIC,
            lambda message: setattr(self, "_land_detected", message),
            qos,
        )
        self._config = runtime_config
        self._isolation = isolation or _default_isolation(0, runtime_config)
        self._period_s = 1.0 / max(1.0, runtime_config.setpoint_rate_hz)

    def close(self) -> None:
        self._node.destroy_node()

    def run(self, case: ArmPoseCase | ControllerCase) -> BenchmarkCaseResult:
        from pymavlink import mavutil

        controller = case.controller if isinstance(case, ControllerCase) else "am"
        isolation = getattr(self, "_isolation", None) or _default_isolation(0, self._config)
        mav = mavutil.mavlink_connection(
            isolation.mavlink_url,
            source_system=MAVLINK_SOURCE_SYSTEM,
            source_component=MAVLINK_SOURCE_COMPONENT,
            autoreconnect=True,
        )
        armed = False
        takeoff_success = False
        max_altitude_m = 0.0
        tracking_duration_s = 0.0
        summary = VelocityTrackingSummary()
        arm_motion_samples: tuple[ArmMotionSample, ...] = ()
        arm_motion_summary = ArmMotionSummary()
        velocity_tracking_samples: tuple[VelocityTrackingSample, ...] = ()
        try:
            self._wait_for_local_position(timeout_s=30.0)
            self._wait_for_global_position(timeout_s=30.0)
            mav.wait_heartbeat(timeout=30.0)
            _send_heartbeat(mav, mavlink=mavutil.mavlink)
            takeoff_position_ned = self._takeoff_target_position_ned()
            takeoff_lat_deg, takeoff_lon_deg, takeoff_alt_amsl_m = self._takeoff_target_global()
            _send_arm_command(mav, True, mavlink=mavutil.mavlink)
            _wait_command_ack(
                mav,
                int(getattr(mavutil.mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", MAV_CMD_COMPONENT_ARM_DISARM)),
                timeout_s=5.0,
                resend=lambda: _send_arm_command(mav, True, mavlink=mavutil.mavlink),
            )
            armed = True
            send_takeoff_command(
                mav,
                lat_deg=takeoff_lat_deg,
                lon_deg=takeoff_lon_deg,
                alt_amsl_m=takeoff_alt_amsl_m,
                mavlink=mavutil.mavlink,
            )
            _wait_command_ack(
                mav,
                int(getattr(mavutil.mavlink, "MAV_CMD_NAV_TAKEOFF", MAV_CMD_NAV_TAKEOFF)),
                timeout_s=5.0,
                resend=lambda: send_takeoff_command(
                    mav,
                    lat_deg=takeoff_lat_deg,
                    lon_deg=takeoff_lon_deg,
                    alt_amsl_m=takeoff_alt_amsl_m,
                    mavlink=mavutil.mavlink,
                ),
            )

            takeoff_success, max_altitude_m = self._takeoff(takeoff_position_ned)
            if takeoff_success:
                self._wait_for_settled_local_position(
                    duration_s=max(0.0, self._config.post_takeoff_settle_s),
                    timeout_s=max(0.0, self._config.post_takeoff_settle_s) + 10.0,
                )
                hold_command = VelocityTrackingCommand(active=True, segment_name="hold", velocity_h=(0.0, 0.0, 0.0))
                if controller == "px4_position":
                    hold_position_ned = self._current_position_ned()
                    self._publish_position_setpoint(hold_position_ned)
                    send_offboard_mode_command(mav, mavlink=mavutil.mavlink)
                    _wait_command_ack(
                        mav,
                        int(getattr(mavutil.mavlink, "MAV_CMD_DO_SET_MODE", MAV_CMD_DO_SET_MODE)),
                        timeout_s=5.0,
                        tick=lambda: self._publish_position_setpoint(hold_position_ned),
                        resend=lambda: send_offboard_mode_command(mav, mavlink=mavutil.mavlink),
                    )
                    arm_motion_samples = tuple(self._move_arm_and_record_base_motion(case, hold_position_ned))
                    arm_motion_summary = _compute_arm_motion_summary(arm_motion_samples)
                    self._wait_for_settled_local_position(
                        duration_s=max(0.0, self._config.post_arm_motion_settle_s),
                        timeout_s=max(0.0, self._config.post_arm_motion_settle_s) + 10.0,
                        position_ned=hold_position_ned,
                    )
                    self._publish_for(hold_command, duration_s=1.0)
                else:
                    self._publish_for(hold_command, duration_s=1.0)
                    send_am_offboard_mode_command(mav, mavlink=mavutil.mavlink)
                    _wait_command_ack(
                        mav,
                        int(getattr(mavutil.mavlink, "MAV_CMD_DO_SET_MODE", MAV_CMD_DO_SET_MODE)),
                        timeout_s=5.0,
                        tick=lambda: self._publish_command(hold_command),
                        resend=lambda: send_am_offboard_mode_command(mav, mavlink=mavutil.mavlink),
                    )
                    self._publish_for(hold_command, duration_s=max(0.0, self._config.am_offboard_settle_s))
                    arm_motion_samples = tuple(self._move_arm_and_record_base_motion(case, hold_command))
                    arm_motion_summary = _compute_arm_motion_summary(arm_motion_samples)
                    self._wait_for_settled_local_position(
                        duration_s=max(0.0, self._config.post_arm_motion_settle_s),
                        timeout_s=max(0.0, self._config.post_arm_motion_settle_s) + 10.0,
                        setpoint_tick=lambda: self._publish_command(hold_command),
                    )
                profile = VelocityTrackingProfile(self._config.profile_config)
                metrics, tracking_duration_s, velocity_tracking_samples = self._run_profile(profile)
                summary = metrics.summary()
            cleanup_error: str | None = None
            try:
                self._land_and_disarm(mav, mavutil.mavlink)
            except Exception as exc:
                cleanup_error = str(exc)
            if not takeoff_success:
                error = f"Timed out reaching takeoff altitude {self._config.takeoff_altitude_m:g} m"
                if cleanup_error:
                    error = f"{error}; cleanup failed: {cleanup_error}"
                return BenchmarkCaseResult(
                    name=case.name,
                    pose=case.pose,
                    controller=controller,
                    takeoff_success=False,
                    max_altitude_m=max_altitude_m,
                    tracking_duration_s=0.0,
                    passed=False,
                    summary=VelocityTrackingSummary(),
                    arm_motion_summary=arm_motion_summary,
                    isolation=isolation,
                    arm_motion_samples=arm_motion_samples,
                    velocity_tracking_samples=velocity_tracking_samples,
                    error=error,
                )
            if cleanup_error:
                return BenchmarkCaseResult(
                    name=case.name,
                    pose=case.pose,
                    controller=controller,
                    takeoff_success=takeoff_success,
                    max_altitude_m=max(max_altitude_m, self._altitude_m()),
                    tracking_duration_s=tracking_duration_s,
                    passed=False,
                    summary=summary,
                    arm_motion_summary=arm_motion_summary,
                    isolation=isolation,
                    arm_motion_samples=arm_motion_samples,
                    velocity_tracking_samples=velocity_tracking_samples,
                    error=cleanup_error,
                )
            return BenchmarkCaseResult(
                name=case.name,
                pose=case.pose,
                controller=controller,
                takeoff_success=takeoff_success,
                max_altitude_m=max_altitude_m,
                tracking_duration_s=tracking_duration_s,
                passed=case_passed(summary, self._config.thresholds, takeoff_success=takeoff_success),
                summary=summary,
                arm_motion_summary=arm_motion_summary,
                isolation=isolation,
                arm_motion_samples=arm_motion_samples,
                velocity_tracking_samples=velocity_tracking_samples,
            )
        except Exception as exc:
            if armed:
                try:
                    self._land_and_disarm(mav, mavutil.mavlink)
                except Exception:
                    pass
            return BenchmarkCaseResult(
                name=case.name,
                pose=case.pose,
                controller=controller,
                takeoff_success=takeoff_success,
                max_altitude_m=max(max_altitude_m, self._altitude_m()),
                tracking_duration_s=tracking_duration_s,
                passed=False,
                summary=summary,
                arm_motion_summary=arm_motion_summary,
                isolation=isolation,
                arm_motion_samples=arm_motion_samples,
                velocity_tracking_samples=velocity_tracking_samples,
                error=str(exc),
            )
        finally:
            mav.close()

    def _now_us(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def _px4_time_us(self) -> int:
        timestamps: list[int] = []
        for message in (
            self._local_position,
            self._global_position,
            self._angular_velocity,
            self._land_detected,
        ):
            if message is None:
                continue
            timestamp = int(getattr(message, "timestamp", 0) or 0)
            if timestamp > 0:
                timestamps.append(timestamp)
        return max(timestamps, default=self._now_us())

    def _next_publish_timestamp_us(self) -> int:
        now_us = self._px4_time_us()
        min_step_us = int(round(self._period_s * 1_000_000.0))
        if self._last_publish_time_us is None:
            timestamp_us = now_us
        else:
            timestamp_us = max(now_us, self._last_publish_time_us + max(1, min_step_us))
        self._last_publish_time_us = timestamp_us
        return timestamp_us

    def _spin_once(self, timeout_s: float = 0.0) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=timeout_s)

    def _sleep_and_spin(self, duration_s: float | None = None) -> None:
        end = time.monotonic() + max(0.0, self._period_s if duration_s is None else duration_s)
        while True:
            self._spin_once(timeout_s=0.0)
            if time.monotonic() >= end:
                return
            wait_s = min(self._period_s, max(0.0, end - time.monotonic()))
            if wait_s <= 0.0:
                return
            time.sleep(wait_s)

    def _heading_w(self) -> float:
        if self._local_position is None:
            return 0.0
        return math.pi / 2.0 - float(getattr(self._local_position, "heading", 0.0))

    def _altitude_m(self) -> float:
        if self._local_position is None:
            return 0.0
        z = float(getattr(self._local_position, "z", 0.0))
        if not math.isfinite(z):
            return 0.0
        if self._altitude_reference_z is None:
            self._altitude_reference_z = z
        return abs(z - self._altitude_reference_z)

    def _local_position_valid(self) -> bool:
        position = self._local_position
        return bool(
            position is not None
            and getattr(position, "xy_valid", False)
            and getattr(position, "z_valid", False)
            and getattr(position, "v_xy_valid", False)
            and getattr(position, "v_z_valid", False)
        )

    def _ground_speed_mps(self) -> float:
        if self._local_position is None:
            return 0.0
        vx = float(getattr(self._local_position, "vx", 0.0))
        vy = float(getattr(self._local_position, "vy", 0.0))
        vz = float(getattr(self._local_position, "vz", 0.0))
        if not all(math.isfinite(value) for value in (vx, vy, vz)):
            return 0.0
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    def _wait_for_local_position(self, *, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._spin_once(timeout_s=0.05)
            if self._local_position_valid():
                self._altitude_m()
                return
        topic_list = ", ".join(PX4_LOCAL_POSITION_TOPICS)
        raise RuntimeError(f"Timed out waiting for valid PX4 vehicle_local_position on {topic_list}")

    def _global_position_valid(self) -> bool:
        position = getattr(self, "_global_position", None)
        return bool(
            position is not None
            and getattr(position, "lat_lon_valid", False)
            and getattr(position, "alt_valid", False)
            and math.isfinite(float(getattr(position, "lat", math.nan)))
            and math.isfinite(float(getattr(position, "lon", math.nan)))
            and math.isfinite(float(getattr(position, "alt", math.nan)))
        )

    def _wait_for_global_position(self, *, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._spin_once(timeout_s=0.05)
            if self._global_position_valid():
                return
        raise RuntimeError("Timed out waiting for valid PX4 vehicle_global_position")

    def _wait_for_settled_local_position(
        self,
        *,
        duration_s: float,
        timeout_s: float,
        position_ned: tuple[float, float, float] | None = None,
        setpoint_tick: Callable[[], None] | None = None,
    ) -> None:
        if duration_s <= 0.0:
            return

        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        while time.monotonic() < deadline:
            if setpoint_tick is not None:
                setpoint_tick()
            elif position_ned is not None:
                self._publish_position_setpoint(position_ned)
            else:
                self._spin_once(timeout_s=0.02)
            stable = self._local_position_valid() and self._ground_speed_mps() <= 0.3
            now = time.monotonic()
            if stable:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= duration_s:
                    return
            else:
                stable_since = None
            if setpoint_tick is not None or position_ned is not None:
                self._sleep_and_spin()
            else:
                time.sleep(self._period_s)
        raise RuntimeError(f"Timed out waiting for local position to settle; speed={self._ground_speed_mps():.2f} m/s")

    def _publish_command(self, command: VelocityTrackingCommand) -> None:
        timestamp_us = self._next_publish_timestamp_us()
        self._offboard_pub.publish(
            _set_message_fields(
                self._offboard_cls(),
                make_offboard_control_mode_payload(timestamp_us=timestamp_us),
            )
        )
        self._setpoint_pub.publish(
            _set_message_fields(
                self._setpoint_cls(),
                make_velocity_setpoint_payload(command, heading_w=self._heading_w(), timestamp_us=timestamp_us),
            )
        )
        self._spin_once(timeout_s=0.0)

    def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._publish_command(command)
            self._sleep_and_spin()

    def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
        current_z = 0.0
        if self._local_position is not None:
            current_z = float(getattr(self._local_position, "z", 0.0))
        if not math.isfinite(current_z):
            current_z = 0.0
        return (math.nan, math.nan, current_z - self._config.takeoff_altitude_m)

    def _takeoff_target_global(self) -> tuple[float, float, float]:
        if not self._global_position_valid():
            raise RuntimeError("Cannot create NAV_TAKEOFF target without valid vehicle_global_position")
        position = self._global_position
        return (
            float(getattr(position, "lat")),
            float(getattr(position, "lon")),
            float(getattr(position, "alt")) + self._config.takeoff_altitude_m,
        )

    def _publish_position_setpoint(self, position_ned: tuple[float, float, float]) -> None:
        timestamp_us = self._next_publish_timestamp_us()
        self._offboard_pub.publish(
            _set_message_fields(
                self._offboard_cls(),
                make_offboard_control_mode_payload(timestamp_us=timestamp_us, position=True, velocity=False),
            )
        )
        self._setpoint_pub.publish(
            _set_message_fields(
                self._setpoint_cls(),
                make_position_setpoint_payload(position_ned=position_ned, timestamp_us=timestamp_us),
            )
        )
        self._spin_once(timeout_s=0.0)

    def _takeoff(self, position_ned: tuple[float, float, float]) -> tuple[bool, float]:
        deadline = time.monotonic() + self._config.takeoff_timeout_s
        max_altitude_m = self._altitude_m()
        reached_altitude_m = max(0.0, self._config.takeoff_altitude_m - self._config.takeoff_altitude_tolerance_m)
        while time.monotonic() < deadline:
            self._spin_once(timeout_s=0.02)
            altitude_m = self._altitude_m()
            max_altitude_m = max(max_altitude_m, altitude_m)
            if altitude_m >= reached_altitude_m:
                return True, max_altitude_m
            time.sleep(self._period_s)
        return False, max_altitude_m

    def _send_arm_motion_command(self, case: ArmPoseCase | ControllerCase) -> dict[str, object]:
        context = zmq.Context.instance()
        endpoint = _connect_endpoint(self._isolation.arm_command_zmq_endpoint, self._config.bridge_mode)
        deadline = time.monotonic() + 5.0
        payload = {
            "type": "move_joint_pose",
            "command_id": case.name,
            "pose": list(case.pose[:5]),
            "duration_s": float(self._config.arm_motion_duration_s),
        }
        while time.monotonic() < deadline:
            socket = context.socket(zmq.REQ)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVTIMEO, 250)
            socket.setsockopt(zmq.SNDTIMEO, 250)
            socket.connect(endpoint)
            try:
                socket.send_json(payload)
                reply = socket.recv_json()
            except zmq.Again:
                socket.close(linger=0)
                time.sleep(0.05)
                continue
            if not isinstance(reply, dict) or not bool(reply.get("ok")):
                socket.close(linger=0)
                raise RuntimeError(f"arm motion command rejected: {reply}")
            socket.close(linger=0)
            return reply
        raise RuntimeError(f"Timed out sending arm motion command to {endpoint}")

    def _local_position_xyz_ned(self) -> tuple[float, float, float]:
        if self._local_position is None:
            return (0.0, 0.0, 0.0)
        return (
            float(getattr(self._local_position, "x", 0.0)),
            float(getattr(self._local_position, "y", 0.0)),
            float(getattr(self._local_position, "z", 0.0)),
        )

    def _current_position_ned(self) -> tuple[float, float, float]:
        return self._local_position_xyz_ned()

    def _local_velocity_ned(self) -> tuple[float, float, float]:
        if self._local_position is None:
            return (0.0, 0.0, 0.0)
        return (
            float(getattr(self._local_position, "vx", 0.0)),
            float(getattr(self._local_position, "vy", 0.0)),
            float(getattr(self._local_position, "vz", 0.0)),
        )

    def _attitude_rpy(self) -> tuple[float, float, float]:
        if getattr(self, "_attitude", None) is None:
            return (0.0, 0.0, self._heading_w())
        q = getattr(self._attitude, "q", None)
        if q is None or len(q) < 4:
            return (0.0, 0.0, self._heading_w())
        qw, qx, qy, qz = (float(value) for value in q[:4])
        if not all(math.isfinite(value) for value in (qw, qx, qy, qz)):
            return (0.0, 0.0, self._heading_w())
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    def _move_arm_and_record_base_motion(
        self,
        case: ArmPoseCase | ControllerCase,
        hold: VelocityTrackingCommand | tuple[float, float, float],
    ) -> list[ArmMotionSample]:
        reply = self._send_arm_motion_command(case)
        try:
            duration_s = float(cast(Any, reply.get("duration_s", self._config.arm_motion_duration_s)))
        except (TypeError, ValueError):
            duration_s = self._config.arm_motion_duration_s
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            duration_s = self._config.arm_motion_duration_s
        start = time.monotonic()
        base_x, base_y, base_z = self._local_position_xyz_ned()
        base_roll, base_pitch, base_yaw = self._attitude_rpy()
        samples: list[ArmMotionSample] = []
        while True:
            elapsed_s = time.monotonic() - start
            if elapsed_s > duration_s:
                break
            if isinstance(hold, VelocityTrackingCommand):
                self._publish_command(hold)
            else:
                self._publish_position_setpoint(hold)
            x, y, z = self._local_position_xyz_ned()
            vx, vy, vz = self._local_velocity_ned()
            roll, pitch, yaw = self._attitude_rpy()
            samples.append(
                ArmMotionSample(
                    elapsed_s=elapsed_s,
                    x_m=x,
                    y_m=y,
                    z_m=z,
                    dx_m=x - base_x,
                    dy_m=y - base_y,
                    dz_m=z - base_z,
                    vx_mps=vx,
                    vy_mps=vy,
                    vz_mps=vz,
                    roll_rad=roll,
                    pitch_rad=pitch,
                    yaw_rad=yaw,
                    droll_rad=roll - base_roll,
                    dpitch_rad=pitch - base_pitch,
                    dyaw_rad=yaw - base_yaw,
                    progress=min(1.0, elapsed_s / max(duration_s, 1e-9)),
                )
            )
            self._sleep_and_spin()
        return samples

    def _run_profile(
        self,
        profile: VelocityTrackingProfile,
    ) -> tuple[VelocityTrackingMetrics, float, tuple[VelocityTrackingSample, ...]]:
        metrics = VelocityTrackingMetrics()
        raw_samples: list[VelocityTrackingSample] = []
        start = time.monotonic()
        while True:
            elapsed_s = time.monotonic() - start
            if elapsed_s >= profile.duration_s:
                return metrics, elapsed_s, tuple(raw_samples)
            command = profile.sample(elapsed_s)
            self._publish_command(command)
            if self._local_position_valid():
                altitude_m = self._altitude_m()
                if altitude_m > self._config.max_profile_altitude_m:
                    raise RuntimeError(
                        f"Profile aborted: altitude {altitude_m:.2f} m exceeded "
                        f"{self._config.max_profile_altitude_m:.2f} m during segment '{command.segment_name}'"
                    )
                metrics.record(
                    heading_w=self._heading_w(),
                    actual_velocity_world_enu=self._velocity_enu(),
                    actual_yaw_rate_flu_radps=self._yaw_rate_flu_radps(),
                    command=command,
                )
                actual_h = self._actual_velocity_heading_frame()
                desired_yaw_rate = 0.0 if command.yaw_rate is None else float(command.yaw_rate)
                speed_error = math.sqrt(
                    (float(command.velocity_h[0]) - actual_h[0]) ** 2
                    + (float(command.velocity_h[1]) - actual_h[1]) ** 2
                    + (float(command.velocity_h[2]) - actual_h[2]) ** 2
                )
                raw_samples.append(
                    VelocityTrackingSample(
                        elapsed_s=elapsed_s,
                        segment_name=command.segment_name,
                        desired_forward_mps=float(command.velocity_h[0]),
                        desired_left_mps=float(command.velocity_h[1]),
                        desired_up_mps=float(command.velocity_h[2]),
                        actual_forward_mps=actual_h[0],
                        actual_left_mps=actual_h[1],
                        actual_up_mps=actual_h[2],
                        desired_yaw_rate_radps=desired_yaw_rate,
                        actual_yaw_rate_radps=self._yaw_rate_flu_radps(),
                        speed_error_norm_mps=speed_error,
                    )
                )
            self._sleep_and_spin()

    def _actual_velocity_heading_frame(self) -> tuple[float, float, float]:
        from acesim.benchmark.x500_arm2x_velocity import world_enu_velocity_to_heading_frame

        return world_enu_velocity_to_heading_frame(self._heading_w(), self._velocity_enu())

    def _velocity_enu(self) -> tuple[float, float, float]:
        if self._local_position is None:
            return (0.0, 0.0, 0.0)
        return (
            float(getattr(self._local_position, "vy", 0.0)),
            float(getattr(self._local_position, "vx", 0.0)),
            -float(getattr(self._local_position, "vz", 0.0)),
        )

    def _yaw_rate_flu_radps(self) -> float:
        if self._angular_velocity is None:
            return 0.0
        xyz = getattr(self._angular_velocity, "xyz", None)
        if xyz is None or len(xyz) < 3:
            return 0.0
        return -float(xyz[2])

    def _land_and_disarm(self, mav: Any, mavlink: Any) -> None:
        zero_command = VelocityTrackingCommand(active=True, segment_name="stop", velocity_h=(0.0, 0.0, 0.0))
        self._publish_for(zero_command, duration_s=0.5)
        send_land_command(mav, mavlink=mavlink)
        _wait_command_ack(
            mav,
            int(getattr(mavlink, "MAV_CMD_NAV_LAND", MAV_CMD_NAV_LAND)),
            timeout_s=5.0,
            tick=lambda: self._publish_command(zero_command),
            resend=lambda: send_land_command(mav, mavlink=mavlink),
        )
        deadline = time.monotonic() + self._config.land_timeout_s
        while time.monotonic() < deadline:
            self._spin_once(timeout_s=0.05)
            if self._landed():
                break
            time.sleep(self._period_s)
        if not self._landed():
            raise RuntimeError(f"Timed out waiting for landing before disarm; altitude={self._altitude_m():.2f} m")
        _send_arm_command(mav, False, mavlink=mavlink)
        _wait_command_ack(
            mav,
            int(getattr(mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", MAV_CMD_COMPONENT_ARM_DISARM)),
            timeout_s=5.0,
            tick=lambda: self._publish_command(zero_command),
            resend=lambda: _send_arm_command(mav, False, mavlink=mavlink),
        )

    def _landed(self) -> bool:
        return bool(self._land_detected is not None and getattr(self._land_detected, "landed", False))


class ManagedBenchmarkStack:
    def __init__(
        self,
        case: ArmPoseCase | ControllerCase,
        runtime_config: BenchmarkRuntimeConfig,
        isolation: BenchmarkIsolationConfig | None = None,
    ) -> None:
        self._case = case
        self._config = runtime_config
        self._isolation = isolation or _default_isolation(0, runtime_config)
        self._tmpdir: tempfile.TemporaryDirectory[str] | None = None
        self._processes: list[tuple[str, subprocess.Popen[bytes]]] = []
        self._case_log_dir: Path | None = None
        self._log_files: list[Any] = []

    def __enter__(self) -> "ManagedBenchmarkStack":
        self._tmpdir = tempfile.TemporaryDirectory(prefix=f"acesim_x500_arm2x_{self._case.name}_")
        if not self._config.verbose_process_logs:
            base_log_dir = Path(
                self._config.process_log_dir or (Path(tempfile.gettempdir()) / "x500_arm2x_benchmark_logs")
            )
            controller = getattr(self._case, "controller", None)
            case_log_name = self._case.name if controller is None else f"{self._case.name}_{controller}"
            self._case_log_dir = base_log_dir / case_log_name
            self._case_log_dir.mkdir(parents=True, exist_ok=True)
            print(
                f"x500_arm2x benchmark: case {case_log_name} process logs: {self._case_log_dir}",
                file=sys.stderr,
                flush=True,
            )
        config_path = self._write_config(Path(self._tmpdir.name))
        try:
            self._start_processes(config_path)
        except Exception:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        for _name, process in reversed(self._processes):
            if process.poll() is None:
                process.terminate()
        for _name, process in reversed(self._processes):
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        for log_file in self._log_files:
            log_file.close()
        self._log_files.clear()
        if self._tmpdir is not None:
            self._tmpdir.cleanup()

    def _write_config(self, root: Path) -> Path:
        from acesim_ros2 import launch_common

        acesim_root = launch_common.detect_acesim_root()
        asset_src = acesim_root / "config" / "mujoco" / "x500_arm2x.toml"
        asset_dst_dir = root / "mujoco"
        asset_dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(asset_src, asset_dst_dir / "x500_arm2x.toml")
        config_path = root / "default.toml"
        config_path.write_text(
            "\n".join(
                [
                    "[basic]",
                    'sim_type = "mujoco"',
                    'env_type = "am"',
                    'scene_name = "default"',
                    'asset_name = "x500_arm2x"',
                    'benchmark = "multirotor"',
                    "",
                ]
            ),
            encoding="utf-8",
        )
        return config_path

    def _start_processes(self, config_path: Path) -> None:
        import yaml
        from acesim_ros2 import launch_common

        from acesim.config.config_loader import ConfigLoader
        from acesim.utils.px4_transport import PX4SensorParams

        config_loader = ConfigLoader(config_path)
        px4_repo = launch_common.load_px4_repo_path(self._config.px4_repo)
        px4_env = launch_common.build_px4_additional_env(config_loader)
        bridge_overrides_path = Path(self._tmpdir.name) / "bridge_overrides.yaml"  # type: ignore[union-attr]
        bridge_entries = launch_common.load_bridge_entries(launch_common.bridge_config_path())
        bridge_host = launch_common.resolve_bridge_host(self._config.bridge_mode)  # type: ignore[arg-type]
        overrides: dict[str, dict[str, dict[str, str]]] = {"overrides": {}}
        for bridge in bridge_entries:
            if not bool(bridge["enabled"]):
                continue
            bridge_name = str(bridge["name"])
            if bridge_name == "simulation_clock":
                endpoint = self._isolation.clock_zmq_endpoint
            elif bridge_name == "arm_state":
                endpoint = self._isolation.arm_state_zmq_endpoint
            else:
                endpoint = str(bridge["endpoint"])
            port = endpoint.rsplit(":", 1)[1]
            overrides["overrides"][bridge_name] = {"input_endpoint": f"tcp://{bridge_host}:{port}"}
        bridge_overrides_path.write_text(yaml.safe_dump(overrides, sort_keys=False), encoding="utf-8")

        common_env = {
            "ROS_DOMAIN_ID": str(self._isolation.ros_domain_id),
            "ACESIM_PX4_MAVLINK_URL": self._isolation.mavlink_url,
        }
        play_env = {
            **common_env,
            "ACESIM_PX4_SIM_TCP_PORT": str(self._isolation.px4_sim_tcp_port),
            "ACESIM_CLOCK_ZMQ_ENDPOINT": self._isolation.clock_zmq_endpoint,
            "ACESIM_VISUAL_ZMQ_ENDPOINT": self._isolation.visual_zmq_endpoint,
            "ACESIM_ARM_STATE_ZMQ_ENDPOINT": self._isolation.arm_state_zmq_endpoint,
            "ACESIM_ARM_COMMAND_ENDPOINT": self._isolation.arm_command_zmq_endpoint,
            "ACESIM_ARM_COMMAND_ONLY": "1",
        }
        px4_env.update(
            {
                "PX4_UXRCE_DDS_PORT": str(self._isolation.xrce_port),
                # Each benchmark slot already has an isolated ROS_DOMAIN_ID, so keep
                # PX4 topics at /fmu/... where the controller subscribes/publishes.
                "PX4_UXRCE_DDS_NS": "",
                "PX4_PARAM_UXRCE_DDS_PRT": str(self._isolation.xrce_port),
            }
        )
        px4_env.update(common_env)

        self._popen(
            "microxrce",
            launch_common.build_graceful_shutdown_command(f"MicroXRCEAgent udp4 -p {self._isolation.xrce_port}"),
            env=common_env,
        )
        self._popen(
            "px4",
            launch_common.build_graceful_shutdown_command(
                build_px4_direct_command(px4_repo, px4_env, self._isolation),
                filter_px4_prompt=True,
            ),
            cwd=px4_repo,
            env=px4_env,
        )
        self._popen(
            "bridge",
            launch_common.build_graceful_shutdown_command(
                launch_common.build_python_module_run_command(
                    "acesim_ros2",
                    "acesim_bridge",
                    extra_args=[
                        "--ros-args",
                        "-r",
                        "__node:=acesim_bridge",
                        "-p",
                        f"bridge_overrides_file:={bridge_overrides_path}",
                    ],
                )
            ),
            env=common_env,
        )
        sensor_params = PX4SensorParams.from_asset_params(
            config_loader.get_asset_params(), dynamic_hil_sensor_fields=False
        )
        post_start = self._popen(
            "px4_post_start_setup",
            launch_common.build_px4_post_start_command(sensor_params),
            env=common_env,
        )
        time.sleep(max(0.0, self._config.play_start_delay_s))
        self._raise_if_long_running_process_exited()
        self._popen(
            "headless",
            launch_common.build_graceful_shutdown_command(
                launch_common.build_python_module_run_command(
                    "acesim_ros2",
                    "acesim_play_headless",
                    additional_env=play_env,
                    extra_args=[
                        "--config",
                        str(config_path),
                    ],
                )
            ),
        )
        try:
            post_start.wait(timeout=self._config.startup_timeout_s)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Timed out waiting for PX4 post-start readiness checks{self._log_suffix()}") from exc
        if post_start.returncode != 0:
            raise RuntimeError(
                f"PX4 post-start setup failed with exit code {post_start.returncode}{self._log_suffix()}"
            )
        self._raise_if_long_running_process_exited()

    def _popen(
        self,
        name: str,
        command: list[str],
        *,
        cwd: str | None = None,
        env: dict[str, str] | None = None,
    ) -> subprocess.Popen[bytes]:
        process_env = os.environ.copy()
        if env:
            process_env.update(env)
        stdout: int | IO[Any] | None = sys.stderr
        stderr: int | IO[Any] | None = sys.stderr
        if self._config.verbose_process_logs:
            print(f"x500_arm2x benchmark: starting {name}", file=sys.stderr, flush=True)
        else:
            if self._case_log_dir is None:
                raise RuntimeError("Process log directory was not initialized")
            log_path = self._case_log_dir / f"{name}.log"
            log_file = log_path.open("ab")
            log_file.write(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} starting {name} ---\n".encode())
            log_file.write((" ".join(command) + "\n").encode(errors="replace"))
            log_file.flush()
            self._log_files.append(log_file)
            stdout = log_file
            stderr = subprocess.STDOUT
        process = subprocess.Popen(command, cwd=cwd, env=process_env, stdout=stdout, stderr=stderr)
        self._processes.append((name, process))
        return process

    def _raise_if_long_running_process_exited(self) -> None:
        for name, process in self._processes:
            if name == "px4_post_start_setup":
                continue
            returncode = process.poll()
            if returncode is not None:
                raise RuntimeError(f"{name} exited unexpectedly with code {returncode}{self._log_suffix()}")

    def _log_suffix(self) -> str:
        if self._case_log_dir is None:
            return ""
        return f"; logs: {self._case_log_dir}"


def _is_retryable_case_start_error(exc: Exception) -> bool:
    message = str(exc)
    retryable_fragments = (
        "PX4 post-start setup failed",
        "Timed out waiting for PX4 post-start readiness checks",
    )
    return any(fragment in message for fragment in retryable_fragments)


def run_case(
    case: ArmPoseCase | ControllerCase,
    runtime_config: BenchmarkRuntimeConfig,
    isolation: BenchmarkIsolationConfig | None = None,
) -> BenchmarkCaseResult:
    validate_arm_pose(case.pose)
    controller_name = getattr(case, "controller", "am")
    isolation = isolation or _default_isolation(0, runtime_config)
    attempts = max(1, int(runtime_config.case_start_attempts))
    for attempt in range(1, attempts + 1):
        try:
            with ManagedBenchmarkStack(case, runtime_config, isolation):
                import rclpy

                initialized = False
                controller: X500Arm2xBenchmarkController | None = None
                try:
                    os.environ["ROS_DOMAIN_ID"] = str(isolation.ros_domain_id)
                    rclpy.init(args=None)
                    initialized = True
                    controller = X500Arm2xBenchmarkController(runtime_config, isolation)
                    return controller.run(case)
                finally:
                    try:
                        if controller is not None:
                            controller.close()
                    finally:
                        if initialized:
                            rclpy.shutdown()
        except Exception as exc:
            if attempt < attempts and _is_retryable_case_start_error(exc):
                print(
                    f"x500_arm2x benchmark: case {case.name} startup attempt {attempt} failed: {exc}; retrying",
                    file=sys.stderr,
                    flush=True,
                )
                continue
            return BenchmarkCaseResult(
                name=case.name,
                pose=case.pose,
                controller=controller_name,
                takeoff_success=False,
                max_altitude_m=0.0,
                tracking_duration_s=0.0,
                passed=False,
                summary=VelocityTrackingSummary(),
                isolation=isolation,
                error=str(exc),
            )
    raise RuntimeError("unreachable")


def _run_case_worker(
    payload: tuple[ArmPoseCase | ControllerCase, BenchmarkRuntimeConfig, BenchmarkIsolationConfig],
) -> BenchmarkCaseResult:
    case, runtime_config, isolation = payload
    return run_case(case, runtime_config, isolation)


def _format_progress(percent: int, message: str) -> str:
    return f"x500_arm2x benchmark: [{percent:3d}%] {message}"


def _case_result_label(case: ArmPoseCase | ControllerCase) -> str:
    controller = getattr(case, "controller", None)
    return case.name if controller is None else f"{case.name}/{controller}"


def run_benchmark(cases: Sequence[ArmPoseCase], runtime_config: BenchmarkRuntimeConfig) -> dict[str, object]:
    controller_cases = expand_controller_cases(cases)
    results: list[BenchmarkCaseResult] = []
    total_cases = len(controller_cases)
    print(_format_progress(0, f"starting {total_cases} case(s)"), file=sys.stderr, flush=True)
    jobs = max(1, min(int(runtime_config.jobs), total_cases or 1))
    if jobs == 1:
        serial_isolation = _default_isolation(0, runtime_config)
        for index, case in enumerate(controller_cases, start=1):
            start_percent = int(round((index - 1) * 100.0 / max(1, total_cases)))
            print(
                _format_progress(start_percent, f"running {_case_result_label(case)} ({index}/{total_cases})"),
                file=sys.stderr,
                flush=True,
            )
            result = run_case(case, runtime_config, serial_isolation)
            results.append(result)
            done_percent = int(round(index * 100.0 / max(1, total_cases)))
            status = "PASS" if result.passed else "FAIL"
            print(
                _format_progress(done_percent, f"completed {_case_result_label(case)}: {status}"),
                file=sys.stderr,
                flush=True,
            )
    else:
        ordered: list[BenchmarkCaseResult | None] = [None] * total_cases
        with concurrent.futures.ProcessPoolExecutor(max_workers=jobs) as executor:
            active: dict[concurrent.futures.Future[BenchmarkCaseResult], tuple[int, ControllerCase, int]] = {}
            next_index = 0

            def submit_next(slot: int) -> None:
                nonlocal next_index
                case = controller_cases[next_index]
                isolation = _default_isolation(slot, runtime_config)
                future = executor.submit(_run_case_worker, (case, runtime_config, isolation))
                active[future] = (next_index, case, slot)
                next_index += 1

            for slot in range(min(jobs, total_cases)):
                submit_next(slot)

            completed_count = 0
            while active:
                for future in concurrent.futures.as_completed(tuple(active)):
                    index, case, slot = active.pop(future)
                    result = future.result()
                    ordered[index] = result
                    completed_count += 1
                    done_percent = int(round(completed_count * 100.0 / max(1, total_cases)))
                    status = "PASS" if result.passed else "FAIL"
                    print(
                        _format_progress(done_percent, f"completed {_case_result_label(case)}: {status}"),
                        file=sys.stderr,
                        flush=True,
                    )
                    if next_index < total_cases:
                        submit_next(slot)
                    break
        results = [result for result in ordered if result is not None]
    return {
        "asset": "x500_arm2x",
        "profile": asdict(runtime_config.profile_config),
        "thresholds": asdict(runtime_config.thresholds),
        "runtime": {
            "jobs": jobs,
            "arm_motion_duration_s": runtime_config.arm_motion_duration_s,
            "post_arm_motion_settle_s": runtime_config.post_arm_motion_settle_s,
        },
        "passed": all(result.passed for result in results),
        "cases": [result.to_dict() for result in results],
    }


def _case_metric(case: dict[str, object], metric_name: str) -> float:
    summary = case.get("summary", {})
    if not isinstance(summary, dict):
        return 0.0
    value = summary.get(metric_name, 0.0)
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _case_arm_metric(case: dict[str, object], metric_name: str) -> float:
    summary = case.get("arm_motion_summary", {})
    if not isinstance(summary, dict):
        return 0.0
    value = summary.get(metric_name, 0.0)
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _paired_pose_cases(cases: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    pairs: list[dict[str, object]] = []
    by_name: dict[str, dict[str, object]] = {}
    for index, case in enumerate(cases):
        name = str(case.get("name", f"case_{index + 1}"))
        pair = by_name.get(name)
        if pair is None:
            pair = {"name": name, "pose": case.get("pose", ()), "controllers": {}}
            by_name[name] = pair
            pairs.append(pair)
        controller = str(case.get("controller", "am"))
        controllers = pair["controllers"]
        if isinstance(controllers, dict):
            controllers[controller] = case
    return pairs


def _pair_metric_delta(pair: dict[str, object], summary_key: str, metric_name: str) -> float | None:
    controllers = pair.get("controllers", {})
    if not isinstance(controllers, dict):
        return None
    am = controllers.get("am")
    px4 = controllers.get("px4_position")
    if not isinstance(am, dict) or not isinstance(px4, dict):
        return None
    am_summary = am.get(summary_key, {})
    px4_summary = px4.get(summary_key, {})
    if not isinstance(am_summary, dict) or not isinstance(px4_summary, dict):
        return None
    try:
        return float(am_summary.get(metric_name, 0.0)) - float(px4_summary.get(metric_name, 0.0))
    except (TypeError, ValueError):
        return None


def _summary_metric(case: dict[str, object], summary_key: str, metric_name: str) -> float | None:
    summary = case.get(summary_key, {})
    if not isinstance(summary, dict):
        return None
    try:
        return float(summary.get(metric_name, 0.0))
    except (TypeError, ValueError):
        return None


def _pair_max_attitude_delta(pair: dict[str, object]) -> float | None:
    controllers = pair.get("controllers", {})
    if not isinstance(controllers, dict):
        return None
    values: dict[str, float] = {}
    for controller in ("am", "px4_position"):
        case = controllers.get(controller)
        if not isinstance(case, dict):
            return None
        components = [
            _summary_metric(case, "arm_motion_summary", "max_abs_roll_rad"),
            _summary_metric(case, "arm_motion_summary", "max_abs_pitch_rad"),
            _summary_metric(case, "arm_motion_summary", "max_abs_yaw_rad"),
        ]
        if any(value is None for value in components):
            return None
        values[controller] = max(float(value) for value in components if value is not None)
    return values["am"] - values["px4_position"]


def _paired_metric_delta_grid(pairs: Sequence[dict[str, object]]) -> tuple[list[str], Any]:
    import numpy as np

    metric_specs: tuple[tuple[str, str, str], ...] = (
        ("arm_motion_summary", "max_horizontal_offset_m", "max XY"),
        ("arm_motion_summary", "rms_horizontal_offset_m", "RMS XY"),
        ("arm_motion_summary", "max_vertical_offset_m", "max Z"),
        ("summary", "rms_speed_error_norm_mps", "RMS speed"),
        ("summary", "max_abs_lateral_velocity_bias_mps", "lateral bias"),
        ("summary", "rms_yaw_rate_error_radps", "yaw rate"),
    )
    labels = ["max XY", "RMS XY", "max Z", "attitude", "RMS speed", "lateral bias", "yaw rate"]
    matrix = np.full((len(labels), len(pairs)), np.nan, dtype=float)
    for column, pair in enumerate(pairs):
        for row, (summary_key, metric_name, _label) in enumerate(metric_specs[:3]):
            value = _pair_metric_delta(pair, summary_key, metric_name)
            if value is not None:
                matrix[row, column] = value
        attitude_delta = _pair_max_attitude_delta(pair)
        if attitude_delta is not None:
            matrix[3, column] = attitude_delta
        for offset, (summary_key, metric_name, _label) in enumerate(metric_specs[3:], start=4):
            value = _pair_metric_delta(pair, summary_key, metric_name)
            if value is not None:
                matrix[offset, column] = value
    return labels, matrix


def _pair_case(pair: dict[str, object], controller: str) -> dict[str, object] | None:
    controllers = pair.get("controllers", {})
    if not isinstance(controllers, dict):
        return None
    case = controllers.get(controller)
    return case if isinstance(case, dict) else None


def _pair_status_text(pair: dict[str, object]) -> str:
    parts: list[str] = []
    for label, controller in (("AM", "am"), ("PX4", "px4_position")):
        case = _pair_case(pair, controller)
        if case is None:
            parts.append(f"{label} missing")
        else:
            parts.append(f"{label} {'PASS' if bool(case.get('passed')) else 'FAIL'}")
    return " | ".join(parts)


_CASE_PALETTE = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#9467bd",
    "#d62728",
    "#8c564b",
    "#17becf",
    "#7f7f7f",
    "#bcbd22",
    "#e377c2",
)
_PASS_COLOR = "#2f855a"
_FAIL_COLOR = "#c53030"
_DELTA_NEGATIVE_COLOR = "#2b6cb0"
_DELTA_POSITIVE_COLOR = "#c05621"
_DELTA_NEUTRAL_COLOR = "#f7fafc"


def _case_palette(count: int) -> list[str]:
    return [_CASE_PALETTE[index % len(_CASE_PALETTE)] for index in range(max(0, int(count)))]


def _controller_line_style(controller: object) -> str:
    return "--" if str(controller) == "px4_position" else "-"


def _write_dict_rows(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


_ARM_BODY_NAMES = ("link_1", "link_2", "link_3", "link_4", "link_5", "gripper_left", "gripper_right")
_ARM_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_gripper_left",
    "joint_gripper_right",
)
_ARM_VISUAL_BODY_NAMES = {"link_1", "link_2", "link_3", "link_4", "link_5", "gripper_left", "gripper_right"}
_ARM_MODEL_CACHE: Any | None = None


def _x500_arm2x_asset_xml_path() -> Path:
    import importlib.util

    spec = importlib.util.find_spec("acesim.env.mujoco.mj_env")
    if spec is None or spec.origin is None:
        raise RuntimeError("Failed to locate acesim.env.mujoco.mj_env")
    return Path(spec.origin).resolve().parent / "asset" / "x500_arm2x" / "x500_arm2x.xml"


def _x500_arm2x_model() -> Any:
    global _ARM_MODEL_CACHE
    if _ARM_MODEL_CACHE is None:
        import mujoco

        _ARM_MODEL_CACHE = mujoco.MjModel.from_xml_path(str(_x500_arm2x_asset_xml_path()))
    return _ARM_MODEL_CACHE


def _set_x500_arm2x_pose(model: Any, data: Any, pose: Sequence[float]) -> None:
    values = validate_arm_pose(pose)
    import mujoco

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    home_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key_id >= 0:
        data.qpos[:] = model.key_qpos[home_key_id]

    for joint_name, value in zip(_ARM_JOINT_NAMES, values):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise RuntimeError(f"x500_arm2x joint not found: {joint_name}")
        data.qpos[int(model.jnt_qposadr[joint_id])] = value

    mujoco.mj_forward(model, data)


def _initialize_x500_arm2x_visual_mocaps(model: Any, data: Any) -> None:
    import re

    import mujoco

    for body_id in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        match = re.fullmatch(r"rotor_(\d+)_vis", body_name)
        if match is None:
            continue
        mocap_id = int(model.body_mocapid[body_id])
        if mocap_id < 0:
            continue
        rotor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{match.group(1)}")
        if rotor_id < 0:
            raise RuntimeError(f"x500_arm2x physical rotor body not found for {body_name}")
        data.mocap_pos[mocap_id] = data.xpos[rotor_id]
        data.mocap_quat[mocap_id] = data.xquat[rotor_id]
    mujoco.mj_forward(model, data)


def _x500_arm2x_pose_body_positions(pose: Sequence[float]) -> dict[str, tuple[float, float, float]]:
    """Evaluate the x500_arm2x target pose through MuJoCo FK for report checks."""

    import mujoco

    model = _x500_arm2x_model()
    data = mujoco.MjData(model)
    _set_x500_arm2x_pose(model, data, pose)
    _initialize_x500_arm2x_visual_mocaps(model, data)
    points: dict[str, tuple[float, float, float]] = {}
    for body_name in _ARM_BODY_NAMES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise RuntimeError(f"x500_arm2x body not found: {body_name}")
        position = data.xpos[body_id]
        points[body_name] = (float(position[0]), float(position[1]), float(position[2]))
    return points


def _mesh_world_bounds(model: Any, data: Any, *, visual_only: bool = True) -> tuple[Any, Any]:
    import mujoco
    import numpy as np

    bounds_min = np.full(3, np.inf, dtype=float)
    bounds_max = np.full(3, -np.inf, dtype=float)
    for geom_id in range(model.ngeom):
        if visual_only and int(model.geom_group[geom_id]) != 1:
            continue
        if int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0:
            continue
        vert_start = int(model.mesh_vertadr[mesh_id])
        vert_count = int(model.mesh_vertnum[mesh_id])
        vertices = np.asarray(model.mesh_vert[vert_start : vert_start + vert_count], dtype=float)
        geom_pos = np.asarray(data.geom_xpos[geom_id], dtype=float)
        geom_rot = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
        transformed = vertices @ geom_rot.T + geom_pos
        bounds_min = np.minimum(bounds_min, transformed.min(axis=0))
        bounds_max = np.maximum(bounds_max, transformed.max(axis=0))
    if not np.all(np.isfinite(bounds_min)) or not np.all(np.isfinite(bounds_max)):
        raise ValueError("x500_arm2x model has no visual mesh geoms to frame")
    return bounds_min, bounds_max


def _color_x500_arm2x_visual_geoms(model: Any, color: str) -> None:
    from matplotlib.colors import to_rgba

    arm_rgba = to_rgba(color, alpha=1.0)
    body_rgba = (0.72, 0.74, 0.78, 1.0)
    rotor_rgba = (0.20, 0.22, 0.25, 1.0)
    for geom_id in range(model.ngeom):
        if int(model.geom_group[geom_id]) != 1:
            continue
        body_id = int(model.geom_bodyid[geom_id])
        body_name = ""
        if body_id >= 0:
            body_name = str(model.body(body_id).name)
        if body_name in _ARM_VISUAL_BODY_NAMES:
            model.geom_rgba[geom_id] = arm_rgba
        elif body_name.startswith("rotor_"):
            model.geom_rgba[geom_id] = rotor_rgba
        else:
            model.geom_rgba[geom_id] = body_rgba


def _render_x500_arm2x_pose_inprocess(
    pose: Sequence[float],
    *,
    width: int = 320,
    height: int = 240,
    color: str = "#1f77b4",
) -> Any:
    import mujoco

    model = _x500_arm2x_model()
    data = mujoco.MjData(model)
    _set_x500_arm2x_pose(model, data, pose)
    _initialize_x500_arm2x_visual_mocaps(model, data)
    _color_x500_arm2x_visual_geoms(model, color)
    bounds_min, bounds_max = _mesh_world_bounds(model, data)
    center = (bounds_min + bounds_max) / 2.0
    span = bounds_max - bounds_min
    radius = max(float((span @ span) ** 0.5) / 2.0, 0.35)

    renderer = mujoco.Renderer(model, height, width)
    try:
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat = center.astype(float)
        camera.lookat[2] = float(bounds_min[2] + span[2] * 0.47)
        camera.distance = max(radius * 2.0, 0.85)
        camera.azimuth = 138.0
        camera.elevation = -24.0
        options = mujoco.MjvOption()
        mujoco.mjv_defaultOption(options)
        options.geomgroup[:] = 0
        options.geomgroup[1] = 1
        renderer.update_scene(data, camera=camera, scene_option=options)
        return renderer.render()
    finally:
        renderer.close()


def _render_x500_arm2x_pose(
    pose: Sequence[float],
    *,
    width: int = 320,
    height: int = 240,
    color: str = "#1f77b4",
) -> Any:
    """Render a pose in a short-lived process so MuJoCo/OpenGL aborts cannot kill the report."""

    import pickle

    output_file = tempfile.NamedTemporaryFile(prefix="acesim_x500_arm2x_render_", suffix=".pkl", delete=False)
    output_path = Path(output_file.name)
    output_file.close()
    worker_code = r"""
import importlib.util
import json
import os
import pickle
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "acesim_matplotlib"))
module_path = Path(sys.argv[1])
output_path = Path(sys.argv[2])
pose = json.loads(sys.argv[3])
width = int(sys.argv[4])
height = int(sys.argv[5])
color = sys.argv[6]
spec = importlib.util.spec_from_file_location("acesim_x500_arm2x_render_worker", module_path)
module = importlib.util.module_from_spec(spec)
sys.modules["acesim_x500_arm2x_render_worker"] = module
spec.loader.exec_module(module)
image = module._render_x500_arm2x_pose_inprocess(pose, width=width, height=height, color=color)
output_path.write_bytes(pickle.dumps(image, protocol=pickle.HIGHEST_PROTOCOL))
"""
    env = dict(os.environ)
    env.setdefault("MUJOCO_GL", "egl")
    env.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "acesim_matplotlib"))
    try:
        completed = subprocess.run(
            [
                sys.executable,
                "-c",
                worker_code,
                str(Path(__file__).resolve()),
                str(output_path),
                json.dumps(list(pose)),
                str(int(width)),
                str(int(height)),
                color,
            ],
            check=False,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30.0,
        )
        if completed.returncode != 0:
            stderr = completed.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(stderr[-600:] or f"renderer exited with {completed.returncode}")
        return pickle.loads(output_path.read_bytes())
    finally:
        output_path.unlink(missing_ok=True)


def _draw_x500_arm2x_pose_gallery_panel(
    ax: Any,
    cases: Sequence[dict[str, object]],
    names: Sequence[str],
    colors: Sequence[str],
    status_texts: Sequence[str] | None = None,
) -> None:
    import numpy as np

    ax.set_title("B  Pose configurations and paired status")
    ax.set_axis_off()
    if not cases:
        ax.text(0.5, 0.5, "no cases", transform=ax.transAxes, ha="center", va="center", fontsize=9)
        return

    try:
        images = [
            _render_x500_arm2x_pose(
                cast(Sequence[float], case.get("pose", [0.0] * 7)),
                width=260,
                height=190,
                color=colors[index],
            )
            for index, case in enumerate(cases)
        ]
    except Exception as exc:
        ax.text(
            0.5,
            0.5,
            f"MuJoCo renderer unavailable\n{exc}",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=9,
            color="#4a5568",
        )
        return

    if len(images) <= 4:
        columns = len(images)
    else:
        columns = min(5, max(1, math.ceil(len(images) / 2.0)))
    rows = math.ceil(len(images) / columns)
    image_h, image_w = images[0].shape[:2]
    label_h = 44
    gap = 12
    tile_h = image_h + label_h
    gallery_h = rows * tile_h + (rows - 1) * gap
    gallery_w = columns * image_w + (columns - 1) * gap
    gallery = np.full((gallery_h, gallery_w, 3), 255, dtype=images[0].dtype)
    for index, image in enumerate(images):
        row = index // columns
        column = index % columns
        y0 = row * (tile_h + gap)
        x0 = column * (image_w + gap)
        gallery[y0 : y0 + image_h, x0 : x0 + image_w] = image
        label_y0 = y0 + image_h
        gallery[label_y0 : label_y0 + label_h, x0 : x0 + image_w] = _render_gallery_label_strip(
            names[index],
            colors[index],
            status_text=status_texts[index] if status_texts is not None and index < len(status_texts) else "",
            width=image_w,
            height=label_h,
        )
    ax.imshow(gallery)


def _render_gallery_label_strip(name: str, color: str, *, width: int, height: int, status_text: str = "") -> Any:
    import numpy as np
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.patches import Rectangle

    figure = Figure(figsize=(width / 100.0, height / 100.0), dpi=100)
    figure.patch.set_facecolor("white")
    canvas = FigureCanvasAgg(figure)
    label_ax = figure.add_axes([0.0, 0.0, 1.0, 1.0])
    label_ax.set_axis_off()
    label_ax.set_facecolor("white")
    label_ax.add_patch(Rectangle((0.0, 0.0), 0.025, 1.0, transform=label_ax.transAxes, color=color, linewidth=0))
    words = str(name).split("_")
    if len(str(name)) > 16 and len(words) > 1:
        midpoint = math.ceil(len(words) / 2)
        label = "_".join(words[:midpoint]) + "\n" + "_".join(words[midpoint:])
        fontsize = 7.0
    else:
        label = str(name)
        fontsize = 7.5 if len(label) > 13 else 8.0
    label_ax.text(
        0.52,
        0.63 if status_text else 0.52,
        label,
        transform=label_ax.transAxes,
        ha="center",
        va="center",
        fontsize=fontsize,
        fontweight="bold",
        color="#1a202c",
        linespacing=0.9,
    )
    if status_text:
        label_ax.text(
            0.52,
            0.22,
            status_text,
            transform=label_ax.transAxes,
            ha="center",
            va="center",
            fontsize=6.2,
            color="#4a5568",
        )
    canvas.draw()
    rgba = np.asarray(canvas.buffer_rgba(), dtype=np.uint8)
    return np.asarray(rgba[:, :, :3], dtype=np.uint8)


def _draw_pass_fail_matrix(ax: Any, pairs: Sequence[dict[str, object]]) -> None:
    import numpy as np
    from matplotlib.colors import ListedColormap

    if not pairs:
        ax.set_title("pass/fail")
        ax.text(0.5, 0.5, "No pose pairs", transform=ax.transAxes, ha="center", va="center")
        ax.set_axis_off()
        return

    controllers = (("am", "AM"), ("px4_position", "PX4 Position"))
    matrix = np.zeros((len(controllers), len(pairs)), dtype=float)
    for column, pair in enumerate(pairs):
        for row, (controller, _label) in enumerate(controllers):
            case = _pair_case(pair, controller)
            if case is None:
                matrix[row, column] = 0.0
            else:
                matrix[row, column] = 1.0 if bool(case.get("passed")) else -1.0
    cmap = ListedColormap([_FAIL_COLOR, "#e2e8f0", _PASS_COLOR])
    ax.imshow(matrix, aspect="auto", cmap=cmap, vmin=-1.0, vmax=1.0)
    ax.set_title("pass/fail")
    ax.set_xticks(range(len(pairs)), [str(index + 1) for index in range(len(pairs))])
    ax.set_xlabel("Pose index, matched to panel B")
    ax.set_yticks(range(len(controllers)), [label for _controller, label in controllers])
    ax.tick_params(axis="both", labelsize=7)
    ax.set_xticks(np.arange(-0.5, len(pairs), 1.0), minor=True)
    ax.set_yticks(np.arange(-0.5, len(controllers), 1.0), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.2)
    ax.tick_params(which="minor", bottom=False, left=False)
    for column, pair in enumerate(pairs):
        for row, (controller, _label) in enumerate(controllers):
            case = _pair_case(pair, controller)
            text = "NA" if case is None else ("P" if bool(case.get("passed")) else "F")
            ax.text(
                column,
                row,
                text,
                ha="center",
                va="center",
                fontsize=8.0,
                fontweight="bold",
                color="white" if text != "NA" else "#4a5568",
            )
    for spine in ax.spines.values():
        spine.set_visible(False)


def _draw_paired_delta_heatmap(ax: Any, pairs: Sequence[dict[str, object]], *, title: str) -> None:
    import numpy as np
    from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm

    labels, matrix = _paired_metric_delta_grid(pairs)
    ax.set_title(title)
    if matrix.size == 0:
        ax.text(0.5, 0.5, "No paired metrics", transform=ax.transAxes, ha="center", va="center")
        ax.set_axis_off()
        return
    finite = matrix[np.isfinite(matrix)]
    limit = float(np.nanmax(np.abs(finite))) if finite.size else 1.0
    if limit <= 0.0:
        limit = 1.0
    cmap = LinearSegmentedColormap.from_list(
        "acesim_delta",
        [_DELTA_NEGATIVE_COLOR, _DELTA_NEUTRAL_COLOR, _DELTA_POSITIVE_COLOR],
    )
    cmap.set_bad("#e2e8f0")
    image = ax.imshow(matrix, aspect="auto", cmap=cmap, norm=TwoSlopeNorm(vcenter=0.0, vmin=-limit, vmax=limit))
    ax.set_xticks(range(len(pairs)), [str(index + 1) for index in range(len(pairs))])
    ax.set_yticks(range(len(labels)), labels)
    ax.tick_params(axis="both", labelsize=7)
    ax.set_xlabel("pose index, matched to B")
    ax.set_xticks(np.arange(-0.5, len(pairs), 1.0), minor=True)
    ax.set_yticks(np.arange(-0.5, len(labels), 1.0), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.0)
    ax.tick_params(which="minor", bottom=False, left=False)
    for row in range(matrix.shape[0]):
        for column in range(matrix.shape[1]):
            value = matrix[row, column]
            if not np.isfinite(value):
                text = "NA"
                color = "#4a5568"
            else:
                text = f"{value:+.2f}"
                color = "#1a202c" if abs(value) < limit * 0.55 else "white"
            ax.text(column, row, text, ha="center", va="center", fontsize=5.8, color=color)
    for spine in ax.spines.values():
        spine.set_visible(False)
    colorbar = ax.figure.colorbar(image, ax=ax, fraction=0.035, pad=0.015)
    colorbar.ax.tick_params(labelsize=6)
    colorbar.set_label("AM - PX4", fontsize=7)


def _draw_paired_overview_panel(ax: Any, pairs: Sequence[dict[str, object]]) -> None:
    ax.set_axis_off()
    ax.set_title("A  Paired benchmark overview", loc="left", fontweight="bold")
    status_ax = ax.inset_axes([0.0, 0.74, 1.0, 0.20])
    delta_ax = ax.inset_axes([0.0, 0.00, 1.0, 0.64])
    _draw_pass_fail_matrix(status_ax, pairs)
    _draw_paired_delta_heatmap(delta_ax, pairs, title="key metric deltas")
    ax.text(
        0.0,
        0.255,
        "Delta cells show AM - PX4 Position; negative values favor AM for these error/disturbance metrics.",
        transform=ax.transAxes,
        fontsize=7.5,
        color="#4a5568",
    )


def _collect_pair_delta_series(
    pairs: Sequence[dict[str, object]],
    metrics: Sequence[tuple[str, str, str]],
    extra_values: Sequence[tuple[str, Sequence[float | None]]] = (),
) -> list[tuple[str, list[float | None]]]:
    series: list[tuple[str, list[float | None]]] = [
        (label, [_pair_metric_delta(pair, summary_key, metric_name) for pair in pairs])
        for summary_key, metric_name, label in metrics
    ]
    series.extend((label, list(values)) for label, values in extra_values)
    return series


def _draw_pair_delta_forest(
    ax: Any,
    pairs: Sequence[dict[str, object]],
    *,
    title: str,
    metrics: Sequence[tuple[str, str, str]],
    ylabel: str,
    extra_values: Sequence[tuple[str, Sequence[float | None]]] = (),
) -> None:
    pass

    names = [str(pair["name"]) for pair in pairs]
    series = _collect_pair_delta_series(pairs, metrics, extra_values)
    if not series:
        ax.set_title(title)
        ax.text(0.5, 0.5, "No paired metrics", transform=ax.transAxes, ha="center", va="center")
        ax.set_axis_off()
        return
    y_positions: list[float] = []
    labels: list[str] = []
    values: list[float | None] = []
    colors: list[str] = []
    metric_colors = ("#2b6cb0", "#805ad5", "#319795", "#dd6b20")
    group_gap = 0.6
    y = 0.0
    for pose_index, pose_name in enumerate(names):
        labels.append(pose_name)
        y_positions.append(y)
        values.append(None)
        colors.append("#000000")
        for metric_index, (metric_label, metric_values) in enumerate(series):
            y += 1.0
            labels.append(f"  {metric_label}")
            y_positions.append(y)
            values.append(metric_values[pose_index] if pose_index < len(metric_values) else None)
            colors.append(metric_colors[metric_index % len(metric_colors)])
        y += group_gap
    all_values: list[float] = []
    for value in values:
        if value is not None:
            all_values.append(abs(float(value)))
    limit = max(all_values) if all_values else 1.0
    if limit <= 0.0:
        limit = 1.0
    ax.axvline(0.0, color="#1a202c", linewidth=0.8)
    for y_pos, label, value, color in zip(y_positions, labels, values, colors):
        if value is None:
            if not label.startswith("  "):
                ax.axhspan(y_pos - 0.45, y_pos + len(series) + 0.45, color="#f7fafc", zorder=0)
            else:
                ax.text(0.0, y_pos, "NA", ha="center", va="center", fontsize=6, color="#718096")
            continue
        ax.hlines(y_pos, 0.0, float(value), color=color, linewidth=1.8, alpha=0.9)
        ax.plot(float(value), y_pos, "o", color=color, markersize=4.0)
    ax.set_xlim(-limit * 1.18, limit * 1.18)
    ax.set_ylim(max(y_positions) + 0.5, -0.6)
    ax.set_title(title, loc="left", fontweight="bold")
    ax.set_xlabel(ylabel)
    ax.set_yticks(y_positions, labels)
    ax.tick_params(axis="y", labelsize=6.4)
    ax.tick_params(axis="x", labelsize=7)
    ax.grid(True, axis="x", alpha=0.22)


def _draw_pair_delta_bars(
    ax: Any,
    pairs: Sequence[dict[str, object]],
    *,
    title: str,
    metrics: Sequence[tuple[str, str, str]],
    ylabel: str,
    extra_values: Sequence[tuple[str, Sequence[float | None]]] = (),
) -> None:
    _draw_pair_delta_forest(
        ax,
        pairs,
        title=title,
        metrics=metrics,
        ylabel=ylabel,
        extra_values=extra_values,
    )


def write_raw_output(result: dict[str, object], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    cases_dir = output_dir / "cases"
    if cases_dir.exists():
        shutil.rmtree(cases_dir)
    compact = json.loads(json.dumps(result))
    for case in compact.get("cases", []):
        if isinstance(case, dict):
            case.pop("arm_motion_samples", None)
            case.pop("velocity_tracking_samples", None)
    (output_dir / "summary.json").write_text(json.dumps(compact, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    for case in cast(list[object], result.get("cases", [])):
        if not isinstance(case, dict):
            continue
        case_name = str(case.get("name", "case"))
        controller = str(case.get("controller", "am"))
        case_dir = cases_dir / f"{case_name}_{controller}"
        case_dir.mkdir(parents=True, exist_ok=True)
        metadata = dict(case)
        arm_rows = metadata.pop("arm_motion_samples", [])
        velocity_rows = metadata.pop("velocity_tracking_samples", [])
        (case_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        _write_dict_rows(case_dir / "arm_motion.csv", arm_rows if isinstance(arm_rows, list) else [])
        _write_dict_rows(
            case_dir / "velocity_tracking.csv",
            velocity_rows if isinstance(velocity_rows, list) else [],
        )


def write_benchmark_report_image(result: dict[str, object], output_path: Path) -> None:
    mpl_config_dir = Path(tempfile.gettempdir()) / "acesim_matplotlib"
    mpl_config_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config_dir))

    import matplotlib

    matplotlib.use("Agg")
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)
        from matplotlib import pyplot as plt

    cases = [case for case in cast(list[object], result.get("cases", [])) if isinstance(case, dict)]
    pairs = _paired_pose_cases(cases)
    pose_names = [str(pair["name"]) for pair in pairs]
    pose_colors = _case_palette(len(pairs))
    pose_cases = [
        _pair_case(pair, "am") or _pair_case(pair, "px4_position") or {"pose": pair.get("pose", ())} for pair in pairs
    ]
    status_texts = [_pair_status_text(pair) for pair in pairs]

    figure = plt.figure(figsize=(32, 20), constrained_layout=True)
    grid = figure.add_gridspec(3, 6, height_ratios=(2.45, 1.62, 1.62), width_ratios=(1.0, 1.0, 1.0, 1.18, 1.18, 1.18))
    figure.suptitle(
        f"{result.get('asset', 'benchmark')} benchmark: {'PASS' if result.get('passed') else 'FAIL'} "
        f"({sum(1 for case in cases if bool(case.get('passed')))}/{len(cases)} controller runs)",
        fontsize=20,
        fontweight="bold",
    )

    complete_pairs = sum(
        1 for pair in pairs if _pair_case(pair, "am") is not None and _pair_case(pair, "px4_position") is not None
    )
    tracking_samples = sum(_case_metric(case, "sample_count") for case in cases)
    figure.text(
        0.5,
        0.965,
        (
            "AM - PX4 Position deltas; lower is better for all plotted error/disturbance metrics, "
            "so negative values favor AM. "
            f"{len(pairs)} poses, {complete_pairs} complete pairs, {int(tracking_samples)} tracking samples; "
            "raw time series remain in CSV."
        ),
        ha="center",
        va="top",
        fontsize=10,
        color="#4a5568",
    )

    overview_ax = figure.add_subplot(grid[0, :3])
    _draw_paired_overview_panel(overview_ax, pairs)

    gallery_ax = figure.add_subplot(grid[0, 3:])
    try:
        _draw_x500_arm2x_pose_gallery_panel(
            gallery_ax,
            pose_cases,
            pose_names,
            pose_colors,
            status_texts=status_texts,
        )
    except Exception as exc:
        gallery_ax.set_title("B  Pose configurations and paired status")
        gallery_ax.text(
            0.5,
            0.5,
            f"MuJoCo renderer unavailable\n{exc}",
            transform=gallery_ax.transAxes,
            ha="center",
            va="center",
            fontsize=9,
            color="#4a5568",
        )
        gallery_ax.set_axis_off()

    arm_position_ax = figure.add_subplot(grid[1:, :2])
    _draw_pair_delta_forest(
        arm_position_ax,
        pairs,
        title="C  Arm-motion position disturbance",
        metrics=(
            ("arm_motion_summary", "max_horizontal_offset_m", "max XY"),
            ("arm_motion_summary", "rms_horizontal_offset_m", "RMS XY"),
            ("arm_motion_summary", "max_vertical_offset_m", "max Z"),
        ),
        ylabel="meters, AM - PX4",
    )

    arm_attitude_ax = figure.add_subplot(grid[1:, 2:4])
    _draw_pair_delta_forest(
        arm_attitude_ax,
        pairs,
        title="D  Arm-motion attitude disturbance",
        metrics=(
            ("arm_motion_summary", "max_abs_roll_rad", "roll"),
            ("arm_motion_summary", "max_abs_pitch_rad", "pitch"),
            ("arm_motion_summary", "max_abs_yaw_rad", "yaw"),
        ),
        ylabel="radians, AM - PX4",
    )

    tracking_ax = figure.add_subplot(grid[1:, 4:])
    _draw_pair_delta_forest(
        tracking_ax,
        pairs,
        title="E  Velocity-tracking error",
        metrics=(
            ("summary", "rms_speed_error_norm_mps", "RMS speed"),
            ("summary", "max_abs_lateral_velocity_bias_mps", "max lateral"),
            ("summary", "rms_yaw_rate_error_radps", "RMS yaw rate"),
        ),
        ylabel="AM - PX4",
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the x500_arm2x takeoff and velocity-tracking benchmark.")
    parser.add_argument("--case", action="append", dest="case_names", help="Run only the named pose case.")
    parser.add_argument("--output", type=Path, help="Optional summary figure path (.png or .pdf).")
    parser.add_argument("--json-output", type=Path, help="Optional compact JSON summary path.")
    parser.add_argument("--raw-output-dir", type=Path, help="Directory for raw per-case benchmark data.")
    parser.add_argument("--px4-repo", help="PX4-Autopilot path override.")
    parser.add_argument("--bridge-mode", choices=["linux", "wsl"], default="linux")
    parser.add_argument("--setpoint-rate-hz", type=float, default=50.0)
    parser.add_argument("--takeoff-altitude-m", type=float, default=1.5)
    parser.add_argument("--takeoff-altitude-tolerance-m", type=float, default=0.02)
    parser.add_argument("--takeoff-timeout-s", type=float, default=25.0)
    parser.add_argument("--startup-timeout-s", type=float, default=180.0)
    parser.add_argument("--case-start-attempts", type=int, default=1)
    parser.add_argument("--log-dir", type=Path, help="Directory for per-process logs.")
    parser.add_argument("--verbose-process-logs", action="store_true", help="Stream PX4 and simulator logs to stderr.")
    parser.add_argument("--strict-exit-code", action="store_true", help="Return 1 when any benchmark case fails.")
    parser.add_argument("--post-takeoff-settle-s", type=float, default=3.0)
    parser.add_argument("--arm-motion-duration-s", type=float, default=10.0)
    parser.add_argument("--post-arm-motion-settle-s", type=float, default=2.0)
    parser.add_argument("--am-offboard-settle-s", type=float, default=2.0)
    parser.add_argument("--max-profile-altitude-m", type=float, default=8.0)
    parser.add_argument("--profile-cycles", type=int, default=1)
    parser.add_argument("--jobs", default="auto", help="'auto' runs all selected cases in parallel; use 1 for serial.")
    parser.add_argument("--port-base", type=int, default=DEFAULT_CLOCK_ZMQ_PORT)
    parser.add_argument("--xrce-port-base", type=int, default=DEFAULT_XRCE_PORT)
    parser.add_argument("--ros-domain-base", type=int, default=DEFAULT_ROS_DOMAIN_ID)
    parser.add_argument("--px4-instance-base", type=int, default=0)
    args = parser.parse_args(argv)
    if args.profile_cycles <= 0:
        parser.error("--profile-cycles must be positive")
    if args.takeoff_altitude_tolerance_m < 0.0:
        parser.error("--takeoff-altitude-tolerance-m must be non-negative")
    if args.case_start_attempts <= 0:
        parser.error("--case-start-attempts must be positive")
    if args.post_takeoff_settle_s < 0.0:
        parser.error("--post-takeoff-settle-s must be non-negative")
    if args.arm_motion_duration_s <= 0.0:
        parser.error("--arm-motion-duration-s must be positive")
    if args.post_arm_motion_settle_s < 0.0:
        parser.error("--post-arm-motion-settle-s must be non-negative")
    if args.am_offboard_settle_s < 0.0:
        parser.error("--am-offboard-settle-s must be non-negative")
    if args.max_profile_altitude_m <= 0.0:
        parser.error("--max-profile-altitude-m must be positive")
    if args.port_base <= 0 or args.xrce_port_base <= 0:
        parser.error("--port-base and --xrce-port-base must be positive")
    if args.ros_domain_base < 0:
        parser.error("--ros-domain-base must be non-negative")
    if args.px4_instance_base < 0:
        parser.error("--px4-instance-base must be non-negative")
    return args


def _resolve_jobs(value: object, case_count: int) -> int:
    if isinstance(value, str) and value.strip().lower() in {"auto", "all"}:
        return max(1, case_count)
    jobs = int(cast(Any, value))
    if jobs <= 0:
        raise ValueError("--jobs must be 'auto' or a positive integer")
    return min(jobs, max(1, case_count))


def _select_cases(case_names: Sequence[str] | None) -> list[ArmPoseCase]:
    cases = default_arm_pose_cases()
    if not case_names:
        return cases
    by_name = {case.name: case for case in cases}
    selected: list[ArmPoseCase] = []
    for name in case_names:
        if name not in by_name:
            supported = ", ".join(by_name)
            raise ValueError(f"Unknown x500_arm2x benchmark case '{name}'. Supported cases: {supported}")
        selected.append(by_name[name])
    return selected


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    selected_cases = _select_cases(args.case_names)
    selected_controller_case_count = len(selected_cases) * len(CONTROLLER_VARIANTS)
    try:
        jobs = _resolve_jobs(args.jobs, selected_controller_case_count)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    max_px4_slots = 10 - int(args.px4_instance_base)
    if jobs > max_px4_slots and str(args.jobs).strip().lower() in {"auto", "all"}:
        jobs = max(1, max_px4_slots)
    if args.px4_instance_base + jobs - 1 > 9:
        raise SystemExit("--px4-instance-base + --jobs - 1 must be <= 9 to avoid PX4 MAVLink port collisions")

    profile_config = VelocityTrackingProfileConfig(cycles=args.profile_cycles)
    process_log_dir = args.log_dir
    raw_output_dir = args.raw_output_dir
    if process_log_dir is None and args.output is not None:
        raw_output_dir = raw_output_dir or (args.output.parent / f"{args.output.stem}_raw")
        process_log_dir = raw_output_dir / "process_logs"
    elif process_log_dir is None and raw_output_dir is not None:
        process_log_dir = raw_output_dir / "process_logs"
    runtime_config = BenchmarkRuntimeConfig(
        setpoint_rate_hz=args.setpoint_rate_hz,
        takeoff_altitude_m=args.takeoff_altitude_m,
        takeoff_altitude_tolerance_m=args.takeoff_altitude_tolerance_m,
        takeoff_timeout_s=args.takeoff_timeout_s,
        startup_timeout_s=args.startup_timeout_s,
        case_start_attempts=args.case_start_attempts,
        post_takeoff_settle_s=args.post_takeoff_settle_s,
        arm_motion_duration_s=args.arm_motion_duration_s,
        post_arm_motion_settle_s=args.post_arm_motion_settle_s,
        am_offboard_settle_s=args.am_offboard_settle_s,
        max_profile_altitude_m=args.max_profile_altitude_m,
        jobs=jobs,
        port_base=args.port_base,
        xrce_port_base=args.xrce_port_base,
        ros_domain_base=args.ros_domain_base,
        px4_instance_base=args.px4_instance_base,
        bridge_mode=args.bridge_mode,
        px4_repo=args.px4_repo,
        process_log_dir=str(process_log_dir) if process_log_dir is not None else None,
        verbose_process_logs=args.verbose_process_logs,
        profile_config=profile_config,
    )
    result = run_benchmark(selected_cases, runtime_config)
    compact_result = json.loads(json.dumps(result))
    for case in cast(list[object], compact_result.get("cases", [])):
        if isinstance(case, dict):
            case.pop("arm_motion_samples", None)
            case.pop("velocity_tracking_samples", None)
    result_text = json.dumps(compact_result, indent=2, sort_keys=True)
    cases = [case for case in cast(list[object], result.get("cases", [])) if isinstance(case, dict)]
    passed_count = sum(1 for case in cases if bool(case.get("passed")))
    print(
        f"x500_arm2x benchmark: result {'PASS' if result['passed'] else 'FAIL'} "
        f"({passed_count}/{len(cases)} cases passed)"
    )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        write_benchmark_report_image(result, args.output)
        print(f"x500_arm2x benchmark: report written to {args.output}")
    if raw_output_dir is not None:
        write_raw_output(result, raw_output_dir)
        print(f"x500_arm2x benchmark: raw data written to {raw_output_dir}")
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(result_text + "\n", encoding="utf-8")
        print(f"x500_arm2x benchmark: json written to {args.json_output}")
    return 1 if args.strict_exit_code and not bool(result["passed"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
