from __future__ import annotations

import math

import pytest

from acesim.benchmark.x500_arm2x_velocity import (
    VelocityTrackingMetrics,
    VelocityTrackingProfile,
    VelocityTrackingProfileConfig,
    heading_frame_velocity_to_world_enu,
    velocity_enu_to_ned,
    world_enu_velocity_to_heading_frame,
    yaw_rate_enu_to_ned,
)


def test_heading_frame_velocity_rotates_with_world_heading() -> None:
    forward_world = heading_frame_velocity_to_world_enu(math.pi / 2.0, (1.0, 0.0, 0.0))
    left_world = heading_frame_velocity_to_world_enu(math.pi / 2.0, (0.0, 1.0, 0.0))

    assert forward_world == pytest.approx((0.0, 1.0, 0.0))
    assert left_world == pytest.approx((-1.0, 0.0, 0.0))
    assert world_enu_velocity_to_heading_frame(math.pi / 2.0, forward_world) == pytest.approx((1.0, 0.0, 0.0))


def test_enu_velocity_and_yaw_rate_convert_to_px4_ned() -> None:
    assert velocity_enu_to_ned((1.0, 2.0, 3.0)) == pytest.approx((2.0, 1.0, -3.0))
    assert yaw_rate_enu_to_ned(0.7) == pytest.approx(-0.7)


def test_velocity_tracking_profile_matches_acepilot_segment_order() -> None:
    profile = VelocityTrackingProfile(
        VelocityTrackingProfileConfig(
            segment_duration_s=2.0,
            rest_duration_s=1.0,
            cycles=1,
            forward_speed_mps=0.4,
            lateral_speed_mps=0.3,
            vertical_speed_mps=0.2,
            yaw_rate_radps=0.3,
            include_yaw_rate=True,
        )
    )

    assert profile.sample(0.5).segment_name == "rest_initial"
    assert profile.sample(1.5).segment_name == "forward"
    assert profile.sample(1.5).velocity_h == pytest.approx((0.4, 0.0, 0.0))
    assert profile.sample(4.5).segment_name == "backward"
    assert profile.sample(7.5).segment_name == "left"
    assert profile.sample(10.5).segment_name == "right"
    assert profile.sample(13.5).segment_name == "up"
    assert profile.sample(16.5).segment_name == "down"
    assert profile.sample(19.5).segment_name == "yaw_positive"
    assert profile.sample(19.5).yaw_rate == pytest.approx(0.3)
    assert profile.sample(22.5).segment_name == "yaw_negative"
    assert profile.sample(22.5).yaw_rate == pytest.approx(-0.3)

    complete = profile.sample(100.0)
    assert not complete.active
    assert complete.segment_name == "complete"


def test_velocity_tracking_metrics_records_heading_frame_errors() -> None:
    metrics = VelocityTrackingMetrics()
    profile = VelocityTrackingProfile(VelocityTrackingProfileConfig(include_yaw_rate=True))
    command = profile.sample(2.5)

    metrics.record(
        heading_w=0.0,
        actual_velocity_world_enu=(0.4, 0.1, 0.0),
        actual_yaw_rate_flu_radps=0.2,
        command=command,
    )

    summary = metrics.summary()

    assert summary.sample_count == 1
    assert summary.mean_left_error_mps == pytest.approx(-0.1)
    assert summary.rms_speed_error_norm_mps == pytest.approx(0.1)
    assert summary.mean_lateral_velocity_bias_mps == pytest.approx(0.1)
    assert summary.forward_segment_lateral_sample_count == 1
    assert summary.mean_forward_segment_actual_left_mps == pytest.approx(0.1)
    assert summary.mean_yaw_rate_error_radps == pytest.approx(0.0)
    assert summary.to_dict()["sample_count"] == 1
