from __future__ import annotations

import unittest

import numpy as np

from acesim.utils.vehicle_visual_state_publisher import (
    VehicleVisualState,
    VehicleVisualStatePublisher,
    VehicleVisualStreamParams,
)


class VehicleVisualStatePublisherTests(unittest.TestCase):
    def test_stream_params_defaults_when_config_missing(self) -> None:
        params = VehicleVisualStreamParams.from_asset_params({})
        self.assertFalse(params.enabled)
        self.assertEqual(params.rate_hz, 120.0)
        self.assertEqual(params.zmq_endpoint, "tcp://0.0.0.0:5601")

    def test_unpack_round_trip_matches_original_values(self) -> None:
        state = VehicleVisualState(
            timestamp_us=123456,
            position_world_m_nwu=np.array([1.0, -2.0, 3.5], dtype=float),
            attitude_world_quat_scalar_first=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            rotor_angle_rad=np.array([0.1, 0.2, 0.3, 0.4], dtype=float),
            rotor_visual_speed_radps=np.array([10.0, 20.0, 30.0, 40.0], dtype=float),
        )

        payload = VehicleVisualStatePublisher._STRUCT.pack(
            int(state.timestamp_us),
            *state.position_world_m_nwu.tolist(),
            *state.attitude_world_quat_scalar_first.tolist(),
            4,
            *state.rotor_angle_rad.tolist(),
            *([0.0] * (VehicleVisualStatePublisher.MAX_ROTORS - 4)),
            *state.rotor_visual_speed_radps.tolist(),
            *([0.0] * (VehicleVisualStatePublisher.MAX_ROTORS - 4)),
        )
        decoded = VehicleVisualStatePublisher.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], state.timestamp_us)
        self.assertEqual(decoded["rotor_count"], 4)
        np.testing.assert_allclose(decoded["position_world_m_nwu"], state.position_world_m_nwu)
        np.testing.assert_allclose(
            decoded["attitude_world_quat_scalar_first"],
            state.attitude_world_quat_scalar_first,
        )
        np.testing.assert_allclose(decoded["rotor_angle_rad"], state.rotor_angle_rad)
        np.testing.assert_allclose(decoded["rotor_visual_speed_radps"], state.rotor_visual_speed_radps)


if __name__ == "__main__":
    unittest.main()
