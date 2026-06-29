from __future__ import annotations

import unittest

import numpy as np

from acesim.utils.dynamics import (
    DownwashParams,
    LumpedDragParams,
    RotorFlowParams,
    RotorInertialTorqueParams,
    ThrottleToOmegaParams,
    first_order_response_step,
    idle_visual_speed_target,
    rotor_thrust_moment_along_axis,
    thruster_wrenches_from_speed,
)


class DynamicsUtilsTests(unittest.TestCase):
    def test_lumped_drag_params_default_to_disabled_zero_drag(self) -> None:
        params = LumpedDragParams.from_config(None)

        self.assertFalse(params.enabled)
        np.testing.assert_allclose(params.d, np.zeros(3, dtype=float))

    def test_lumped_drag_params_parse_mass_normalized_config(self) -> None:
        params = LumpedDragParams.from_config(
            {
                "enabled": True,
                "units": "mass_normalized",
                "D": [0.20, 0.10, 0.00],
            }
        )

        self.assertTrue(params.enabled)
        np.testing.assert_allclose(params.d, np.array([0.20, 0.10, 0.00], dtype=float))

    def test_lumped_drag_params_reject_unsupported_units(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unsupported lumped_drag units"):
            LumpedDragParams.from_config({"enabled": True, "units": "raw", "D": [0.20, 0.10, 0.00]})

    def test_lumped_drag_params_require_three_diagonal_coefficients(self) -> None:
        with self.assertRaisesRegex(ValueError, "lumped_drag D must contain exactly three"):
            LumpedDragParams.from_config({"enabled": True, "units": "mass_normalized", "D": [0.20, 0.10]})

    def test_rotor_flow_params_default_to_disabled_no_correction(self) -> None:
        params = RotorFlowParams.from_config(None)

        self.assertFalse(params.enabled)
        self.assertEqual(params.advance_c_mu, 0.0)
        self.assertEqual(params.advance_scale_min, 1.0)
        self.assertFalse(params.ground_effect_enabled)
        self.assertAlmostEqual(params.ground_effect_normal_min_dot, 0.5)

    def test_rotor_flow_params_parse_configured_defaults(self) -> None:
        params = RotorFlowParams.from_config(
            {
                "enabled": True,
                "advance_c_lambda": 0.0,
                "advance_c_mu": 0.0,
                "advance_scale_min": 0.85,
                "advance_scale_max": 1.10,
                "ground_effect_enabled": True,
                "ground_effect_max_scale": 1.25,
                "ground_effect_height_rotor_diameters": 1.0,
                "ground_effect_normal_min_dot": 0.6,
            }
        )

        self.assertTrue(params.enabled)
        self.assertAlmostEqual(params.advance_c_mu, 0.0)
        self.assertAlmostEqual(params.advance_scale_min, 0.85)
        self.assertTrue(params.ground_effect_enabled)
        self.assertAlmostEqual(params.ground_effect_max_scale, 1.25)
        self.assertAlmostEqual(params.ground_effect_normal_min_dot, 0.6)

    def test_rotor_flow_params_reject_invalid_bounds(self) -> None:
        invalid_configs: list[dict[str, object]] = [
            {"advance_scale_min": float("inf")},
            {"advance_scale_min": -0.1},
            {"advance_scale_min": 1.2, "advance_scale_max": 1.1},
            {"ground_effect_max_scale": 0.9},
            {"ground_effect_height_rotor_diameters": -0.1},
            {"ground_effect_normal_min_dot": 1.1},
        ]

        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    RotorFlowParams.from_config(config)

    def test_downwash_params_default_to_disabled_no_targets(self) -> None:
        params = DownwashParams.from_config(None)

        self.assertFalse(params.enabled)
        self.assertEqual(params.exclude_body_patterns, ())
        self.assertFalse(hasattr(params, "force_coeff"))
        self.assertFalse(hasattr(params, "model"))

    def test_downwash_params_parse_configured_exclusion_patterns(self) -> None:
        params = DownwashParams.from_config(
            {
                "enabled": True,
                "exclude_body_patterns": ["base_link", "rotor_*"],
                "drag_coefficient": 1.1,
                "area_scale": 1.0,
                "wake_speed_scale": 1.0,
                "wake_spread_angle_rad": 0.20,
                "axial_decay_m": 0.45,
            }
        )

        self.assertTrue(params.enabled)
        self.assertEqual(params.exclude_body_patterns, ("base_link", "rotor_*"))
        self.assertAlmostEqual(params.drag_coefficient, 1.1)
        self.assertAlmostEqual(params.area_scale, 1.0)
        self.assertAlmostEqual(params.wake_speed_scale, 1.0)
        self.assertAlmostEqual(params.wake_spread_angle_rad, 0.20)
        self.assertAlmostEqual(params.axial_decay_m, 0.45)
        self.assertFalse(hasattr(params, "force_coeff"))
        self.assertFalse(hasattr(params, "model"))

    def test_downwash_params_reject_nonphysical_coefficients(self) -> None:
        invalid_configs: list[dict[str, object]] = [
            {"drag_coefficient": -0.1},
            {"area_scale": -0.1},
            {"wake_speed_scale": -0.1},
            {"wake_spread_angle_rad": -0.1},
            {"axial_decay_m": 0.0},
        ]

        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    DownwashParams.from_config(config)

    def test_rotor_inertial_torque_params_default_to_disabled(self) -> None:
        params = RotorInertialTorqueParams.from_config(None)

        self.assertFalse(params.enabled)
        self.assertEqual(params.inertia_kg_m2, 0.0)
        self.assertTrue(params.apply_acceleration_torque)
        self.assertTrue(params.apply_gyro_torque)
        self.assertFalse(params.randomize_enabled)
        self.assertEqual(params.enabled_probability, 1.0)

    def test_rotor_inertial_torque_params_parse_configured_values(self) -> None:
        params = RotorInertialTorqueParams.from_config(
            {
                "enabled": True,
                "inertia_kg_m2": 1.5e-5,
                "apply_acceleration_torque": False,
                "apply_gyro_torque": True,
                "randomize_enabled": True,
                "enabled_probability": 0.25,
            }
        )

        self.assertTrue(params.enabled)
        self.assertAlmostEqual(params.inertia_kg_m2, 1.5e-5)
        self.assertFalse(params.apply_acceleration_torque)
        self.assertTrue(params.apply_gyro_torque)
        self.assertTrue(params.randomize_enabled)
        self.assertAlmostEqual(params.enabled_probability, 0.25)

    def test_rotor_inertial_torque_params_reject_invalid_values(self) -> None:
        invalid_configs: list[dict[str, object]] = [
            {"inertia_kg_m2": -1.0},
            {"inertia_kg_m2": float("inf")},
            {"enabled_probability": -0.1},
            {"enabled_probability": 1.1},
        ]

        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaises(ValueError):
                    RotorInertialTorqueParams.from_config(config)

    def test_first_order_response_step_uses_up_time_constant_for_spin_up(self) -> None:
        updated = first_order_response_step(
            current=np.array([0.0, 10.0], dtype=float),
            target=np.array([10.0, 30.0], dtype=float),
            dt_s=0.1,
            time_constant_up=0.2,
            time_constant_down=0.5,
        )
        expected = np.array([0.0, 10.0], dtype=float) + (
            np.array([10.0, 30.0], dtype=float) - np.array([0.0, 10.0], dtype=float)
        ) * (1.0 - np.exp(-0.1 / 0.2))
        np.testing.assert_allclose(updated, expected)

    def test_first_order_response_step_uses_down_time_constant_for_spin_down(self) -> None:
        updated = first_order_response_step(
            current=10.0,
            target=2.0,
            dt_s=0.1,
            time_constant_up=0.2,
            time_constant_down=0.5,
        )
        expected = 10.0 + (2.0 - 10.0) * (1.0 - np.exp(-0.1 / 0.5))
        self.assertAlmostEqual(updated, expected)

    def test_idle_visual_speed_target_respects_armed_and_low_speed_blend(self) -> None:
        self.assertEqual(
            idle_visual_speed_target(
                physical_speed=120.0,
                actuator_output=0.5,
                armed=False,
                idle_speed=300.0,
                low_speed_blend_end=400.0,
            ),
            120.0,
        )

        self.assertEqual(
            idle_visual_speed_target(
                physical_speed=120.0,
                actuator_output=0.0,
                armed=True,
                idle_speed=300.0,
                low_speed_blend_end=400.0,
            ),
            300.0,
        )

        blended = idle_visual_speed_target(
            physical_speed=120.0,
            actuator_output=0.4,
            armed=True,
            idle_speed=300.0,
            low_speed_blend_end=400.0,
        )
        expected_blended = max(120.0, (1.0 - 120.0 / 400.0) * 300.0 + (120.0 / 400.0) * 120.0)
        self.assertAlmostEqual(blended, expected_blended)

        self.assertEqual(
            idle_visual_speed_target(
                physical_speed=500.0,
                actuator_output=0.6,
                armed=True,
                idle_speed=300.0,
                low_speed_blend_end=400.0,
            ),
            500.0,
        )

    def test_rotor_thrust_moment_formula_uses_axis_and_spin_direction(self) -> None:
        axis = np.array([0.0, 0.0, 1.0], dtype=float)

        thrust, moment = rotor_thrust_moment_along_axis(
            omega_radps=4.0,
            axis=axis,
            spin_direction=-1.0,
            motor_constant=0.5,
            moment_constant=0.2,
        )

        np.testing.assert_allclose(thrust, np.array([0.0, 0.0, 8.0], dtype=float))
        np.testing.assert_allclose(moment, np.array([0.0, 0.0, 1.6], dtype=float))

    def test_rotor_thrust_moment_zero_speed_has_zero_force_and_moment(self) -> None:
        thrust, moment = rotor_thrust_moment_along_axis(
            omega_radps=0.0,
            axis=np.array([0.0, 0.0, 1.0], dtype=float),
            spin_direction=1.0,
            motor_constant=0.5,
            moment_constant=0.2,
        )

        np.testing.assert_allclose(thrust, np.zeros(3, dtype=float))
        np.testing.assert_allclose(moment, np.zeros(3, dtype=float))

    def test_rotor_thrust_moment_nonzero_speed_has_positive_thrust(self) -> None:
        thrust, moment = rotor_thrust_moment_along_axis(
            omega_radps=1.0,
            axis=np.array([0.0, 0.0, 1.0], dtype=float),
            spin_direction=1.0,
            motor_constant=0.5,
            moment_constant=0.2,
        )

        np.testing.assert_allclose(thrust, np.array([0.0, 0.0, 0.5], dtype=float))
        np.testing.assert_allclose(moment, np.array([0.0, 0.0, -0.1], dtype=float))

    def test_throttle_to_omega_defaults_to_linear_mapping(self) -> None:
        params = ThrottleToOmegaParams.from_config(None)

        np.testing.assert_allclose(params.coefficients, np.array([0.0, 1.0, 0.0], dtype=float))
        np.testing.assert_allclose(
            params.evaluate(np.array([0.0, 0.25, 1.0], dtype=float)),
            np.array([0.0, 0.25, 1.0], dtype=float),
        )

    def test_throttle_to_omega_parses_quadratic_coefficients(self) -> None:
        params = ThrottleToOmegaParams.from_config({"coefficients": [0.0, 1.75553745, -0.75498727]})

        self.assertAlmostEqual(params.evaluate(0.5), 1.75553745 * 0.5 - 0.75498727 * 0.25)
        self.assertEqual(params.evaluate(1.0), 1.0)

    def test_throttle_to_omega_rejects_model_field(self) -> None:
        with self.assertRaisesRegex(ValueError, "throttle_to_omega does not support a model field"):
            ThrottleToOmegaParams.from_config({"model": "omega_fraction_quadratic", "coefficients": [0.0, 1.0, 0.0]})

    def test_throttle_to_omega_rejects_invalid_coefficients(self) -> None:
        with self.assertRaisesRegex(ValueError, "coefficients must contain exactly three finite numbers"):
            ThrottleToOmegaParams.from_config({"coefficients": [0.0, 1.0]})

    def test_thruster_wrenches_from_speed_vectorizes_body_frame_formula(self) -> None:
        axes_b = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=float)
        omega = np.array([3.0, -2.0], dtype=float)
        motor_constant = np.array([0.5, 2.0], dtype=float)
        moment_constant = np.array([0.1, 0.25], dtype=float)
        rotor_direction = np.array([1.0, -1.0], dtype=float)

        force_b, moment_b = thruster_wrenches_from_speed(
            omega_radps=omega,
            axes_b=axes_b,
            motor_constant=motor_constant,
            moment_constant=moment_constant,
            rotor_direction=rotor_direction,
        )

        expected_scalar = omega * np.abs(omega) * motor_constant
        np.testing.assert_allclose(force_b, axes_b * expected_scalar[:, None])
        np.testing.assert_allclose(
            moment_b,
            -rotor_direction[:, None] * expected_scalar[:, None] * moment_constant[:, None] * axes_b,
        )


if __name__ == "__main__":
    unittest.main()
