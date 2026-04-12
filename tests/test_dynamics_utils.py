from __future__ import annotations

import unittest

import numpy as np

from acesim.utils.dynamics import first_order_response_step, idle_visual_speed_target


class DynamicsUtilsTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
