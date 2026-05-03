from __future__ import annotations

import unittest

from acesim.utils.simulation_clock import SimulationClock


class SimulationClockTests(unittest.TestCase):
    def test_clock_tracks_time_without_transport_side_effects(self) -> None:
        clock = SimulationClock(start_time_us=100)

        self.assertEqual(clock.current_time_us, 100)
        self.assertEqual(clock.advance_us(50), 150)
        self.assertEqual(clock.advance_seconds(0.001), 1150)
        clock.reset(25)
        self.assertEqual(clock.current_time_us, 25)
        self.assertFalse(hasattr(clock, "_socket"))

    def test_clock_rejects_negative_time(self) -> None:
        with self.assertRaisesRegex(ValueError, "start_time_us must be non-negative"):
            SimulationClock(start_time_us=-1)


if __name__ == "__main__":
    unittest.main()
