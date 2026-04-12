from __future__ import annotations

import unittest


class URDF2MJCFRuntimeHandlerTests(unittest.TestCase):
    def test_all_vehicle_families_have_registered_runtime_handlers(self) -> None:
        from acesim.tools.urdf2mjcf.runtime_handler_registry import RUNTIME_MODEL_HANDLERS

        self.assertIn("multirotor", RUNTIME_MODEL_HANDLERS)
        self.assertIn("fixedwing", RUNTIME_MODEL_HANDLERS)
        self.assertIn("vtol", RUNTIME_MODEL_HANDLERS)
        self.assertIn("uuv", RUNTIME_MODEL_HANDLERS)


if __name__ == "__main__":
    unittest.main()
