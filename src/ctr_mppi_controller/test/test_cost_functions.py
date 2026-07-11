import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_mppi_controller.cost_functions import (  # noqa: E402
    control_magnitude_cost,
    control_smoothness_cost,
    obstacle_cost,
    shape_tracking_cost,
    stability_cost,
    tactile_cost,
    terminal_tip_cost,
    tip_tracking_cost,
)


class CostFunctionsTest(unittest.TestCase):
    def test_enabled_cost_terms(self):
        self.assertAlmostEqual(0.25, tip_tracking_cost(np.array([0.5, 0.0, 0.0]), np.zeros(3)))
        self.assertAlmostEqual(0.25, terminal_tip_cost(np.array([0.5, 0.0, 0.0]), np.zeros(3)))
        self.assertAlmostEqual(0.05, control_magnitude_cost(np.array([0.1, 0.2, 0, 0, 0, 0])))
        self.assertAlmostEqual(
            0.05,
            control_smoothness_cost(np.array([0.1, 0.2, 0, 0, 0, 0]), np.zeros(6)),
        )

    def test_disabled_interfaces_return_zero_when_disabled(self):
        self.assertEqual(0.0, shape_tracking_cost(enabled=False))
        self.assertEqual(0.0, obstacle_cost(enabled=False))
        self.assertEqual(0.0, tactile_cost(enabled=False))
        self.assertEqual(0.0, stability_cost(enabled=False))

    def test_disabled_interfaces_raise_when_enabled(self):
        for cost_fn in (shape_tracking_cost, obstacle_cost, tactile_cost, stability_cost):
            with self.assertRaises(NotImplementedError):
                cost_fn(enabled=True)


if __name__ == "__main__":
    unittest.main()
