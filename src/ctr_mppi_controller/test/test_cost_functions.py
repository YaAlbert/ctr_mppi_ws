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
from ctr_mppi_controller.tactile_cost import (  # noqa: E402
    REGION_CONTACT,
    REGION_NO_CONTACT,
    REGION_STOP,
    TactileCostConfig,
    snapshot_from_values,
    tactile_cost_value,
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
        for cost_fn in (shape_tracking_cost, obstacle_cost, stability_cost):
            with self.assertRaises(NotImplementedError):
                cost_fn(enabled=True)

    def test_tactile_cost_is_candidate_dependent_and_no_contact_is_neutral(self):
        config = TactileCostConfig(
            enabled=True,
            max_age_s=0.1,
            tactile_weight=10.0,
            force_saturation_n=10.0,
            proximity_margin_m=0.002,
            no_contact_multiplier=0.0,
            contact_multiplier=1.0,
            warning_multiplier=2.0,
            stop_multiplier=4.0,
        )
        contact = snapshot_from_values(
            timestamp_s=1.0, frame_id="base_link", source="simulated", valid=True,
            contact=True, warning=False, stop=False, region=REGION_CONTACT, force_magnitude_n=1.0,
        )
        no_contact = snapshot_from_values(
            timestamp_s=1.0, frame_id="base_link", source="simulated", valid=True,
            contact=False, warning=False, stop=False, region=REGION_NO_CONTACT, force_magnitude_n=0.0,
        )
        near = tactile_cost_value(enabled=True, snapshot=contact, predicted_clearance_m=0.0005, config=config)
        far = tactile_cost_value(enabled=True, snapshot=contact, predicted_clearance_m=0.003, config=config)
        neutral = tactile_cost_value(enabled=True, snapshot=no_contact, predicted_clearance_m=0.0005, config=config)
        self.assertGreater(near, far)
        self.assertEqual(0.0, neutral)
        stop = contact.__class__(**{**contact.__dict__, "region": REGION_STOP, "stop": True, "force_magnitude_n": 5.0})
        self.assertGreater(
            tactile_cost_value(enabled=True, snapshot=stop, predicted_clearance_m=0.0005, config=config),
            near,
        )

    def test_tactile_cost_is_finite_at_clearance_boundaries(self):
        config = TactileCostConfig(True, 0.1, 10.0, 10.0, 0.002, 0.0, 1.0, 2.0, 4.0)
        snapshot = snapshot_from_values(
            timestamp_s=1.0, frame_id="base_link", source="simulated", valid=True,
            contact=True, warning=False, stop=False, region=REGION_CONTACT, force_magnitude_n=10.0,
        )
        self.assertEqual(0.0, tactile_cost_value(enabled=True, snapshot=snapshot, predicted_clearance_m=0.002, config=config))
        self.assertTrue(np.isfinite(tactile_cost_value(enabled=True, snapshot=snapshot, predicted_clearance_m=-1.0, config=config)))


if __name__ == "__main__":
    unittest.main()
