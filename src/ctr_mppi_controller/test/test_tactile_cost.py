import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ctr_mppi_controller.tactile_cost import (
    REGION_CONTACT,
    REGION_NO_CONTACT,
    TactileCostConfig,
    snapshot_eligibility,
    snapshot_from_values,
)


class TactileSnapshotTest(unittest.TestCase):
    def setUp(self):
        self.snapshot = snapshot_from_values(
            timestamp_s=10.0,
            frame_id="base_link",
            source="simulated",
            valid=True,
            contact=True,
            warning=False,
            stop=False,
            region=REGION_CONTACT,
            force_magnitude_n=0.2,
        )

    def test_fresh_snapshot_is_eligible_at_age_boundary(self):
        self.assertEqual((True, "eligible"), snapshot_eligibility(
            self.snapshot, now_s=10.1, max_age_s=0.1, expected_frame="base_link"
        ))

    def test_stale_invalid_future_and_frame_mismatch_are_neutral_reasons(self):
        for now, frame, valid, expected in (
            (10.100001, "base_link", True, "stale_snapshot"),
            (10.0, "other", True, "frame_mismatch"),
            (10.0, "base_link", False, "snapshot_invalid"),
            (9.9, "base_link", True, "future_timestamp"),
        ):
            snapshot = self.snapshot if valid else self.snapshot.__class__(**{**self.snapshot.__dict__, "valid": False})
            self.assertEqual(expected, snapshot_eligibility(
                snapshot, now_s=now, max_age_s=0.1, expected_frame=frame
            )[1])

    def test_zero_timestamp_and_nonfinite_force_rejected(self):
        zero = self.snapshot.__class__(**{**self.snapshot.__dict__, "timestamp_s": 0.0})
        bad_force = self.snapshot.__class__(**{**self.snapshot.__dict__, "force_magnitude_n": math.nan})
        self.assertEqual("invalid_timestamp", snapshot_eligibility(zero, now_s=1.0, max_age_s=0.1, expected_frame="base_link")[1])
        self.assertEqual("invalid_force", snapshot_eligibility(bad_force, now_s=10.0, max_age_s=0.1, expected_frame="base_link")[1])

    def test_no_contact_region_is_valid_but_neutral_by_cost_owner(self):
        snapshot = self.snapshot.__class__(**{**self.snapshot.__dict__, "region": REGION_NO_CONTACT, "force_magnitude_n": 0.0})
        self.assertEqual(REGION_NO_CONTACT, snapshot.region)
        self.assertTrue(snapshot.valid)

    def test_config_values_are_explicit_and_finite(self):
        config = {"mppi": {"weights": {"force": 0.0}, "tactile": {
            "enabled": False, "max_age_s": 0.1, "force_saturation_n": 10.0,
            "proximity_margin_m": 0.002, "no_contact_multiplier": 0.0,
            "contact_multiplier": 1.0, "warning_multiplier": 2.0, "stop_multiplier": 4.0,
        }}}
        parsed = TactileCostConfig.from_project_config(config)
        self.assertFalse(parsed.enabled)
        self.assertTrue(math.isfinite(parsed.proximity_margin_m))

    def test_enabled_configuration_requires_positive_force_weight(self):
        config = {"mppi": {"weights": {"force": 0.0}, "tactile": {
            "enabled": True, "max_age_s": 0.1, "force_saturation_n": 10.0,
            "proximity_margin_m": 0.002, "no_contact_multiplier": 0.0,
            "contact_multiplier": 1.0, "warning_multiplier": 2.0, "stop_multiplier": 4.0,
        }}}
        with self.assertRaises(ValueError):
            TactileCostConfig.from_project_config(config)


if __name__ == "__main__":
    unittest.main()
