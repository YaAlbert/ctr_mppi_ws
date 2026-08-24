import math
import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_tactile.simulated_tactile import (  # noqa: E402
    REGION_CONTACT,
    REGION_NO_CONTACT,
    REGION_STOP,
    REGION_WARNING,
)
from ctr_tactile.tactile_processing import (  # noqa: E402
    TactileProcessingParameters,
    TactileProcessor,
)


def parameters(**overrides):
    values = dict(
        zero_offset=1.0,
        scale=2.0,
        force_saturation_n=1.0,
        alpha=0.25,
        contact_on_n=0.10,
        contact_off_n=0.08,
        warning_on_n=0.30,
        warning_off_n=0.28,
        stop_on_n=0.50,
        stop_off_n=0.48,
    )
    values.update(overrides)
    return TactileProcessingParameters(**values)


def sample(processor, force, clearance=0.01):
    return processor.process([1.0 + force / 2.0], clearance_m=clearance, geometric_contact=False)


class TactileProcessingTest(unittest.TestCase):
    def test_calibration_force_inversion_and_filter_units(self):
        result = TactileProcessor(parameters(alpha=1.0)).process(
            [1.5], clearance_m=-0.001, geometric_contact=True
        )
        self.assertEqual(1.5, result.raw_signal)
        self.assertEqual(1.5, result.filtered_signal)
        self.assertEqual(1.0, result.force_n)

    def test_force_is_clamped_and_saturates(self):
        result = TactileProcessor(parameters(alpha=1.0)).process(
            [100.0], clearance_m=-0.1, geometric_contact=True
        )
        self.assertEqual(1.0, result.force_n)
        self.assertTrue(math.isfinite(result.force_n))

    def test_ema_initialization_recurrence_and_reset(self):
        processor = TactileProcessor(parameters(alpha=0.25))
        self.assertAlmostEqual(0.0, sample(processor, 0.0).force_n)
        self.assertAlmostEqual(0.5, sample(processor, 2.0).force_n)
        processor.reset()
        self.assertAlmostEqual(1.0, sample(processor, 2.0).force_n)

    def test_alpha_one_is_passthrough(self):
        result = sample(TactileProcessor(parameters(alpha=1.0)), 0.35)
        self.assertAlmostEqual(0.35, result.force_n)
        self.assertAlmostEqual(1.175, result.filtered_signal)

    def test_invalid_configuration_is_rejected(self):
        for field, value in (
            ("alpha", 0.0), ("alpha", math.nan), ("scale", 0.0),
            ("force_saturation_n", math.inf), ("contact_off_n", 0.10),
            ("warning_off_n", 0.09), ("stop_off_n", 0.29),
            ("stop_on_n", 2.0),
        ):
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    parameters(**{field: value})

    def test_invalid_input_preserves_state(self):
        processor = TactileProcessor(parameters(alpha=1.0))
        sample(processor, 0.35)
        before = processor.region
        invalid = processor.process([math.nan], clearance_m=0.01, geometric_contact=False)
        self.assertFalse(invalid.valid)
        self.assertEqual(before, invalid.region)
        self.assertEqual(before, processor.region)
        self.assertAlmostEqual(0.35, sample(processor, 0.35).force_n)
        for values in (None, [], [1.0, 2.0], [math.inf]):
            self.assertFalse(
                processor.process(values, clearance_m=0.01, geometric_contact=False).valid
            )

    def test_regions_and_rising_boundaries(self):
        processor = TactileProcessor(parameters(alpha=1.0, zero_offset=0.0, scale=1.0))
        def exact(force):
            return processor.process([force], clearance_m=0.01, geometric_contact=False)
        self.assertEqual(REGION_NO_CONTACT, exact(0.0).region)
        self.assertEqual(REGION_CONTACT, exact(0.10).region)
        self.assertEqual(REGION_WARNING, exact(0.30).region)
        self.assertEqual(REGION_STOP, exact(0.50).region)
        self.assertTrue(exact(0.50).stop)
        self.assertTrue(exact(0.30).warning)

    def test_hysteresis_release_boundaries_and_direct_fall(self):
        processor = TactileProcessor(parameters(alpha=1.0, zero_offset=0.0, scale=1.0))
        def exact(force):
            return processor.process([force], clearance_m=0.01, geometric_contact=False)
        exact(0.60)
        self.assertEqual(REGION_STOP, exact(0.48).region)
        self.assertEqual(REGION_WARNING, exact(0.47).region)
        self.assertEqual(REGION_WARNING, exact(0.28).region)
        self.assertEqual(REGION_CONTACT, exact(0.27).region)
        self.assertEqual(REGION_CONTACT, exact(0.08).region)
        self.assertEqual(REGION_NO_CONTACT, exact(0.079).region)

    def test_invalid_sample_does_not_release_region(self):
        processor = TactileProcessor(parameters(alpha=1.0))
        sample(processor, 0.60)
        invalid = processor.process([math.nan], clearance_m=0.01, geometric_contact=False)
        self.assertEqual(REGION_STOP, invalid.region)
        self.assertTrue(invalid.stop)

    def test_exact_zero_clearance_is_geometric_contact_without_force(self):
        result = TactileProcessor(parameters(alpha=1.0)).process(
            [1.0], clearance_m=0.0, geometric_contact=True
        )
        self.assertTrue(result.contact)
        self.assertEqual(0.0, result.force_n)
        self.assertEqual(REGION_NO_CONTACT, result.region)
        self.assertFalse(result.warning)
        self.assertFalse(result.stop)

    def test_deterministic_replay(self):
        sequence = [(0.0, 0.01, False), (0.12, 0.0, True), (0.35, -0.001, True), (0.0, 0.01, False)]
        outputs = []
        for _ in range(2):
            processor = TactileProcessor(parameters(alpha=1.0))
            outputs.append([
                processor.process([1.0 + force / 2.0], clearance_m=clearance, geometric_contact=contact)
                for force, clearance, contact in sequence
            ])
        self.assertEqual(outputs[0], outputs[1])

    def test_stop_is_observation_and_direction_is_zero(self):
        result = sample(TactileProcessor(parameters(alpha=1.0)), 0.60)
        self.assertTrue(result.stop)
        self.assertIn("directional_force_unavailable", result.diagnostic_status)


if __name__ == "__main__":
    unittest.main()
