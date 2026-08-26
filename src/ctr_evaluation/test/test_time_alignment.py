import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_evaluation.time_alignment import (  # noqa: E402
    AlignmentConfig,
    TimedState,
    align_samples,
    aligned_arrays,
    command_sample,
    reference_sample,
    solve_sample,
    state_sample,
)


def state(t):
    return state_sample(t, np.zeros(6), np.zeros(6), [t, 0.0, 0.0])


class TimeAlignmentTest(unittest.TestCase):
    def test_reference_interpolation(self):
        result = align_samples(
            states=[state(0.5)],
            references=[
                reference_sample(0.0, [0.0, 0.0, 0.0], progress=0.0),
                reference_sample(1.0, [1.0, 0.0, 0.0], progress=1.0),
            ],
            commands=[],
            solves=[],
            config=AlignmentConfig(1.0, 1.0, 1.0),
        )
        self.assertEqual(1, len(result.samples))
        self.assertTrue(np.allclose([0.5, 0.0, 0.0], result.samples[0].reference_position))
        self.assertTrue(result.samples[0].used_reference_interpolation)
        self.assertAlmostEqual(0.5, result.samples[0].reference_progress)
        self.assertEqual(1, result.diagnostics.reference_interpolation_count)

    def test_nearest_reference_fallback(self):
        result = align_samples(
            states=[state(0.05)],
            references=[reference_sample(0.0, [1.0, 0.0, 0.0])],
            commands=[],
            solves=[],
            config=AlignmentConfig(0.1, 1.0, 1.0),
        )
        self.assertEqual(1, len(result.samples))
        self.assertTrue(result.samples[0].used_nearest_reference)
        self.assertEqual(1, result.diagnostics.nearest_reference_fallback_count)

    def test_alignment_gap_rejection(self):
        result = align_samples(
            states=[state(10.0)],
            references=[reference_sample(0.0, [0.0, 0.0, 0.0])],
            commands=[],
            solves=[],
            config=AlignmentConfig(0.1, 1.0, 1.0),
        )
        self.assertEqual(0, len(result.samples))
        self.assertEqual(1, result.diagnostics.rejected_aligned_sample_count)
        self.assertEqual(1, result.diagnostics.rejection_reasons["reference_gap"])

    def test_command_association_not_later_than_state(self):
        result = align_samples(
            states=[state(1.0)],
            references=[reference_sample(1.0, [0.0, 0.0, 0.0])],
            commands=[
                command_sample(0.5, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                command_sample(1.5, [9.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ],
            solves=[solve_sample(0.75, 0.02)],
            config=AlignmentConfig(0.1, 1.0, 1.0),
        )
        self.assertEqual(1, len(result.samples))
        self.assertAlmostEqual(1.0, result.samples[0].command[0])
        self.assertAlmostEqual(0.5, result.samples[0].command_gap)
        self.assertAlmostEqual(0.25, result.samples[0].solve_gap)

    def test_stale_command_becomes_missing_when_optional(self):
        result = align_samples(
            states=[state(10.0)],
            references=[reference_sample(10.0, [0.0, 0.0, 0.0])],
            commands=[command_sample(0.0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            solves=[],
            config=AlignmentConfig(0.1, 1.0, 1.0, require_command=False),
        )
        self.assertEqual(1, len(result.samples))
        self.assertTrue(result.samples[0].missing_command)
        self.assertTrue(np.allclose(np.zeros(6), result.samples[0].command))
        self.assertEqual(1, result.diagnostics.missing_command_count)

    def test_stale_command_rejected_when_required(self):
        result = align_samples(
            states=[state(10.0)],
            references=[reference_sample(10.0, [0.0, 0.0, 0.0])],
            commands=[command_sample(0.0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])],
            solves=[],
            config=AlignmentConfig(0.1, 1.0, 1.0, require_command=True),
        )
        self.assertEqual(0, len(result.samples))
        self.assertEqual(1, result.diagnostics.rejection_reasons["command_gap"])

    def test_nonfinite_sample_is_counted(self):
        bad_state = TimedState(
            timestamp=0.0,
            q=np.zeros(6),
            q_dot=np.zeros(6),
            tip_position=np.array([math.nan, 0.0, 0.0]),
        )
        result = align_samples(
            states=[bad_state],
            references=[reference_sample(0.0, [0.0, 0.0, 0.0])],
            commands=[],
            solves=[],
            config=AlignmentConfig(0.1, 1.0, 1.0),
        )
        self.assertEqual(0, len(result.samples))
        self.assertEqual(1, result.diagnostics.invalid_nonfinite_sample_count)

    def test_aligned_arrays_have_expected_shapes(self):
        result = align_samples(
            states=[state(0.0), state(1.0)],
            references=[reference_sample(0.0, [0.0, 0.0, 0.0]), reference_sample(1.0, [1.0, 0.0, 0.0])],
            commands=[command_sample(0.0, np.zeros(6)), command_sample(1.0, np.ones(6))],
            solves=[solve_sample(0.0, 0.01), solve_sample(1.0, 0.02)],
            config=AlignmentConfig(0.1, 1.0, 1.0),
        )
        arrays = aligned_arrays(result.samples)
        self.assertEqual((2,), arrays["timestamps"].shape)
        self.assertEqual((2, 3), arrays["tip_positions"].shape)
        self.assertEqual((2, 6), arrays["commands"].shape)
        self.assertEqual((2,), arrays["solve_times"].shape)


if __name__ == "__main__":
    unittest.main()
