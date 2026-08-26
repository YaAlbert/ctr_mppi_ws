import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_mppi_controller.curved_lumen import CurvedLumen, circular_arc_centerline  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.reference_validation import (  # noqa: E402
    INVALID_REFERENCE,
    SOURCE_TRAJECTORY_HORIZON,
    VALID_REFERENCE,
    accept_trajectory_reference,
    initial_trajectory_reference,
    reject_trajectory_update,
    trajectory_kwargs_from_active,
    trajectory_state_log_line,
    validate_reference_sequence,
)


def cylinder_lumen():
    return CylindricalLumen(
        frame_id="base_link",
        axis_origin=[0.0, 0.0, 0.0],
        axis_direction=[0.0, 0.0, 1.0],
        radius=0.030,
        length=0.120,
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


def curved_lumen():
    centerline = circular_arc_centerline(
        inlet_position=[0.0, 0.0, 0.0],
        initial_tangent=[0.0, 0.0, 1.0],
        bend_normal=[1.0, 0.0, 0.0],
        curvature_radius=0.180,
        arc_angle=0.70,
        sample_spacing=0.010,
    )
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=centerline,
        lumen_radius=0.030,
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


def valid_points(count=3):
    return np.array(
        [
            [0.0, 0.0, 0.020 + 0.010 * index]
            for index in range(count)
        ],
        dtype=float,
    )


class ReferenceSequenceValidationTest(unittest.TestCase):
    def test_valid_no_lumen_sequence_is_copied_float64_and_write_protected(self):
        source = valid_points(3)
        accepted = validate_reference_sequence(
            source,
            received_frame="base_link",
            expected_frame="base_link",
            expected_count=3,
        )
        self.assertEqual((3, 3), accepted.shape)
        self.assertEqual(np.float64, accepted.dtype)
        self.assertTrue(np.array_equal(source, accepted))
        self.assertFalse(accepted.flags.writeable)
        source[0, 0] = 9.0
        self.assertNotEqual(source[0, 0], accepted[0, 0])
        with self.assertRaises(ValueError):
            accepted[0, 0] = 1.0

    def test_valid_one_point_sequence_is_allowed_when_expected(self):
        accepted = validate_reference_sequence(
            [[0.0, 0.0, 0.0]],
            received_frame="base_link",
            expected_frame="base_link",
            expected_count=1,
        )
        self.assertEqual((1, 3), accepted.shape)

    def test_empty_sequence_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "at least one point"):
            validate_reference_sequence([], received_frame="base_link", expected_frame="base_link", expected_count=3)

    def test_wrong_rank_is_rejected(self):
        for points in ([0.0, 0.0, 0.0], np.zeros((1, 3, 1))):
            with self.subTest(shape=np.asarray(points).shape):
                with self.assertRaisesRegex(ValueError, "shape"):
                    validate_reference_sequence(
                        points,
                        received_frame="base_link",
                        expected_frame="base_link",
                        expected_count=3,
                    )

    def test_wrong_coordinate_dimension_is_rejected(self):
        for points in (np.zeros((3, 2)), np.zeros((3, 4))):
            with self.subTest(shape=points.shape):
                with self.assertRaisesRegex(ValueError, "shape"):
                    validate_reference_sequence(
                        points,
                        received_frame="base_link",
                        expected_frame="base_link",
                        expected_count=3,
                    )

    def test_wrong_point_count_is_rejected(self):
        for points in (valid_points(2), valid_points(4)):
            with self.subTest(count=points.shape[0]):
                with self.assertRaisesRegex(ValueError, "shape"):
                    validate_reference_sequence(
                        points,
                        received_frame="base_link",
                        expected_frame="base_link",
                        expected_count=3,
                    )

    def test_nonfinite_values_report_point_index(self):
        cases = (float("nan"), float("inf"), -float("inf"))
        for value in cases:
            points = valid_points(3)
            points[1, 2] = value
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, r"point\[1\].*non-finite"):
                    validate_reference_sequence(
                        points,
                        received_frame="base_link",
                        expected_frame="base_link",
                        expected_count=3,
                    )

    def test_empty_and_wrong_frames_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "frame_id"):
            validate_reference_sequence(valid_points(3), received_frame="", expected_frame="base_link", expected_count=3)
        with self.assertRaisesRegex(ValueError, "frame mismatch"):
            validate_reference_sequence(
                valid_points(3),
                received_frame="map",
                expected_frame="base_link",
                expected_count=3,
            )

    def test_lumen_frame_mismatch_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "lumen frame"):
            validate_reference_sequence(
                valid_points(3),
                received_frame="base_link",
                expected_frame="base_link",
                lumen_geometry=CylindricalLumen(
                    frame_id="world",
                    axis_origin=[0.0, 0.0, 0.0],
                    axis_direction=[0.0, 0.0, 1.0],
                    radius=0.030,
                    length=0.120,
                    ctr_outer_radius=0.0015,
                    safety_margin=0.0020,
                ),
                expected_count=3,
            )

    def test_valid_cylinder_sequence_is_accepted(self):
        accepted = validate_reference_sequence(
            valid_points(3),
            received_frame="base_link",
            expected_frame="base_link",
            lumen_geometry=cylinder_lumen(),
            expected_count=3,
        )
        self.assertTrue(np.array_equal(valid_points(3), accepted))

    def test_cylinder_wall_inlet_and_outlet_invalid_points_are_rejected(self):
        cases = (
            ([0.040, 0.0, 0.050], "point\\[1\\]"),
            ([0.0, 0.0, -0.001], "point\\[1\\]"),
            ([0.0, 0.0, 0.121], "point\\[1\\]"),
        )
        for point, pattern in cases:
            points = valid_points(3)
            points[1] = point
            with self.subTest(point=point):
                with self.assertRaisesRegex(ValueError, pattern):
                    validate_reference_sequence(
                        points,
                        received_frame="base_link",
                        expected_frame="base_link",
                        lumen_geometry=cylinder_lumen(),
                        expected_count=3,
                    )

    def test_valid_curved_sequence_is_accepted(self):
        lumen = curved_lumen()
        points = lumen.centerline_points[[1, 3, 5]]
        accepted = validate_reference_sequence(
            points,
            received_frame="base_link",
            expected_frame="base_link",
            lumen_geometry=lumen,
            expected_count=3,
        )
        self.assertTrue(np.array_equal(points, accepted))

    def test_curved_invalid_middle_point_reports_index(self):
        lumen = curved_lumen()
        points = lumen.centerline_points[[1, 3, 5]].copy()
        points[1] = [0.080, 0.0, 0.030]
        with self.assertRaisesRegex(ValueError, r"point\[1\]"):
            validate_reference_sequence(
                points,
                received_frame="base_link",
                expected_frame="base_link",
                lumen_geometry=lumen,
                expected_count=3,
            )

    def test_repeated_zero_and_signed_values_are_preserved_when_valid(self):
        points = np.array([[0.0, -0.0, 0.020], [0.0, -0.0, 0.020], [-0.001, 0.001, 0.030]])
        accepted = validate_reference_sequence(
            points,
            received_frame="base_link",
            expected_frame="base_link",
            lumen_geometry=cylinder_lumen(),
            expected_count=3,
        )
        self.assertTrue(np.array_equal(points, accepted))


class ActiveTrajectoryReferenceTest(unittest.TestCase):
    def test_first_acceptance_sets_revision_one_and_preserves_snapshot(self):
        previous = initial_trajectory_reference()
        points = valid_points(3)
        active = accept_trajectory_reference(previous, points=points, frame="base_link", stamp_s=10.0)
        self.assertEqual(VALID_REFERENCE, active.state)
        self.assertEqual(SOURCE_TRAJECTORY_HORIZON, active.source)
        self.assertEqual(1, active.revision)
        self.assertEqual("base_link", active.frame_id)
        self.assertTrue(np.array_equal(points, active.points))
        points[0, 0] = 5.0
        self.assertNotEqual(points[0, 0], active.points[0, 0])

    def test_duplicate_updates_refresh_stamp_without_revision_change(self):
        first = accept_trajectory_reference(
            initial_trajectory_reference(),
            points=valid_points(3),
            frame="base_link",
            stamp_s=10.0,
        )
        duplicate = accept_trajectory_reference(first, points=valid_points(3), frame="base_link", stamp_s=10.1)
        self.assertEqual(first.revision, duplicate.revision)
        self.assertAlmostEqual(10.1, duplicate.stamp_s)

    def test_changed_reordered_and_count_content_affect_revision_or_validity(self):
        first = accept_trajectory_reference(
            initial_trajectory_reference(),
            points=valid_points(3),
            frame="base_link",
            stamp_s=10.0,
        )
        changed = valid_points(3)
        changed[[0, 1]] = changed[[1, 0]]
        second = accept_trajectory_reference(first, points=changed, frame="base_link", stamp_s=10.1)
        self.assertEqual(2, second.revision)
        with self.assertRaisesRegex(ValueError, "shape"):
            validate_reference_sequence(
                valid_points(4),
                received_frame="base_link",
                expected_frame="base_link",
                expected_count=3,
            )

    def test_reject_first_update_records_error_without_points(self):
        rejected = reject_trajectory_update(initial_trajectory_reference(), "bad horizon")
        self.assertEqual(INVALID_REFERENCE, rejected.state)
        self.assertEqual(0, rejected.revision)
        self.assertIsNone(rejected.points)
        self.assertIn("bad horizon", rejected.last_validation_error)

    def test_reject_replacement_retains_previous_valid_snapshot(self):
        first = accept_trajectory_reference(
            initial_trajectory_reference(),
            points=valid_points(3),
            frame="base_link",
            stamp_s=10.0,
        )
        retained = reject_trajectory_update(first, "bad replacement")
        self.assertEqual(VALID_REFERENCE, retained.state)
        self.assertEqual(first.revision, retained.revision)
        self.assertEqual(first.frame_id, retained.frame_id)
        self.assertTrue(np.array_equal(first.points, retained.points))
        self.assertIn("bad replacement", retained.last_validation_error)

    def test_trajectory_kwargs_enforce_freshness_and_return_copy(self):
        active = accept_trajectory_reference(
            initial_trajectory_reference(),
            points=valid_points(3),
            frame="base_link",
            stamp_s=10.0,
        )
        kwargs = trajectory_kwargs_from_active(active, current_time_s=10.1, stale_timeout=0.2)
        self.assertTrue(np.array_equal(active.points, kwargs["target_tip_sequence"]))
        kwargs["target_tip_sequence"][0, 0] = 4.0
        self.assertNotEqual(4.0, active.points[0, 0])
        with self.assertRaisesRegex(ValueError, "stale"):
            trajectory_kwargs_from_active(active, current_time_s=10.3, stale_timeout=0.2)
        with self.assertRaisesRegex(ValueError, "future"):
            trajectory_kwargs_from_active(active, current_time_s=9.9, stale_timeout=0.2)

    def test_trajectory_log_line_is_machine_parseable(self):
        active = accept_trajectory_reference(
            initial_trajectory_reference(),
            points=valid_points(3),
            frame="base_link",
            stamp_s=10.0,
        )
        line = trajectory_state_log_line(active, reason="accepted horizon")
        self.assertIn("REFERENCE_STATE", line)
        self.assertIn("mode=trajectory", line)
        self.assertIn("source=trajectory_horizon", line)
        self.assertIn("revision=1", line)
        self.assertIn("points=3", line)
        self.assertNotIn("accepted horizon", line)


if __name__ == "__main__":
    unittest.main()
