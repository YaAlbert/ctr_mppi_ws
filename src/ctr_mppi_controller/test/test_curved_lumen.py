import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_mppi_controller.curved_lumen import (  # noqa: E402
    CurvedLumen,
    circular_arc_centerline,
    s_curve_centerline,
)
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.lumen_geometry import LumenGeometry  # noqa: E402


def straight_curved_lumen(**overrides):
    values = {
        "frame_id": "base_link",
        "centerline_points": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.120]],
        "lumen_radius": 0.030,
        "ctr_outer_radius": 0.0015,
        "safety_margin": 0.0020,
    }
    values.update(overrides)
    return CurvedLumen(**values)


def kinked_lumen(**overrides):
    values = {
        "frame_id": "base_link",
        "centerline_points": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.050], [0.050, 0.0, 0.050]],
        "lumen_radius": 0.030,
        "ctr_outer_radius": 0.0015,
        "safety_margin": 0.0020,
    }
    values.update(overrides)
    return CurvedLumen(**values)


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


class CurvedLumenInterfaceAndValidationTest(unittest.TestCase):
    def test_curved_lumen_satisfies_common_protocol(self):
        self.assertIsInstance(straight_curved_lumen(), LumenGeometry)

    def test_cylindrical_lumen_satisfies_common_protocol(self):
        self.assertIsInstance(cylinder_lumen(), LumenGeometry)

    def test_invalid_centerline_shape_rejected(self):
        with self.assertRaisesRegex(ValueError, "centerline_points"):
            straight_curved_lumen(centerline_points=[0.0, 0.0, 0.0])

    def test_fewer_than_two_centerline_points_rejected(self):
        with self.assertRaisesRegex(ValueError, "at least two"):
            straight_curved_lumen(centerline_points=[[0.0, 0.0, 0.0]])

    def test_nan_centerline_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            straight_curved_lumen(centerline_points=[[0.0, 0.0, 0.0], [math.nan, 0.0, 1.0]])

    def test_inf_centerline_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            straight_curved_lumen(centerline_points=[[0.0, 0.0, 0.0], [math.inf, 0.0, 1.0]])

    def test_duplicate_consecutive_points_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate consecutive"):
            straight_curved_lumen(centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    def test_zero_length_segment_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate consecutive"):
            straight_curved_lumen(
                centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.020], [0.0, 0.0, 0.020]]
            )

    def test_invalid_scalar_radius_rejected(self):
        with self.assertRaisesRegex(ValueError, "lumen_radius"):
            straight_curved_lumen(lumen_radius=0.0)

    def test_invalid_radius_profile_shape_rejected(self):
        with self.assertRaisesRegex(ValueError, "one value per centerline point"):
            straight_curved_lumen(lumen_radius=[0.030])

    def test_nonpositive_radius_profile_value_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            straight_curved_lumen(lumen_radius=[0.030, 0.0])

    def test_invalid_ctr_outer_radius_rejected(self):
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            straight_curved_lumen(ctr_outer_radius=-0.001)

    def test_invalid_safety_margin_rejected(self):
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            straight_curved_lumen(safety_margin=-0.001)

    def test_radius_must_exceed_outer_radius(self):
        with self.assertRaisesRegex(ValueError, "exceed ctr_outer_radius"):
            straight_curved_lumen(lumen_radius=0.001, ctr_outer_radius=0.0015)

    def test_usable_radius_must_support_safety_margin(self):
        with self.assertRaisesRegex(ValueError, "safety_margin"):
            straight_curved_lumen(lumen_radius=0.003, ctr_outer_radius=0.0015, safety_margin=0.0020)


class CurvedLumenProjectionTest(unittest.TestCase):
    def test_point_exactly_on_segment(self):
        projection = straight_curved_lumen().project_point([0.0, 0.0, 0.060])
        self.assertEqual(0, projection.segment_index)
        self.assertAlmostEqual(0.5, projection.segment_parameter)
        self.assertAlmostEqual(0.060, projection.progress)
        self.assertAlmostEqual(0.0, projection.radial_distance)

    def test_projection_to_segment_interior(self):
        projection = straight_curved_lumen().project_point([0.006, 0.0, 0.060])
        self.assertEqual(0, projection.segment_index)
        self.assertAlmostEqual(0.5, projection.segment_parameter)
        self.assertAlmostEqual(0.006, projection.radial_distance)

    def test_projection_clamps_to_segment_start(self):
        projection = straight_curved_lumen().project_point([0.006, 0.0, -0.010])
        self.assertEqual(0, projection.segment_index)
        self.assertAlmostEqual(0.0, projection.segment_parameter)
        self.assertTrue(np.allclose([0.0, 0.0, 0.0], projection.closest_point))

    def test_projection_clamps_to_segment_end(self):
        projection = straight_curved_lumen().project_point([0.006, 0.0, 0.140])
        self.assertEqual(0, projection.segment_index)
        self.assertAlmostEqual(1.0, projection.segment_parameter)
        self.assertTrue(np.allclose([0.0, 0.0, 0.120], projection.closest_point))

    def test_deterministic_equal_distance_tie_uses_lowest_segment(self):
        lumen = CurvedLumen(
            frame_id="base_link",
            centerline_points=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            lumen_radius=1.0,
            ctr_outer_radius=0.0,
            safety_margin=0.0,
        )
        projection = lumen.project_point([1.0, 0.1, 0.0])
        self.assertEqual(0, projection.segment_index)
        self.assertAlmostEqual(1.0, projection.segment_parameter)

    def test_nonuniform_centerline_sampling(self):
        lumen = straight_curved_lumen(centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.020], [0.0, 0.0, 0.120]])
        projection = lumen.project_point([0.004, 0.0, 0.070])
        self.assertEqual(1, projection.segment_index)
        self.assertAlmostEqual(0.5, projection.segment_parameter)
        self.assertAlmostEqual(0.070, projection.progress)

    def test_variable_radius_interpolation(self):
        lumen = straight_curved_lumen(lumen_radius=[0.030, 0.040])
        clearance = lumen.point_clearance([0.010, 0.0, 0.060])
        self.assertAlmostEqual(0.035, clearance.local_radius)
        self.assertAlmostEqual(0.0235, clearance.physical_clearance)

    def test_closest_point_and_segment_index_correctness(self):
        lumen = kinked_lumen()
        projection = lumen.project_point([0.020, 0.0, 0.060])
        self.assertEqual(1, projection.segment_index)
        self.assertTrue(np.allclose([0.020, 0.0, 0.050], projection.closest_point))

    def test_arc_length_progress_correctness(self):
        lumen = kinked_lumen()
        projection = lumen.project_point([0.030, 0.0, 0.050])
        self.assertAlmostEqual(0.080, projection.progress)

    def test_numerical_continuity_near_segment_boundary(self):
        lumen = kinked_lumen()
        before = lumen.project_point([0.0, 0.0, 0.050 - 1.0e-9])
        after = lumen.project_point([1.0e-9, 0.0, 0.050])
        self.assertLess(abs(before.progress - after.progress), 5.0e-9)


class CurvedLumenTubeAndEndCapTest(unittest.TestCase):
    def test_point_on_centerline(self):
        clearance = straight_curved_lumen().point_clearance([0.0, 0.0, 0.060])
        self.assertAlmostEqual(0.0285, clearance.physical_clearance)
        self.assertFalse(clearance.collision)

    def test_point_inside_physical_wall(self):
        clearance = straight_curved_lumen().point_clearance([0.010, 0.0, 0.060])
        self.assertAlmostEqual(0.0185, clearance.physical_clearance)
        self.assertFalse(clearance.collision)

    def test_point_exactly_at_physical_wall(self):
        lumen = straight_curved_lumen()
        clearance = lumen.point_clearance([lumen.minimum_usable_radius, 0.0, 0.060])
        self.assertAlmostEqual(0.0, clearance.physical_clearance)
        self.assertFalse(clearance.collision)
        self.assertTrue(clearance.safety_margin_violation)

    def test_point_outside_wall(self):
        lumen = straight_curved_lumen()
        clearance = lumen.point_clearance([lumen.minimum_usable_radius + 0.001, 0.0, 0.060])
        self.assertTrue(clearance.radial_collision)
        self.assertTrue(clearance.collision)

    def test_point_at_safety_margin(self):
        lumen = straight_curved_lumen()
        clearance = lumen.point_clearance([lumen.minimum_usable_radius - lumen.safety_margin, 0.0, 0.060])
        self.assertAlmostEqual(lumen.safety_margin, clearance.physical_clearance)
        self.assertFalse(clearance.safety_margin_violation)

    def test_point_before_inlet(self):
        clearance = straight_curved_lumen().point_clearance([0.0, 0.0, -0.001])
        self.assertTrue(clearance.inlet_violation)
        self.assertTrue(clearance.collision)

    def test_point_after_outlet(self):
        clearance = straight_curved_lumen().point_clearance([0.0, 0.0, 0.121])
        self.assertTrue(clearance.outlet_violation)
        self.assertTrue(clearance.collision)

    def test_exact_inlet_valid(self):
        validation = straight_curved_lumen().validate_target([0.0, 0.0, 0.0], frame_id="base_link")
        self.assertTrue(validation.valid)

    def test_exact_outlet_valid(self):
        validation = straight_curved_lumen().validate_target([0.0, 0.0, 0.120], frame_id="base_link")
        self.assertTrue(validation.valid)

    def test_radially_valid_before_inlet_rejected(self):
        validation = straight_curved_lumen().validate_target([0.001, 0.0, -0.001], frame_id="base_link")
        self.assertFalse(validation.valid)
        self.assertTrue(any("inlet" in reason for reason in validation.reasons))

    def test_radially_valid_after_outlet_rejected(self):
        validation = straight_curved_lumen().validate_target([0.001, 0.0, 0.121], frame_id="base_link")
        self.assertFalse(validation.valid)
        self.assertTrue(any("outlet" in reason for reason in validation.reasons))

    def test_frame_mismatch_invalidates_target(self):
        validation = straight_curved_lumen().validate_target([0.0, 0.0, 0.060], frame_id="world")
        self.assertFalse(validation.valid)
        self.assertTrue(any("frame_id" in reason for reason in validation.reasons))

    def test_nonfinite_target_invalid(self):
        validation = straight_curved_lumen().validate_target([math.nan, 0.0, 0.060], frame_id="base_link")
        self.assertFalse(validation.valid)


class CurvedLumenBackboneTest(unittest.TestCase):
    def test_entire_backbone_valid(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.0], [0.010, 0.0, 0.060], [0.0, 0.0, 0.120]])
        self.assertTrue(result.collision_free)
        self.assertEqual(0, result.collision_count)

    def test_valid_tip_with_middle_radial_collision(self):
        lumen = straight_curved_lumen()
        result = lumen.backbone_clearance(
            [[0.0, 0.0, 0.0], [lumen.minimum_usable_radius + 0.001, 0.0, 0.060], [0.0, 0.0, 0.100]]
        )
        self.assertFalse(result.collision_free)
        self.assertTrue(result.radial_collision_mask[1])
        self.assertFalse(result.radial_collision_mask[-1])

    def test_valid_tip_with_middle_inlet_violation(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.020], [0.0, 0.0, -0.001], [0.0, 0.0, 0.100]])
        self.assertFalse(result.collision_free)
        self.assertTrue(result.inlet_violation_mask[1])
        self.assertFalse(result.inlet_violation_mask[-1])

    def test_valid_tip_with_middle_outlet_violation(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.020], [0.0, 0.0, 0.121], [0.0, 0.0, 0.100]])
        self.assertFalse(result.collision_free)
        self.assertTrue(result.outlet_violation_mask[1])
        self.assertFalse(result.outlet_violation_mask[-1])

    def test_correct_minimum_clearance_point_index(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.0], [0.020, 0.0, 0.060], [0.005, 0.0, 0.100]])
        self.assertEqual(1, result.closest_backbone_index)
        self.assertEqual(1, result.closest_backbone_point_index)

    def test_correct_maximum_penetration(self):
        lumen = straight_curved_lumen()
        result = lumen.backbone_clearance([[0.0, 0.0, 0.0], [lumen.minimum_usable_radius + 0.003, 0.0, 0.060]])
        self.assertAlmostEqual(0.003, result.maximum_penetration_depth)
        self.assertAlmostEqual(result.maximum_penetration_depth, result.maximum_penetration)

    def test_correct_mean_clearance(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.0], [0.010, 0.0, 0.060]])
        self.assertAlmostEqual((0.0285 + 0.0185) / 2.0, result.mean_clearance)

    def test_correct_p05_clearance(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.0], [0.010, 0.0, 0.060], [0.020, 0.0, 0.100]])
        self.assertAlmostEqual(np.percentile([0.0285, 0.0185, 0.0085], 5.0), result.p05_clearance)

    def test_correct_min_and_max_progress(self):
        result = straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.020], [0.0, 0.0, 0.100]])
        self.assertAlmostEqual(0.020, result.minimum_progress)
        self.assertAlmostEqual(0.100, result.maximum_progress)

    def test_nonfinite_backbone_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            straight_curved_lumen().backbone_clearance([[0.0, 0.0, 0.0], [math.inf, 0.0, 0.0]])

    def test_deterministic_repeated_result(self):
        lumen = kinked_lumen()
        points = [[0.0, 0.0, 0.0], [0.010, 0.0, 0.040], [0.020, 0.0, 0.050]]
        first = lumen.backbone_clearance(points)
        second = lumen.backbone_clearance(points)
        self.assertTrue(np.array_equal(first.physical_clearances, second.physical_clearances))
        self.assertTrue(np.array_equal(first.closest_geometry_indices, second.closest_geometry_indices))

    def test_representative_workload_has_finite_shapes(self):
        centerline = circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.0021,
        )
        lumen = CurvedLumen("base_link", centerline, 0.030, 0.0015, 0.0020)
        backbone = centerline[np.linspace(0, centerline.shape[0] - 1, 50).astype(int)]
        result = lumen.backbone_clearance(backbone)
        self.assertEqual((50,), result.physical_clearances.shape)
        self.assertEqual((50, 3), result.closest_geometry_points.shape)
        self.assertTrue(np.all(np.isfinite(result.physical_clearances)))


class CurvedLumenGeneratorTest(unittest.TestCase):
    def test_circular_arc_includes_exact_inlet(self):
        points = circular_arc_centerline(
            inlet_position=[0.1, 0.2, 0.3],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.010,
        )
        self.assertTrue(np.allclose([0.1, 0.2, 0.3], points[0]))

    def test_circular_arc_expected_outlet_geometry(self):
        radius = 0.180
        angle = 0.70
        points = circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=radius,
            arc_angle=angle,
            sample_spacing=0.002,
        )
        expected = [radius * (1.0 - math.cos(angle)), 0.0, radius * math.sin(angle)]
        self.assertTrue(np.allclose(expected, points[-1]))

    def test_circular_arc_inlet_tangent_matches_requested_tangent(self):
        points = circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.001,
        )
        first_segment = points[1] - points[0]
        first_segment /= np.linalg.norm(first_segment)
        self.assertGreater(float(np.dot(first_segment, [0.0, 0.0, 1.0])), 0.99999)

    def test_circular_arc_supports_arbitrary_orientation(self):
        points = circular_arc_centerline(
            inlet_position=[1.0, 2.0, 3.0],
            initial_tangent=[1.0, 0.0, 0.0],
            bend_normal=[0.0, 1.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.010,
        )
        self.assertGreater(points[-1, 0], points[0, 0])
        self.assertGreater(points[-1, 1], points[0, 1])

    def test_circular_arc_approximate_spacing_is_bounded(self):
        points = circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.010,
        )
        segment_lengths = np.linalg.norm(points[1:] - points[:-1], axis=1)
        self.assertLessEqual(float(np.max(segment_lengths)), 0.0101)
        self.assertGreater(float(np.min(segment_lengths)), 0.0)

    def test_circular_arc_invalid_basis_rejected(self):
        with self.assertRaisesRegex(ValueError, "parallel"):
            circular_arc_centerline(
                inlet_position=[0.0, 0.0, 0.0],
                initial_tangent=[0.0, 0.0, 1.0],
                bend_normal=[0.0, 0.0, 2.0],
                curvature_radius=0.180,
                arc_angle=0.70,
                sample_spacing=0.010,
            )

    def test_circular_arc_negative_angle_bends_opposite_direction(self):
        positive = circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.010,
        )
        negative = circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=-0.70,
            sample_spacing=0.010,
        )
        self.assertAlmostEqual(positive[-1, 2], negative[-1, 2])
        self.assertAlmostEqual(positive[-1, 0], -negative[-1, 0])

    def test_s_curve_includes_exact_inlet(self):
        points = s_curve_centerline(
            inlet_position=[0.1, 0.2, 0.3],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.010,
            sample_spacing=0.005,
        )
        self.assertTrue(np.allclose([0.1, 0.2, 0.3], points[0]))

    def test_s_curve_endpoint_properties_are_correct(self):
        points = s_curve_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.010,
            sample_spacing=0.005,
        )
        self.assertTrue(np.allclose([0.0, 0.0, 0.120], points[-1], atol=1.0e-15))

    def test_s_curve_contains_opposite_bend_directions(self):
        points = s_curve_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.010,
            sample_spacing=0.005,
        )
        self.assertGreater(float(np.max(points[:, 0])), 0.005)
        self.assertLess(float(np.min(points[:, 0])), -0.005)

    def test_s_curve_tangent_is_continuous_numerically(self):
        points = s_curve_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.010,
            sample_spacing=0.002,
        )
        segments = points[1:] - points[:-1]
        unit_segments = segments / np.linalg.norm(segments, axis=1)[:, None]
        adjacent_dots = np.sum(unit_segments[1:] * unit_segments[:-1], axis=1)
        self.assertGreater(float(np.min(adjacent_dots)), 0.95)

    def test_s_curve_supports_arbitrary_orientation(self):
        points = s_curve_centerline(
            inlet_position=[1.0, 2.0, 3.0],
            initial_tangent=[1.0, 0.0, 0.0],
            bend_plane_normal=[0.0, 1.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.010,
            sample_spacing=0.005,
        )
        self.assertTrue(np.allclose([1.120, 2.0, 3.0], points[-1], atol=1.0e-15))
        self.assertGreater(float(np.max(points[:, 1])), 2.005)
        self.assertLess(float(np.min(points[:, 1])), 1.995)

    def test_s_curve_generator_output_is_deterministic(self):
        kwargs = dict(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.010,
            sample_spacing=0.005,
        )
        first = s_curve_centerline(**kwargs)
        second = s_curve_centerline(**kwargs)
        self.assertTrue(np.array_equal(first, second))


class StraightCylinderRegressionCompatibilityTest(unittest.TestCase):
    def test_common_result_conversion_preserves_cylindrical_minimum_clearance(self):
        cyl = cylinder_lumen()
        result = cyl.backbone_clearance([[0.0, 0.0, 0.0], [0.010, 0.0, 0.060], [0.020, 0.0, 0.100]])
        self.assertAlmostEqual(0.0085, result.minimum_radial_clearance)
        self.assertAlmostEqual(result.minimum_radial_clearance, result.minimum_clearance)

    def test_common_result_conversion_preserves_cylindrical_collision_masks(self):
        cyl = cylinder_lumen()
        result = cyl.backbone_clearance([[0.0, 0.0, 0.0], [cyl.usable_radius + 0.001, 0.0, 0.060]])
        self.assertTrue(np.array_equal([False, True], result.radial_collision_mask))
        self.assertTrue(np.array_equal([False, True], result.collision_mask))

    def test_common_result_conversion_preserves_cylindrical_inlet_masks(self):
        result = cylinder_lumen().backbone_clearance([[0.0, 0.0, -0.001], [0.0, 0.0, 0.060]])
        self.assertTrue(np.array_equal([True, False], result.inlet_violation_mask))

    def test_common_result_conversion_preserves_cylindrical_outlet_masks(self):
        result = cylinder_lumen().backbone_clearance([[0.0, 0.0, 0.060], [0.0, 0.0, 0.121]])
        self.assertTrue(np.array_equal([False, True], result.outlet_violation_mask))

    def test_straight_centerline_curved_lumen_matches_cylinder_for_interior_points(self):
        cyl = cylinder_lumen()
        curved = straight_curved_lumen()
        for point in ([0.0, 0.0, 0.060], [0.010, 0.0, 0.060], [0.020, 0.0, 0.100]):
            self.assertAlmostEqual(cyl.point_clearance(point).radial_clearance, curved.point_clearance(point).physical_clearance)

    def test_cylindrical_point_exposes_common_fields(self):
        clearance = cylinder_lumen().point_clearance([0.010, 0.0, 0.060])
        self.assertAlmostEqual(clearance.radial_clearance, clearance.physical_clearance)
        self.assertAlmostEqual(clearance.radial_clearance - 0.0020, clearance.safety_margin_clearance)
        self.assertEqual(0, clearance.closest_geometry_index)

    def test_cylindrical_backbone_exposes_common_fields(self):
        result = cylinder_lumen().backbone_clearance([[0.0, 0.0, 0.0], [0.010, 0.0, 0.060]])
        self.assertEqual((2,), result.physical_clearances.shape)
        self.assertEqual((2,), result.safety_margin_clearances.shape)
        self.assertEqual((2, 3), result.closest_geometry_points.shape)


if __name__ == "__main__":
    unittest.main()
