import dataclasses
import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_evaluation.lumen_metrics import (  # noqa: E402
    CONSTRAINT_INLET,
    CONSTRAINT_OUTLET,
    CONSTRAINT_WALL,
    compute_lumen_evaluation_metrics,
    event_count,
    event_duration,
)
from ctr_evaluation.metrics import compute_lumen_safety_metrics  # noqa: E402
from ctr_mppi_controller.curved_lumen import (  # noqa: E402
    CurvedLumen,
    circular_arc_centerline,
    s_curve_centerline,
)
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.lumen_geometry import BackboneClearance  # noqa: E402


def cylinder_lumen():
    return CylindricalLumen(
        frame_id="base_link",
        axis_origin=[0.0, 0.0, 0.0],
        axis_direction=[0.0, 0.0, 1.0],
        radius=0.030,
        length=0.120,
        ctr_outer_radius=0.0015,
        safety_margin=0.002,
    )


def circular_arc_lumen():
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.180,
            arc_angle=0.70,
            sample_spacing=0.004,
        ),
        lumen_radius=0.030,
        ctr_outer_radius=0.0015,
        safety_margin=0.002,
    )


def s_curve_lumen():
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=s_curve_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.120,
            lateral_amplitude=0.015,
            sample_spacing=0.004,
        ),
        lumen_radius=0.030,
        ctr_outer_radius=0.0015,
        safety_margin=0.002,
    )


def variable_radius_lumen():
    centerline = np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.100]], dtype=float)
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=centerline,
        lumen_radius=np.asarray([0.020, 0.040], dtype=float),
        ctr_outer_radius=0.0015,
        safety_margin=0.002,
    )


def point_at(lumen, fraction, radial_offset=0.0):
    if hasattr(lumen, "centerline_points"):
        distance = float(np.clip(fraction, 0.0, 1.0) * lumen.length)
        segment = int(np.searchsorted(lumen.cumulative_arc_lengths, distance, side="right") - 1)
        segment = int(np.clip(segment, 0, len(lumen.segment_lengths) - 1))
        start = lumen.centerline_points[segment]
        length = lumen.segment_lengths[segment]
        parameter = 0.0 if length == 0.0 else (distance - lumen.cumulative_arc_lengths[segment]) / length
        center = start + parameter * lumen.segment_vectors[segment]
        return center + np.asarray([0.0, radial_offset, 0.0], dtype=float)
    return np.asarray([radial_offset, 0.0, float(fraction) * lumen.length], dtype=float)


def backbone_for(lumen, radial_offset, *, fraction=0.5):
    return np.vstack(
        [
            point_at(lumen, 0.10, 0.0),
            point_at(lumen, fraction, radial_offset),
            point_at(lumen, 0.90, 0.0),
        ]
    )


def centerline_backbone(lumen, fractions):
    return np.vstack([point_at(lumen, fraction, 0.0) for fraction in fractions])


def assert_no_nonfinite(testcase, value):
    if dataclasses.is_dataclass(value):
        for field in dataclasses.fields(value):
            assert_no_nonfinite(testcase, getattr(value, field.name))
    elif isinstance(value, dict):
        for item in value.values():
            assert_no_nonfinite(testcase, item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            assert_no_nonfinite(testcase, item)
    elif isinstance(value, np.ndarray):
        testcase.assertTrue(np.all(np.isfinite(value)))
    elif isinstance(value, float):
        testcase.assertTrue(math.isfinite(value))


def signature(result):
    return (
        result.safety.minimum_physical_clearance,
        result.safety.minimum_safety_clearance,
        result.safety.physical_collision_event_count,
        result.safety.safety_margin_violation_event_count,
        result.safety.worst_physical_constraint,
        result.safety.worst_physical_sample_index,
        result.safety.worst_physical_backbone_index,
        result.safety.worst_safety_sample_index,
        result.safety.worst_safety_backbone_index,
        result.progress.final_centerline_arc_length,
        result.progress.final_normalized_progress,
    )


def custom_clearance(
    *,
    physical,
    safety=None,
    collision=None,
    safety_mask=None,
    radial_collision=None,
    inlet=None,
    outlet=None,
    wall_penetration=None,
    inlet_penetration=None,
    outlet_penetration=None,
):
    physical = np.asarray(physical, dtype=float)
    count = physical.shape[0]
    points = np.column_stack((np.zeros(count), np.zeros(count), np.arange(count, dtype=float)))
    safety_values = physical - 0.002 if safety is None else np.asarray(safety, dtype=float)
    radial = np.zeros(count, dtype=bool) if radial_collision is None else np.asarray(radial_collision, dtype=bool)
    inlet_mask = np.zeros(count, dtype=bool) if inlet is None else np.asarray(inlet, dtype=bool)
    outlet_mask = np.zeros(count, dtype=bool) if outlet is None else np.asarray(outlet, dtype=bool)
    collision_mask = radial | inlet_mask | outlet_mask if collision is None else np.asarray(collision, dtype=bool)
    safety_flags = safety_values < 0.0 if safety_mask is None else np.asarray(safety_mask, dtype=bool)
    wall_pen = np.maximum(-physical, 0.0) if wall_penetration is None else np.asarray(wall_penetration, dtype=float)
    inlet_pen = np.zeros(count, dtype=float) if inlet_penetration is None else np.asarray(inlet_penetration, dtype=float)
    outlet_pen = np.zeros(count, dtype=float) if outlet_penetration is None else np.asarray(outlet_penetration, dtype=float)
    return BackboneClearance(
        points=points,
        physical_clearances=physical,
        safety_margin_clearances=safety_values,
        collision_mask=collision_mask,
        safety_margin_violation_mask=safety_flags,
        radial_collision_mask=radial,
        inlet_violation_mask=inlet_mask,
        outlet_violation_mask=outlet_mask,
        wall_penetrations=wall_pen,
        inlet_penetrations=inlet_pen,
        outlet_penetrations=outlet_pen,
        end_cap_penetrations=np.maximum(inlet_pen, outlet_pen),
        maximum_penetration_depth=float(np.max(np.maximum.reduce([wall_pen, inlet_pen, outlet_pen]))),
        minimum_clearance=float(np.min(physical)),
        mean_clearance=float(np.mean(physical)),
        p05_clearance=float(np.percentile(physical, 5.0)),
        closest_backbone_index=int(np.argmin(physical)),
        closest_geometry_indices=np.zeros(count, dtype=int),
        closest_geometry_parameters=np.zeros(count, dtype=float),
        closest_geometry_points=points.copy(),
        centerline_progress=points[:, 2],
        radial_distance=np.zeros(count, dtype=float),
        local_radius=np.ones(count, dtype=float),
        axial_position=points[:, 2],
        radial_clearance=physical,
        axial_clearance=np.ones(count, dtype=float),
        closest_backbone_point_index=int(np.argmin(physical)),
        minimum_radial_clearance=float(np.min(physical)),
        minimum_axial_clearance=1.0,
        collision_count=int(np.sum(collision_mask)),
        safety_margin_violation_count=int(np.sum(safety_flags)),
    )


class SpyGeometry:
    frame_id = "base_link"
    ctr_outer_radius = 0.0015
    safety_margin = 0.002
    length = 2.0

    def __init__(self, clearances):
        self.clearances = list(clearances)
        self.calls = 0

    def backbone_clearance(self, points):
        result = self.clearances[self.calls]
        self.calls += 1
        return result

    def point_clearance(self, point):
        raise AssertionError("point_clearance must not be used by evaluation metrics")


class LumenMetricsTest(unittest.TestCase):
    def test_safe_circular_arc_full_backbone(self):
        lumen = circular_arc_lumen()
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0],
            backbone_points=[centerline_backbone(lumen, [0.1, 0.4]), centerline_backbone(lumen, [0.2, 0.6])],
        )
        self.assertTrue(result.safety.physical_safety_pass)
        self.assertTrue(result.safety.safety_margin_pass)
        self.assertGreater(result.safety.minimum_physical_clearance, 0.0)
        self.assertGreater(result.safety.minimum_safety_clearance, 0.0)
        self.assertEqual(0, result.safety.physical_collision_event_count)
        self.assertEqual(0.0, result.safety.physical_collision_duration)
        assert_no_nonfinite(self, result)

    def test_safety_margin_violation_without_physical_collision(self):
        lumen = circular_arc_lumen()
        offset = lumen.minimum_lumen_radius - lumen.ctr_outer_radius - 0.5 * lumen.safety_margin
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0, 2.5],
            backbone_points=[
                centerline_backbone(lumen, [0.1, 0.3]),
                backbone_for(lumen, offset),
                backbone_for(lumen, offset),
            ],
        )
        self.assertTrue(result.safety.physical_safety_pass)
        self.assertFalse(result.safety.safety_margin_pass)
        self.assertEqual(0, result.safety.physical_collision_sample_count)
        self.assertEqual(2, result.safety.safety_margin_violation_sample_count)
        self.assertEqual(1, result.safety.safety_margin_violation_event_count)
        self.assertAlmostEqual(2.5, result.safety.safety_margin_violation_duration)
        self.assertGreaterEqual(result.safety.minimum_physical_clearance, 0.0)
        self.assertLess(result.safety.minimum_safety_clearance, 0.0)

    def test_wall_physical_collision(self):
        lumen = circular_arc_lumen()
        offset = lumen.minimum_lumen_radius - lumen.ctr_outer_radius + 0.003
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 0.5],
            backbone_points=[centerline_backbone(lumen, [0.1, 0.2]), backbone_for(lumen, offset)],
        )
        self.assertTrue(result.safety.physical_collision_detected)
        self.assertEqual(1, result.safety.physical_collision_sample_count)
        self.assertEqual(1, result.safety.physical_collision_event_count)
        self.assertAlmostEqual(0.5, result.safety.physical_collision_duration)
        self.assertLess(result.safety.minimum_physical_clearance, 0.0)
        self.assertEqual(CONSTRAINT_WALL, result.safety.worst_physical_constraint)
        self.assertEqual(1, result.safety.worst_physical_backbone_index)

    def test_inlet_violation(self):
        lumen = circular_arc_lumen()
        offending = lumen.centerline_points[0] - 0.003 * lumen.inlet_tangent
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0],
            backbone_points=[np.vstack([offending, point_at(lumen, 0.1)])],
        )
        self.assertTrue(result.safety.physical_collision_detected)
        self.assertEqual(CONSTRAINT_INLET, result.safety.worst_physical_constraint)
        self.assertEqual(0.0, result.safety.first_physical_collision_time)
        inlet = result.safety.per_constraint_breakdown[1]
        self.assertEqual(CONSTRAINT_INLET, inlet.constraint_type)
        self.assertEqual(1, inlet.physical_violation_sample_count)
        self.assertGreater(inlet.maximum_penetration, 0.0)

    def test_outlet_violation(self):
        lumen = circular_arc_lumen()
        offending = lumen.centerline_points[-1] + 0.004 * lumen.outlet_tangent
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0],
            backbone_points=[centerline_backbone(lumen, [0.2, 0.4]), np.vstack([point_at(lumen, 0.8), offending])],
        )
        self.assertTrue(result.safety.physical_collision_detected)
        self.assertEqual(CONSTRAINT_OUTLET, result.safety.worst_physical_constraint)
        outlet = result.safety.per_constraint_breakdown[2]
        self.assertEqual(1, outlet.physical_violation_sample_count)
        self.assertAlmostEqual(1.0, outlet.physical_violation_duration)

    def test_exact_contact_preserves_geometry_classification(self):
        lumen = cylinder_lumen()
        contact_offset = lumen.radius - lumen.ctr_outer_radius
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0],
            backbone_points=[backbone_for(lumen, contact_offset)],
        )
        self.assertAlmostEqual(0.0, result.safety.minimum_physical_clearance)
        self.assertFalse(result.safety.physical_collision_detected)
        self.assertTrue(result.safety.safety_margin_violation_detected)
        self.assertEqual(0, result.safety.physical_collision_event_count)
        self.assertEqual(1, result.safety.safety_margin_violation_event_count)

    def test_multiple_collision_events(self):
        lumen = cylinder_lumen()
        safe = 0.0
        collision = lumen.radius - lumen.ctr_outer_radius + 0.002
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0, 2.5, 4.0, 6.0],
            backbone_points=[
                backbone_for(lumen, safe),
                backbone_for(lumen, collision),
                backbone_for(lumen, collision),
                backbone_for(lumen, safe),
                backbone_for(lumen, collision),
            ],
        )
        self.assertEqual(2, result.safety.physical_collision_event_count)
        self.assertEqual(3, result.safety.physical_collision_sample_count)
        self.assertAlmostEqual(4.5, result.safety.physical_collision_duration)

    def test_first_sample_violating_uses_existing_duration_convention(self):
        lumen = cylinder_lumen()
        collision = lumen.radius - lumen.ctr_outer_radius + 0.002
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[5.0, 6.0, 8.0],
            backbone_points=[backbone_for(lumen, collision), backbone_for(lumen, collision), backbone_for(lumen, 0.0)],
        )
        self.assertEqual(1, result.safety.physical_collision_event_count)
        self.assertAlmostEqual(1.0, result.safety.physical_collision_duration)
        self.assertEqual(0.0, result.safety.first_physical_collision_time)

    def test_final_sample_violating_has_no_extrapolated_duration(self):
        lumen = cylinder_lumen()
        collision = lumen.radius - lumen.ctr_outer_radius + 0.002
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 2.0],
            backbone_points=[backbone_for(lumen, 0.0), backbone_for(lumen, collision)],
        )
        self.assertEqual(1, result.safety.physical_collision_event_count)
        self.assertAlmostEqual(2.0, result.safety.physical_collision_duration)

    def test_duplicate_timestamps_contribute_zero_duration(self):
        flags = [False, True, True, False]
        self.assertEqual(1, event_count(flags))
        self.assertAlmostEqual(1.0, event_duration([0.0, 1.0, 1.0, 2.0], flags))

    def test_decreasing_timestamps_are_rejected(self):
        lumen = cylinder_lumen()
        with self.assertRaisesRegex(ValueError, "monotonically"):
            compute_lumen_evaluation_metrics(
                geometry=lumen,
                times=[0.0, 1.0, 0.5],
                backbone_points=[backbone_for(lumen, 0.0)] * 3,
            )

    def test_nonfinite_timestamps_and_backbones_are_rejected(self):
        lumen = cylinder_lumen()
        with self.assertRaisesRegex(ValueError, "finite"):
            compute_lumen_evaluation_metrics(
                geometry=lumen,
                times=[0.0, math.inf],
                backbone_points=[backbone_for(lumen, 0.0), backbone_for(lumen, 0.0)],
            )
        with self.assertRaisesRegex(ValueError, "finite"):
            compute_lumen_evaluation_metrics(
                geometry=lumen,
                times=[0.0],
                backbone_points=[np.asarray([[0.0, math.nan, 0.0]])],
            )

    def test_mismatched_lengths_are_rejected(self):
        lumen = cylinder_lumen()
        with self.assertRaisesRegex(ValueError, "matching lengths"):
            compute_lumen_evaluation_metrics(
                geometry=lumen,
                times=[0.0, 1.0],
                backbone_points=[backbone_for(lumen, 0.0)],
            )

    def test_variable_radius_curved_lumen_reports_local_radius(self):
        lumen = variable_radius_lumen()
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0],
            backbone_points=[np.vstack([point_at(lumen, 0.0), point_at(lumen, 0.5), point_at(lumen, 1.0)])],
        )
        self.assertAlmostEqual(0.040, result.progress.final_local_lumen_radius)
        self.assertAlmostEqual(np.mean([0.040]), result.progress.mean_local_lumen_radius)
        self.assertAlmostEqual(0.040, result.samples[0].local_lumen_radius)

    def test_circular_arc_progress_is_monotonic(self):
        lumen = circular_arc_lumen()
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0, 2.0],
            backbone_points=[
                centerline_backbone(lumen, [0.1, 0.2]),
                centerline_backbone(lumen, [0.1, 0.5]),
                centerline_backbone(lumen, [0.1, 0.8]),
            ],
            compute_centerline_tracking_rmse=True,
        )
        arcs = [sample.tip_centerline_arc_length for sample in result.samples]
        self.assertLess(arcs[0], arcs[1])
        self.assertLess(arcs[1], arcs[2])
        self.assertAlmostEqual(0.0, result.progress.centerline_tracking_rmse)

    def test_s_curve_progress_uses_centerline_order_not_world_axis(self):
        lumen = s_curve_lumen()
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0, 2.0],
            backbone_points=[
                centerline_backbone(lumen, [0.25]),
                centerline_backbone(lumen, [0.50]),
                centerline_backbone(lumen, [0.75]),
            ],
        )
        progress = [sample.normalized_tip_progress for sample in result.samples]
        self.assertLess(progress[0], progress[1])
        self.assertLess(progress[1], progress[2])
        x_values = [sample.tip_centerline_point[0] for sample in result.samples]
        self.assertGreater(x_values[0], x_values[1])
        self.assertGreater(x_values[1], x_values[2])

    def test_before_inlet_and_after_outlet_progress_behavior(self):
        lumen = cylinder_lumen()
        result = compute_lumen_evaluation_metrics(
            geometry=lumen,
            times=[0.0, 1.0],
            backbone_points=[
                np.asarray([[0.0, 0.0, -0.010]], dtype=float),
                np.asarray([[0.0, 0.0, 0.130]], dtype=float),
            ],
        )
        self.assertTrue(result.samples[0].tip_progress_out_of_extent)
        self.assertTrue(result.samples[1].tip_progress_out_of_extent)
        self.assertAlmostEqual(-0.010, result.samples[0].tip_centerline_arc_length)
        self.assertAlmostEqual(0.0, result.samples[0].normalized_tip_progress)
        self.assertAlmostEqual(1.0, result.samples[1].normalized_tip_progress)

    def test_deterministic_tie_breaking_prefers_wall_then_low_index(self):
        clearance = custom_clearance(
            physical=[-0.01, -0.01],
            collision=[True, True],
            radial_collision=[True, True],
            inlet=[True, False],
            wall_penetration=[0.01, 0.01],
            inlet_penetration=[0.01, 0.0],
        )
        geometry = SpyGeometry([clearance])
        result = compute_lumen_evaluation_metrics(
            geometry=geometry,
            times=[0.0],
            backbone_points=[clearance.points],
        )
        self.assertEqual(CONSTRAINT_WALL, result.safety.worst_physical_constraint)
        self.assertEqual(0, result.safety.worst_physical_backbone_index)
        self.assertEqual(CONSTRAINT_WALL, result.samples[0].selected_constraint_type)

    def test_physical_and_safety_minima_are_independent(self):
        first = custom_clearance(physical=[0.001], safety=[0.010], safety_mask=[False])
        second = custom_clearance(physical=[0.003], safety=[-0.002], safety_mask=[True])
        geometry = SpyGeometry([first, second])
        result = compute_lumen_evaluation_metrics(
            geometry=geometry,
            times=[0.0, 1.0],
            backbone_points=[first.points, second.points],
        )
        self.assertEqual(0, result.safety.worst_physical_sample_index)
        self.assertEqual(1, result.safety.worst_safety_sample_index)
        self.assertAlmostEqual(0.001, result.safety.minimum_physical_clearance)
        self.assertAlmostEqual(-0.002, result.safety.minimum_safety_clearance)

    def test_backbone_clearance_called_once_per_sample(self):
        lumen = cylinder_lumen()
        backbones = [backbone_for(lumen, 0.0), backbone_for(lumen, 0.001), backbone_for(lumen, 0.002)]
        geometry = SpyGeometry([lumen.backbone_clearance(points) for points in backbones])
        compute_lumen_evaluation_metrics(geometry=geometry, times=[0.0, 1.0, 2.0], backbone_points=backbones)
        self.assertEqual(3, geometry.calls)

    def test_input_mutation_protection_and_readonly_points(self):
        lumen = circular_arc_lumen()
        backbone = centerline_backbone(lumen, [0.2, 0.5])
        result = compute_lumen_evaluation_metrics(geometry=lumen, times=[0.0], backbone_points=[backbone])
        stored = result.samples[0].tip_centerline_point.copy()
        backbone[:] = 99.0
        self.assertTrue(np.allclose(stored, result.samples[0].tip_centerline_point))
        with self.assertRaises(ValueError):
            result.samples[0].tip_centerline_point[0] = 1.0

    def test_repeatability(self):
        lumen = circular_arc_lumen()
        backbones = [centerline_backbone(lumen, [0.1, 0.4]), backbone_for(lumen, 0.010)]
        first = compute_lumen_evaluation_metrics(geometry=lumen, times=[0.0, 1.0], backbone_points=backbones)
        second = compute_lumen_evaluation_metrics(geometry=lumen, times=[0.0, 1.0], backbone_points=backbones)
        self.assertEqual(signature(first), signature(second))

    def test_cylinder_backward_compatibility_with_existing_safety_api(self):
        lumen = cylinder_lumen()
        backbones = [
            backbone_for(lumen, 0.0),
            backbone_for(lumen, 0.027),
            backbone_for(lumen, 0.040),
        ]
        old = compute_lumen_safety_metrics(times=[0.0, 0.5, 1.0], backbone_points=backbones, lumen=lumen)
        new = compute_lumen_evaluation_metrics(geometry=lumen, times=[0.0, 0.5, 1.0], backbone_points=backbones)
        self.assertAlmostEqual(old.minimum_backbone_wall_clearance, new.safety.minimum_physical_clearance)
        self.assertAlmostEqual(old.mean_minimum_backbone_clearance, np.mean([sample.physical_clearance for sample in new.samples]))
        self.assertAlmostEqual(old.p05_clearance, np.percentile([sample.physical_clearance for sample in new.samples], 5.0))
        self.assertEqual(old.safety_margin_violation_count, new.safety.safety_margin_violation_sample_count)
        self.assertEqual(old.radial_collision_count, new.safety.per_constraint_breakdown[0].physical_violation_sample_count)
        self.assertEqual(old.inlet_violation_count, new.safety.per_constraint_breakdown[1].physical_violation_sample_count)
        self.assertEqual(old.outlet_violation_count, new.safety.per_constraint_breakdown[2].physical_violation_sample_count)
        self.assertEqual(old.collision_free_pass, new.safety.physical_safety_pass)
        self.assertEqual(old.safety_margin_pass, new.safety.safety_margin_pass)


if __name__ == "__main__":
    unittest.main()
