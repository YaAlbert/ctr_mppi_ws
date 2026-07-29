import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))


from ctr_mppi_controller.curved_lumen import (  # noqa: E402
    CurvedLumen,
    circular_arc_centerline,
    s_curve_centerline,
)
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_sim.lumen_diagnostics import (  # noqa: E402
    CONSTRAINT_INLET,
    CONSTRAINT_OUTLET,
    CONSTRAINT_WALL,
    STATUS_COLLISION,
    STATUS_MARGIN,
    STATUS_SAFE,
    STATUS_UNAVAILABLE,
    LumenRuntimeDiagnostic,
    build_lumen_runtime_diagnostic,
    unavailable_lumen_runtime_diagnostic,
)


def cylinder() -> CylindricalLumen:
    return CylindricalLumen(
        frame_id="base_link",
        axis_origin=np.array([0.0, 0.0, 0.0], dtype=float),
        axis_direction=np.array([0.0, 0.0, 1.0], dtype=float),
        radius=0.030,
        length=0.120,
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


def circular_arc() -> CurvedLumen:
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.18,
            arc_angle=0.35,
            sample_spacing=0.01,
        ),
        lumen_radius=0.030,
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


def s_curve() -> CurvedLumen:
    centerline = s_curve_centerline(
        inlet_position=[0.0, 0.0, 0.0],
        initial_tangent=[0.0, 0.0, 1.0],
        bend_plane_normal=[1.0, 0.0, 0.0],
        total_length=0.120,
        lateral_amplitude=0.010,
        sample_spacing=0.01,
    )
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=centerline,
        lumen_radius=np.linspace(0.030, 0.026, centerline.shape[0]),
        ctr_outer_radius=0.0015,
        safety_margin=0.0020,
    )


class LumenRuntimeDiagnosticTest(unittest.TestCase):
    def test_safe_wall_case_uses_authoritative_clearance_and_witness_points(self):
        lumen = cylinder()
        diagnostic = build_lumen_runtime_diagnostic(lumen, [[0.010, 0.0, 0.050]], "cylindrical")
        self.assertEqual(STATUS_SAFE, diagnostic.status)
        self.assertEqual(CONSTRAINT_WALL, diagnostic.constraint_type)
        self.assertEqual(0, diagnostic.backbone_index)
        self.assertAlmostEqual(0.0185, diagnostic.physical_clearance)
        self.assertAlmostEqual(0.0165, diagnostic.safety_clearance)
        np.testing.assert_allclose(diagnostic.backbone_center_point, [0.010, 0.0, 0.050])
        np.testing.assert_allclose(diagnostic.lumen_reference_point, [0.0, 0.0, 0.050])
        np.testing.assert_allclose(diagnostic.ctr_surface_point, [0.0115, 0.0, 0.050])
        np.testing.assert_allclose(diagnostic.lumen_boundary_point, [0.030, 0.0, 0.050])
        self.assertAlmostEqual(
            diagnostic.physical_clearance,
            np.linalg.norm(diagnostic.lumen_boundary_point - diagnostic.ctr_surface_point),
        )

    def test_margin_violation_is_distinct_from_physical_collision(self):
        diagnostic = build_lumen_runtime_diagnostic(cylinder(), [[0.0278, 0.0, 0.050]], "cylindrical")
        self.assertEqual(STATUS_MARGIN, diagnostic.status)
        self.assertFalse(diagnostic.physical_collision)
        self.assertTrue(diagnostic.safety_margin_violation)
        self.assertGreaterEqual(diagnostic.physical_clearance, 0.0)
        self.assertLess(diagnostic.safety_clearance, 0.0)

    def test_exact_physical_contact_preserves_existing_threshold_semantics(self):
        lumen = cylinder()
        diagnostic = build_lumen_runtime_diagnostic(lumen, [[lumen.usable_radius, 0.0, 0.050]], "cylindrical")
        self.assertEqual(STATUS_MARGIN, diagnostic.status)
        self.assertAlmostEqual(0.0, diagnostic.physical_clearance)
        self.assertFalse(diagnostic.physical_collision)
        self.assertTrue(diagnostic.safety_margin_violation)

    def test_physical_wall_collision_reports_negative_clearance(self):
        diagnostic = build_lumen_runtime_diagnostic(cylinder(), [[0.032, 0.0, 0.050]], "cylindrical")
        self.assertEqual(STATUS_COLLISION, diagnostic.status)
        self.assertEqual(CONSTRAINT_WALL, diagnostic.constraint_type)
        self.assertTrue(diagnostic.physical_collision)
        self.assertLess(diagnostic.physical_clearance, 0.0)

    def test_inlet_violation_uses_plane_witness(self):
        diagnostic = build_lumen_runtime_diagnostic(cylinder(), [[0.0, 0.0, -0.003]], "cylindrical")
        self.assertEqual(STATUS_COLLISION, diagnostic.status)
        self.assertEqual(CONSTRAINT_INLET, diagnostic.constraint_type)
        self.assertAlmostEqual(-0.003, diagnostic.physical_clearance)
        np.testing.assert_allclose(diagnostic.ctr_surface_point, [0.0, 0.0, -0.003])
        np.testing.assert_allclose(diagnostic.lumen_reference_point, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(diagnostic.lumen_boundary_point, [0.0, 0.0, 0.0])

    def test_outlet_violation_uses_plane_witness(self):
        diagnostic = build_lumen_runtime_diagnostic(cylinder(), [[0.0, 0.0, 0.124]], "cylindrical")
        self.assertEqual(STATUS_COLLISION, diagnostic.status)
        self.assertEqual(CONSTRAINT_OUTLET, diagnostic.constraint_type)
        self.assertAlmostEqual(-0.004, diagnostic.physical_clearance)
        np.testing.assert_allclose(diagnostic.ctr_surface_point, [0.0, 0.0, 0.124])
        np.testing.assert_allclose(diagnostic.lumen_reference_point, [0.0, 0.0, 0.120])
        np.testing.assert_allclose(diagnostic.lumen_boundary_point, [0.0, 0.0, 0.120])

    def test_curved_arc_and_s_curve_wall_cases_are_finite_and_deterministic(self):
        for lumen in (circular_arc(), s_curve()):
            point = lumen.centerline_points[3] + np.array([0.006, 0.0, 0.0], dtype=float)
            first = build_lumen_runtime_diagnostic(lumen, [point], "curved")
            second = build_lumen_runtime_diagnostic(lumen, [point], "curved")
            self.assertEqual(STATUS_SAFE, first.status)
            self.assertEqual(CONSTRAINT_WALL, first.constraint_type)
            for values in (
                first.backbone_center_point,
                first.ctr_surface_point,
                first.lumen_reference_point,
                first.lumen_boundary_point,
            ):
                self.assertTrue(np.all(np.isfinite(values)))
                self.assertFalse(values.flags.writeable)
            np.testing.assert_allclose(first.ctr_surface_point, second.ctr_surface_point)
            np.testing.assert_allclose(first.lumen_boundary_point, second.lumen_boundary_point)

    def test_variable_radius_changes_wall_witness_boundary(self):
        lumen = s_curve()
        index = lumen.centerline_points.shape[0] - 2
        point = lumen.centerline_points[index] + np.array([0.004, 0.0, 0.0], dtype=float)
        diagnostic = build_lumen_runtime_diagnostic(lumen, [point], "curved")
        radius = np.linalg.norm(diagnostic.lumen_boundary_point - diagnostic.lumen_reference_point)
        self.assertLess(radius, 0.030)
        self.assertGreater(radius, 0.025)

    def test_worst_backbone_index_can_be_first_middle_or_final(self):
        cases = (
            ([[0.032, 0.0, 0.040], [0.010, 0.0, 0.060], [0.010, 0.0, 0.080]], 0),
            ([[0.010, 0.0, 0.040], [0.032, 0.0, 0.060], [0.010, 0.0, 0.080]], 1),
            ([[0.010, 0.0, 0.040], [0.010, 0.0, 0.060], [0.032, 0.0, 0.080]], 2),
        )
        for backbone, expected in cases:
            with self.subTest(expected=expected):
                diagnostic = build_lumen_runtime_diagnostic(cylinder(), backbone, "cylindrical")
                self.assertEqual(expected, diagnostic.backbone_index)

    def test_collision_priority_and_tie_breaking_are_deterministic(self):
        diagnostic = build_lumen_runtime_diagnostic(
            cylinder(),
            [[0.030, 0.0, -0.003], [0.030, 0.0, -0.003]],
            "cylindrical",
        )
        self.assertEqual(0, diagnostic.backbone_index)
        self.assertEqual(CONSTRAINT_INLET, diagnostic.constraint_type)

        tied = build_lumen_runtime_diagnostic(cylinder(), [[0.0305, 0.0, -0.002]], "cylindrical")
        self.assertEqual(CONSTRAINT_WALL, tied.constraint_type)

    def test_input_backbone_mutation_cannot_change_diagnostic(self):
        backbone = np.array([[0.010, 0.0, 0.050]], dtype=float)
        diagnostic = build_lumen_runtime_diagnostic(cylinder(), backbone, "cylindrical")
        backbone[0, 0] = 0.032
        np.testing.assert_allclose(diagnostic.backbone_center_point, [0.010, 0.0, 0.050])
        with self.assertRaises(ValueError):
            diagnostic.backbone_center_point[0] = 0.0

    def test_zero_radial_direction_is_reported_without_fabricated_witness_line(self):
        diagnostic = build_lumen_runtime_diagnostic(cylinder(), [[0.0, 0.0, 0.050]], "cylindrical")
        self.assertEqual(STATUS_SAFE, diagnostic.status)
        self.assertFalse(diagnostic.witness_available)
        np.testing.assert_allclose(diagnostic.ctr_surface_point, diagnostic.backbone_center_point)

    def test_invalid_backbone_inputs_are_rejected(self):
        invalid = (
            [],
            [1.0, 2.0, 3.0],
            np.zeros((1, 1, 3), dtype=float),
            np.zeros((2, 2), dtype=float),
            [[math.nan, 0.0, 0.0]],
            [[math.inf, 0.0, 0.0]],
            [[-math.inf, 0.0, 0.0]],
        )
        for backbone in invalid:
            with self.subTest(backbone=repr(backbone)):
                with self.assertRaises(ValueError):
                    build_lumen_runtime_diagnostic(cylinder(), backbone, "cylindrical")

    def test_mode_and_dataclass_consistency_are_validated(self):
        with self.assertRaisesRegex(ValueError, "geometry_mode"):
            build_lumen_runtime_diagnostic(cylinder(), [[0.0, 0.0, 0.0]], "curved")
        with self.assertRaisesRegex(ValueError, "status"):
            LumenRuntimeDiagnostic(
                frame_id="base_link",
                geometry_mode="curved",
                constraint_type=CONSTRAINT_WALL,
                backbone_index=0,
                backbone_center_point=np.zeros(3),
                ctr_surface_point=np.zeros(3),
                lumen_reference_point=np.zeros(3),
                lumen_boundary_point=np.zeros(3),
                physical_clearance=0.01,
                safety_clearance=0.008,
                physical_collision=True,
                safety_margin_violation=True,
                status=STATUS_SAFE,
                valid=True,
                reason="bad",
            )
        with self.assertRaisesRegex(ValueError, "physical_collision"):
            LumenRuntimeDiagnostic(
                frame_id="base_link",
                geometry_mode="curved",
                constraint_type=CONSTRAINT_WALL,
                backbone_index=0,
                backbone_center_point=np.zeros(3),
                ctr_surface_point=np.zeros(3),
                lumen_reference_point=np.zeros(3),
                lumen_boundary_point=np.zeros(3),
                physical_clearance=-0.01,
                safety_clearance=-0.012,
                physical_collision=True,
                safety_margin_violation=False,
                status=STATUS_COLLISION,
                valid=True,
                reason="bad",
            )

    def test_unavailable_diagnostic_is_explicit(self):
        diagnostic = unavailable_lumen_runtime_diagnostic(geometry_mode="curved", reason="disabled")
        self.assertFalse(diagnostic.valid)
        self.assertEqual(STATUS_UNAVAILABLE, diagnostic.status)
        self.assertEqual("disabled", diagnostic.reason)


if __name__ == "__main__":
    unittest.main()
