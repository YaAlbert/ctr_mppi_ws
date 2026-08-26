import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))


from builtin_interfaces.msg import Time  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from ctr_mppi_controller.curved_lumen import (  # noqa: E402
    CurvedLumen,
    circular_arc_centerline,
    s_curve_centerline,
)
from ctr_sim.lumen_markers import (  # noqa: E402
    BoundedTipTrajectory,
    CURVED_STATIC_LUMEN_MARKER_KEYS,
    CURVED_STATIC_LUMEN_MARKER_KEYS_WITH_SURFACE,
    DYNAMIC_LUMEN_MARKER_KEYS,
    LumenMarkerConfig,
    build_actual_tip_path_marker,
    build_dynamic_lumen_delete_markers,
    build_dynamic_lumen_diagnostic_markers,
    build_curved_static_lumen_markers,
    build_reference_path_markers,
    build_static_lumen_delete_markers,
    compute_parallel_transport_frames,
    marker_keys,
    sample_ring,
    static_lumen_cache_key,
)
from ctr_sim.lumen_diagnostics import (  # noqa: E402
    CONSTRAINT_INLET,
    CONSTRAINT_OUTLET,
    CONSTRAINT_WALL,
    STATUS_COLLISION,
    STATUS_MARGIN,
    STATUS_SAFE,
    LumenRuntimeDiagnostic,
    unavailable_lumen_runtime_diagnostic,
)


def straight_centerline(axis: str = "z") -> np.ndarray:
    axes = {
        "x": np.array([1.0, 0.0, 0.0], dtype=float),
        "y": np.array([0.0, 1.0, 0.0], dtype=float),
        "z": np.array([0.0, 0.0, 1.0], dtype=float),
    }
    direction = axes[axis]
    return np.asarray([index * 0.01 * direction for index in range(6)], dtype=float)


def circular_arc_geometry() -> CurvedLumen:
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=circular_arc_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_normal=[1.0, 0.0, 0.0],
            curvature_radius=0.18,
            arc_angle=0.25,
            sample_spacing=0.01,
        ),
        lumen_radius=0.03,
        ctr_outer_radius=0.0015,
        safety_margin=0.002,
    )


def s_curve_geometry() -> CurvedLumen:
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=s_curve_centerline(
            inlet_position=[0.0, 0.0, 0.0],
            initial_tangent=[0.0, 0.0, 1.0],
            bend_plane_normal=[1.0, 0.0, 0.0],
            total_length=0.12,
            lateral_amplitude=0.01,
            sample_spacing=0.01,
        ),
        lumen_radius=np.linspace(0.03, 0.025, 13),
        ctr_outer_radius=0.0015,
        safety_margin=0.002,
    )


def marker_by_namespace(markers: list[Marker], namespace: str) -> Marker:
    for marker in markers:
        if marker.ns == namespace:
            return marker
    raise AssertionError(f"missing marker namespace {namespace}")


def marker_by_key(markers: list[Marker], key: tuple[str, int]) -> Marker:
    for marker in markers:
        if (marker.ns, marker.id) == key:
            return marker
    raise AssertionError(f"missing marker key {key}")


def marker_points(marker: Marker) -> np.ndarray:
    return np.asarray([[point.x, point.y, point.z] for point in marker.points], dtype=float)


def marker_position(marker: Marker) -> np.ndarray:
    return np.asarray([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z], dtype=float)


def diagnostic(
    *,
    status: str = STATUS_SAFE,
    constraint_type: str = CONSTRAINT_WALL,
    physical_clearance: float = 0.004,
    safety_clearance: float = 0.002,
    witness_available: bool = True,
) -> LumenRuntimeDiagnostic:
    physical_collision = status == STATUS_COLLISION
    margin_violation = status in {STATUS_COLLISION, STATUS_MARGIN}
    return LumenRuntimeDiagnostic(
        frame_id="base_link",
        geometry_mode="curved",
        constraint_type=constraint_type,
        backbone_index=4,
        backbone_center_point=np.array([0.010, 0.0, 0.050], dtype=float),
        ctr_surface_point=np.array([0.0115, 0.0, 0.050], dtype=float),
        lumen_reference_point=np.array([0.0, 0.0, 0.050], dtype=float),
        lumen_boundary_point=np.array([0.030, 0.0, 0.050], dtype=float),
        physical_clearance=physical_clearance,
        safety_clearance=safety_clearance,
        physical_collision=physical_collision,
        safety_margin_violation=margin_violation,
        status=status,
        valid=True,
        reason="updated",
        witness_available=witness_available,
    )


class ParallelTransportFrameTest(unittest.TestCase):
    def assert_frame_basis(self, centerline: np.ndarray) -> None:
        frames = compute_parallel_transport_frames(centerline)
        for values in (frames.tangents, frames.normals, frames.binormals):
            self.assertEqual(centerline.shape, values.shape)
            self.assertTrue(np.all(np.isfinite(values)))
            self.assertFalse(values.flags.writeable)
        np.testing.assert_allclose(np.linalg.norm(frames.tangents, axis=1), 1.0, atol=1.0e-12)
        np.testing.assert_allclose(np.linalg.norm(frames.normals, axis=1), 1.0, atol=1.0e-12)
        np.testing.assert_allclose(np.linalg.norm(frames.binormals, axis=1), 1.0, atol=1.0e-12)
        np.testing.assert_allclose(np.sum(frames.tangents * frames.normals, axis=1), 0.0, atol=1.0e-12)
        np.testing.assert_allclose(np.sum(frames.tangents * frames.binormals, axis=1), 0.0, atol=1.0e-12)
        np.testing.assert_allclose(np.sum(frames.normals * frames.binormals, axis=1), 0.0, atol=1.0e-12)
        cross = np.cross(frames.normals, frames.binormals)
        np.testing.assert_allclose(cross, frames.tangents, atol=1.0e-12)
        repeated = compute_parallel_transport_frames(centerline)
        np.testing.assert_allclose(repeated.normals, frames.normals, atol=1.0e-12)

    def test_straight_axes_have_finite_right_handed_frames(self):
        for axis in ("x", "y", "z"):
            with self.subTest(axis=axis):
                self.assert_frame_basis(straight_centerline(axis))

    def test_arc_and_s_curve_frames_are_deterministic_without_sign_flips(self):
        for centerline in (circular_arc_geometry().centerline_points, s_curve_geometry().centerline_points):
            with self.subTest(point_count=centerline.shape[0]):
                self.assert_frame_basis(centerline)
                frames = compute_parallel_transport_frames(centerline)
                adjacent = np.sum(frames.normals[1:] * frames.normals[:-1], axis=1)
                self.assertTrue(np.all(adjacent > -0.25), adjacent)

    def test_near_axis_tangents_and_repeated_samples_are_handled(self):
        centerline = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0e-14, 0.0, 0.0],
                [0.01, 0.0, 0.0],
                [0.02, 0.0, 0.0],
            ],
            dtype=float,
        )
        self.assert_frame_basis(centerline)

    def test_invalid_centerlines_are_rejected(self):
        invalid_cases = (
            np.zeros((1, 3), dtype=float),
            np.zeros((3, 2), dtype=float),
            np.zeros((3, 4), dtype=float),
            np.array([[0.0, 0.0, 0.0], [math.nan, 0.0, 0.0]], dtype=float),
            np.zeros((3, 3), dtype=float),
        )
        for centerline in invalid_cases:
            with self.subTest(shape=centerline.shape):
                with self.assertRaises(ValueError):
                    compute_parallel_transport_frames(centerline)


class RingSamplingTest(unittest.TestCase):
    def test_sample_ring_line_list_geometry(self):
        center = np.array([1.0, 2.0, 3.0], dtype=float)
        normal = np.array([1.0, 0.0, 0.0], dtype=float)
        binormal = np.array([0.0, 1.0, 0.0], dtype=float)
        points = sample_ring(center, normal, binormal, 0.25, 16)
        self.assertEqual((32, 3), points.shape)
        self.assertTrue(np.all(np.isfinite(points)))
        sampled = points[0::2]
        np.testing.assert_allclose(np.linalg.norm(sampled - center, axis=1), 0.25, atol=1.0e-12)
        np.testing.assert_allclose((sampled - center) @ np.array([0.0, 0.0, 1.0]), 0.0, atol=1.0e-12)
        np.testing.assert_allclose(points[-1], sampled[0], atol=1.0e-12)
        np.testing.assert_allclose(sample_ring(center, normal, binormal, 0.25, 16), points, atol=0.0)

    def test_sample_ring_rejects_invalid_inputs(self):
        for radius in (0.0, -0.1, math.nan, math.inf):
            with self.subTest(radius=radius):
                with self.assertRaises(ValueError):
                    sample_ring([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], radius, 16)
        for segments in (7, 129, 8.0, True):
            with self.subTest(segments=segments):
                with self.assertRaises(ValueError):
                    sample_ring([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.1, segments)


class CurvedMarkerConstructionTest(unittest.TestCase):
    def test_curved_markers_have_deterministic_namespaces_ids_types_and_frames(self):
        geometry = circular_arc_geometry()
        config = LumenMarkerConfig(centerline_stride=2, ring_stride=3, ring_segments=12)
        markers = build_curved_static_lumen_markers(geometry, "abc123", "base_link", config, Time())
        self.assertEqual(CURVED_STATIC_LUMEN_MARKER_KEYS, marker_keys(markers))
        expected_types = {
            ("lumen_centerline", 0): Marker.LINE_STRIP,
            ("lumen_wireframe", 0): Marker.LINE_LIST,
            ("lumen_wireframe", 1): Marker.LINE_LIST,
            ("lumen_wireframe", 2): Marker.LINE_LIST,
            ("lumen_wireframe", 3): Marker.LINE_LIST,
        }
        for marker in markers:
            self.assertEqual("base_link", marker.header.frame_id)
            self.assertEqual(expected_types[(marker.ns, marker.id)], marker.type)
            self.assertEqual(Marker.ADD, marker.action)
            self.assertGreater(marker.scale.x, 0.0)
            self.assertTrue(np.all(np.isfinite(marker_points(marker))))
            self.assertNotIn(marker.ns, {"lumen_closest_pair", "lumen_status", "collision_status", "safety_status"})

    def test_centerline_preserves_order_and_includes_final_point_after_stride(self):
        geometry = circular_arc_geometry()
        markers = build_curved_static_lumen_markers(
            geometry,
            "abc123",
            "base_link",
            LumenMarkerConfig(centerline_stride=4, ring_stride=4, ring_segments=8),
            Time(),
        )
        centerline = marker_points(marker_by_namespace(markers, "lumen_centerline"))
        np.testing.assert_allclose(centerline[0], geometry.centerline_points[0], atol=0.0)
        np.testing.assert_allclose(centerline[-1], geometry.centerline_points[-1], atol=0.0)

    def test_physical_and_safety_ring_counts_and_radii(self):
        geometry = s_curve_geometry()
        config = LumenMarkerConfig(centerline_stride=1, ring_stride=5, ring_segments=10)
        markers = build_curved_static_lumen_markers(geometry, "abc123", "base_link", config, Time())
        expected_rings = 1 + len(range(5, geometry.centerline_points.shape[0], 5))
        if (geometry.centerline_points.shape[0] - 1) % 5 != 0:
            expected_rings += 1
        physical = marker_points(marker_by_key(markers, ("lumen_wireframe", 0)))
        safety = marker_points(marker_by_key(markers, ("lumen_wireframe", 1)))
        self.assertEqual(expected_rings, physical.shape[0] // (2 * config.ring_segments))
        self.assertEqual(expected_rings, safety.shape[0] // (2 * config.ring_segments))
        first_physical_radius = np.linalg.norm(physical[0] - geometry.centerline_points[0])
        first_safety_radius = np.linalg.norm(safety[0] - geometry.centerline_points[0])
        self.assertAlmostEqual(float(geometry.radius_profile[0]), first_physical_radius, places=12)
        self.assertAlmostEqual(
            float(geometry.radius_profile[0] - geometry.ctr_outer_radius - geometry.safety_margin),
            first_safety_radius,
            places=12,
        )

    def test_inlet_and_outlet_ring_centers_match_geometry_endpoints(self):
        geometry = circular_arc_geometry()
        markers = build_curved_static_lumen_markers(
            geometry,
            "abc123",
            "base_link",
            LumenMarkerConfig(ring_segments=20),
            Time(),
        )
        inlet = marker_points(marker_by_key(markers, ("lumen_wireframe", 2)))[0::2]
        outlet = marker_points(marker_by_key(markers, ("lumen_wireframe", 3)))[0::2]
        np.testing.assert_allclose(inlet.mean(axis=0), geometry.centerline_points[0], atol=1.0e-12)
        np.testing.assert_allclose(outlet.mean(axis=0), geometry.centerline_points[-1], atol=1.0e-12)

    def test_surface_is_finite_triangle_mesh_using_exact_curved_lumen_radius(self):
        geometry = s_curve_geometry()
        segments = 16
        markers = build_curved_static_lumen_markers(
            geometry,
            "abc123",
            "base_link",
            LumenMarkerConfig(
                ring_segments=segments,
                publish_lumen_surface=True,
                surface_alpha=0.20,
            ),
            Time(),
        )
        self.assertEqual(CURVED_STATIC_LUMEN_MARKER_KEYS_WITH_SURFACE, marker_keys(markers))
        surface = marker_by_key(markers, ("lumen_surface", 0))
        points = marker_points(surface)
        self.assertEqual(Marker.TRIANGLE_LIST, surface.type)
        self.assertEqual((geometry.centerline_points.shape[0] - 1) * segments * 6, len(points))
        self.assertEqual(0, len(points) % 3)
        self.assertTrue(np.all(np.isfinite(points)))
        self.assertAlmostEqual(0.20, surface.color.a)
        self.assertAlmostEqual(
            float(geometry.radius_profile[0]),
            float(np.linalg.norm(points[0] - geometry.centerline_points[0])),
            places=12,
        )

    def test_reference_path_markers_render_exact_single_and_multi_pose_data(self):
        singleton = np.array([[0.01, -0.02, 0.08]], dtype=float)
        singleton_markers = build_reference_path_markers(singleton, "base_link", Time())
        self.assertEqual((('reference_path', 1),), marker_keys(singleton_markers))
        self.assertEqual(Marker.SPHERE_LIST, singleton_markers[0].type)
        np.testing.assert_allclose(marker_points(singleton_markers[0]), singleton)
        self.assertEqual((1.0, 0.0, 1.0, 1.0), (
            singleton_markers[0].color.r,
            singleton_markers[0].color.g,
            singleton_markers[0].color.b,
            singleton_markers[0].color.a,
        ))

        points = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.04]], dtype=float)
        markers = build_reference_path_markers(points, "base_link", Time())
        self.assertEqual((('reference_path', 0), ('reference_path', 1)), marker_keys(markers))
        np.testing.assert_allclose(marker_points(markers[0]), points)
        self.assertAlmostEqual(0.004, markers[0].scale.x)

    def test_actual_tip_history_is_time_decimated_bounded_and_reset_on_time_rollback(self):
        history = BoundedTipTrajectory(max_points=3, minimum_interval=0.05)
        self.assertTrue(history.append([0.0, 0.0, 0.0], 1.0))
        self.assertFalse(history.append([1.0, 0.0, 0.0], 1.01))
        for index in range(1, 5):
            self.assertTrue(history.append([float(index), 0.0, 0.0], 1.0 + 0.1 * index))
        self.assertEqual((3, 3), history.points().shape)
        np.testing.assert_allclose(history.points()[:, 0], [2.0, 3.0, 4.0])
        marker = build_actual_tip_path_marker(history.points(), "base_link", Time())
        self.assertIsNotNone(marker)
        self.assertEqual("actual_tip_path", marker.ns)
        self.assertTrue(history.append([9.0, 0.0, 0.0], 0.5))
        np.testing.assert_allclose(history.points(), [[9.0, 0.0, 0.0]])

    def test_disabled_config_returns_no_markers(self):
        markers = build_curved_static_lumen_markers(
            circular_arc_geometry(),
            "abc123",
            "base_link",
            LumenMarkerConfig(publish_lumen_markers=False),
            Time(),
        )
        self.assertEqual([], markers)

    def test_invalid_marker_inputs_are_rejected(self):
        geometry = circular_arc_geometry()
        with self.assertRaisesRegex(ValueError, "frame_id"):
            build_curved_static_lumen_markers(geometry, "abc123", "map", LumenMarkerConfig(), Time())
        with self.assertRaisesRegex(ValueError, "geometry_fingerprint"):
            build_curved_static_lumen_markers(geometry, "", "base_link", LumenMarkerConfig(), Time())
        with self.assertRaisesRegex(ValueError, "simulation.visualization.ring_segments"):
            build_curved_static_lumen_markers(geometry, "abc123", "base_link", {"ring_segments": 7}, Time())

    def test_cache_key_uses_geometry_and_visual_settings_only(self):
        config = LumenMarkerConfig(centerline_stride=1, ring_stride=4, ring_segments=20, marker_publish_rate=5.0)
        same_content = LumenMarkerConfig(centerline_stride=1, ring_stride=4, ring_segments=20, marker_publish_rate=10.0)
        diagnostic_disabled = LumenMarkerConfig(
            centerline_stride=1,
            ring_stride=4,
            ring_segments=20,
            marker_publish_rate=5.0,
            publish_lumen_diagnostics=False,
        )
        self.assertEqual(static_lumen_cache_key("abc", config), static_lumen_cache_key("abc", same_content))
        self.assertEqual(static_lumen_cache_key("abc", config), static_lumen_cache_key("abc", diagnostic_disabled))
        self.assertNotEqual(static_lumen_cache_key("abc", config), static_lumen_cache_key("def", config))
        self.assertNotEqual(
            static_lumen_cache_key("abc", config),
            static_lumen_cache_key("abc", LumenMarkerConfig(ring_stride=5)),
        )
        self.assertNotIn((1.0, 2.0, 3.0), static_lumen_cache_key("abc", config))

    def test_delete_markers_are_targeted_and_never_delete_all(self):
        deletes = build_static_lumen_delete_markers(
            [("lumen_outlet", 0), ("lumen_centerline", 0), ("lumen_outlet", 0)],
            "base_link",
            Time(),
        )
        self.assertEqual((("lumen_outlet", 0), ("lumen_centerline", 0)), marker_keys(deletes))
        for marker in deletes:
            self.assertEqual(Marker.DELETE, marker.action)
            self.assertNotEqual(Marker.DELETEALL, marker.action)


class DynamicLumenMarkerTest(unittest.TestCase):
    def test_dynamic_markers_have_expected_namespaces_ids_types_and_frame(self):
        markers = build_dynamic_lumen_diagnostic_markers(diagnostic(), Time())
        self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, marker_keys(markers))
        expected_types = {
            ("lumen_closest_pair", 0): Marker.LINE_LIST,
            ("lumen_closest_pair", 1): Marker.SPHERE,
            ("lumen_closest_pair", 2): Marker.SPHERE,
            ("lumen_status", 0): Marker.TEXT_VIEW_FACING,
        }
        for marker in markers:
            self.assertEqual("base_link", marker.header.frame_id)
            self.assertEqual(expected_types[(marker.ns, marker.id)], marker.type)
            self.assertEqual(Marker.ADD, marker.action)
            self.assertGreater(marker.color.a, 0.0)
            self.assertLessEqual(marker.color.a, 1.0)
            self.assertNotIn((marker.ns, marker.id), CURVED_STATIC_LUMEN_MARKER_KEYS)
            self.assertNotIn(marker.ns, {"ctr_backbone", "ctr_tip", "ctr_target", "collision_status", "safety_status"})

    def test_dynamic_line_and_witness_positions_are_exact(self):
        markers = build_dynamic_lumen_diagnostic_markers(diagnostic(), Time())
        line = [marker for marker in markers if marker.ns == "lumen_closest_pair" and marker.id == 0][0]
        backbone = [marker for marker in markers if marker.ns == "lumen_closest_pair" and marker.id == 1][0]
        boundary = [marker for marker in markers if marker.ns == "lumen_closest_pair" and marker.id == 2][0]
        np.testing.assert_allclose(marker_points(line), [[0.0115, 0.0, 0.050], [0.030, 0.0, 0.050]])
        np.testing.assert_allclose(marker_position(backbone), [0.0115, 0.0, 0.050])
        np.testing.assert_allclose(marker_position(boundary), [0.030, 0.0, 0.050])

    def test_dynamic_marker_colors_encode_status(self):
        cases = (
            (STATUS_SAFE, 0.0, 0.8, 0.2),
            (STATUS_MARGIN, 1.0, 0.62, 0.0),
            (STATUS_COLLISION, 1.0, 0.0, 0.0),
        )
        for status, red, green, blue in cases:
            with self.subTest(status=status):
                markers = build_dynamic_lumen_diagnostic_markers(
                    diagnostic(
                        status=status,
                        physical_clearance=-0.001 if status == STATUS_COLLISION else 0.001,
                        safety_clearance=-0.001 if status != STATUS_SAFE else 0.003,
                    ),
                    Time(),
                )
                for marker in markers:
                    self.assertAlmostEqual(red, marker.color.r)
                    self.assertAlmostEqual(green, marker.color.g)
                    self.assertAlmostEqual(blue, marker.color.b)

    def test_status_text_is_compact_and_preserves_negative_values(self):
        markers = build_dynamic_lumen_diagnostic_markers(
            diagnostic(
                status=STATUS_COLLISION,
                constraint_type=CONSTRAINT_OUTLET,
                physical_clearance=-0.004321,
                safety_clearance=-0.006321,
            ),
            Time(),
        )
        text = [marker.text for marker in markers if marker.ns == "lumen_status"][0]
        self.assertIn("state=PHYSICAL_COLLISION", text)
        self.assertIn("physical_clearance=-0.004321 m", text)
        self.assertIn("safety_clearance=-0.006321 m", text)
        self.assertIn("constraint=outlet", text)
        self.assertIn("backbone_index=4", text)
        self.assertNotIn("[[", text)

    def test_inlet_and_outlet_diagnostics_keep_constraint_text(self):
        for constraint_type in (CONSTRAINT_INLET, CONSTRAINT_OUTLET):
            with self.subTest(constraint_type=constraint_type):
                markers = build_dynamic_lumen_diagnostic_markers(
                    diagnostic(status=STATUS_COLLISION, constraint_type=constraint_type, physical_clearance=-0.002),
                    Time(),
                )
                text = [marker.text for marker in markers if marker.ns == "lumen_status"][0]
                self.assertIn(f"constraint={constraint_type}", text)

    def test_unavailable_witness_publishes_status_only(self):
        markers = build_dynamic_lumen_diagnostic_markers(diagnostic(witness_available=False), Time())
        self.assertEqual((("lumen_status", 0),), marker_keys(markers))
        self.assertEqual(Marker.TEXT_VIEW_FACING, markers[0].type)

    def test_unavailable_diagnostic_returns_no_active_markers(self):
        self.assertEqual(
            [],
            build_dynamic_lumen_diagnostic_markers(
                unavailable_lumen_runtime_diagnostic(geometry_mode="curved", reason="disabled"),
                Time(),
            ),
        )

    def test_dynamic_delete_markers_are_targeted_and_never_delete_all(self):
        deletes = build_dynamic_lumen_delete_markers(
            [("lumen_status", 0), ("lumen_closest_pair", 2), ("lumen_status", 0)],
            "base_link",
            Time(),
        )
        self.assertEqual((("lumen_status", 0), ("lumen_closest_pair", 2)), marker_keys(deletes))
        for marker in deletes:
            self.assertEqual(Marker.DELETE, marker.action)
            self.assertNotEqual(Marker.DELETEALL, marker.action)

    def test_dynamic_marker_points_scales_and_positions_are_finite(self):
        markers = build_dynamic_lumen_diagnostic_markers(diagnostic(), Time())
        for marker in markers:
            self.assertTrue(np.isfinite([marker.scale.x, marker.scale.y, marker.scale.z]).all())
            self.assertTrue(np.isfinite([marker.color.r, marker.color.g, marker.color.b, marker.color.a]).all())
            self.assertTrue(np.isfinite(marker_position(marker)).all())
            for point in marker.points:
                self.assertTrue(np.isfinite([point.x, point.y, point.z]).all())

    def test_dynamic_marker_calls_are_deterministic(self):
        first = build_dynamic_lumen_diagnostic_markers(diagnostic(), Time())
        second = build_dynamic_lumen_diagnostic_markers(diagnostic(), Time())
        self.assertEqual(marker_keys(first), marker_keys(second))
        for left, right in zip(first, second):
            self.assertEqual(left.type, right.type)
            self.assertEqual(left.text, right.text)
            np.testing.assert_allclose(marker_points(left), marker_points(right))
            np.testing.assert_allclose(marker_position(left), marker_position(right))


if __name__ == "__main__":
    unittest.main()
