import sys
import unittest
from pathlib import Path

import numpy as np


TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))

from test_simulator_node import (  # noqa: E402
    cylinder_config,
    curved_config,
    no_lumen_config,
    simulator_shell,
    _time_msg,
)

from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.lumen_factory import lumen_geometry_from_config  # noqa: E402
from ctr_sim.lumen_markers import LumenMarkerConfig  # noqa: E402
from ctr_sim.nodes.simulator_node import _point_from_array  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


C3_STATIC_KEYS = {
    ("lumen_centerline", 0),
    ("lumen_physical_boundary", 0),
    ("lumen_safety_boundary", 0),
    ("lumen_inlet", 0),
    ("lumen_outlet", 0),
}

C4_DYNAMIC_KEYS = {
    ("lumen_closest_pair", 0),
    ("lumen_closest_pair", 1),
    ("lumen_closest_pair", 2),
    ("lumen_status", 0),
}

CTR_DYNAMIC_KEYS = {
    ("ctr_backbone", 0),
    ("ctr_tip", 1),
    ("ctr_target", 2),
}

CYLINDER_KEYS = {
    ("cylindrical_lumen", 10),
    ("cylindrical_lumen", 11),
    ("cylindrical_lumen", 12),
    ("cylindrical_lumen", 13),
}

ALL_OWNED_KEYS = C3_STATIC_KEYS | C4_DYNAMIC_KEYS | CTR_DYNAMIC_KEYS | CYLINDER_KEYS


def message_for(node, stamp, backbone: np.ndarray):
    return node._marker_array_msg(stamp, [_point_from_array(point) for point in backbone], backbone)


def curved_wall_backbone(node, radial_offset: float) -> np.ndarray:
    assert isinstance(node.lumen, CurvedLumen)
    index = min(5, node.lumen.centerline_points.shape[0] - 1)
    return np.asarray([node.lumen.centerline_points[index] + np.array([radial_offset, 0.0, 0.0])], dtype=float)


def cylinder_backbone(radial_offset: float) -> np.ndarray:
    return np.asarray([[radial_offset, 0.0, 0.050]], dtype=float)


def active_markers(msg):
    return [marker for marker in msg.markers if marker.action == Marker.ADD]


def active_keys(msg) -> set[tuple[str, int]]:
    return {(marker.ns, marker.id) for marker in active_markers(msg)}


def delete_markers(msg):
    return [marker for marker in msg.markers if marker.action == Marker.DELETE]


def delete_keys(msg) -> set[tuple[str, int]]:
    return {(marker.ns, marker.id) for marker in delete_markers(msg)}


def marker_by_key(msg, key: tuple[str, int]) -> Marker:
    for marker in msg.markers:
        if marker.ns == key[0] and marker.id == key[1] and marker.action == Marker.ADD:
            return marker
    raise AssertionError(f"missing active marker {key}")


def marker_points(marker: Marker) -> np.ndarray:
    return np.asarray([[point.x, point.y, point.z] for point in marker.points], dtype=float)


def marker_position(marker: Marker) -> np.ndarray:
    return np.asarray([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z], dtype=float)


def static_cache_signature(node):
    signature = {}
    for marker in node._static_lumen_markers:
        key = (marker.ns, marker.id)
        signature[key] = {
            "type": marker.type,
            "scale": np.array([marker.scale.x, marker.scale.y, marker.scale.z], dtype=float),
            "pose": marker_position(marker),
            "points": marker_points(marker),
        }
    return signature


def set_diagnostics_enabled(node, enabled: bool) -> None:
    node.lumen_marker_config = LumenMarkerConfig(
        publish_lumen_markers=node.lumen_marker_config.publish_lumen_markers,
        publish_lumen_diagnostics=enabled,
        centerline_stride=node.lumen_marker_config.centerline_stride,
        ring_stride=node.lumen_marker_config.ring_stride,
        ring_segments=node.lumen_marker_config.ring_segments,
        marker_publish_rate=node.lumen_marker_config.marker_publish_rate,
    )


def set_static_lumen_enabled(node, enabled: bool) -> None:
    node.lumen_marker_config = LumenMarkerConfig(
        publish_lumen_markers=enabled,
        publish_lumen_diagnostics=node.lumen_marker_config.publish_lumen_diagnostics,
        centerline_stride=node.lumen_marker_config.centerline_stride,
        ring_stride=node.lumen_marker_config.ring_stride,
        ring_segments=node.lumen_marker_config.ring_segments,
        marker_publish_rate=node.lumen_marker_config.marker_publish_rate,
    )


class LumenSimulationAcceptanceTest(unittest.TestCase):
    def assert_no_delete_all(self, msg) -> None:
        self.assertFalse(any(marker.action == Marker.DELETEALL for marker in msg.markers))

    def assert_finite_active_markers(self, msg) -> None:
        for marker in active_markers(msg):
            self.assertTrue(marker.header.frame_id, (marker.ns, marker.id))
            self.assertTrue(np.isfinite([marker.scale.x, marker.scale.y, marker.scale.z]).all(), marker.ns)
            self.assertTrue(
                np.isfinite([marker.color.r, marker.color.g, marker.color.b, marker.color.a]).all(),
                marker.ns,
            )
            self.assertGreaterEqual(marker.color.a, 0.0, marker.ns)
            self.assertLessEqual(marker.color.a, 1.0, marker.ns)
            if marker.type in (Marker.LINE_STRIP, Marker.LINE_LIST):
                self.assertGreater(marker.scale.x, 0.0, marker.ns)
            elif marker.type in (Marker.SPHERE, Marker.CYLINDER):
                self.assertGreater(marker.scale.x, 0.0, marker.ns)
                self.assertGreater(marker.scale.y, 0.0, marker.ns)
                self.assertGreater(marker.scale.z, 0.0, marker.ns)
            elif marker.type == Marker.TEXT_VIEW_FACING:
                self.assertGreater(marker.scale.z, 0.0, marker.ns)
                self.assertTrue(marker.text)
            self.assertTrue(np.isfinite(marker_position(marker)).all(), marker.ns)
            points = marker_points(marker)
            if points.size:
                self.assertTrue(np.isfinite(points).all(), marker.ns)

    def assert_static_cache_unchanged(self, node, cache_key, build_count, signature) -> None:
        self.assertEqual(cache_key, node._static_lumen_cache_key)
        self.assertEqual(build_count, node._static_lumen_build_count)
        current = static_cache_signature(node)
        self.assertEqual(set(signature), set(current))
        for key, expected in signature.items():
            self.assertEqual(expected["type"], current[key]["type"])
            np.testing.assert_allclose(expected["scale"], current[key]["scale"], rtol=0.0, atol=0.0)
            np.testing.assert_allclose(expected["pose"], current[key]["pose"], rtol=0.0, atol=0.0)
            np.testing.assert_allclose(expected["points"], current[key]["points"], rtol=0.0, atol=0.0)

    def assert_active_keys_exact(self, msg, expected_keys: set[tuple[str, int]]) -> None:
        self.assertEqual(expected_keys, active_keys(msg))
        self.assertTrue(active_keys(msg).issubset(ALL_OWNED_KEYS))
        self.assert_no_delete_all(msg)
        self.assert_finite_active_markers(msg)

    def test_mode_marker_ownership_matrix(self):
        cases = (
            ("none", simulator_shell(no_lumen_config()), None, CTR_DYNAMIC_KEYS),
            ("cylindrical", simulator_shell(cylinder_config()), cylinder_backbone(0.010), CTR_DYNAMIC_KEYS | CYLINDER_KEYS),
            ("circular_arc", simulator_shell(curved_config("circular_arc")), None, CTR_DYNAMIC_KEYS | C3_STATIC_KEYS | C4_DYNAMIC_KEYS),
            ("s_curve", simulator_shell(curved_config("s_curve")), None, CTR_DYNAMIC_KEYS | C3_STATIC_KEYS | C4_DYNAMIC_KEYS),
        )
        for mode, node, explicit_backbone, expected_keys in cases:
            with self.subTest(mode=mode):
                backbone = explicit_backbone if explicit_backbone is not None else curved_wall_backbone(node, 0.006) if mode != "none" else cylinder_backbone(0.0)
                msg = message_for(node, _time_msg(1.0), backbone)
                self.assert_active_keys_exact(msg, expected_keys)
                if mode == "cylindrical":
                    self.assertEqual(
                        {
                            ("cylindrical_lumen", 10): Marker.CYLINDER,
                            ("cylindrical_lumen", 11): Marker.CYLINDER,
                            ("cylindrical_lumen", 12): Marker.LINE_STRIP,
                            ("cylindrical_lumen", 13): Marker.SPHERE,
                        },
                        {key: marker_by_key(msg, key).type for key in CYLINDER_KEYS},
                    )

    def test_static_dynamic_ctr_and_cylinder_keys_are_disjoint_and_stable(self):
        self.assertTrue(C3_STATIC_KEYS.isdisjoint(C4_DYNAMIC_KEYS))
        self.assertTrue(C3_STATIC_KEYS.isdisjoint(CTR_DYNAMIC_KEYS))
        self.assertTrue(C4_DYNAMIC_KEYS.isdisjoint(CTR_DYNAMIC_KEYS))
        self.assertTrue((C3_STATIC_KEYS | C4_DYNAMIC_KEYS).isdisjoint(CYLINDER_KEYS))

        node = simulator_shell(curved_config("circular_arc"))
        statuses = (
            curved_wall_backbone(node, 0.006),
            curved_wall_backbone(node, 0.0278),
            curved_wall_backbone(node, 0.0320),
        )
        dynamic_key_sets = []
        for index, backbone in enumerate(statuses):
            msg = message_for(node, _time_msg(1.0 + 0.05 * index), backbone)
            keys = active_keys(msg)
            self.assertTrue(C4_DYNAMIC_KEYS.issubset(keys))
            self.assertFalse(keys & CYLINDER_KEYS)
            self.assertTrue(keys.issubset(CTR_DYNAMIC_KEYS | C3_STATIC_KEYS | C4_DYNAMIC_KEYS))
            dynamic_key_sets.append(keys & C4_DYNAMIC_KEYS)
        self.assertEqual([C4_DYNAMIC_KEYS, C4_DYNAMIC_KEYS, C4_DYNAMIC_KEYS], dynamic_key_sets)

    def test_dynamic_status_transitions_preserve_static_cache_geometry(self):
        node = simulator_shell(curved_config("circular_arc"))
        safe = curved_wall_backbone(node, 0.006)
        margin = curved_wall_backbone(node, 0.0278)
        collision = curved_wall_backbone(node, 0.0320)

        first = message_for(node, _time_msg(1.0), safe)
        self.assertTrue(C3_STATIC_KEYS.issubset(active_keys(first)))
        cache_key = node._static_lumen_cache_key
        build_count = node._static_lumen_build_count
        signature = static_cache_signature(node)

        messages = [
            first,
            message_for(node, _time_msg(1.05), margin),
            message_for(node, _time_msg(1.10), collision),
            message_for(node, _time_msg(1.15), safe),
        ]
        expected_states = (
            "state=SAFE",
            "state=SAFETY_MARGIN_VIOLATION",
            "state=PHYSICAL_COLLISION",
            "state=SAFE",
        )
        expected_colors = (
            (0.0, 0.8, 0.2),
            (1.0, 0.62, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.8, 0.2),
        )
        for msg, expected_state, expected_color in zip(messages, expected_states, expected_colors):
            self.assertEqual(C4_DYNAMIC_KEYS, active_keys(msg) & C4_DYNAMIC_KEYS)
            status = marker_by_key(msg, ("lumen_status", 0))
            self.assertIn(expected_state, status.text)
            self.assertEqual(expected_color, (status.color.r, status.color.g, status.color.b))
            self.assert_no_delete_all(msg)

        self.assertNotIn("PHYSICAL_COLLISION", marker_by_key(messages[-1], ("lumen_status", 0)).text)
        self.assert_static_cache_unchanged(node, cache_key, build_count, signature)

    def test_diagnostics_disabled_preserves_static_cache_and_deletes_only_dynamic_keys(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = curved_wall_backbone(node, 0.006)
        message_for(node, _time_msg(1.0), backbone)
        cache_key = node._static_lumen_cache_key
        build_count = node._static_lumen_build_count
        signature = static_cache_signature(node)
        diagnostic_count = node._lumen_diagnostic_update_count

        set_diagnostics_enabled(node, False)
        msg = message_for(node, _time_msg(1.25), backbone)

        self.assertTrue(C3_STATIC_KEYS.issubset(active_keys(msg)))
        self.assertFalse(active_keys(msg) & C4_DYNAMIC_KEYS)
        self.assertEqual(C4_DYNAMIC_KEYS, delete_keys(msg))
        self.assertFalse(delete_keys(msg) & C3_STATIC_KEYS)
        self.assertFalse(delete_keys(msg) & CTR_DYNAMIC_KEYS)
        self.assertEqual(diagnostic_count, node._lumen_diagnostic_update_count)
        self.assert_static_cache_unchanged(node, cache_key, build_count, signature)
        self.assert_no_delete_all(msg)

        repeat = message_for(node, _time_msg(1.50), backbone)
        self.assertEqual(set(), delete_keys(repeat))

    def test_static_visualization_disabled_deletes_curved_lumen_keys_only(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = curved_wall_backbone(node, 0.006)
        message_for(node, _time_msg(1.0), backbone)
        diagnostic_count = node._lumen_diagnostic_update_count

        set_static_lumen_enabled(node, False)
        msg = message_for(node, _time_msg(1.10), backbone)

        self.assertEqual(CTR_DYNAMIC_KEYS, active_keys(msg))
        self.assertEqual(C3_STATIC_KEYS | C4_DYNAMIC_KEYS, delete_keys(msg))
        self.assertFalse(delete_keys(msg) & CTR_DYNAMIC_KEYS)
        self.assertFalse(delete_keys(msg) & CYLINDER_KEYS)
        self.assertEqual(diagnostic_count, node._lumen_diagnostic_update_count)
        self.assertEqual((), node._static_lumen_marker_keys)
        self.assertEqual((), node._dynamic_lumen_marker_keys)
        self.assert_no_delete_all(msg)

    def test_curved_to_cylinder_cleanup_preserves_cylinder_marker_compatibility(self):
        node = simulator_shell(curved_config("circular_arc"))
        message_for(node, _time_msg(1.0), curved_wall_backbone(node, 0.006))

        config = cylinder_config()
        node.config = config
        node.lumen_mode = "cylindrical"
        node.lumen_geometry = lumen_geometry_from_config(config)
        node.lumen = node.lumen_geometry
        msg = message_for(node, _time_msg(1.10), cylinder_backbone(0.010))

        self.assertEqual(C3_STATIC_KEYS | C4_DYNAMIC_KEYS, delete_keys(msg))
        self.assertFalse(active_keys(msg) & (C3_STATIC_KEYS | C4_DYNAMIC_KEYS))
        self.assertTrue(CYLINDER_KEYS.issubset(active_keys(msg)))
        self.assertTrue(CTR_DYNAMIC_KEYS.issubset(active_keys(msg)))
        self.assert_no_delete_all(msg)
        self.assertEqual(
            {
                ("cylindrical_lumen", 10): Marker.CYLINDER,
                ("cylindrical_lumen", 11): Marker.CYLINDER,
                ("cylindrical_lumen", 12): Marker.LINE_STRIP,
                ("cylindrical_lumen", 13): Marker.SPHERE,
            },
            {key: marker_by_key(msg, key).type for key in CYLINDER_KEYS},
        )

        cases = (
            (cylinder_backbone(0.010), (0.0, 0.8, 0.2)),
            (cylinder_backbone(0.0278), (1.0, 0.6, 0.0)),
            (cylinder_backbone(0.0320), (1.0, 0.0, 0.0)),
        )
        for index, (backbone, expected_color) in enumerate(cases):
            with self.subTest(expected_color=expected_color):
                current = message_for(node, _time_msg(1.20 + 0.1 * index), backbone)
                closest = marker_by_key(current, ("cylindrical_lumen", 13))
                self.assertEqual(Marker.SPHERE, closest.type)
                self.assertEqual(expected_color, (closest.color.r, closest.color.g, closest.color.b))
                np.testing.assert_allclose(marker_position(closest), backbone[0], atol=0.0)
                self.assertFalse(active_keys(current) & (C3_STATIC_KEYS | C4_DYNAMIC_KEYS))
                self.assert_no_delete_all(current)

    def test_curved_to_no_lumen_cleanup_deletes_once_and_preserves_ctr_markers(self):
        node = simulator_shell(curved_config("s_curve"))
        backbone = curved_wall_backbone(node, 0.006)
        message_for(node, _time_msg(1.0), backbone)

        node.config = no_lumen_config()
        node.lumen_mode = "none"
        node.lumen_geometry = None
        node.lumen = None
        msg = message_for(node, _time_msg(1.10), cylinder_backbone(0.0))

        self.assertEqual(CTR_DYNAMIC_KEYS, active_keys(msg))
        self.assertEqual(C3_STATIC_KEYS | C4_DYNAMIC_KEYS, delete_keys(msg))
        self.assertEqual((), node._static_lumen_marker_keys)
        self.assertEqual((), node._dynamic_lumen_marker_keys)
        self.assert_no_delete_all(msg)
        self.assert_finite_active_markers(msg)

        repeat = message_for(node, _time_msg(1.20), cylinder_backbone(0.0))
        self.assertEqual(CTR_DYNAMIC_KEYS, active_keys(repeat))
        self.assertEqual(set(), delete_keys(repeat))
        self.assert_no_delete_all(repeat)


if __name__ == "__main__":
    unittest.main()
