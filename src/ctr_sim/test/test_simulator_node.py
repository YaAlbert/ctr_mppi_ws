import copy
import sys
import types
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))


try:
    import ctr_interfaces.msg as ctr_interfaces_msg_module  # noqa: F401
    for required_name in ("CtrBackbone", "CtrJointCommand", "CtrJointState", "CtrState"):
        if not hasattr(ctr_interfaces_msg_module, required_name):
            raise ImportError(required_name)
except ImportError:
    ctr_interfaces_module = types.ModuleType("ctr_interfaces")
    ctr_interfaces_msg_module = types.ModuleType("ctr_interfaces.msg")
    for name in ("CtrBackbone", "CtrJointCommand", "CtrJointState", "CtrState"):
        setattr(ctr_interfaces_msg_module, name, type(name, (), {}))
    sys.modules["ctr_interfaces"] = ctr_interfaces_module
    sys.modules["ctr_interfaces.msg"] = ctr_interfaces_msg_module


from builtin_interfaces.msg import Time  # noqa: E402
from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402
from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_lumen_overrides,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)
from ctr_sim.lumen_markers import (  # noqa: E402
    CURVED_STATIC_LUMEN_MARKER_KEYS,
    DYNAMIC_LUMEN_MARKER_KEYS,
    LumenMarkerConfig,
)
from ctr_sim.lumen_diagnostics import STATUS_COLLISION, STATUS_MARGIN  # noqa: E402
import ctr_sim.nodes.simulator_node as simulator_node_module  # noqa: E402
from ctr_sim.nodes.simulator_node import CTRSimulatorNode, _point_from_array  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def load_config():
    return copy.deepcopy(load_parameter_files(CONFIG_FILES))


def no_lumen_config():
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=False,
    )


def cylinder_config():
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=True,
        enable_curved_lumen=False,
    )


def curved_config(lumen_type="circular_arc"):
    return config_with_lumen_overrides(
        load_config(),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type=lumen_type,
    )


def simulator_shell(config):
    node = object.__new__(CTRSimulatorNode)
    node.config = config
    node.frame_id = config["robot"]["frames"]["base"]
    node.target_position = np.asarray(config["goal"]["position"], dtype=float)
    node.lumen_mode = lumen_mode_from_config(config)
    node.lumen_geometry = lumen_geometry_from_config(config)
    node.lumen = node.lumen_geometry
    node.lumen_marker_config = LumenMarkerConfig.from_mapping(config["simulation"].get("visualization", {}))
    node._static_lumen_cache_key = None
    node._static_lumen_markers = []
    node._static_lumen_marker_keys = ()
    node._static_lumen_marker_frame_id = node.frame_id
    node._last_static_lumen_publish_time_s = None
    node._static_lumen_build_count = 0
    node._static_lumen_cache_hit_logged = False
    node._dynamic_lumen_marker_keys = ()
    node._dynamic_lumen_marker_frame_id = node.frame_id
    node._last_lumen_diagnostic_log_signature = None
    node._lumen_diagnostic_update_count = 0
    return node


def sample_backbone():
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [0.003, 0.0, 0.040],
            [0.006, 0.0, 0.080],
        ],
        dtype=float,
    )


class SimulatorNodeLumenRuntimeTest(unittest.TestCase):
    def test_simulator_source_uses_shared_lumen_factory(self):
        source = (PACKAGE_ROOT / "ctr_sim" / "nodes" / "simulator_node.py").read_text(encoding="utf-8")
        self.assertIn("config_with_lumen_overrides", source)
        self.assertIn("lumen_geometry_from_config", source)
        self.assertIn("lumen_mode_from_config", source)
        self.assertIn("parse_launch_bool", source)
        self.assertNotIn("config_with_cylinder_overrides", source)

    def test_factory_modes_construct_expected_geometry_for_simulator(self):
        cases = (
            (no_lumen_config(), "none", type(None)),
            (cylinder_config(), "cylindrical", CylindricalLumen),
            (curved_config("circular_arc"), "curved", CurvedLumen),
            (curved_config("s_curve"), "curved", CurvedLumen),
        )
        for config, expected_mode, expected_type in cases:
            with self.subTest(expected_mode=expected_mode, expected_type=expected_type):
                self.assertEqual(expected_mode, lumen_mode_from_config(config))
                self.assertIsInstance(lumen_geometry_from_config(config), expected_type)

    def test_no_lumen_marker_array_keeps_geometry_independent_markers(self):
        node = simulator_shell(no_lumen_config())
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], [marker.ns for marker in msg.markers])

    def test_curved_marker_array_publishes_static_and_dynamic_diagnostic_markers(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        namespaces = [marker.ns for marker in msg.markers]
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], namespaces[:3])
        self.assertEqual(
            [key[0] for key in CURVED_STATIC_LUMEN_MARKER_KEYS],
            namespaces[3 : 3 + len(CURVED_STATIC_LUMEN_MARKER_KEYS)],
        )
        self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, tuple((marker.ns, marker.id) for marker in msg.markers[-4:]))
        self.assertNotIn("cylindrical_lumen", namespaces)
        self.assertIn("lumen_closest_pair", namespaces)
        self.assertIn("lumen_status", namespaces)

    def test_cylindrical_marker_array_retains_existing_lumen_markers(self):
        node = simulator_shell(cylinder_config())
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        namespaces = [marker.ns for marker in msg.markers]
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], namespaces[:3])
        self.assertGreater(namespaces.count("cylindrical_lumen"), 0)
        cylinder_markers = [marker for marker in msg.markers if marker.ns == "cylindrical_lumen"]
        self.assertEqual([10, 11, 12, 13], [marker.id for marker in cylinder_markers])
        self.assertEqual(
            [Marker.CYLINDER, Marker.CYLINDER, Marker.LINE_STRIP, Marker.SPHERE],
            [marker.type for marker in cylinder_markers],
        )
        for namespace, _marker_id in CURVED_STATIC_LUMEN_MARKER_KEYS:
            self.assertNotIn(namespace, namespaces)
        for namespace, _marker_id in DYNAMIC_LUMEN_MARKER_KEYS:
            self.assertNotIn(namespace, namespaces)

    def test_cylindrical_marker_13_color_semantics_are_preserved(self):
        node = simulator_shell(cylinder_config())
        cases = (
            (np.array([[0.010, 0.0, 0.050]], dtype=float), (0.0, 0.8, 0.2)),
            (np.array([[0.0278, 0.0, 0.050]], dtype=float), (1.0, 0.6, 0.0)),
            (np.array([[0.0320, 0.0, 0.050]], dtype=float), (1.0, 0.0, 0.0)),
        )
        for backbone, expected_color in cases:
            with self.subTest(expected_color=expected_color):
                msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
                closest = [marker for marker in msg.markers if marker.ns == "cylindrical_lumen" and marker.id == 13][0]
                self.assertEqual(Marker.SPHERE, closest.type)
                self.assertEqual(expected_color, (closest.color.r, closest.color.g, closest.color.b))
                np.testing.assert_allclose(
                    [closest.pose.position.x, closest.pose.position.y, closest.pose.position.z],
                    backbone[0],
                )

    def test_curved_static_markers_are_cached_and_republished_at_bounded_rate(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()

        first = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        first_namespaces = [marker.ns for marker in first.markers]
        self.assertIn("lumen_centerline", first_namespaces)
        self.assertEqual(1, node._static_lumen_build_count)

        second = node._marker_array_msg(_time_msg(1.05), [_point_from_array(point) for point in backbone], backbone)
        second_namespaces = [marker.ns for marker in second.markers]
        self.assertNotIn("lumen_centerline", second_namespaces)
        self.assertEqual(1, node._static_lumen_build_count)

        node.target_position = np.array([0.001, 0.002, 0.003], dtype=float)
        third = node._marker_array_msg(_time_msg(1.25), [_point_from_array(point) for point in backbone], backbone)
        third_namespaces = [marker.ns for marker in third.markers]
        self.assertIn("lumen_centerline", third_namespaces)
        self.assertEqual(1, node._static_lumen_build_count)
        self.assertEqual(3, node._lumen_diagnostic_update_count)

    def test_curved_static_marker_points_are_finite(self):
        node = simulator_shell(curved_config("s_curve"))
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        for marker in msg.markers:
            for point in marker.points:
                self.assertTrue(np.isfinite([point.x, point.y, point.z]).all(), marker.ns)
            self.assertTrue(np.isfinite([marker.scale.x, marker.scale.y, marker.scale.z]).all(), marker.ns)

    def test_visualization_disabled_suppresses_curved_static_markers(self):
        config = curved_config("circular_arc")
        config["simulation"]["visualization"]["publish_lumen_markers"] = False
        node = simulator_shell(config)
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        self.assertEqual(["ctr_backbone", "ctr_tip", "ctr_target"], [marker.ns for marker in msg.markers])

    def test_lumen_diagnostics_disabled_keeps_static_markers_and_dynamic_markers_absent(self):
        config = curved_config("circular_arc")
        config["simulation"]["visualization"]["publish_lumen_diagnostics"] = False
        node = simulator_shell(config)
        backbone = sample_backbone()
        msg = node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        keys = tuple((marker.ns, marker.id) for marker in msg.markers)
        for key in CURVED_STATIC_LUMEN_MARKER_KEYS:
            self.assertIn(key, keys)
        for key in DYNAMIC_LUMEN_MARKER_KEYS:
            self.assertNotIn(key, keys)
        self.assertEqual(0, node._lumen_diagnostic_update_count)

    def test_no_lumen_clears_stale_curved_static_markers_with_targeted_deletes(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)

        node.lumen_mode = "none"
        node.lumen = None
        msg = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        deletes = [marker for marker in msg.markers if marker.action == Marker.DELETE]
        self.assertEqual(
            CURVED_STATIC_LUMEN_MARKER_KEYS + DYNAMIC_LUMEN_MARKER_KEYS,
            tuple((marker.ns, marker.id) for marker in deletes),
        )
        self.assertTrue(all(marker.action != Marker.DELETEALL for marker in deletes))

    def test_cylindrical_mode_clears_prior_curved_static_markers_without_delete_all(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)

        cylinder = lumen_geometry_from_config(cylinder_config())
        node.config = cylinder_config()
        node.lumen_mode = "cylindrical"
        node.lumen = cylinder
        node.lumen_geometry = cylinder
        msg = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        deletes = [marker for marker in msg.markers if marker.action == Marker.DELETE]
        self.assertEqual(
            CURVED_STATIC_LUMEN_MARKER_KEYS + DYNAMIC_LUMEN_MARKER_KEYS,
            tuple((marker.ns, marker.id) for marker in deletes),
        )
        self.assertTrue(any(marker.ns == "cylindrical_lumen" and marker.id == 10 for marker in msg.markers))

    def test_dynamic_diagnostic_reuses_full_fk_backbone_without_static_cache_rebuild(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        calls = []
        original = simulator_node_module.build_lumen_runtime_diagnostic

        def spy(lumen, points, mode):
            calls.append((lumen, np.asarray(points).copy(), mode))
            return original(lumen, points, mode)

        simulator_node_module.build_lumen_runtime_diagnostic = spy
        try:
            first = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
            second = node._marker_array_msg(_time_msg(1.01), [_point_from_array(point) for point in backbone], backbone)
        finally:
            simulator_node_module.build_lumen_runtime_diagnostic = original

        self.assertEqual(2, len(calls))
        np.testing.assert_allclose(calls[0][1], backbone)
        self.assertEqual("curved", calls[0][2])
        self.assertEqual(1, node._static_lumen_build_count)
        self.assertIn("lumen_centerline", [marker.ns for marker in first.markers])
        self.assertNotIn("lumen_centerline", [marker.ns for marker in second.markers])
        self.assertIn("lumen_status", [marker.ns for marker in second.markers])

    def test_dynamic_status_transitions_reuse_stable_marker_ids(self):
        node = simulator_shell(curved_config("circular_arc"))
        lumen = node.lumen
        assert isinstance(lumen, CurvedLumen)
        safe = np.asarray([lumen.centerline_points[5] + np.array([0.006, 0.0, 0.0])], dtype=float)
        margin = np.asarray([lumen.centerline_points[5] + np.array([0.0278, 0.0, 0.0])], dtype=float)
        collision = np.asarray([lumen.centerline_points[5] + np.array([0.032, 0.0, 0.0])], dtype=float)
        messages = [
            node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in safe], safe),
            node._marker_array_msg(_time_msg(1.1), [_point_from_array(point) for point in margin], margin),
            node._marker_array_msg(_time_msg(1.2), [_point_from_array(point) for point in collision], collision),
            node._marker_array_msg(_time_msg(1.3), [_point_from_array(point) for point in safe], safe),
        ]
        for msg in messages:
            self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, tuple((marker.ns, marker.id) for marker in msg.markers[-4:]))
        status_text = [[marker.text for marker in msg.markers if marker.ns == "lumen_status"][0] for msg in messages]
        self.assertIn(STATUS_MARGIN, status_text[1])
        self.assertIn(STATUS_COLLISION, status_text[2])
        self.assertNotIn(STATUS_COLLISION, status_text[3])

    def test_dynamic_generation_failure_deletes_stale_keys_and_keeps_static_markers(self):
        node = simulator_shell(curved_config("circular_arc"))
        backbone = sample_backbone()
        node._marker_array_msg(Time(), [_point_from_array(point) for point in backbone], backbone)
        original = simulator_node_module.build_lumen_runtime_diagnostic

        def failing(*_args, **_kwargs):
            raise ValueError("diagnostic failure")

        simulator_node_module.build_lumen_runtime_diagnostic = failing
        try:
            msg = node._marker_array_msg(_time_msg(1.0), [_point_from_array(point) for point in backbone], backbone)
        finally:
            simulator_node_module.build_lumen_runtime_diagnostic = original

        deletes = tuple((marker.ns, marker.id) for marker in msg.markers if marker.action == Marker.DELETE)
        self.assertEqual(DYNAMIC_LUMEN_MARKER_KEYS, deletes)
        self.assertIn("lumen_centerline", [marker.ns for marker in msg.markers])


def _time_msg(seconds: float) -> Time:
    stamp = Time()
    stamp.sec = int(seconds)
    stamp.nanosec = int(round((seconds - stamp.sec) * 1.0e9))
    return stamp


if __name__ == "__main__":
    unittest.main()
