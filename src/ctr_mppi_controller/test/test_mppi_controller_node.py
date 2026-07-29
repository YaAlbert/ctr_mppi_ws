import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))


try:
    import ctr_interfaces.msg as ctr_interfaces_msg_module  # noqa: F401
    for required_name in ("CtrControllerMetrics", "CtrJointCommand", "CtrState"):
        if not hasattr(ctr_interfaces_msg_module, required_name):
            raise ImportError(required_name)
except ImportError:
    ctr_interfaces_module = types.ModuleType("ctr_interfaces")
    ctr_interfaces_msg_module = types.ModuleType("ctr_interfaces.msg")
    class CtrControllerMetrics:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")

    class CtrJointCommand:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.q_dot = []
            self.valid = False
            self.diagnostic_status = ""

    ctr_interfaces_msg_module.CtrControllerMetrics = CtrControllerMetrics
    ctr_interfaces_msg_module.CtrJointCommand = CtrJointCommand
    ctr_interfaces_msg_module.CtrState = type("CtrState", (), {})
    sys.modules["ctr_interfaces"] = ctr_interfaces_module
    sys.modules["ctr_interfaces.msg"] = ctr_interfaces_msg_module


from ctr_mppi_controller.nodes.mppi_controller_node import (  # noqa: E402
    MPPIControllerNode,
    active_reference_point,
    reference_mode_from_config,
    reference_type_from_config,
    should_publish_metrics,
    solve_reference_kwargs,
    target_sequence_from_path,
    trajectory_metrics_diagnostic_array,
)
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.nodes.reference_manager_node import path_from_points, pose_from_point  # noqa: E402
from ctr_mppi_controller.reference_validation import (  # noqa: E402
    EXTERNAL_TARGET,
    FIXED_TARGET,
    INVALID_REFERENCE,
    NO_ACTIVE_REFERENCE,
    SOURCE_TRAJECTORY_HORIZON,
    TRAJECTORY,
    VALID_REFERENCE,
    accept_point_reference,
    initial_reference,
    initial_trajectory_reference,
    reference_state_log_line,
    validate_reference_point,
)
from ctr_mppi_controller.trajectory_metrics import TrajectoryMetricsAccumulator, TrajectoryMetricsConfig  # noqa: E402


def horizon_path(points, *, frame_id="base_link", stamp_s=10):
    path = path_from_points(points, frame_id)
    path.header.stamp.sec = int(stamp_s)
    path.header.stamp.nanosec = int((float(stamp_s) - int(stamp_s)) * 1.0e9)
    for pose in path.poses:
        pose.header.stamp = path.header.stamp
    return path


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(str(message))

    def warn(self, message):
        self.warnings.append(str(message))

    def error(self, message):
        self.errors.append(str(message))


class FakeTime:
    def __init__(self, seconds=10.0):
        self.nanoseconds = int(seconds * 1.0e9)

    def to_msg(self):
        return SimpleNamespace(sec=int(self.nanoseconds // 1_000_000_000), nanosec=int(self.nanoseconds % 1_000_000_000))


class FakeClock:
    def __init__(self, seconds=10.0):
        self.seconds = seconds

    def now(self):
        return FakeTime(self.seconds)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeCore:
    def __init__(self):
        self.solve_calls = 0
        self.solve_kwargs = []
        self.horizon = 3
        self.dt = 0.05
        self.control_dimension = 6

    def solve(self, **kwargs):
        self.solve_calls += 1
        self.solve_kwargs.append(kwargs)
        return SimpleNamespace(
            command=np.zeros(6),
            solve_time=0.001,
            minimum_cost=0.0,
            mean_cost=0.0,
            effective_sample_weight=1.0,
            command_magnitude=0.0,
            command_saturated=False,
            diagnostic_status="ok",
        )


def controller_shell(*, mode=EXTERNAL_TARGET, lumen_geometry=None):
    node = object.__new__(MPPIControllerNode)
    node.reference_mode = mode
    node.reference_frame_id = "base_link"
    node.reference_stale_timeout = 0.20
    node.lumen_geometry = lumen_geometry
    node.active_reference = initial_reference(mode)
    node.active_trajectory_reference = initial_trajectory_reference()
    node.target_tip = np.zeros(3)
    node.latest_reference_horizon = None
    node.latest_reference_horizon_stamp_s = None
    node.latest_state = SimpleNamespace(q=[0.0] * 6, q_dot=[0.0] * 6)
    node.core = FakeCore()
    node.command_pub = FakePublisher()
    node.safe_command_pub = FakePublisher()
    node.metrics_pub = FakePublisher()
    node.trajectory_metrics_pub = FakePublisher()
    node.frame_id = "base_link"
    node.config = {"mppi": {"control_frequency": 20.0}}
    node.tracking_metrics_config = SimpleNamespace(enabled=False)
    node._warned_missing_horizon = False
    node._warned_missing_reference = False
    node._last_invalid_reference_warning_s = None
    node._fake_logger = FakeLogger()
    node.get_logger = lambda: node._fake_logger
    node.get_clock = lambda: FakeClock()
    return node


def target_pose(point, frame_id="base_link", stamp_s=0):
    msg = pose_from_point([0.0, 0.0, 0.0], frame_id)
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = int((float(stamp_s) - int(stamp_s)) * 1.0e9)
    msg.pose.position.x = float(point[0])
    msg.pose.position.y = float(point[1])
    msg.pose.position.z = float(point[2])
    return msg


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


def fixed_init_shell(*, position=(0.015, 0.005, 0.100), goal_frame="base_link", reference_frame="base_link", lumen_geometry=None):
    node = object.__new__(MPPIControllerNode)
    node.reference_mode = FIXED_TARGET
    node.reference_frame_id = reference_frame
    node.lumen_geometry = lumen_geometry
    node.active_reference = initial_reference(FIXED_TARGET)
    node.target_tip = np.zeros(3)
    node.config = {
        "goal": {
            "position": list(position),
            "frame_id": goal_frame,
        },
    }
    return node


class MPPIControllerNodeHelpersTest(unittest.TestCase):
    def test_path_to_numpy_conversion(self):
        points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]])
        sequence, stamp_s = target_sequence_from_path(
            horizon_path(points),
            expected_horizon=3,
            expected_frame_id="base_link",
            current_time_s=10.05,
            stale_timeout=0.20,
        )
        self.assertEqual((3, 3), sequence.shape)
        self.assertTrue(np.allclose(points, sequence))
        self.assertAlmostEqual(10.0, stamp_s)

    def test_exact_horizon_point_count_is_required(self):
        points = np.zeros((2, 3))
        with self.assertRaises(ValueError):
            target_sequence_from_path(
                horizon_path(points),
                expected_horizon=3,
                expected_frame_id="base_link",
                current_time_s=10.0,
                stale_timeout=0.20,
            )

    def test_nan_pose_is_rejected(self):
        points = np.array([[0.0, 0.0, 0.0], [np.nan, 0.0, 0.0], [0.0, 0.0, 0.0]])
        with self.assertRaises(ValueError):
            target_sequence_from_path(
                horizon_path(points),
                expected_horizon=3,
                expected_frame_id="base_link",
                current_time_s=10.0,
                stale_timeout=0.20,
            )

    def test_frame_mismatch_is_rejected(self):
        points = np.zeros((3, 3))
        with self.assertRaises(ValueError):
            target_sequence_from_path(
                horizon_path(points, frame_id="world"),
                expected_horizon=3,
                expected_frame_id="base_link",
                current_time_s=10.0,
                stale_timeout=0.20,
            )

    def test_pose_frame_mismatch_is_rejected(self):
        points = np.zeros((3, 3))
        path = horizon_path(points)
        path.poses[1].header.frame_id = "world"
        with self.assertRaises(ValueError):
            target_sequence_from_path(
                path,
                expected_horizon=3,
                expected_frame_id="base_link",
                current_time_s=10.0,
                stale_timeout=0.20,
            )

    def test_stale_horizon_is_rejected(self):
        points = np.zeros((3, 3))
        with self.assertRaises(ValueError):
            target_sequence_from_path(
                horizon_path(points, stamp_s=10.0),
                expected_horizon=3,
                expected_frame_id="base_link",
                current_time_s=10.30,
                stale_timeout=0.20,
            )

    def test_fixed_target_behavior_uses_target_tip(self):
        target = np.array([0.0192, 0.0, 0.08])
        sequence = np.ones((3, 3))
        kwargs = solve_reference_kwargs(
            reference_mode="fixed_target",
            target_tip=target,
            target_tip_sequence=sequence,
            horizon_stamp_s=10.0,
            current_time_s=10.30,
            stale_timeout=0.20,
        )
        self.assertEqual(["target_tip"], list(kwargs.keys()))
        self.assertTrue(np.allclose(target, kwargs["target_tip"]))

    def test_trajectory_mode_uses_target_tip_sequence(self):
        target = np.array([0.0192, 0.0, 0.08])
        sequence = np.ones((3, 3))
        kwargs = solve_reference_kwargs(
            reference_mode="trajectory",
            target_tip=target,
            target_tip_sequence=sequence,
            horizon_stamp_s=10.0,
            current_time_s=10.05,
            stale_timeout=0.20,
        )
        self.assertEqual(["target_tip_sequence"], list(kwargs.keys()))
        self.assertTrue(np.allclose(sequence, kwargs["target_tip_sequence"]))

    def test_trajectory_mode_requires_valid_horizon(self):
        with self.assertRaises(ValueError):
            solve_reference_kwargs(
                reference_mode="trajectory",
                target_tip=np.zeros(3),
                target_tip_sequence=None,
                horizon_stamp_s=None,
                current_time_s=10.0,
                stale_timeout=0.20,
            )

    def test_reference_mode_override_validation(self):
        config = {"reference": {"mode": "fixed_target"}}
        self.assertEqual("fixed_target", reference_mode_from_config(config, ""))
        self.assertEqual("trajectory", reference_mode_from_config(config, "trajectory"))
        self.assertEqual("external_target", reference_mode_from_config(config, "external_target"))
        with self.assertRaises(ValueError):
            reference_mode_from_config(config, "invalid")

    def test_controller_applies_reference_override_before_config_validation(self):
        source = (PACKAGE_ROOT / "ctr_mppi_controller" / "nodes" / "mppi_controller_node.py").read_text(encoding="utf-8")
        self.assertLess(
            source.index("reference_config = project_config_with_overrides("),
            source.index("validate_or_raise(self.config)"),
        )
        self.assertLess(
            source.index("validate_or_raise(self.config)"),
            source.index('self.reference_mode = reference_mode_from_config(self.config, "")'),
        )

    def test_reference_type_override_validation(self):
        config = {"reference": {"trajectory_type": "circle"}}
        self.assertEqual("circle", reference_type_from_config(config, ""))
        self.assertEqual("helix", reference_type_from_config(config, "helix"))
        with self.assertRaises(ValueError):
            reference_type_from_config(config, "square")

    def test_active_reference_point_selects_fixed_or_sequence(self):
        fixed = np.array([0.1, 0.2, 0.3])
        self.assertTrue(np.allclose(fixed, active_reference_point({"target_tip": fixed})))
        sequence = np.array([[0.4, 0.5, 0.6], [0.7, 0.8, 0.9]])
        self.assertTrue(np.allclose(sequence[0], active_reference_point({"target_tip_sequence": sequence})))

    def test_reference_point_validation_preserves_exact_coordinates(self):
        point = np.array([0.015, 0.005, 0.100])
        accepted = validate_reference_point(
            point,
            received_frame="base_link",
            expected_frame="base_link",
            lumen_geometry=None,
            label="external",
        )
        self.assertTrue(np.array_equal(point, accepted))
        accepted[0] = 1.0
        self.assertNotEqual(point[0], accepted[0])

    def test_reference_point_validation_rejects_frame_mismatch(self):
        with self.assertRaisesRegex(ValueError, "frame_id"):
            validate_reference_point(
                [0.0, 0.0, 0.0],
                received_frame="world",
                expected_frame="base_link",
                lumen_geometry=None,
                label="external",
            )

    def test_reference_point_validation_uses_lumen_geometry(self):
        with self.assertRaisesRegex(ValueError, "lumen"):
            validate_reference_point(
                [0.040, 0.0, 0.050],
                received_frame="base_link",
                expected_frame="base_link",
                lumen_geometry=cylinder_lumen(),
                label="external",
            )

    def test_reference_state_log_line_is_machine_parseable(self):
        reference = accept_point_reference(
            initial_reference(EXTERNAL_TARGET),
            source=EXTERNAL_TARGET,
            point=[0.01, 0.02, 0.03],
            frame="base_link",
        )
        line = reference_state_log_line(reference, reason="accepted target")
        self.assertIn("REFERENCE_STATE", line)
        self.assertIn("mode=external_target", line)
        self.assertIn("state=VALID_REFERENCE", line)
        self.assertIn("revision=1", line)
        self.assertNotIn("accepted target", line)

    def test_external_startup_has_no_active_reference(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        self.assertEqual(NO_ACTIVE_REFERENCE, node.active_reference.state)
        self.assertEqual(0, node.active_reference.revision)
        with self.assertRaisesRegex(ValueError, "valid active reference"):
            node._timer_reference_kwargs(current_time_s=10.0)

    def test_fixed_startup_validates_and_activates_goal_position(self):
        node = fixed_init_shell()
        status = node._initialize_active_reference()
        self.assertEqual("fixed_target_valid", status)
        self.assertEqual(VALID_REFERENCE, node.active_reference.state)
        self.assertEqual(FIXED_TARGET, node.active_reference.source)
        self.assertEqual(1, node.active_reference.revision)
        self.assertEqual("base_link", node.active_reference.target_frame)
        self.assertTrue(np.array_equal([0.015, 0.005, 0.100], node.active_reference.target))

    def test_fixed_startup_rejects_nonfinite_goal(self):
        node = fixed_init_shell(position=(0.015, float("nan"), 0.100))
        with self.assertRaisesRegex(ValueError, "configured fixed target"):
            node._initialize_active_reference()

    def test_fixed_startup_rejects_frame_mismatch(self):
        node = fixed_init_shell(goal_frame="world", reference_frame="base_link")
        with self.assertRaisesRegex(ValueError, "frame_id"):
            node._initialize_active_reference()

    def test_fixed_startup_rejects_wall_inlet_and_outlet_invalid_goals(self):
        lumen = cylinder_lumen()
        cases = (
            [0.040, 0.0, 0.050],
            [0.0, 0.0, -0.001],
            [0.0, 0.0, 0.121],
        )
        for point in cases:
            with self.subTest(point=point):
                node = fixed_init_shell(position=point, lumen_geometry=lumen)
                with self.assertRaisesRegex(ValueError, "configured fixed target"):
                    node._initialize_active_reference()

    def test_external_timer_does_not_solve_or_publish_before_target(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        node._on_timer()
        self.assertEqual(0, node.core.solve_calls)
        self.assertEqual([], node.command_pub.messages)
        self.assertEqual([], node.safe_command_pub.messages)

    def test_external_valid_pose_is_accepted_exactly(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        msg = pose_from_point([0.015, 0.005, 0.100], "base_link")
        node._on_target(msg)
        self.assertEqual(VALID_REFERENCE, node.active_reference.state)
        self.assertEqual(EXTERNAL_TARGET, node.active_reference.source)
        self.assertEqual(1, node.active_reference.revision)
        self.assertEqual("base_link", node.active_reference.target_frame)
        self.assertTrue(np.array_equal([0.015, 0.005, 0.100], node.active_reference.target))
        kwargs = node._timer_reference_kwargs(current_time_s=10.0)
        self.assertTrue(np.array_equal([0.015, 0.005, 0.100], kwargs["target_tip"]))

    def test_external_invalid_first_update_keeps_no_active_reference(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        msg = pose_from_point([0.015, 0.005, 0.100], "world")
        node._on_target(msg)
        self.assertEqual(INVALID_REFERENCE, node.active_reference.state)
        self.assertEqual(0, node.active_reference.revision)
        self.assertFalse(node.active_reference.has_valid_target)

    def test_external_nonfinite_first_update_is_rejected(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        msg = target_pose([0.015, float("nan"), 0.100], "base_link")
        node._on_target(msg)
        self.assertEqual(INVALID_REFERENCE, node.active_reference.state)
        self.assertEqual(0, node.active_reference.revision)

    def test_external_geometry_invalid_first_update_is_rejected(self):
        node = controller_shell(mode=EXTERNAL_TARGET, lumen_geometry=cylinder_lumen())
        msg = pose_from_point([0.040, 0.0, 0.050], "base_link")
        node._on_target(msg)
        self.assertEqual(INVALID_REFERENCE, node.active_reference.state)
        self.assertEqual(0, node.active_reference.revision)

    def test_external_inlet_and_outlet_invalid_points_are_rejected(self):
        lumen = cylinder_lumen()
        cases = ([0.0, 0.0, -0.001], [0.0, 0.0, 0.121])
        for point in cases:
            with self.subTest(point=point):
                node = controller_shell(mode=EXTERNAL_TARGET, lumen_geometry=lumen)
                node._on_target(pose_from_point(point, "base_link"))
                self.assertEqual(INVALID_REFERENCE, node.active_reference.state)
                self.assertEqual(0, node.active_reference.revision)

    def test_external_invalid_replacement_retains_previous_valid_target_and_revision(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        node._on_target(pose_from_point([0.015, 0.005, 0.100], "base_link"))
        previous_target = node.active_reference.target.copy()
        previous_frame = node.active_reference.target_frame
        previous_revision = node.active_reference.revision
        node._on_target(pose_from_point([0.010, 0.000, 0.090], "world"))
        self.assertEqual(VALID_REFERENCE, node.active_reference.state)
        self.assertEqual(previous_revision, node.active_reference.revision)
        self.assertEqual(previous_frame, node.active_reference.target_frame)
        self.assertTrue(np.array_equal(previous_target, node.active_reference.target))

    def test_external_later_valid_changed_target_replaces_and_increments_revision(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        node._on_target(pose_from_point([0.015, 0.005, 0.100], "base_link"))
        node._on_target(pose_from_point([0.010, 0.000, 0.090], "base_link"))
        self.assertEqual(VALID_REFERENCE, node.active_reference.state)
        self.assertEqual(2, node.active_reference.revision)
        self.assertTrue(np.array_equal([0.010, 0.000, 0.090], node.active_reference.target))

    def test_external_duplicate_target_does_not_increment_revision(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        msg = pose_from_point([0.015, 0.005, 0.100], "base_link")
        node._on_target(msg)
        node._on_target(msg)
        self.assertEqual(1, node.active_reference.revision)

    def test_zero_duplicate_and_out_of_order_stamps_do_not_change_coordinates(self):
        node = controller_shell(mode=EXTERNAL_TARGET)
        first = pose_from_point([0.015, 0.005, 0.100], "base_link")
        first.header.stamp.sec = 20
        second = pose_from_point([0.015, 0.005, 0.100], "base_link")
        second.header.stamp.sec = 0
        node._on_target(first)
        node._on_target(second)
        self.assertEqual(1, node.active_reference.revision)
        self.assertTrue(np.array_equal([0.015, 0.005, 0.100], node.active_reference.target))

    def test_fixed_target_ignores_different_tip_update(self):
        node = controller_shell(mode=FIXED_TARGET)
        node.active_reference = accept_point_reference(
            node.active_reference,
            source=FIXED_TARGET,
            point=[0.015, 0.005, 0.100],
            frame="base_link",
        )
        node._on_target(pose_from_point([0.010, 0.000, 0.090], "base_link"))
        self.assertEqual(1, node.active_reference.revision)
        self.assertTrue(np.array_equal([0.015, 0.005, 0.100], node.active_reference.target))

    def test_fixed_and_external_solve_kwargs_use_target_tip(self):
        for mode in (FIXED_TARGET, EXTERNAL_TARGET):
            with self.subTest(mode=mode):
                target = np.array([0.0192, 0.0, 0.08])
                kwargs = solve_reference_kwargs(
                    reference_mode=mode,
                    target_tip=target,
                    target_tip_sequence=np.ones((3, 3)),
                    horizon_stamp_s=10.0,
                    current_time_s=10.30,
                    stale_timeout=0.20,
                )
                self.assertEqual(["target_tip"], list(kwargs.keys()))
                self.assertTrue(np.allclose(target, kwargs["target_tip"]))

    def test_trajectory_mode_still_requires_horizon_before_solve(self):
        node = controller_shell(mode=TRAJECTORY)
        node.latest_reference_horizon = None
        node.latest_reference_horizon_stamp_s = None
        with self.assertRaisesRegex(ValueError, "horizon"):
            node._timer_reference_kwargs(current_time_s=10.0)
        node._on_timer()
        self.assertEqual(0, node.core.solve_calls)
        self.assertEqual([], node.command_pub.messages)
        self.assertEqual([], node.safe_command_pub.messages)

    def test_trajectory_valid_horizon_installs_snapshot_and_enables_solve(self):
        node = controller_shell(mode=TRAJECTORY)
        points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]])
        msg = horizon_path(points, stamp_s=10.0)
        node._on_reference_horizon(msg)
        self.assertEqual(VALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertEqual(SOURCE_TRAJECTORY_HORIZON, node.active_trajectory_reference.source)
        self.assertEqual(1, node.active_trajectory_reference.revision)
        self.assertEqual("base_link", node.active_trajectory_reference.frame_id)
        self.assertTrue(np.array_equal(points, node.active_trajectory_reference.points))
        self.assertFalse(node.active_trajectory_reference.points.flags.writeable)
        self.assertTrue(np.array_equal(points[0], node.target_tip))

        msg.poses[0].pose.position.x = 9.0
        self.assertNotEqual(9.0, node.active_trajectory_reference.points[0, 0])

        kwargs = node._timer_reference_kwargs(current_time_s=10.05)
        self.assertTrue(np.array_equal(points, kwargs["target_tip_sequence"]))
        node._on_timer()
        self.assertEqual(1, node.core.solve_calls)
        self.assertTrue(np.array_equal(points, node.core.solve_kwargs[0]["target_tip_sequence"]))
        self.assertEqual(1, len(node.command_pub.messages))
        self.assertEqual(1, len(node.safe_command_pub.messages))

    def test_trajectory_invalid_first_horizon_leaves_no_active_snapshot(self):
        node = controller_shell(mode=TRAJECTORY)
        invalid = horizon_path(np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]]))
        invalid.poses[1].pose.position.x = float("nan")
        node._on_reference_horizon(invalid)
        self.assertEqual(INVALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertEqual(0, node.active_trajectory_reference.revision)
        self.assertIsNone(node.active_trajectory_reference.points)
        self.assertIsNone(node.latest_reference_horizon)
        node._on_timer()
        self.assertEqual(0, node.core.solve_calls)
        self.assertEqual([], node.command_pub.messages)
        self.assertEqual([], node.safe_command_pub.messages)

        valid = horizon_path(np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]]))
        node._on_reference_horizon(valid)
        self.assertEqual(VALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertEqual(1, node.active_trajectory_reference.revision)

    def test_trajectory_rejects_wrong_length_and_mixed_pose_frame(self):
        node = controller_shell(mode=TRAJECTORY)
        with self.assertRaisesRegex(ValueError, "shape"):
            target_sequence_from_path(
                horizon_path(np.zeros((2, 3))),
                expected_horizon=3,
                expected_frame_id="base_link",
                current_time_s=10.0,
                stale_timeout=0.20,
            )
        mixed = horizon_path(np.zeros((3, 3)))
        mixed.poses[1].header.frame_id = "world"
        node._on_reference_horizon(mixed)
        self.assertEqual(INVALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertEqual(0, node.active_trajectory_reference.revision)

    def test_trajectory_geometry_invalid_first_horizon_is_rejected(self):
        node = controller_shell(mode=TRAJECTORY, lumen_geometry=cylinder_lumen())
        points = np.array([[0.0, 0.0, 0.020], [0.040, 0.0, 0.050], [0.0, 0.0, 0.080]])
        node._on_reference_horizon(horizon_path(points))
        self.assertEqual(INVALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertEqual(0, node.active_trajectory_reference.revision)
        self.assertIsNone(node.active_trajectory_reference.points)
        self.assertIn("point[1]", node.active_trajectory_reference.last_validation_error)

    def test_trajectory_invalid_replacement_retains_previous_snapshot_and_solve_input(self):
        node = controller_shell(mode=TRAJECTORY)
        node.get_clock = lambda: FakeClock(10.11)
        first_points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]])
        node._on_reference_horizon(horizon_path(first_points, stamp_s=10.0))
        previous_revision = node.active_trajectory_reference.revision
        previous_points = node.active_trajectory_reference.points.copy()
        invalid = horizon_path(np.array([[0.010, 0.0, 0.08], [0.011, 0.0, 0.08], [0.012, 0.0, 0.08]]), stamp_s=10.1)
        invalid.poses[1].pose.position.y = float("inf")
        node._on_reference_horizon(invalid)
        self.assertEqual(VALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertEqual(previous_revision, node.active_trajectory_reference.revision)
        self.assertTrue(np.array_equal(previous_points, node.active_trajectory_reference.points))
        self.assertTrue(np.array_equal(previous_points, node.latest_reference_horizon))
        node._on_timer()
        self.assertEqual(1, node.core.solve_calls)
        self.assertTrue(np.array_equal(previous_points, node.core.solve_kwargs[0]["target_tip_sequence"]))

    def test_trajectory_valid_replacement_duplicate_and_timestamp_policy(self):
        node = controller_shell(mode=TRAJECTORY)
        node.get_clock = lambda: FakeClock(10.11)
        first_points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]])
        second_points = np.array([[0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08], [0.0195, 0.0, 0.08]])
        node._on_reference_horizon(horizon_path(first_points, stamp_s=10.0))
        node._on_reference_horizon(horizon_path(second_points, stamp_s=10.05))
        self.assertEqual(2, node.active_trajectory_reference.revision)
        self.assertTrue(np.array_equal(second_points, node.active_trajectory_reference.points))
        node._on_reference_horizon(horizon_path(second_points, stamp_s=10.10))
        self.assertEqual(2, node.active_trajectory_reference.revision)
        self.assertAlmostEqual(10.10, node.active_trajectory_reference.stamp_s)
        node._on_timer()
        self.assertTrue(np.array_equal(second_points, node.core.solve_kwargs[-1]["target_tip_sequence"]))

    def test_trajectory_tip_callback_cannot_bypass_or_mutate_horizon_snapshot(self):
        node = controller_shell(mode=TRAJECTORY)
        node._on_target(pose_from_point([0.010, 0.000, 0.090], "base_link"))
        self.assertFalse(node.active_trajectory_reference.has_valid_horizon)
        self.assertEqual(0, node.active_trajectory_reference.revision)
        points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]])
        node._on_reference_horizon(horizon_path(points))
        previous = node.active_trajectory_reference.points.copy()
        node._on_target(pose_from_point([0.010, 0.000, 0.090], "base_link"))
        self.assertEqual(1, node.active_trajectory_reference.revision)
        self.assertTrue(np.array_equal(previous, node.active_trajectory_reference.points))

    def test_trajectory_stale_active_snapshot_gates_solve_without_clearing_snapshot(self):
        node = controller_shell(mode=TRAJECTORY)
        points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08], [0.0194, 0.0, 0.08]])
        node._on_reference_horizon(horizon_path(points, stamp_s=10.0))
        with self.assertRaisesRegex(ValueError, "stale"):
            node._timer_reference_kwargs(current_time_s=10.30)
        self.assertEqual(VALID_REFERENCE, node.active_trajectory_reference.state)
        self.assertTrue(np.array_equal(points, node.active_trajectory_reference.points))

    def test_hold_final_repeated_horizon_is_valid_and_duplicate_does_not_churn_revision(self):
        node = controller_shell(mode=TRAJECTORY)
        node.get_clock = lambda: FakeClock(10.06)
        final_points = np.array([[0.0192, 0.0, 0.08], [0.0192, 0.0, 0.08], [0.0192, 0.0, 0.08]])
        node._on_reference_horizon(horizon_path(final_points, stamp_s=10.0))
        node._on_reference_horizon(horizon_path(final_points, stamp_s=10.05))
        self.assertEqual(1, node.active_trajectory_reference.revision)
        self.assertTrue(np.array_equal(final_points, node.active_trajectory_reference.points))

    def test_metrics_publish_rate_gate(self):
        self.assertTrue(should_publish_metrics(last_publish_time_s=None, current_time_s=1.0, publish_frequency=5.0))
        self.assertFalse(should_publish_metrics(last_publish_time_s=1.0, current_time_s=1.1, publish_frequency=5.0))
        self.assertTrue(should_publish_metrics(last_publish_time_s=1.0, current_time_s=1.2, publish_frequency=5.0))
        self.assertTrue(should_publish_metrics(last_publish_time_s=2.0, current_time_s=1.0, publish_frequency=5.0))

    def test_trajectory_metrics_diagnostic_contains_required_fields(self):
        config = TrajectoryMetricsConfig(
            enabled=True,
            publish_frequency=5.0,
            transient_tolerance=0.001,
            stable_cycles=1,
            reset_on_new_trajectory=True,
        )
        accumulator = TrajectoryMetricsAccumulator(config=config, command_dimension=2, trajectory_type="circle")
        accumulator.add_sample(
            timestamp=0.0,
            tip_position=[0.0, 0.0, 0.0],
            reference_position=[0.0, 0.0, 0.0],
            command=[0.1, -0.2],
            dt=0.1,
            solve_time=0.01,
            command_saturated=True,
        )
        msg = trajectory_metrics_diagnostic_array(accumulator.snapshot(), frame_id="base_link", stamp=None)
        values = {item.key: item.value for item in msg.status[0].values}
        for key in (
            "trajectory_type",
            "sample_count",
            "rmse",
            "mean_error",
            "max_error",
            "control_effort",
            "transient_duration",
            "mean_solve_time",
            "max_solve_time",
            "command_saturation_count",
            "maximum_command_per_joint",
            "experiment_elapsed_time",
            "completion_state",
        ):
            self.assertIn(key, values)
        self.assertEqual("circle", values["trajectory_type"])
        self.assertEqual("1", values["sample_count"])


if __name__ == "__main__":
    unittest.main()
