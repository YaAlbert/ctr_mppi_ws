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


try:
    import ctr_interfaces.msg  # noqa: F401
except ImportError:
    ctr_interfaces_module = types.ModuleType("ctr_interfaces")
    ctr_interfaces_msg_module = types.ModuleType("ctr_interfaces.msg")
    ctr_interfaces_msg_module.CtrControllerMetrics = type("CtrControllerMetrics", (), {})
    ctr_interfaces_msg_module.CtrJointCommand = type("CtrJointCommand", (), {})
    ctr_interfaces_msg_module.CtrState = type("CtrState", (), {})
    sys.modules["ctr_interfaces"] = ctr_interfaces_module
    sys.modules["ctr_interfaces.msg"] = ctr_interfaces_msg_module


from ctr_mppi_controller.nodes.mppi_controller_node import (  # noqa: E402
    active_reference_point,
    reference_mode_from_config,
    reference_type_from_config,
    should_publish_metrics,
    solve_reference_kwargs,
    target_sequence_from_path,
    trajectory_metrics_diagnostic_array,
)
from ctr_mppi_controller.nodes.reference_manager_node import path_from_points  # noqa: E402
from ctr_mppi_controller.trajectory_metrics import TrajectoryMetricsAccumulator, TrajectoryMetricsConfig  # noqa: E402


def horizon_path(points, *, frame_id="base_link", stamp_s=10):
    path = path_from_points(points, frame_id)
    path.header.stamp.sec = int(stamp_s)
    path.header.stamp.nanosec = int((float(stamp_s) - int(stamp_s)) * 1.0e9)
    for pose in path.poses:
        pose.header.stamp = path.header.stamp
    return path


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
        with self.assertRaises(ValueError):
            reference_mode_from_config(config, "invalid")

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
