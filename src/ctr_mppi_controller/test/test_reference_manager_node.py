import copy
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import config_with_cylinder_overrides, goal_position_from_config  # noqa: E402
from ctr_mppi_controller.lumen_factory import config_with_lumen_overrides, lumen_mode_from_config  # noqa: E402
from ctr_mppi_controller.nodes.reference_manager_node import (  # noqa: E402
    adjusted_trajectory_start_time,
    build_reference_trajectory,
    path_from_points,
    pose_from_point,
    reference_settings_from_config,
    trajectory_start_time_from_policy,
)


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def make_config():
    config = load_parameter_files(CONFIG_FILES)
    validate_or_raise(config)
    return copy.deepcopy(config)


class ReferenceManagerNodeHelpersTest(unittest.TestCase):
    def test_reference_settings_default_to_fixed_target(self):
        config = make_config()
        settings = reference_settings_from_config(config)
        self.assertEqual("fixed_target", settings.mode)
        self.assertEqual("circle", settings.trajectory_type)
        self.assertEqual("base_link", settings.frame_id)
        self.assertEqual(config["mppi"]["horizon"], settings.horizon)
        self.assertTrue(np.allclose(config["reference"]["fixed_target"], settings.fixed_target))

    def test_reference_type_override_builds_ellipse(self):
        config = make_config()
        settings = reference_settings_from_config(config, mode_override="trajectory", type_override="ellipse")
        trajectory = build_reference_trajectory(config, settings=settings)
        self.assertEqual("trajectory", settings.mode)
        self.assertEqual("ellipse", trajectory.trajectory_type)
        self.assertEqual("base_link", trajectory.frame_id)
        self.assertTrue(np.all(np.isfinite(trajectory.points)))

    def test_cylinder_fixed_target_uses_goal_override(self):
        config = config_with_cylinder_overrides(
            make_config(),
            enabled=True,
            target_position=[0.010, 0.012, 0.095],
            mppi_profile="cylinder_fast",
            random_seed=11,
        )
        settings = reference_settings_from_config(config, mode_override="fixed_target", type_override="circle")
        fixed_settings = settings.__class__(
            mode=settings.mode,
            trajectory_type=settings.trajectory_type,
            frame_id=settings.frame_id,
            completion_behavior=settings.completion_behavior,
            sample_period=settings.sample_period,
            duration=settings.duration,
            publish_frequency=settings.publish_frequency,
            stale_timeout=settings.stale_timeout,
            fixed_target=goal_position_from_config(config),
            horizon=settings.horizon,
        )
        self.assertTrue(np.allclose([0.010, 0.012, 0.095], fixed_settings.fixed_target))

    def test_curved_fixed_target_uses_goal_override_without_geometry_construction(self):
        config = config_with_lumen_overrides(
            make_config(),
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type="circular_arc",
            target=[0.010, 0.012, 0.095],
            cylinder_profile="cylinder_fast",
            random_seed=11,
        )
        self.assertEqual("curved", lumen_mode_from_config(config))
        settings = reference_settings_from_config(config, mode_override="fixed_target", type_override="circle")
        fixed_settings = settings.__class__(
            mode=settings.mode,
            trajectory_type=settings.trajectory_type,
            frame_id=settings.frame_id,
            completion_behavior=settings.completion_behavior,
            sample_period=settings.sample_period,
            duration=settings.duration,
            publish_frequency=settings.publish_frequency,
            stale_timeout=settings.stale_timeout,
            fixed_target=goal_position_from_config(config),
            horizon=settings.horizon,
        )
        self.assertTrue(np.allclose([0.010, 0.012, 0.095], fixed_settings.fixed_target))

    def test_elapsed_ros_time_progression_uses_sample_period(self):
        config = make_config()
        settings = reference_settings_from_config(config, mode_override="trajectory", type_override="circle")
        trajectory = build_reference_trajectory(config, settings=settings)
        horizon = trajectory.horizon_at_time(
            current_time=1.10,
            start_time=1.00,
            horizon_length=settings.horizon,
        )
        self.assertEqual((settings.horizon, 3), horizon.points.shape)
        self.assertEqual(2, horizon.start_index)

    def test_ros_time_reset_restarts_trajectory_timing(self):
        self.assertAlmostEqual(
            2.0,
            adjusted_trajectory_start_time(previous_time_s=5.0, current_time_s=2.0, start_time_s=1.0),
        )
        self.assertAlmostEqual(
            1.0,
            adjusted_trajectory_start_time(previous_time_s=5.0, current_time_s=6.0, start_time_s=1.0),
        )

    def test_scheduled_start_policy_uses_configured_epoch(self):
        self.assertAlmostEqual(
            10.0,
            trajectory_start_time_from_policy(
                policy="scheduled_time",
                now_s=2.0,
                scheduled_reference_epoch_s=10.0,
            ),
        )
        self.assertAlmostEqual(
            2.0,
            trajectory_start_time_from_policy(
                policy="node_start",
                now_s=2.0,
                scheduled_reference_epoch_s=10.0,
            ),
        )

    def test_scheduled_start_rejects_nonfinite_epoch(self):
        with self.assertRaises(ValueError):
            trajectory_start_time_from_policy(
                policy="scheduled_time",
                now_s=2.0,
                scheduled_reference_epoch_s=float("nan"),
            )

    def test_scheduled_pre_epoch_behavior_keeps_first_horizon_point(self):
        config = make_config()
        settings = reference_settings_from_config(config, mode_override="trajectory", type_override="circle")
        trajectory = build_reference_trajectory(config, settings=settings)
        horizon = trajectory.horizon_at_time(
            current_time=9.0,
            start_time=10.0,
            horizon_length=settings.horizon,
        )
        self.assertEqual(0, horizon.start_index)
        self.assertEqual(0, horizon.current_index)
        self.assertTrue(np.allclose(trajectory.points[0], horizon.current_point))

    def test_scheduled_transition_at_epoch_uses_elapsed_sample_period(self):
        config = make_config()
        settings = reference_settings_from_config(config, mode_override="trajectory", type_override="circle")
        trajectory = build_reference_trajectory(config, settings=settings)
        at_epoch = trajectory.horizon_at_time(current_time=10.0, start_time=10.0, horizon_length=settings.horizon)
        after_epoch = trajectory.horizon_at_time(
            current_time=10.0 + 2.0 * settings.sample_period,
            start_time=10.0,
            horizon_length=settings.horizon,
        )
        self.assertEqual(0, at_epoch.start_index)
        self.assertEqual(2, after_epoch.start_index)

    def test_scheduled_time_backward_before_epoch_keeps_scheduled_epoch(self):
        self.assertAlmostEqual(
            10.0,
            adjusted_trajectory_start_time(
                previous_time_s=9.0,
                current_time_s=2.0,
                start_time_s=10.0,
                policy="scheduled_time",
                scheduled_reference_epoch_s=10.0,
            ),
        )

    def test_loop_behavior_wraps_horizon_indices(self):
        config = make_config()
        settings = reference_settings_from_config(config, mode_override="trajectory", type_override="circle")
        trajectory = build_reference_trajectory(config, settings=settings)
        horizon = trajectory.horizon_at_index(start_index=trajectory.points.shape[0] - 2, horizon_length=4)
        self.assertEqual(
            [
                trajectory.points.shape[0] - 2,
                trajectory.points.shape[0] - 1,
                0,
                1,
            ],
            horizon.indices.tolist(),
        )
        self.assertFalse(horizon.completed)

    def test_hold_final_behavior_clamps_horizon_indices(self):
        config = make_config()
        config["reference"]["loop"] = False
        config["reference"]["completion_behavior"] = "hold_final"
        settings = reference_settings_from_config(config, mode_override="trajectory", type_override="circle")
        trajectory = build_reference_trajectory(config, settings=settings)
        last_index = trajectory.points.shape[0] - 1
        horizon = trajectory.horizon_at_index(start_index=last_index - 1, horizon_length=4)
        self.assertEqual([last_index - 1, last_index, last_index, last_index], horizon.indices.tolist())
        self.assertTrue(horizon.completed)

    def test_path_and_pose_messages_use_identity_orientation(self):
        points = np.array([[0.0192, 0.0, 0.08], [0.0193, 0.0, 0.08]])
        path = path_from_points(points, "base_link")
        self.assertEqual("base_link", path.header.frame_id)
        self.assertEqual(2, len(path.poses))
        self.assertAlmostEqual(1.0, path.poses[0].pose.orientation.w)
        self.assertTrue(np.allclose(points[1], [path.poses[1].pose.position.x, path.poses[1].pose.position.y, path.poses[1].pose.position.z]))

        pose = pose_from_point(points[0], "base_link")
        self.assertEqual("base_link", pose.header.frame_id)
        self.assertAlmostEqual(1.0, pose.pose.orientation.w)

    def test_reference_manager_console_script_is_registered(self):
        setup_text = (REPO_ROOT / "src" / "ctr_mppi_controller" / "setup.py").read_text(encoding="utf-8")
        self.assertIn(
            "reference_manager_node = ctr_mppi_controller.nodes.reference_manager_node:main",
            setup_text,
        )


if __name__ == "__main__":
    unittest.main()
