import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_evaluation.metrics import (  # noqa: E402
    DataQualityMetrics,
    EvaluationThresholds,
    NumericalSafetyMetrics,
    aggregate_trial_summaries,
    compare_summaries,
    compute_acceptance,
    compute_control_effort_series,
    compute_control_metrics,
    compute_goal_metrics,
    compute_lumen_safety_metrics,
    compute_motion_metrics,
    compute_timing_metrics,
    compute_tracking_metrics,
    publication_rate,
    relative_improvement_percent,
)
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen  # noqa: E402
from ctr_mppi_controller.trajectory_metrics import (  # noqa: E402
    TrajectoryMetricsAccumulator,
    TrajectoryMetricsConfig,
)


class EvaluationMetricsTest(unittest.TestCase):
    def test_tracking_error_statistics(self):
        times = [0.0, 1.0, 2.0, 3.0]
        tips = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]]
        refs = np.zeros((4, 3))
        metrics = compute_tracking_metrics(
            times=times,
            tip_positions=tips,
            reference_positions=refs,
            tolerance=2.1,
            stable_cycles=2,
            steady_state_window=1.0,
            steady_state_fraction=0.5,
            path_progress=[0.0, 0.6, 0.2, 0.4],
        )
        self.assertAlmostEqual(math.sqrt((0.0 + 1.0 + 4.0 + 16.0) / 4.0), metrics.rmse)
        self.assertAlmostEqual(1.75, metrics.mean_error)
        self.assertAlmostEqual(1.5, metrics.median_error)
        self.assertAlmostEqual(np.percentile([0.0, 1.0, 2.0, 4.0], 95.0), metrics.p95_error)
        self.assertAlmostEqual(4.0, metrics.max_error)
        self.assertAlmostEqual(4.0, metrics.final_error)
        self.assertAlmostEqual(3.0, metrics.steady_state_error)
        self.assertAlmostEqual(0.0, metrics.time_to_first_tolerance_entry)
        self.assertAlmostEqual(0.0, metrics.transient_duration)
        self.assertAlmostEqual(75.0, metrics.time_inside_tolerance_percentage)
        self.assertAlmostEqual(60.0, metrics.path_completion_percentage)

    def test_tracking_no_transient(self):
        metrics = compute_tracking_metrics(
            times=[0.0, 1.0, 2.0],
            tip_positions=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            reference_positions=np.zeros((3, 3)),
            tolerance=0.5,
            stable_cycles=2,
            steady_state_window=0.0,
            steady_state_fraction=0.5,
        )
        self.assertEqual(-1.0, metrics.time_to_first_tolerance_entry)
        self.assertEqual(-1.0, metrics.transient_duration)

    def test_control_metrics(self):
        metrics = compute_control_metrics(
            times=[0.0, 0.5, 1.0],
            commands=[
                [1.0, 0.0, 0.0, 2.0, 0.0, 0.0],
                [2.0, 0.0, 0.0, 0.0, 3.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 4.0],
            ],
            saturation_flags=[False, True, False],
            missing_command_flags=[False, False, True],
        )
        self.assertAlmostEqual(15.0, metrics.total_control_effort)
        self.assertAlmostEqual(2.5, metrics.insertion_control_effort)
        self.assertAlmostEqual(12.5, metrics.rotation_control_effort)
        self.assertEqual(1, metrics.saturation_count)
        self.assertAlmostEqual(100.0 / 3.0, metrics.saturation_percentage)
        self.assertEqual(1, metrics.missing_command_sample_count)
        self.assertEqual(6, len(metrics.command_rms_per_joint))
        self.assertEqual([2.0, 1.0, 0.0, 2.0, 3.0, 4.0], metrics.maximum_command_per_joint)
        self.assertGreater(metrics.total_command_variation, 0.0)

    def test_control_effort_series_known_irregular_and_duplicate_timestamps(self):
        times = [0.0, 0.25, 1.0, 1.0, 2.0]
        commands = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 3.0, 0.0, 0.0],
            [0.0, 4.0, 0.0, 0.0, 5.0, 0.0],
            [9.0, 9.0, 9.0, 9.0, 9.0, 9.0],
            [1.0, 1.0, 1.0, 2.0, 2.0, 2.0],
        ]
        effort = compute_control_effort_series(times=times, commands=commands)
        self.assertEqual([0.0, 0.25, 0.75, 0.0, 1.0], effort.interval_durations)
        self.assertAlmostEqual(49.0, effort.total_control_effort)
        self.assertAlmostEqual(16.0, effort.insertion_control_effort)
        self.assertAlmostEqual(33.0, effort.rotation_control_effort)

        metrics = compute_control_metrics(times=times, commands=commands)
        self.assertAlmostEqual(metrics.total_control_effort, effort.cumulative_total_effort[-1])
        self.assertAlmostEqual(metrics.insertion_control_effort, effort.cumulative_insertion_effort[-1])
        self.assertAlmostEqual(metrics.rotation_control_effort, effort.cumulative_rotation_effort[-1])

    def test_control_effort_series_one_sample_contributes_zero_duration(self):
        effort = compute_control_effort_series(
            times=[10.0],
            commands=[[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]],
        )
        self.assertEqual([0.0], effort.interval_durations)
        self.assertEqual([0.0], effort.cumulative_total_effort)
        self.assertEqual(0.0, effort.total_control_effort)

    def test_control_effort_rejects_out_of_order_timestamps(self):
        with self.assertRaisesRegex(ValueError, "sorted by time"):
            compute_control_effort_series(
                times=[0.0, 1.0, 0.5],
                commands=np.zeros((3, 6)),
            )

    def test_timing_metrics_and_publication_rate(self):
        metrics = compute_timing_metrics(
            solve_times=[0.20, 0.01, 0.10],
            solve_timestamps=[0.0, 1.0, 2.0],
            state_timestamps=[0.0, 0.1, 0.2, 0.3],
            reference_timestamps=[0.0, 0.05, 0.10],
            command_timestamps=[0.0, 1.0],
            configured_control_frequency=20.0,
            experiment_wall_duration=3.0,
            valid_aligned_evaluation_duration=0.3,
        )
        self.assertAlmostEqual(0.1033333333, metrics.mean_solve_time)
        self.assertAlmostEqual(0.10, metrics.median_solve_time)
        self.assertEqual(2, metrics.deadline_overrun_count)
        self.assertAlmostEqual(100.0 * 2.0 / 3.0, metrics.deadline_overrun_percentage)
        self.assertAlmostEqual(10.0, metrics.state_publication_rate)
        self.assertAlmostEqual(1.0, publication_rate([0.0, 1.0, 2.0]))

    def test_goal_metrics_hold_duration(self):
        metrics = compute_goal_metrics(
            times=[0.0, 0.2, 0.4, 0.7, 1.0],
            tip_positions=[
                [0.010, 0.0, 0.0],
                [0.004, 0.0, 0.0],
                [0.002, 0.0, 0.0],
                [0.001, 0.0, 0.0],
                [0.001, 0.0, 0.0],
            ],
            goal_position=[0.0, 0.0, 0.0],
            tolerance=0.003,
            required_hold_duration=0.5,
        )
        self.assertTrue(metrics.goal_reached)
        self.assertAlmostEqual(0.4, metrics.time_to_goal)
        self.assertAlmostEqual(0.6, metrics.goal_hold_duration)
        self.assertAlmostEqual(0.001, metrics.final_goal_error)

    def test_lumen_safety_metrics_detect_backbone_collision_and_margin(self):
        lumen = CylindricalLumen(
            frame_id="base_link",
            axis_origin=[0.0, 0.0, 0.0],
            axis_direction=[0.0, 0.0, 1.0],
            radius=0.030,
            length=0.120,
            ctr_outer_radius=0.0015,
            safety_margin=0.002,
        )
        metrics = compute_lumen_safety_metrics(
            times=[0.0, 0.5, 1.0],
            backbone_points=[
                np.array([[0.0, 0.0, 0.0], [0.010, 0.0, 0.05]]),
                np.array([[0.0, 0.0, 0.0], [0.027, 0.0, 0.05]]),
                np.array([[0.0, 0.0, 0.0], [0.040, 0.0, 0.05]]),
            ],
            lumen=lumen,
        )
        self.assertFalse(metrics.collision_free_pass)
        self.assertFalse(metrics.safety_margin_pass)
        self.assertEqual(1, metrics.radial_collision_count)
        self.assertEqual(2, metrics.safety_margin_violation_count)
        self.assertGreater(metrics.maximum_penetration_depth, 0.0)

    def test_motion_metrics_path_efficiency_and_control_fields(self):
        control = compute_control_metrics(
            times=[0.0, 1.0],
            commands=[[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        )
        metrics = compute_motion_metrics(
            times=[0.0, 1.0],
            tip_positions=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            q_values=np.zeros((2, 6)),
            goal_position=[1.0, 0.0, 0.0],
            control=control,
        )
        self.assertAlmostEqual(1.0, metrics.tip_path_length)
        self.assertAlmostEqual(1.0, metrics.straight_line_target_distance)
        self.assertAlmostEqual(1.0, metrics.path_efficiency)
        self.assertAlmostEqual(control.total_control_effort, metrics.total_control_effort)

    def test_acceptance_categories_are_separate(self):
        tracking = compute_tracking_metrics(
            times=[0.0, 0.1],
            tip_positions=np.zeros((2, 3)),
            reference_positions=np.zeros((2, 3)),
            tolerance=0.1,
            stable_cycles=1,
            steady_state_window=0.0,
            steady_state_fraction=1.0,
        )
        control = compute_control_metrics(times=[0.0, 0.1], commands=np.zeros((2, 6)))
        timing = compute_timing_metrics(
            solve_times=[1.0],
            solve_timestamps=[0.0],
            state_timestamps=[0.0, 0.1],
            reference_timestamps=[0.0, 0.1],
            command_timestamps=[],
            configured_control_frequency=20.0,
            experiment_wall_duration=0.1,
            valid_aligned_evaluation_duration=0.1,
        )
        thresholds = EvaluationThresholds(
            configured_duration=1.0,
            configured_control_frequency=20.0,
            tracking_tolerance=0.1,
            transient_stable_cycles=1,
            steady_state_window=0.0,
            steady_state_fraction=1.0,
            minimum_valid_sample_count=2,
            maximum_invalid_sample_percentage=10.0,
            maximum_saturation_percentage=1.0,
            maximum_deadline_overrun_percentage=5.0,
            required_minimum_baseline_improvement=0.0,
            near_zero_baseline_epsilon=1.0e-12,
        )
        result = compute_acceptance(
            tracking=tracking,
            control=control,
            timing=timing,
            numerical_safety=NumericalSafetyMetrics(0, 0, 0, 0, 0, 0, 0, 0),
            data_quality=DataQualityMetrics(2, 2, 0, 2, 0, 0, 0.0, 0.0, 0, 0, 2, 0),
            thresholds=thresholds,
            baseline_improvement_valid=True,
        )
        self.assertTrue(result.functional_pass)
        self.assertTrue(result.goal_reached_pass)
        self.assertTrue(result.collision_free_pass)
        self.assertTrue(result.safety_margin_pass)
        self.assertTrue(result.numerical_safety_pass)
        self.assertFalse(result.real_time_pass)
        self.assertFalse(result.timing_pass)
        self.assertNotIn("controller timing", result.reasons)
        self.assertFalse(result.physical_validation_pass)
        self.assertFalse(result.hardware_validation_pass)

    def test_relative_improvement_and_near_zero(self):
        improvement, valid, reason = relative_improvement_percent(
            candidate_value=4.0,
            baseline_value=5.0,
            lower_is_better=True,
            near_zero_epsilon=1.0e-12,
        )
        self.assertTrue(valid)
        self.assertEqual("ok", reason)
        self.assertAlmostEqual(20.0, improvement)
        improvement, valid, reason = relative_improvement_percent(
            candidate_value=0.0,
            baseline_value=0.0,
            lower_is_better=True,
            near_zero_epsilon=1.0e-12,
        )
        self.assertIsNone(improvement)
        self.assertFalse(valid)
        self.assertIn("near zero", reason)

    def test_comparison_rejects_incompatible_configuration(self):
        candidate = {"tracking": {"rmse": 1.0}}
        baseline = {"tracking": {"rmse": 2.0}}
        meta_a = {"configuration": {"trajectory_type": "circle", "frame_id": "base_link"}}
        meta_b = {"configuration": {"trajectory_type": "ellipse", "frame_id": "base_link"}}
        result = compare_summaries(
            candidate_summary=candidate,
            baseline_summary=baseline,
            candidate_metadata=meta_a,
            baseline_metadata=meta_b,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=1.0,
            initial_state_tolerance=1.0e-6,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertFalse(result.metric_comparisons[0].comparison_valid)

    def test_timing_measurements_do_not_change_comparison_validity(self):
        candidate = {
            "tracking": {"rmse": 1.0},
            "timing": {"mean_solve_time": 0.01, "effective_solve_frequency": 100.0},
        }
        baseline = {
            "tracking": {"rmse": 2.0},
            "timing": {"mean_solve_time": 9.0, "effective_solve_frequency": 0.1},
        }
        config = {"trajectory_type": "circle", "frame_id": "base_link"}
        result = compare_summaries(
            candidate_summary=candidate,
            baseline_summary=baseline,
            candidate_metadata={"configuration": config},
            baseline_metadata={"configuration": config},
            near_zero_epsilon=1.0e-12,
            duration_tolerance=1.0,
            initial_state_tolerance=1.0e-6,
        )
        self.assertTrue(result.compatibility_valid)
        self.assertTrue(result.metric_comparisons[0].comparison_valid)

    def test_actual_duration_difference_rejects_comparison(self):
        candidate = {"tracking": {"rmse": 1.0}}
        baseline = {"tracking": {"rmse": 2.0}}
        config = {"trajectory_type": "circle", "frame_id": "base_link"}
        result = compare_summaries(
            candidate_summary=candidate,
            baseline_summary=baseline,
            candidate_metadata={"configuration": config, "actual_duration": 2.0},
            baseline_metadata={"configuration": config, "actual_duration": 1.0},
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=1.0e-6,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertIn("actual duration", result.compatibility_reasons[0])

    def test_repeated_trial_aggregation(self):
        aggregate = aggregate_trial_summaries(
            [
                {"tracking": {"rmse": 1.0}, "timing": {"mean_solve_time": 2.0}},
                {"tracking": {"rmse": 3.0}, "timing": {"mean_solve_time": 4.0}},
            ]
        )
        self.assertEqual(2, aggregate["count"])
        self.assertAlmostEqual(2.0, aggregate["metrics"]["rmse"]["mean"])
        self.assertIn("confidence_interval_95", aggregate["metrics"]["rmse"])

    def test_higher_is_better_clearance_comparison(self):
        result = compare_summaries(
            candidate_summary={"lumen_safety": {"minimum_backbone_wall_clearance": 0.004}},
            baseline_summary={"lumen_safety": {"minimum_backbone_wall_clearance": 0.002}},
            candidate_metadata={"configuration": {"trajectory_type": "circle", "frame_id": "base_link"}},
            baseline_metadata={"configuration": {"trajectory_type": "circle", "frame_id": "base_link"}},
            near_zero_epsilon=1.0e-12,
            duration_tolerance=1.0,
            initial_state_tolerance=1.0e-6,
        )
        self.assertTrue(result.metric_comparisons[0].comparison_valid)
        self.assertEqual("higher", result.metric_comparisons[0].direction)
        self.assertAlmostEqual(100.0, result.metric_comparisons[0].relative_improvement_percent)

    def test_online_metric_cross_check_for_shared_definitions(self):
        online_config = TrajectoryMetricsConfig(
            enabled=True,
            publish_frequency=5.0,
            transient_tolerance=0.5,
            stable_cycles=2,
            reset_on_new_trajectory=True,
        )
        online = TrajectoryMetricsAccumulator(
            config=online_config,
            command_dimension=6,
            trajectory_type="circle",
        )
        commands = np.zeros((2, 6))
        for timestamp, error in [(0.0, 1.0), (1.0, 2.0)]:
            online.add_sample(
                timestamp=timestamp,
                tip_position=[error, 0.0, 0.0],
                reference_position=[0.0, 0.0, 0.0],
                command=np.zeros(6),
                dt=1.0,
                solve_time=0.1,
                command_saturated=False,
            )
        offline_tracking = compute_tracking_metrics(
            times=[0.0, 1.0],
            tip_positions=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            reference_positions=np.zeros((2, 3)),
            tolerance=0.5,
            stable_cycles=2,
            steady_state_window=0.0,
            steady_state_fraction=1.0,
        )
        offline_control = compute_control_metrics(times=[0.0, 1.0], commands=commands)
        snapshot = online.snapshot()
        self.assertAlmostEqual(snapshot.rmse, offline_tracking.rmse)
        self.assertAlmostEqual(snapshot.mean_error, offline_tracking.mean_error)
        self.assertAlmostEqual(snapshot.max_error, offline_tracking.max_error)
        self.assertAlmostEqual(snapshot.control_effort, offline_control.total_control_effort)


if __name__ == "__main__":
    unittest.main()
