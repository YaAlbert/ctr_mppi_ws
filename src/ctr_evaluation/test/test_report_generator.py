import sys
import tempfile
import unittest
import csv
import json
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_evaluation.report_generator import (  # noqa: E402
    generate_plot_artifact,
    generate_plots,
    plot_artifact_names,
    plot_producer_registry,
    generate_report,
)
from ctr_evaluation.time_alignment import AlignedSample  # noqa: E402


def sample(t):
    return AlignedSample(
        timestamp=t,
        q=np.zeros(6),
        q_dot=np.zeros(6),
        tip_position=np.array([t, 0.0, 0.0]),
        backbone_points=None,
        reference_position=np.array([0.0, 0.0, 0.0]),
        command=np.ones(6) * 0.1,
        solve_time=0.01,
        command_saturated=False,
        missing_command=False,
        reference_gap=0.0,
        command_gap=0.0,
        solve_gap=0.0,
        used_reference_interpolation=False,
        used_nearest_reference=False,
        reference_progress=t,
    )


class ReportGeneratorTest(unittest.TestCase):
    def test_plot_generation_creates_png_files(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            paths = generate_plots(Path(temp_dir), [sample(0.0), sample(1.0)])
            self.assertEqual(7, len(paths))
            self.assertIn("tip_trajectory.png", [path.name for path in paths])
            for path in paths:
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 0)

    def test_empty_plot_generation_still_creates_files(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            paths = generate_plots(Path(temp_dir), [])
            self.assertEqual(7, len(paths))
            self.assertIn("tip_trajectory.png", [path.name for path in paths])
            self.assertTrue(all(path.is_file() for path in paths))

    def test_plot_dispatch_is_deterministic_and_single_artifact(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            names = plot_artifact_names(run_dir)
            self.assertEqual(7, len(names))
            self.assertEqual(names, plot_artifact_names(run_dir))
            registry = plot_producer_registry(
                run_dir, [sample(0.0), sample(1.0)]
            )
            self.assertEqual(names, tuple(registry))
            path = registry["tracking_error_plot"](run_dir)
            self.assertEqual("tracking_error.png", path.name)
            self.assertTrue(path.is_file())
            self.assertFalse((run_dir / "trajectory_xy.png").exists())

    def test_cylinder_registry_uses_explicit_applicability(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            metadata = {
                "configuration": {"cylindrical_lumen": {"enabled": True}}
            }
            names = plot_artifact_names(
                run_dir, metadata, include_cylinder_plots=True
            )
            registry = plot_producer_registry(
                run_dir, [sample(0.0)], metadata, include_cylinder_plots=True
            )
            self.assertIn("wall_clearance_plot", names)
            self.assertIn("cylinder_backbone_target_plot", names)
            self.assertEqual(names, tuple(registry))
            self.assertFalse((run_dir / "cylinder_navigation.csv").exists())

    def test_curved_lumen_registry_generates_clearance_centerline_and_geometry_plots(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            with (run_dir / "lumen_evaluation.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=("timestamp_s", "physical_clearance_m", "radial_offset_m"),
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "timestamp_s": 0.0,
                        "physical_clearance_m": 0.02,
                        "radial_offset_m": 0.001,
                    }
                )
                writer.writerow(
                    {
                        "timestamp_s": 1.0,
                        "physical_clearance_m": 0.019,
                        "radial_offset_m": 0.002,
                    }
                )
            (run_dir / "summary.json").write_text(
                json.dumps(
                    {
                        "lumen_evaluation": {
                            "identity": {"executed_target": [0.0, 0.0, 0.1]},
                            "geometry": {
                                "fingerprint_payload": {
                                    "centerline_points": [
                                        [0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.1],
                                    ],
                                    "lumen_radius": [0.03, 0.03],
                                }
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            names = plot_artifact_names(run_dir, include_lumen_plots=True)
            self.assertEqual(
                {
                    "curved_wall_clearance_plot",
                    "centerline_tracking_error_plot",
                    "curved_lumen_trajectory_plot",
                },
                set(names)
                - {
                    "tracking_error_plot",
                    "trajectory_xy_plot",
                    "trajectory_3d_plot",
                    "tip_trajectory_plot",
                    "command_history_plot",
                    "solve_time_plot",
                    "cumulative_control_effort_plot",
                },
            )
            registry = plot_producer_registry(
                run_dir,
                [sample(0.0), sample(1.0)],
                include_lumen_plots=True,
            )
            produced = [registry[name](run_dir) for name in names if name.startswith("curved_") or name.startswith("centerline_")]
            self.assertEqual(3, len(produced))
            self.assertTrue(all(path.is_file() and path.stat().st_size > 0 for path in produced))

    def test_plot_dispatch_unknown_name_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            with self.assertRaises(KeyError):
                generate_plot_artifact("missing_plot", Path(temp_dir), [])

    def test_markdown_report_contains_required_sections(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            plot_paths = generate_plots(run_dir, [sample(0.0), sample(1.0)])
            report = generate_report(
                run_dir=run_dir,
                metadata={
                    "run_id": "run_1",
                    "experiment_group": "group",
                    "controller_label": "mppi",
                    "configured_duration": 1.0,
                    "requested_evaluation_duration_s": 25.0,
                    "evaluation_window_duration_s": 25.0,
                    "actual_duration": 1.0,
                    "git": {"short_commit": "abc", "branch": "main", "dirty": False},
                    "topics": {"/ctr/state": {"required": True, "received": True, "count": 2}},
                    "configuration": {
                        "trajectory_type": "circle",
                        "frame_id": "base_link",
                        "configured_control_period": 0.05,
                        "reference_sample_period": 0.05,
                        "software_mode": "simulation",
                    },
                },
                summary={
                    "tracking": {"rmse": 0.1},
                    "control": {"total_control_effort": 0.2},
                    "timing": {"mean_solve_time": 0.3},
                    "data_quality": {"valid_aligned_sample_count": 2},
                    "numerical_safety": {"nonfinite_state_samples": 0},
                    "acceptance": {
                        "functional_pass": True,
                        "numerical_safety_pass": True,
                        "data_quality_pass": True,
                        "baseline_improvement_pass": True,
                        "timing_pass": False,
                        "real_time_pass": False,
                        "physical_validation_pass": False,
                        "hardware_validation_pass": False,
                        "reasons": [],
                    },
                },
                comparison={
                    "compatibility_valid": True,
                    "comparison_valid": True,
                    "compatibility_reasons": [],
                    "metric_comparisons": [
                        {
                            "metric": "rmse",
                            "candidate_value": 0.1,
                            "baseline_value": 0.2,
                            "absolute_difference": -0.1,
                            "relative_improvement_percent": 50.0,
                            "comparison_valid": True,
                        }
                    ],
                },
                plot_paths=plot_paths,
            )
            text = report.read_text(encoding="utf-8")
            self.assertIn("CTR Evaluation Report", text)
            self.assertIn("Topic Status", text)
            self.assertIn("Baseline Comparison", text)
            self.assertIn("Warnings And Limitations", text)
            self.assertIn("requested_evaluation_duration_s: `25.0`", text)
            self.assertIn("evaluation_window_duration_s: `25.0`", text)
            self.assertIn("actual_recording_duration_s: `1.0`", text)
            self.assertNotIn("- configured_duration_s:", text)
            self.assertIn(
                "Timing and solver-performance metrics are descriptive only and are not navigation acceptance criteria.",
                text,
            )
            self.assertIn("| mean_solve_time | 0.3 |", text)
            self.assertNotIn("| timing_pass |", text)
            self.assertNotIn("| real_time_pass |", text)
            self.assertNotIn("not real time", text.lower())

    def test_fixed_target_report_distinguishes_goal_and_centerline_rmse(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            report = generate_report(
                run_dir=run_dir,
                metadata={"run_id": "fixed", "configured_duration": 1.0},
                summary={
                    "run_status": {
                        "status": "completed",
                        "interrupted": False,
                        "completed_evaluation_window": True,
                    },
                    "metric_semantics": {
                        "tracking_rmse_name": "tip_to_target_rmse_m",
                        "tracking_rmse_formula": "sqrt(mean(||tip_i-reference_i||_2^2))",
                        "tracking_rmse_units": "m",
                        "reference_pose_count": 1,
                    },
                    "tracking": {"rmse": 0.002},
                    "goal": {"tip_to_target_rmse_m": 0.002, "final_goal_error": 0.001},
                    "lumen_evaluation": {
                        "progress": {"centerline_tracking_rmse_m": 0.0015}
                    },
                    "acceptance": {"reasons": []},
                },
                comparison=None,
                plot_paths=[],
            )
            text = report.read_text(encoding="utf-8")
            self.assertIn("tip_to_target_rmse_m", text)
            self.assertIn("centerline_tracking_rmse_m", text)
            self.assertNotIn("reference-path tracking RMSE", text)

    def test_development_target_selection_is_explained_in_report(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            report = generate_report(
                run_dir=run_dir,
                metadata={
                    "development_simulation": True,
                    "development_target_selection": {
                        "target_source": "rviz",
                        "raw_input_point": [0.02, 0.03, 0.08],
                        "raw_input_frame": "world",
                        "validated_target": [0.02, 0.0, 0.08],
                        "controller_target_frame": "base_link",
                        "projection_distance_m": 0.03,
                        "acceptance_status": "target_accepted",
                        "orientation_used": False,
                        "reference_pose_count": 1,
                    },
                },
                summary={"acceptance": {"reasons": []}},
                comparison=None,
                plot_paths=[],
            )
            text = report.read_text(encoding="utf-8")
            self.assertIn("Development Target Selection", text)
            self.assertIn("target_source: `rviz`", text)
            self.assertIn("validated_target_m: `[0.02, 0.0, 0.08]`", text)
            self.assertIn("reference_pose_count: `1`", text)

    def test_invalid_comparison_is_reported_without_improvement_claim(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            report = generate_report(
                run_dir=run_dir,
                metadata={},
                summary={},
                comparison={
                    "compatibility_valid": True,
                    "comparison_valid": False,
                    "improvement_evaluated": False,
                    "improvement_pass": None,
                    "compatibility_reasons": [],
                    "metric_comparisons": [],
                },
                plot_paths=[],
            )
            text = report.read_text(encoding="utf-8")
            self.assertIn("Comparison is not valid; improvement was not evaluated.", text)
            self.assertNotIn("candidate improved", text.lower())

    def test_legacy_comparison_without_validity_field_uses_compatibility_fallback(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            report = generate_report(
                run_dir=run_dir,
                metadata={},
                summary={},
                comparison={
                    "compatibility_valid": True,
                    "compatibility_reasons": [],
                    "metric_comparisons": [
                        {
                            "metric": "rmse",
                            "candidate_value": 0.1,
                            "baseline_value": 0.2,
                            "absolute_difference": -0.1,
                            "relative_improvement_percent": 50.0,
                            "comparison_valid": True,
                        }
                    ],
                },
                plot_paths=[],
            )
            text = report.read_text(encoding="utf-8")
            self.assertIn("Baseline Comparison", text)
            self.assertIn("| rmse | 0.1 | 0.2 | -0.1 | 50 | True |", text)
            self.assertNotIn("Comparison is not compatibility-valid.", text)
            self.assertNotIn("Comparison is not valid; improvement was not evaluated.", text)


if __name__ == "__main__":
    unittest.main()
