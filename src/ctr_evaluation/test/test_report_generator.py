import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_evaluation.report_generator import generate_plots, generate_report  # noqa: E402
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
                        "reasons": ["not real time"],
                    },
                },
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
                plot_paths=plot_paths,
            )
            text = report.read_text(encoding="utf-8")
            self.assertIn("CTR Evaluation Report", text)
            self.assertIn("Topic Status", text)
            self.assertIn("Baseline Comparison", text)
            self.assertIn("Warnings And Limitations", text)


if __name__ == "__main__":
    unittest.main()
