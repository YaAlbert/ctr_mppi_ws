import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_evaluation.experiment_recorder import (  # noqa: E402
    EvaluationRecorderConfig,
    ExperimentRecorder,
    STATE_COMPLETED,
    STATE_FINALIZING,
    STATE_RECORDING,
    write_json,
)
from ctr_evaluation.compare_results import write_json as write_comparison_json  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "evaluation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def project_config(temp_dir, *, baseline_result_dir=""):
    config = copy.deepcopy(load_parameter_files(CONFIG_FILES))
    validate_or_raise(config)
    config["evaluation"]["output_root"] = str(Path(temp_dir) / "evaluation_results")
    config["evaluation"]["experiment_group"] = "unit_group"
    config["evaluation"]["configured_duration"] = 1.0
    config["evaluation"]["minimum_valid_sample_count"] = 2
    config["evaluation"]["plot_generation"] = True
    config["evaluation"]["report_generation"] = True
    config["evaluation"]["baseline_result_dir"] = baseline_result_dir
    config["cylindrical_lumen"]["enabled"] = False
    return config


def make_recorder(temp_dir, *, baseline_result_dir=""):
    config = project_config(temp_dir, baseline_result_dir=baseline_result_dir)
    return ExperimentRecorder(
        config=EvaluationRecorderConfig.from_project_config(config),
        project_config=config,
    )


def add_samples(recorder, *, tip_offset=0.0):
    recorder.record_state(timestamp=0.0, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=[tip_offset, 0.0, 0.0])
    recorder.record_reference(timestamp=0.0, position=[0.0, 0.0, 0.0], progress=0.0)
    recorder.record_command(timestamp=0.0, command=[0.0] * 6, saturated=False, source="safe_command")
    recorder.record_solve_timing(timestamp=0.0, solve_time=0.01, saturated=False)
    recorder.record_horizon(timestamp=0.0, count=10, first_point=[0.0, 0.0, 0.0], final_point=[1.0, 0.0, 0.0])
    recorder.record_path(timestamp=0.0, count=201)
    recorder.record_tip(timestamp=0.0, position=[tip_offset, 0.0, 0.0])
    recorder.record_state(timestamp=1.0, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=[tip_offset, 0.0, 0.0])
    recorder.record_reference(timestamp=1.0, position=[0.0, 0.0, 0.0], progress=1.0)
    recorder.record_command(timestamp=1.0, command=[0.0] * 6, saturated=False, source="safe_command")
    recorder.record_solve_timing(timestamp=1.0, solve_time=0.02, saturated=False)


def add_cylinder_samples(recorder):
    for timestamp, tip in ((0.0, [0.0192, 0.0, 0.080]), (1.0, [0.015, 0.005, 0.100])):
        recorder.record_state(
            timestamp=timestamp,
            q=[0.0] * 6,
            q_dot=[0.0] * 6,
            tip_position=tip,
            backbone_points=[
                [0.0, 0.0, 0.0],
                [0.010, 0.0, 0.050],
                tip,
            ],
        )
        recorder.record_tip(timestamp=timestamp, position=tip)
        recorder.record_reference(timestamp=timestamp, position=[0.015, 0.005, 0.100], progress=1.0)
        recorder.record_command(timestamp=timestamp, command=[0.0] * 6, saturated=False, source="safe_command")
        recorder.record_solve_timing(timestamp=timestamp, solve_time=0.01, saturated=False)


def strict_json_load(path: Path):
    text = path.read_text(encoding="utf-8")
    for token in ("NaN", "Infinity", "-Infinity"):
        if token in text:
            raise AssertionError(f"non-strict JSON token {token} found in {path}")

    def reject_constant(value):
        raise AssertionError(f"non-strict JSON constant {value} found in {path}")

    return json.loads(text, parse_constant=reject_constant)


class ExperimentRecorderTest(unittest.TestCase):
    def test_lifecycle_writes_raw_summary_report_and_plots(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            run_id = recorder.start(experiment_name="mppi_circle", monotonic_time=0.0)
            self.assertEqual(STATE_RECORDING, recorder.lifecycle_state)
            add_samples(recorder)
            result = recorder.stop(monotonic_time=1.0)
            self.assertEqual(run_id, result.run_id)
            self.assertEqual(STATE_COMPLETED, recorder.lifecycle_state)
            self.assertTrue((result.run_dir / "metadata.yaml").is_file())
            self.assertTrue((result.run_dir / "summary.json").is_file())
            self.assertTrue((result.run_dir / "state.csv").is_file())
            self.assertTrue((result.run_dir / "tip.csv").is_file())
            self.assertTrue((result.run_dir / "aligned_samples.csv").is_file())
            self.assertTrue((result.run_dir / "backbone.csv").is_file())
            self.assertTrue((result.run_dir / "report.md").is_file())
            self.assertTrue((result.run_dir / "tracking_error.png").is_file())
            self.assertFalse((result.run_dir.parent / f"{run_id}.partial").exists())

    def test_cylinder_navigation_outputs_are_written_when_enabled(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            config["cylindrical_lumen"]["enabled"] = True
            recorder = ExperimentRecorder(
                config=EvaluationRecorderConfig.from_project_config(config),
                project_config=config,
            )
            recorder.start(experiment_name="cylinder", monotonic_time=0.0)
            add_cylinder_samples(recorder)
            result = recorder.stop(monotonic_time=1.0)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertIn("goal", summary)
            self.assertIn("lumen_safety", summary)
            self.assertIn("motion", summary)
            self.assertTrue((result.run_dir / "cylinder_navigation.csv").is_file())
            self.assertTrue((result.run_dir / "wall_clearance.png").is_file())
            self.assertTrue((result.run_dir / "cylinder_backbone_target_3d.png").is_file())
            self.assertTrue(summary["lumen_safety"]["collision_free_pass"])

    def test_metadata_and_summary_are_machine_readable(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="metadata", metadata={"case": "unit"}, monotonic_time=0.0)
            add_samples(recorder)
            result = recorder.stop(monotonic_time=1.0)
            metadata = yaml.safe_load((result.run_dir / "metadata.yaml").read_text(encoding="utf-8"))
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertEqual("unit", metadata["metadata_override"]["case"])
            self.assertEqual(2, summary["data_quality"]["valid_aligned_sample_count"])
            self.assertTrue(summary["acceptance"]["functional_pass"])

    def test_repeated_experiment_lifecycle_resets_buffers(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="first", monotonic_time=0.0)
            add_samples(recorder)
            first = recorder.stop(monotonic_time=1.0)
            recorder.start(experiment_name="second", monotonic_time=2.0)
            add_samples(recorder, tip_offset=1.0)
            second = recorder.stop(monotonic_time=3.0)
            self.assertNotEqual(first.run_id, second.run_id)
            second_summary = strict_json_load(second.run_dir / "summary.json")
            self.assertAlmostEqual(1.0, second_summary["tracking"]["rmse"])

    def test_start_while_recording_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="active", monotonic_time=0.0)
            with self.assertRaisesRegex(RuntimeError, "already recording"):
                recorder.start(experiment_name="again", monotonic_time=0.1)

    def test_start_while_finalizing_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="active", monotonic_time=0.0)
            recorder.lifecycle_state = STATE_FINALIZING
            with self.assertRaisesRegex(RuntimeError, "finalizing"):
                recorder.start(experiment_name="again", monotonic_time=0.1)

    def test_stop_while_idle_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            with self.assertRaisesRegex(RuntimeError, "no experiment"):
                recorder.stop(monotonic_time=0.0)

    def test_finalize_and_stop_after_completed_return_existing_result_once(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="single_finalize", monotonic_time=0.0)
            add_samples(recorder)
            result = recorder.stop(monotonic_time=1.0)
            summary_path = result.run_dir / "summary.json"
            mtime_ns = summary_path.stat().st_mtime_ns

            repeated_finalize = recorder.finalize()
            repeated_stop = recorder.stop(monotonic_time=2.0)

            self.assertIs(result, repeated_finalize)
            self.assertIs(result, repeated_stop)
            self.assertEqual(mtime_ns, summary_path.stat().st_mtime_ns)
            self.assertEqual(STATE_COMPLETED, recorder.lifecycle_state)

    def test_samples_are_not_appended_while_finalizing(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="finalizing", monotonic_time=0.0)
            recorder.lifecycle_state = STATE_FINALIZING
            recorder.record_state(timestamp=0.0, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=[0.0, 0.0, 0.0])
            recorder.record_reference(timestamp=0.0, position=[0.0, 0.0, 0.0], progress=0.0)
            self.assertEqual([], recorder.states)
            self.assertEqual([], recorder.references)
            self.assertEqual({}, recorder.topic_counts)

    def test_existing_partial_directory_is_preserved_and_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            run_id = recorder.start(experiment_name="partial_collision", monotonic_time=0.0)
            partial_dir = recorder.config.output_root / recorder.config.experiment_group / f"{run_id}.partial"
            partial_dir.mkdir(parents=True)
            marker = partial_dir / "preserve.txt"
            marker.write_text("keep\n", encoding="utf-8")

            with self.assertRaisesRegex(FileExistsError, "partial result directory"):
                recorder.stop(monotonic_time=1.0)

            self.assertEqual(STATE_FINALIZING, recorder.lifecycle_state)
            self.assertEqual("keep\n", marker.read_text(encoding="utf-8"))
            self.assertTrue(partial_dir.is_dir())

    def test_partial_directory_is_preserved_after_finalization_failure(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            missing_baseline = Path(temp_dir) / "missing_baseline"
            recorder = make_recorder(temp_dir, baseline_result_dir=str(missing_baseline))
            run_id = recorder.start(experiment_name="failure", monotonic_time=0.0)
            add_samples(recorder)

            with self.assertRaises(FileNotFoundError):
                recorder.stop(monotonic_time=1.0)

            partial_dir = recorder.config.output_root / recorder.config.experiment_group / f"{run_id}.partial"
            final_dir = recorder.config.output_root / recorder.config.experiment_group / run_id
            self.assertEqual(STATE_FINALIZING, recorder.lifecycle_state)
            self.assertTrue(partial_dir.is_dir())
            self.assertFalse(final_dir.exists())
            self.assertTrue((partial_dir / "state.csv").is_file())
            error = strict_json_load(partial_dir / "finalization_error.json")
            self.assertEqual(run_id, error["run_id"])
            self.assertEqual(STATE_FINALIZING, error["state"])

    def test_baseline_comparison_is_written_for_compatible_runs(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            baseline = make_recorder(temp_dir)
            baseline.start(experiment_name="zero_command", monotonic_time=0.0)
            add_samples(baseline, tip_offset=1.0)
            baseline_result = baseline.stop(monotonic_time=1.0)

            candidate = make_recorder(temp_dir, baseline_result_dir=str(baseline_result.run_dir))
            candidate.start(experiment_name="mppi", monotonic_time=0.0)
            add_samples(candidate, tip_offset=0.5)
            candidate_result = candidate.stop(monotonic_time=1.0)

            self.assertTrue((candidate_result.run_dir / "comparison.json").is_file())
            self.assertTrue((candidate_result.run_dir / "comparison.md").is_file())
            comparison = strict_json_load(candidate_result.run_dir / "comparison.json")
            summary = strict_json_load(candidate_result.run_dir / "summary.json")
            rmse = [item for item in comparison["metric_comparisons"] if item["metric"] == "rmse"][0]
            self.assertTrue(rmse["comparison_valid"])
            self.assertAlmostEqual(50.0, rmse["relative_improvement_percent"])
            self.assertTrue(summary["acceptance"]["baseline_improvement_pass"])

    def test_strict_json_writers_sanitize_numpy_and_nonfinite_values(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            paths = [Path(temp_dir) / "summary.json", Path(temp_dir) / "comparison.json"]
            data = {
                "finite": np.float64(1.25),
                "nan_value": math.nan,
                "pos_inf": math.inf,
                "neg_inf": -math.inf,
                "array": np.asarray([1.0, math.nan, math.inf]),
                "path": Path("relative/result"),
                "reason": "baseline value is near zero",
            }
            write_json(paths[0], data)
            write_comparison_json(paths[1], data)

            for path in paths:
                parsed = strict_json_load(path)
                self.assertEqual(1.25, parsed["finite"])
                self.assertIsNone(parsed["nan_value"])
                self.assertIsNone(parsed["pos_inf"])
                self.assertIsNone(parsed["neg_inf"])
                self.assertEqual([1.0, None, None], parsed["array"])
                self.assertEqual("relative/result", parsed["path"])
                self.assertEqual("baseline value is near zero", parsed["reason"])

    def test_missing_topic_accounting(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="missing", monotonic_time=0.0)
            result = recorder.stop(monotonic_time=0.1)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertGreater(summary["data_quality"]["missing_topic_count"], 0)
            self.assertFalse(summary["acceptance"]["functional_pass"])
            self.assertIsNone(summary["tracking"]["rmse"])

    def test_out_of_order_solve_timestamps_are_sorted_for_summary(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(experiment_name="out_of_order", monotonic_time=0.0)
            recorder.record_state(timestamp=0.0, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=[0.0, 0.0, 0.0])
            recorder.record_reference(timestamp=0.0, position=[0.0, 0.0, 0.0], progress=0.0)
            recorder.record_state(timestamp=1.0, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=[0.0, 0.0, 0.0])
            recorder.record_reference(timestamp=1.0, position=[0.0, 0.0, 0.0], progress=1.0)
            recorder.record_solve_timing(timestamp=1.0, solve_time=0.02, saturated=False)
            recorder.record_solve_timing(timestamp=0.0, solve_time=0.01, saturated=False)
            result = recorder.stop(monotonic_time=1.0)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertAlmostEqual(1.0, summary["timing"]["effective_solve_frequency"])


if __name__ == "__main__":
    unittest.main()
