import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

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
    PromotionStatus,
    write_json,
)
from ctr_evaluation.compare_results import write_json as write_comparison_json  # noqa: E402
import ctr_evaluation.report_generator as report_module  # noqa: E402
import ctr_evaluation.experiment_recorder as recorder_module  # noqa: E402
from ctr_evaluation.publication_model import (  # noqa: E402
    Applicability,
    ArtifactRepresentation,
    ArtifactSpec,
    LayerASnapshot,
    PublicationStatus,
    RecordPhase,
)
from ctr_evaluation.experiment_recorder import (  # noqa: E402
    StagingSetupError,
    ProducerRenderError,
    ProducerStagingError,
    prepare_prepromotion_ledger,
    promote_prepromotion_ledger,
)


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


def make_uninstrumented_recorder(temp_dir):
    config = project_config(temp_dir)

    class NoProfilingConfig:
        def __init__(self, delegate):
            self._delegate = delegate

        def __getattribute__(self, name):
            if name == "enable_finalization_profiling":
                raise AssertionError(
                    "profiling configuration must not be required"
                )
            if name == "_delegate":
                return object.__getattribute__(self, name)
            return getattr(object.__getattribute__(self, "_delegate"), name)

    class NoInstrumentationRecorder(ExperimentRecorder):
        def __getattribute__(self, name):
            if name in {
                "record_diagnostic_event",
                "_record_metric_stage_end",
                "_finalization_trace_path",
            }:
                raise AssertionError(
                    "forbidden instrumentation lookup: " f"{name}"
                )
            return super().__getattribute__(name)

        def __setattr__(self, name, value):
            if (
                name == "_finalization_trace_path"
                and object.__getattribute__(self, "__dict__").get(
                    "_isolation_active", False
                )
            ):
                raise AssertionError(
                    "forbidden instrumentation write: "
                    "_finalization_trace_path"
                )
            object.__setattr__(self, name, value)

    recorder = NoInstrumentationRecorder(
        config=EvaluationRecorderConfig.from_project_config(config),
        project_config=config,
    )
    recorder.config = NoProfilingConfig(recorder.config)
    return recorder


class ExperimentRecorderTest(unittest.TestCase):
    def test_report_is_generated_when_report_generation_flag_is_disabled(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            config["evaluation"]["plot_generation"] = False
            config["evaluation"]["report_generation"] = False
            recorder = ExperimentRecorder(
                config=EvaluationRecorderConfig.from_project_config(config),
                project_config=config,
            )
            recorder.start(experiment_name="automatic_report", monotonic_time=0.0)
            add_samples(recorder)
            result = recorder.stop(monotonic_time=1.0)

            self.assertTrue((result.run_dir / "summary.json").is_file())
            self.assertTrue((result.run_dir / "report.md").is_file())

    def test_baseline_report_is_generated_once_after_comparison(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            baseline = make_recorder(temp_dir)
            baseline.start(experiment_name="baseline", monotonic_time=0.0)
            add_samples(baseline, tip_offset=1.0)
            baseline_result = baseline.stop(monotonic_time=1.0)

            config = project_config(temp_dir, baseline_result_dir=str(baseline_result.run_dir))
            config["evaluation"]["plot_generation"] = False
            config["evaluation"]["report_generation"] = False
            candidate = ExperimentRecorder(
                config=EvaluationRecorderConfig.from_project_config(config),
                project_config=config,
            )
            calls = []
            original_generate_report = report_module.generate_report

            def spy_generate_report(**kwargs):
                calls.append(kwargs["comparison"])
                return original_generate_report(**kwargs)

            report_module.generate_report = spy_generate_report
            try:
                candidate.start(experiment_name="candidate", monotonic_time=0.0)
                add_samples(candidate, tip_offset=0.5)
                result = candidate.stop(monotonic_time=1.0)
            finally:
                report_module.generate_report = original_generate_report

            self.assertEqual(1, len(calls))
            self.assertIsNotNone(calls[0])
            self.assertTrue(calls[0]["comparison_valid"])
            report_text = (result.run_dir / "report.md").read_text(encoding="utf-8")
            self.assertIn("Baseline Comparison", report_text)
            self.assertIn("| rmse | 0.5 | 1 | -0.5 | 50 | True |", report_text)

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

            result = recorder.stop(monotonic_time=1.0)

            partial_dir = recorder.config.output_root / recorder.config.experiment_group / f"{run_id}.partial"
            final_dir = recorder.config.output_root / recorder.config.experiment_group / run_id
            self.assertEqual(STATE_COMPLETED, recorder.lifecycle_state)
            self.assertTrue(partial_dir.is_dir())
            self.assertFalse(final_dir.exists())
            self.assertTrue((partial_dir / "state.csv").is_file())
            error = strict_json_load(partial_dir / "finalization_error.json")
            self.assertEqual(run_id, error["run_id"])
            self.assertEqual(STATE_FINALIZING, error["state"])
            self.assertIs(
                PromotionStatus.PROMOTION_REFUSED, result.promotion.status
            )

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

    def test_disconnected_prepromotion_pipeline_continues_and_terminalizes(
        self,
    ):
        def make_layer():
            return LayerASnapshot(
                snapshot_id="slice2",
                operational_reason="none",
                workflow_classification="COMPLETED",
                workflow_exit_code=0,
                comparison_valid=True,
                compatibility_valid=True,
                cancellation_evidence=(),
                timing_data=(),
            )

        specs = (
            ArtifactSpec(
                "root", "root.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "sibling", "sibling.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "child", "child.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file", ("root",),
            ),
            ArtifactSpec(
                "optional", "optional.json", False,
                Applicability.NOT_APPLICABLE, ArtifactRepresentation.OPAQUE,
                "regular_file",
            ),
            ArtifactSpec(
                "optional_child", "optional_child.json", True,
                Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE,
                "regular_file", ("optional",),
            ),
            ArtifactSpec(
                "report", "report.md", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
                ("sibling", "child"),
            ),
            ArtifactSpec(
                "orchestration", "orchestration.json", True,
                Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE,
                "regular_file",
            ),
        )
        calls = []

        def fail_root(path):
            calls.append("root")
            raise RuntimeError("root failure")

        def write(name):
            def producer(path):
                calls.append(name)
                path.write_text(name, encoding="utf-8")
            return producer

        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=make_layer(),
                inventory=specs,
                staging_root=Path(temp_dir) / "run.partial",
                applicability={},
                producer_registry={
                    "root": fail_root,
                    "sibling": write("sibling"),
                    "optional_child": write("optional_child"),
                },
                report_producer=write("report"),
                orchestration_producer=write("orchestration"),
            )
            by_name = ledger.by_name
            self.assertEqual(
                PublicationStatus.RENDER_FAILED,
                by_name["root"].publication_status,
            )
            self.assertEqual(
                PublicationStatus.DEPENDENCY_FAILED,
                by_name["child"].publication_status,
            )
            self.assertEqual(
                "sibling",
                (Path(temp_dir) / "run.partial" / "sibling.json").read_text(),
            )
            self.assertIsNone(by_name["optional_child"].publication_status)
            self.assertEqual(0, calls.count("report"))
            self.assertNotIn("child", calls)
            self.assertTrue(all(
                record.record_phase is RecordPhase.PRE_PROMOTION
                or record.publication_status
                is PublicationStatus.NOT_APPLICABLE
                for record in ledger.records
            ))
            self.assertTrue(all(
                record.visibility_status.name == "NOT_OBSERVED"
                for record in ledger.records
                if record.publication_status
                is not PublicationStatus.NOT_APPLICABLE
            ))
            self.assertFalse((Path(temp_dir) / "run").exists())

    def test_disconnected_prepromotion_orchestration_policy_and_empty_staging(
        self,
    ):
        layer = LayerASnapshot(
            "slice2", "none", "COMPLETED", 0, True,
            compatibility_valid=True,
        )
        specs = (
            ArtifactSpec(
                "orchestration", "orchestration.json", True,
                Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE,
                "regular_file",
            ),
            ArtifactSpec(
                "optional", "optional.json", False, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=specs,
                staging_root=Path(temp_dir) / "empty.partial",
                applicability={"optional": Applicability.NOT_APPLICABLE},
                producer_registry={},
                report_producer=None,
            )
            self.assertEqual(
                PublicationStatus.DEPENDENCY_FAILED,
                ledger.by_name["orchestration"].publication_status,
            )
            self.assertEqual(
                PublicationStatus.NOT_APPLICABLE,
                ledger.by_name["optional"].publication_status,
            )
            self.assertFalse(any((Path(temp_dir) / "empty.partial").iterdir()))
            with self.assertRaises(StagingSetupError):
                prepare_prepromotion_ledger(
                    layer_a=layer,
                    inventory=specs,
                    staging_root=Path(temp_dir) / "empty.partial",
                    applicability={},
                    producer_registry={},
                    report_producer=None,
                )

    def test_disconnected_report_is_attempted_once_after_comparison(self):
        layer = LayerASnapshot(
            "slice2", "none", "COMPLETED", 0, True,
            compatibility_valid=True,
        )
        specs = (
            ArtifactSpec(
                "metadata", "metadata.yaml", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "comparison", "comparison.json", False,
                Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE,
                "regular_file", ("metadata",),
            ),
            ArtifactSpec(
                "report", "report.md", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file", ("comparison",),
            ),
        )
        order = []

        def producer(name):
            def write(path):
                order.append(name)
                path.write_text(name, encoding="utf-8")
            return write

        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=specs,
                staging_root=Path(temp_dir) / "report.partial",
                applicability={},
                producer_registry={"metadata": producer("metadata")},
                comparison_producer=producer("comparison"),
                report_producer=producer("report"),
            )
            self.assertEqual(["metadata", "comparison", "report"], order)
            self.assertEqual(1, order.count("report"))
            self.assertIsNone(ledger.by_name["report"].publication_status)

    def test_disconnected_multiple_failed_parents_are_deterministic(self):
        layer = LayerASnapshot("slice2", "none", "COMPLETED", 0, True)
        specs = (
            ArtifactSpec(
                "a", "a.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "b", "b.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "child", "child.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file", ("b", "a"),
            ),
            ArtifactSpec(
                "sibling", "sibling.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
        )
        calls = []

        def fail(path):
            raise ProducerRenderError("root failed")

        def sibling(path):
            calls.append("sibling")
            path.write_text("sibling", encoding="utf-8")

        def child(path):
            calls.append("child")
            path.write_text("child", encoding="utf-8")

        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=specs,
                staging_root=Path(temp_dir) / "parents.partial",
                applicability={},
                producer_registry={
                    "a": fail,
                    "b": fail,
                    "child": child,
                    "sibling": sibling,
                },
                report_producer=None,
            )
            child_record = ledger.by_name["child"]
            self.assertEqual(
                PublicationStatus.DEPENDENCY_FAILED,
                child_record.publication_status,
            )
            self.assertEqual(
                "applicable dependency failed for child: a, b",
                child_record.failure_reason,
            )
            self.assertNotIn("child", calls)
            self.assertIn("sibling", calls)

    def test_orchestration_not_applicable_preserves_requiredness(self):
        layer = LayerASnapshot("slice2", "none", "COMPLETED", 0, True)
        orchestration = ArtifactSpec(
            "orchestration", "orchestration.json", True,
            Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(orchestration,),
                staging_root=Path(temp_dir) / "orchestration.partial",
                applicability={"orchestration": Applicability.NOT_APPLICABLE},
                producer_registry={},
                report_producer=None,
            )
            record = ledger.by_name["orchestration"]
            self.assertEqual(
                PublicationStatus.NOT_APPLICABLE,
                record.publication_status,
            )
            self.assertTrue(record.required)
            self.assertIs(record.applicability, Applicability.APPLICABLE)

    def test_finalization_trace_write_failure_is_artifact_local(self):
        layer = LayerASnapshot("slice2", "none", "COMPLETED", 0, True)
        trace = ArtifactSpec(
            "finalization_trace", "finalization_trace.json", False,
            Applicability.APPLICABLE, ArtifactRepresentation.OPAQUE,
            "regular_file",
        )
        sibling = ArtifactSpec(
            "sibling", "sibling.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )

        original_write_text = Path.write_text

        def write_text(path, *args, **kwargs):
            if path.name == "finalization_trace.json":
                raise OSError("trace write failed")
            return original_write_text(path, *args, **kwargs)

        with tempfile.TemporaryDirectory() as temp_dir:
            with mock.patch.object(Path, "write_text", new=write_text):
                ledger = prepare_prepromotion_ledger(
                    layer_a=layer,
                    inventory=(trace, sibling),
                    staging_root=Path(temp_dir) / "trace.partial",
                    applicability={},
                    producer_registry={
                        "sibling": lambda path: path.write_text("ok")
                    },
                    report_producer=None,
                )
            self.assertEqual(
                PublicationStatus.STAGE_FAILED,
                ledger.by_name["finalization_trace"].publication_status,
            )
            self.assertIsNone(ledger.by_name["sibling"].publication_status)

    def test_disconnected_partial_output_render_failure_is_explicit(self):
        layer = LayerASnapshot("slice2", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "broken", "broken.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )

        def broken(path):
            path.write_text("partial", encoding="utf-8")
            raise ProducerRenderError("render interrupted")

        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=Path(temp_dir) / "broken.partial",
                applicability={},
                producer_registry={"broken": broken},
                report_producer=None,
            )
            self.assertEqual(
                PublicationStatus.RENDER_FAILED,
                ledger.by_name["broken"].publication_status,
            )
            self.assertFalse(
                (Path(temp_dir) / "broken.partial" / "broken.json").exists()
            )

    def test_disconnected_explicit_stage_failure_before_target_continues(self):
        layer = LayerASnapshot("slice2", "none", "COMPLETED", 0, True)
        specs = (
            ArtifactSpec(
                "broken", "broken.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "sibling", "sibling.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
        )
        calls = []

        def broken(path):
            calls.append("broken")
            raise ProducerStagingError("stage interrupted")

        def sibling(path):
            calls.append("sibling")
            path.write_text("sibling", encoding="utf-8")

        with tempfile.TemporaryDirectory() as temp_dir:
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=specs,
                staging_root=Path(temp_dir) / "explicit.partial",
                applicability={},
                producer_registry={"broken": broken, "sibling": sibling},
                report_producer=None,
            )
            self.assertEqual(
                PublicationStatus.STAGE_FAILED,
                ledger.by_name["broken"].publication_status,
            )
            self.assertIsNone(ledger.by_name["sibling"].publication_status)
            self.assertEqual(["broken", "sibling"], calls)

    def test_nested_parent_failure_continues(self):
        layer = LayerASnapshot("slice2", "none", "COMPLETED", 0, True)
        specs = (
            ArtifactSpec(
                "nested", "nested/artifact.json", True,
                Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
            ArtifactSpec(
                "sibling", "sibling.json", True, Applicability.APPLICABLE,
                ArtifactRepresentation.OPAQUE, "regular_file",
            ),
        )

        def write(path):
            path.write_text("ok", encoding="utf-8")

        original_mkdir = Path.mkdir

        def fail_nested(path, *args, **kwargs):
            if path.name == "nested":
                raise OSError("cannot create nested parent")
            return original_mkdir(path, *args, **kwargs)

        with tempfile.TemporaryDirectory() as temp_dir:
            with mock.patch.object(Path, "mkdir", new=fail_nested):
                ledger = prepare_prepromotion_ledger(
                    layer_a=layer,
                    inventory=specs,
                    staging_root=Path(temp_dir) / "mkdir.partial",
                    applicability={},
                    producer_registry={"nested": write, "sibling": write},
                    report_producer=None,
                )
            self.assertEqual(
                PublicationStatus.STAGE_FAILED,
                ledger.by_name["nested"].publication_status,
            )
            self.assertIsNone(ledger.by_name["sibling"].publication_status)

    def test_finalize_succeeds_without_profiling_or_diagnostic_symbols(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_uninstrumented_recorder(temp_dir)
            recorder.start(
                experiment_name="uninstrumented", monotonic_time=0.0
            )
            add_samples(recorder)
            if "_finalization_trace_path" in recorder.__dict__:
                delattr(recorder, "_finalization_trace_path")
            object.__setattr__(recorder, "_isolation_active", True)
            with mock.patch.object(
                recorder_module.cProfile,
                "Profile",
                side_effect=AssertionError(
                    "profiling must not be instantiated"
                ),
            ):
                result = recorder.finalize()
            self.assertIs(
                PromotionStatus.PROMOTED_AND_OBSERVED,
                result.promotion.status,
            )
            self.assertEqual(STATE_COMPLETED, recorder.lifecycle_state)
            self.assertNotIn("_finalization_trace_path", recorder.__dict__)

    def test_required_failure_returns_evidence_without_profiling_symbols(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_uninstrumented_recorder(temp_dir)
            recorder.start(
                experiment_name="uninstrumented_failure",
                monotonic_time=0.0,
            )
            add_samples(recorder)
            if "_finalization_trace_path" in recorder.__dict__:
                delattr(recorder, "_finalization_trace_path")
            object.__setattr__(recorder, "_isolation_active", True)
            captured = {}
            original_promote = recorder_module.promote_prepromotion_ledger

            def capture(ledger, final_dir):
                captured["ledger"] = ledger
                return original_promote(ledger, final_dir)

            with mock.patch.object(
                recorder,
                "_write_raw_files",
                side_effect=RuntimeError("raw failure"),
            ), mock.patch.object(
                recorder_module,
                "promote_prepromotion_ledger",
                side_effect=capture,
            ), mock.patch.object(
                recorder_module.cProfile,
                "Profile",
                side_effect=AssertionError(
                    "profiling must not be instantiated"
                ),
            ):
                result = recorder.finalize()
            self.assertIs(
                PromotionStatus.PROMOTION_REFUSED,
                result.promotion.status,
            )
            self.assertIs(
                PublicationStatus.STAGE_FAILED,
                captured["ledger"].by_name["raw_state"].publication_status,
            )
            self.assertTrue(result.run_dir.name.endswith(".partial"))
            self.assertTrue(result.run_dir.is_dir())
            self.assertNotIn("_finalization_trace_path", recorder.__dict__)

    def test_applicable_cylinder_failure_is_not_reclassified_as_inapplicable(
        self,
    ):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            config["cylindrical_lumen"]["enabled"] = True
            recorder = ExperimentRecorder(
                config=EvaluationRecorderConfig.from_project_config(config),
                project_config=config,
            )
            captured = {}
            original_promote = recorder_module.promote_prepromotion_ledger

            def capture(ledger, final_dir):
                captured["ledger"] = ledger
                return original_promote(ledger, final_dir)

            recorder.start(
                experiment_name="cylinder_failure", monotonic_time=0.0
            )
            add_cylinder_samples(recorder)
            with mock.patch.object(
                recorder,
                "_write_raw_files",
                side_effect=RuntimeError("raw failure"),
            ), mock.patch.object(
                recorder_module,
                "promote_prepromotion_ledger",
                side_effect=capture,
            ):
                result = recorder.stop(monotonic_time=1.0)
            record = captured["ledger"].by_name["cylinder_navigation"]
            self.assertIs(
                Applicability.APPLICABLE, record.execution_applicability
            )
            self.assertIs(
                PublicationStatus.STAGE_FAILED, record.publication_status
            )
            self.assertIs(
                PromotionStatus.PROMOTION_REFUSED, result.promotion.status
            )

    def test_applicable_lumen_failure_is_not_reclassified_as_inapplicable(
        self,
    ):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            captured = {}
            original_promote = recorder_module.promote_prepromotion_ledger

            def capture(ledger, final_dir):
                captured["ledger"] = ledger
                return original_promote(ledger, final_dir)

            recorder.start(
                experiment_name="lumen_failure",
                metadata={"curved_lumen_type": "circular_arc"},
                monotonic_time=0.0,
            )
            add_samples(recorder)
            with mock.patch.object(
                recorder_module,
                "_has_curved_lumen_metadata",
                return_value=True,
            ), mock.patch.object(
                recorder,
                "_lumen_evaluation_result",
                side_effect=RuntimeError("lumen failure"),
            ), mock.patch.object(
                recorder_module,
                "promote_prepromotion_ledger",
                side_effect=capture,
            ):
                result = recorder.stop(monotonic_time=1.0)
            record = captured["ledger"].by_name["lumen_evaluation"]
            self.assertIs(
                Applicability.APPLICABLE,
                record.execution_applicability,
            )
            self.assertIs(
                PublicationStatus.STAGE_FAILED,
                record.publication_status,
            )
            self.assertIs(
                PromotionStatus.PROMOTED_AND_OBSERVED,
                result.promotion.status,
            )

    def test_non_applicable_lumen_and_cylinder_remain_not_applicable(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            captured = {}
            original_promote = recorder_module.promote_prepromotion_ledger

            def capture(ledger, final_dir):
                captured["ledger"] = ledger
                return original_promote(ledger, final_dir)

            recorder.start(experiment_name="no_lumen", monotonic_time=0.0)
            add_samples(recorder)
            with mock.patch.object(
                recorder_module,
                "promote_prepromotion_ledger",
                side_effect=capture,
            ):
                recorder.stop(monotonic_time=1.0)
            ledger = captured["ledger"]
            self.assertIs(
                Applicability.NOT_APPLICABLE,
                ledger.by_name["lumen_evaluation"].execution_applicability,
            )
            self.assertIs(
                Applicability.NOT_APPLICABLE,
                ledger.by_name["cylinder_navigation"].execution_applicability,
            )

    def test_slice3_valid_ledger_promotes_and_observes_final_paths(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    )
                },
                report_producer=None,
            )
            result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(
                PromotionStatus.PROMOTED_AND_OBSERVED,
                result.status,
            )
            self.assertTrue((root / "run" / "payload.json").is_file())
            self.assertFalse((root / "run.partial").exists())

    def test_slice3_required_failure_refuses_and_preserves_staging(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: (_ for _ in ()).throw(
                        ProducerRenderError("render failed")
                    )
                },
                report_producer=None,
            )
            result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(PromotionStatus.PROMOTION_REFUSED, result.status)
            self.assertTrue((root / "run.partial").is_dir())
            self.assertFalse((root / "run").exists())

    def test_slice3_optional_failure_does_not_block_required_promotion(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        required = ArtifactSpec(
            "required", "required.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        optional = ArtifactSpec(
            "optional", "optional.json", False, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(required, optional),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "required": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    ),
                    "optional": lambda path: (_ for _ in ()).throw(
                        ProducerRenderError("optional failed")
                    ),
                },
                report_producer=None,
            )
            result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(
                PromotionStatus.PROMOTED_AND_OBSERVED,
                result.status,
            )
            self.assertTrue((root / "run" / "required.json").is_file())

    def test_slice3_missing_final_path_is_observation_failure(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    )
                },
                report_producer=None,
            )
            original_replace = Path.replace

            def replace_then_remove(path, target):
                result = original_replace(path, target)
                (target / "payload.json").unlink()
                return result

            with mock.patch.object(
                Path, "replace", new=replace_then_remove
            ):
                result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(
                PromotionStatus.PROMOTED_OBSERVATION_FAILED,
                result.status,
            )
            self.assertTrue((root / "run").is_dir())

    def test_slice3_promotion_failure_is_distinct_from_refusal(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    )
                },
                report_producer=None,
            )
            with mock.patch.object(
                Path, "replace", side_effect=OSError("rename failed")
            ):
                result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(PromotionStatus.PROMOTION_FAILED, result.status)
            self.assertTrue((root / "run.partial").is_dir())

    def test_slice3_existing_final_directory_refuses_overwrite(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "run").mkdir()
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    )
                },
                report_producer=None,
            )
            result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(PromotionStatus.PROMOTION_REFUSED, result.status)
            self.assertTrue((root / "run" / "payload.json").exists() is False)

    def test_slice3_empty_staging_is_eligible(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "optional", "optional.json", False,
            Applicability.NOT_APPLICABLE, ArtifactRepresentation.OPAQUE,
            "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "empty.partial",
                applicability=None,
                producer_registry={},
                report_producer=None,
            )
            result = promote_prepromotion_ledger(ledger, root / "empty")
            self.assertIs(
                PromotionStatus.PROMOTED_AND_OBSERVED,
                result.status,
            )
            self.assertTrue((root / "empty").is_dir())

    def test_slice3_nonregular_and_symlink_outputs_do_not_promote(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        for mode in ("directory", "symlink"):
            with self.subTest(mode=mode):
                with tempfile.TemporaryDirectory() as temp_dir:
                    root = Path(temp_dir)
                    outside = root / "outside.txt"
                    outside.write_text("outside", encoding="utf-8")

                    def producer(path, mode=mode):
                        if mode == "directory":
                            path.mkdir()
                        else:
                            path.symlink_to(outside)

                    ledger = prepare_prepromotion_ledger(
                        layer_a=layer,
                        inventory=(spec,),
                        staging_root=root / "run.partial",
                        applicability=None,
                        producer_registry={"payload": producer},
                        report_producer=None,
                    )
                    result = promote_prepromotion_ledger(ledger, root / "run")
                    self.assertIs(
                        PromotionStatus.PROMOTION_REFUSED,
                        result.status,
                    )
                    self.assertTrue((root / "run.partial").is_dir())

    def test_slice3_repeated_promotion_is_refused(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    )
                },
                report_producer=None,
            )
            first = promote_prepromotion_ledger(ledger, root / "run")
            second = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(PromotionStatus.PROMOTED_AND_OBSERVED, first.status)
            self.assertIs(PromotionStatus.PROMOTION_REFUSED, second.status)

    def test_slice3_unexpected_directory_fails_observation(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)

            def producer(path):
                path.write_text("ok", encoding="utf-8")
                path.parent.joinpath("unexpected").mkdir()

            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={"payload": producer},
                report_producer=None,
            )
            result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(
                PromotionStatus.PROMOTED_OBSERVATION_FAILED,
                result.status,
            )

    def test_slice3_observation_error_write_failure_preserves_status(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = make_recorder(temp_dir)
            recorder.start(
                experiment_name="observation_error",
                monotonic_time=0.0,
            )
            add_samples(recorder)
            original_replace = Path.replace

            def replace_then_remove(path, target):
                result = original_replace(path, target)
                (target / "state.csv").unlink()
                return result

            with mock.patch.object(
                Path, "replace", new=replace_then_remove
            ), mock.patch.object(
                recorder,
                "_write_finalization_error",
                side_effect=OSError("cannot write evidence"),
            ):
                result = recorder.stop(monotonic_time=1.0)
            self.assertIs(
                PromotionStatus.PROMOTED_OBSERVATION_FAILED,
                result.promotion.status,
            )
            self.assertTrue(result.run_dir.is_dir())

    def test_slice3_final_directory_race_reports_promotion_failure(self):
        layer = LayerASnapshot("slice3", "none", "COMPLETED", 0, True)
        spec = ArtifactSpec(
            "payload", "payload.json", True, Applicability.APPLICABLE,
            ArtifactRepresentation.OPAQUE, "regular_file",
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            ledger = prepare_prepromotion_ledger(
                layer_a=layer,
                inventory=(spec,),
                staging_root=root / "run.partial",
                applicability=None,
                producer_registry={
                    "payload": lambda path: path.write_text(
                        "ok", encoding="utf-8"
                    )
                },
                report_producer=None,
            )

            def race(path, target):
                target.mkdir()
                raise OSError("final path appeared")

            with mock.patch.object(Path, "replace", new=race):
                result = promote_prepromotion_ledger(ledger, root / "run")
            self.assertIs(PromotionStatus.PROMOTION_FAILED, result.status)
            self.assertTrue((root / "run.partial").is_dir())
            self.assertTrue((root / "run").is_dir())

if __name__ == "__main__":
    unittest.main()
