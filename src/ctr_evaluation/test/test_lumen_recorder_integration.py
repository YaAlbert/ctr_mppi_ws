import copy
import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))

import ctr_evaluation.experiment_recorder as recorder_module  # noqa: E402
from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_evaluation.curved_lumen_scenarios import (  # noqa: E402
    CENTERLINE_TARGET,
    LATERAL_OFFSET_TARGET,
    NEAR_SAFETY_BOUNDARY_TARGET,
    resolve_curved_lumen_scenario,
)
from ctr_evaluation.experiment_recorder import EvaluationRecorderConfig, ExperimentRecorder  # noqa: E402
from ctr_evaluation.time_alignment import AlignedSample, AlignmentDiagnostics, AlignmentResult  # noqa: E402
from ctr_mppi_controller.lumen_factory import config_with_lumen_overrides, lumen_geometry_from_config  # noqa: E402


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


def project_config(temp_dir):
    config = copy.deepcopy(load_parameter_files(CONFIG_FILES))
    config["evaluation"]["output_root"] = str(Path(temp_dir) / "evaluation_results")
    config["evaluation"]["experiment_group"] = "d2c1_unit"
    config["evaluation"]["configured_duration"] = 1.0
    config["evaluation"]["minimum_valid_sample_count"] = 2
    config["evaluation"]["plot_generation"] = False
    config["evaluation"]["report_generation"] = False
    config["cylindrical_lumen"]["enabled"] = False
    config["curved_lumen"]["enabled"] = False
    config["reference"]["mode"] = "fixed_target"
    validate_or_raise(config)
    return config


def make_recorder(config):
    return ExperimentRecorder(
        config=EvaluationRecorderConfig.from_project_config(config),
        project_config=config,
    )


def scenario_and_metadata(config, *, curved_type="circular_arc", scenario_id=CENTERLINE_TARGET, run_id="curved_run"):
    effective = config_with_lumen_overrides(
        config,
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type=curved_type,
    )
    scenario = resolve_curved_lumen_scenario(
        effective,
        scenario_id,
        curved_lumen_type=curved_type,
    )
    identity = {
        "requested_target": [float(value) for value in scenario.requested_target],
        "executed_target": [float(value) for value in scenario.validated_target],
        "validated_target": [float(value) for value in scenario.validated_target],
        "derived_target": [float(value) for value in scenario.derived_target],
        "target_replaced": False,
        "target_identity_valid": True,
        "target_identity_tolerance": 1.0e-9,
        "override_used": bool(scenario.override_used),
        "target_override_used": bool(scenario.override_used),
        "reference_mode": "fixed_target",
        "target_mode": scenario.target_mode,
        "curved_lumen_type": scenario.curved_lumen_type,
        "scenario_id": scenario.scenario_id,
        "scenario_policy_version": scenario.policy_version,
        "scenario_fingerprint": scenario.scenario_fingerprint,
        "geometry_frame": scenario.geometry_frame,
        "geometry_fingerprint": scenario.geometry_fingerprint,
        "centerline_fraction": float(scenario.centerline_fraction),
        "centerline_arc_length": float(scenario.centerline_arc_length),
        "radial_offset": float(scenario.radial_offset),
    }
    metadata = {
        "requested_run_id": run_id,
        "run_role": "candidate",
        "task": "curved_lumen_navigation",
        "shared_environment_hash": "shared_curved_environment",
        **identity,
        "reference_configuration": {
            "task": "curved_lumen_navigation",
            "reference_mode": "fixed_target",
            "goal_position": identity["executed_target"],
            "curved_scenario": dict(identity),
        },
    }
    return scenario, metadata


def add_curved_samples(recorder, scenario, *, backbones=None, times=(0.0, 1.0)):
    target = np.asarray(scenario.validated_target, dtype=float)
    center = np.asarray(scenario.centerline_point, dtype=float)
    if backbones is None:
        midpoint = 0.5 * (center + target)
        backbones = [
            np.vstack([center, target]),
            np.vstack([center, midpoint, target]),
        ]
    for timestamp, backbone in zip(times, backbones):
        tip = np.asarray(backbone[-1], dtype=float)
        recorder.record_state(
            timestamp=timestamp,
            q=[0.0] * 6,
            q_dot=[0.0] * 6,
            tip_position=tip,
            backbone_points=backbone,
        )
        recorder.record_tip(timestamp=timestamp, position=tip)
        recorder.record_reference(timestamp=timestamp, position=target, progress=None)
        recorder.record_command(timestamp=timestamp, command=[0.0] * 6, saturated=False, source="safe_command")
        recorder.record_solve_timing(timestamp=timestamp, solve_time=0.01, saturated=False)


def add_simple_trajectory_samples(recorder):
    for timestamp in (0.0, 1.0):
        recorder.record_state(timestamp=timestamp, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=[0.0, 0.0, 0.0])
        recorder.record_reference(timestamp=timestamp, position=[0.0, 0.0, 0.0], progress=timestamp)
        recorder.record_command(timestamp=timestamp, command=[0.0] * 6, saturated=False, source="safe_command")


def aligned_lumen_sample(timestamp, backbone, tip):
    return AlignedSample(
        timestamp=timestamp,
        q=np.zeros(6),
        q_dot=np.zeros(6),
        tip_position=np.asarray(tip, dtype=float),
        backbone_points=backbone,
        reference_position=np.asarray(tip, dtype=float),
        command=np.zeros(6),
        solve_time=0.01,
        command_saturated=False,
        missing_command=False,
        reference_gap=0.0,
        command_gap=0.0,
        solve_gap=0.0,
        used_reference_interpolation=False,
        used_nearest_reference=False,
        reference_progress=None,
    )


def lumen_alignment(samples):
    return AlignmentResult(
        samples=list(samples),
        diagnostics=AlignmentDiagnostics(
            raw_state_sample_count=len(samples),
            raw_reference_sample_count=len(samples),
            raw_command_sample_count=len(samples),
            valid_aligned_sample_count=len(samples),
            rejected_aligned_sample_count=0,
            invalid_nonfinite_sample_count=0,
            mean_alignment_gap=0.0,
            maximum_alignment_gap=0.0,
            reference_interpolation_count=0,
            nearest_reference_fallback_count=0,
            missing_command_count=0,
            rejection_reasons={},
        ),
    )


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
        recorder.record_reference(timestamp=timestamp, position=[0.015, 0.005, 0.100], progress=1.0)
        recorder.record_command(timestamp=timestamp, command=[0.0] * 6, saturated=False, source="safe_command")


def strict_json_load(path: Path):
    text = path.read_text(encoding="utf-8")
    for token in ("NaN", "Infinity", "-Infinity"):
        if token in text:
            raise AssertionError(f"non-strict JSON token {token} found in {path}")
    return json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(AssertionError(value)))


def assert_no_nonfinite_json_numbers(testcase, value):
    if isinstance(value, dict):
        for item in value.values():
            assert_no_nonfinite_json_numbers(testcase, item)
    elif isinstance(value, list):
        for item in value:
            assert_no_nonfinite_json_numbers(testcase, item)
    elif isinstance(value, float):
        testcase.assertTrue(math.isfinite(value))


class LumenRecorderIntegrationTest(unittest.TestCase):
    def test_curved_summary_invokes_d1_once_for_each_curved_type_and_writes_csv(self):
        for curved_type in ("circular_arc", "s_curve"):
            with self.subTest(curved_type=curved_type), tempfile.TemporaryDirectory() as temp_dir:
                config = project_config(temp_dir)
                original_lumen_modes = (
                    config["cylindrical_lumen"]["enabled"],
                    config["curved_lumen"]["enabled"],
                    config["curved_lumen"]["type"],
                )
                scenario, metadata = scenario_and_metadata(config, curved_type=curved_type, run_id=f"{curved_type}_run")
                recorder = make_recorder(config)
                calls = []
                original = recorder_module.compute_lumen_evaluation_metrics

                def spy(**kwargs):
                    calls.append(kwargs)
                    return original(**kwargs)

                recorder_module.compute_lumen_evaluation_metrics = spy
                try:
                    recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                    add_curved_samples(recorder, scenario)
                    result = recorder.stop(monotonic_time=1.0)
                finally:
                    recorder_module.compute_lumen_evaluation_metrics = original

                self.assertEqual(1, len(calls))
                self.assertEqual(2, len(calls[0]["backbone_points"]))
                self.assertEqual((3,), calls[0]["tip_points"][0].shape)
                self.assertEqual((2, 3), calls[0]["backbone_points"][0].shape)
                self.assertEqual((3, 3), calls[0]["backbone_points"][1].shape)
                summary = strict_json_load(result.run_dir / "summary.json")
                lumen = summary["lumen_evaluation"]
                self.assertTrue(lumen["available"])
                self.assertTrue(lumen["run_valid"])
                self.assertEqual("lumen_evaluation_v1", lumen["schema_version"])
                self.assertEqual(curved_type, lumen["identity"]["curved_lumen_type"])
                self.assertEqual(scenario.scenario_fingerprint, lumen["identity"]["scenario_fingerprint"])
                self.assertEqual(scenario.geometry_fingerprint, lumen["identity"]["expected_geometry_fingerprint"])
                self.assertTrue(lumen["identity"]["geometry_fingerprint_match"])
                self.assertTrue(lumen["physical_safety"]["physical_safety_pass"])
                self.assertTrue(summary["navigation"]["navigation_success"])
                self.assertEqual(
                    original_lumen_modes,
                    (
                        config["cylindrical_lumen"]["enabled"],
                        config["curved_lumen"]["enabled"],
                        config["curved_lumen"]["type"],
                    ),
                )
                self.assert_no_numpy_objects(result.summary)
                assert_no_nonfinite_json_numbers(self, summary)
                with (result.run_dir / "lumen_evaluation.csv").open(encoding="utf-8", newline="") as handle:
                    rows = list(csv.DictReader(handle))
                self.assertEqual(2, len(rows))
                self.assertEqual("0.0", rows[0]["timestamp_s"])
                self.assertEqual("1.0", rows[1]["timestamp_s"])

    def test_geometry_is_constructed_once_and_fingerprint_helpers_use_that_object(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, curved_type="circular_arc")
            recorder = make_recorder(config)
            geometry_objects = []
            payload_inputs = []
            fingerprint_inputs = []
            original_geometry = recorder_module.lumen_geometry_from_config
            original_payload = recorder_module.lumen_geometry_fingerprint_payload
            original_fingerprint = recorder_module.lumen_geometry_fingerprint

            def geometry_spy(config):
                geometry = original_geometry(config)
                geometry_objects.append(geometry)
                return geometry

            def payload_spy(value):
                payload_inputs.append(value)
                return original_payload(value)

            def fingerprint_spy(value):
                fingerprint_inputs.append(value)
                return original_fingerprint(value)

            recorder_module.lumen_geometry_from_config = geometry_spy
            recorder_module.lumen_geometry_fingerprint_payload = payload_spy
            recorder_module.lumen_geometry_fingerprint = fingerprint_spy
            try:
                recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                add_curved_samples(recorder, scenario)
                recorder.stop(monotonic_time=1.0)
            finally:
                recorder_module.lumen_geometry_from_config = original_geometry
                recorder_module.lumen_geometry_fingerprint_payload = original_payload
                recorder_module.lumen_geometry_fingerprint = original_fingerprint

            self.assertEqual(1, len(geometry_objects))
            self.assertIs(payload_inputs[0], geometry_objects[0])
            self.assertIs(fingerprint_inputs[0], geometry_objects[0])

    def test_fingerprint_mismatch_marks_unavailable_and_preserves_raw_artifacts(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, curved_type="s_curve")
            metadata["geometry_fingerprint"] = "not_the_recorded_fingerprint"
            metadata["reference_configuration"]["curved_scenario"]["geometry_fingerprint"] = "not_the_recorded_fingerprint"
            recorder = make_recorder(config)
            calls = []
            original = recorder_module.compute_lumen_evaluation_metrics
            recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: calls.append(kwargs)
            try:
                recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                add_curved_samples(recorder, scenario)
                result = recorder.stop(monotonic_time=1.0)
            finally:
                recorder_module.compute_lumen_evaluation_metrics = original

            self.assertEqual([], calls)
            self.assertTrue((result.run_dir / "state.csv").is_file())
            self.assertTrue((result.run_dir / "backbone.csv").is_file())
            self.assertFalse((result.run_dir / "lumen_evaluation.csv").exists())
            summary = strict_json_load(result.run_dir / "summary.json")
            lumen = summary["lumen_evaluation"]
            self.assertFalse(lumen["available"])
            self.assertFalse(lumen["run_valid"])
            self.assertIn("geometry_fingerprint_mismatch: reconstructed geometry fingerprint does not match metadata", lumen["unavailable_reasons"])
            self.assertEqual("not_the_recorded_fingerprint", lumen["identity"]["expected_geometry_fingerprint"])
            self.assertFalse(lumen["identity"]["geometry_fingerprint_match"])
            self.assertFalse(summary["navigation"]["navigation_success"])
            self.assertFalse(summary["acceptance"]["collision_free_pass"])

    def test_missing_identity_and_missing_backbone_block_geometry_and_d1(self):
        cases = ("missing_identity", "missing_backbone")
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as temp_dir:
                config = project_config(temp_dir)
                scenario, metadata = scenario_and_metadata(config)
                if case == "missing_identity":
                    metadata.pop("scenario_id")
                    metadata["reference_configuration"]["curved_scenario"].pop("scenario_id")
                recorder = make_recorder(config)
                geometry_calls = []
                d1_calls = []
                original_geometry = recorder_module.lumen_geometry_from_config
                original_d1 = recorder_module.compute_lumen_evaluation_metrics
                recorder_module.lumen_geometry_from_config = lambda config: geometry_calls.append(config)
                recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: d1_calls.append(kwargs)
                try:
                    recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                    if case == "missing_backbone":
                        for timestamp in (0.0, 1.0):
                            target = scenario.validated_target
                            recorder.record_state(timestamp=timestamp, q=[0.0] * 6, q_dot=[0.0] * 6, tip_position=target)
                            recorder.record_reference(timestamp=timestamp, position=target, progress=None)
                    else:
                        add_curved_samples(recorder, scenario)
                    result = recorder.stop(monotonic_time=1.0)
                finally:
                    recorder_module.lumen_geometry_from_config = original_geometry
                    recorder_module.compute_lumen_evaluation_metrics = original_d1

                self.assertEqual([], geometry_calls)
                self.assertEqual([], d1_calls)
                summary = strict_json_load(result.run_dir / "summary.json")
                self.assertFalse(summary["lumen_evaluation"]["available"])
                self.assertFalse(summary["navigation"]["navigation_success"])
                self.assertFalse((result.run_dir / "lumen_evaluation.csv").exists())

    def test_duplicate_timestamps_are_counted_and_tip_backbone_mismatch_blocks_metrics(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, run_id="duplicate_timestamps")
            recorder = make_recorder(config)
            recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
            add_curved_samples(recorder, scenario, times=(0.0, 0.0))
            result = recorder.stop(monotonic_time=1.0)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertTrue(summary["lumen_evaluation"]["available"])
            self.assertEqual(1, summary["lumen_evaluation"]["data_quality"]["duplicate_timestamp_count"])

        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, run_id="tip_mismatch")
            recorder = make_recorder(config)
            target = np.asarray(scenario.validated_target, dtype=float)
            bad_backbone = np.vstack([scenario.centerline_point, target + np.asarray([0.0, 0.0, 1.0e-5])])
            recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
            for timestamp in (0.0, 1.0):
                recorder.record_state(
                    timestamp=timestamp,
                    q=[0.0] * 6,
                    q_dot=[0.0] * 6,
                    tip_position=target,
                    backbone_points=bad_backbone,
                )
                recorder.record_reference(timestamp=timestamp, position=target, progress=None)
            result = recorder.stop(monotonic_time=1.0)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertFalse(summary["lumen_evaluation"]["available"])
            self.assertFalse(summary["lumen_evaluation"]["data_quality"]["tip_backbone_consistent"])
            self.assertIn("tip_backbone_mismatch", summary["lumen_evaluation"]["unavailable_reasons"])

    def test_safety_margin_failure_does_not_override_physical_navigation_success(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, scenario_id=LATERAL_OFFSET_TARGET, run_id="margin_only")
            effective = config_with_lumen_overrides(
                config,
                enable_cylindrical_lumen=False,
                enable_curved_lumen=True,
                curved_lumen_type=scenario.curved_lumen_type,
            )
            geometry = lumen_geometry_from_config(effective)
            unsafe_offset = scenario.preferred_radius + 0.5 * geometry.safety_margin
            unsafe_point = scenario.centerline_point + unsafe_offset * scenario.radial_direction
            target = np.asarray(scenario.validated_target, dtype=float)
            recorder = make_recorder(config)
            recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
            add_curved_samples(
                recorder,
                scenario,
                backbones=[
                    np.vstack([unsafe_point, target]),
                    np.vstack([unsafe_point, target]),
                ],
            )
            result = recorder.stop(monotonic_time=1.0)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertTrue(summary["lumen_evaluation"]["physical_safety"]["physical_safety_pass"])
            self.assertFalse(summary["lumen_evaluation"]["safety_margin"]["safety_margin_pass"])
            self.assertTrue(summary["navigation"]["goal_success"])
            self.assertTrue(summary["navigation"]["physical_safety_pass"])
            self.assertFalse(summary["navigation"]["safety_margin_pass"])
            self.assertTrue(summary["navigation"]["navigation_success"])

    def test_metric_and_lumen_csv_failures_are_unavailable_without_losing_raw_artifacts(self):
        for case in ("metric", "csv"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as temp_dir:
                config = project_config(temp_dir)
                scenario, metadata = scenario_and_metadata(config, run_id=f"{case}_failure")
                recorder = make_recorder(config)
                original_d1 = recorder_module.compute_lumen_evaluation_metrics
                original_csv = ExperimentRecorder._write_lumen_evaluation_csv
                if case == "metric":
                    recorder_module.compute_lumen_evaluation_metrics = (
                        lambda **kwargs: (_ for _ in ()).throw(RuntimeError("controlled D1 failure"))
                    )
                else:
                    ExperimentRecorder._write_lumen_evaluation_csv = (
                        lambda self, run_dir, rows: (_ for _ in ()).throw(RuntimeError("controlled CSV failure"))
                    )
                try:
                    recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                    add_curved_samples(recorder, scenario)
                    result = recorder.stop(monotonic_time=1.0)
                finally:
                    recorder_module.compute_lumen_evaluation_metrics = original_d1
                    ExperimentRecorder._write_lumen_evaluation_csv = original_csv

                summary = strict_json_load(result.run_dir / "summary.json")
                self.assertFalse(summary["lumen_evaluation"]["available"])
                self.assertFalse(summary["lumen_evaluation"]["run_valid"])
                self.assertTrue((result.run_dir / "state.csv").is_file())
                self.assertTrue((result.run_dir / "backbone.csv").is_file())
                self.assertFalse((result.run_dir / "lumen_evaluation.csv").exists())
                expected_reason = "lumen_metric_computation_failed" if case == "metric" else "lumen_csv_write_failed"
                self.assertTrue(any(reason.startswith(expected_reason) for reason in summary["lumen_evaluation"]["unavailable_reasons"]))
                self.assertIsNone(summary["lumen_evaluation"]["physical_safety"]["minimum_physical_clearance_m"])

    def test_trajectory_and_cylinder_paths_do_not_use_generic_d1_lumen_integration(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            recorder = make_recorder(config)
            calls = []
            original = recorder_module.compute_lumen_evaluation_metrics
            recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: calls.append(kwargs)
            try:
                recorder.start(experiment_name="trajectory", metadata={"requested_run_id": "trajectory"}, monotonic_time=0.0)
                add_simple_trajectory_samples(recorder)
                trajectory_result = recorder.stop(monotonic_time=1.0)
            finally:
                recorder_module.compute_lumen_evaluation_metrics = original
            trajectory_summary = strict_json_load(trajectory_result.run_dir / "summary.json")
            self.assertEqual([], calls)
            self.assertNotIn("lumen_evaluation", trajectory_summary)
            self.assertFalse((trajectory_result.run_dir / "lumen_evaluation.csv").exists())

        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            config["cylindrical_lumen"]["enabled"] = True
            recorder = make_recorder(config)
            calls = []
            original = recorder_module.compute_lumen_evaluation_metrics
            recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: calls.append(kwargs)
            try:
                recorder.start(experiment_name="cylinder", metadata={"requested_run_id": "cylinder"}, monotonic_time=0.0)
                add_cylinder_samples(recorder)
                cylinder_result = recorder.stop(monotonic_time=1.0)
            finally:
                recorder_module.compute_lumen_evaluation_metrics = original
            cylinder_summary = strict_json_load(cylinder_result.run_dir / "summary.json")
            self.assertEqual([], calls)
            self.assertIn("lumen_safety", cylinder_summary)
            self.assertNotIn("lumen_evaluation", cylinder_summary)
            self.assertTrue((cylinder_result.run_dir / "cylinder_navigation.csv").is_file())
            self.assertFalse((cylinder_result.run_dir / "lumen_evaluation.csv").exists())

    def test_legacy_tasks_ignore_stray_curved_metadata_and_preserve_their_paths(self):
        for task in ("trajectory", "cylinder_navigation"):
            with self.subTest(task=task), tempfile.TemporaryDirectory() as temp_dir:
                config = project_config(temp_dir)
                scenario, metadata = scenario_and_metadata(config, run_id=f"{task}_stray")
                metadata["task"] = task
                metadata["reference_configuration"]["task"] = task
                recorder = make_recorder(config)
                geometry_calls = []
                d1_calls = []
                original_geometry = recorder_module.lumen_geometry_from_config
                original_d1 = recorder_module.compute_lumen_evaluation_metrics
                recorder_module.lumen_geometry_from_config = lambda value: geometry_calls.append(value)
                recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: d1_calls.append(kwargs)
                if task == "cylinder_navigation":
                    config["cylindrical_lumen"]["enabled"] = True
                    recorder = make_recorder(config)
                try:
                    recorder.start(experiment_name=task, metadata=metadata, monotonic_time=0.0)
                    if task == "trajectory":
                        add_simple_trajectory_samples(recorder)
                    else:
                        add_cylinder_samples(recorder)
                    result = recorder.stop(monotonic_time=1.0)
                finally:
                    recorder_module.lumen_geometry_from_config = original_geometry
                    recorder_module.compute_lumen_evaluation_metrics = original_d1

                self.assertEqual([], geometry_calls)
                self.assertEqual([], d1_calls)
                summary = strict_json_load(result.run_dir / "summary.json")
                self.assertNotIn("lumen_evaluation", summary)
                if task == "cylinder_navigation":
                    self.assertIn("lumen_safety", summary)
                    self.assertTrue((result.run_dir / "cylinder_navigation.csv").is_file())

    def test_canonical_task_controls_curved_activation_over_nested_metadata(self):
        cases = (
            ("trajectory", "curved_lumen_navigation", False, False),
            ("cylinder_navigation", "curved_lumen_navigation", False, False),
            ("curved_lumen_navigation", "trajectory", True, True),
            ("curved_lumen_navigation", "cylinder_navigation", True, True),
            (None, "curved_lumen_navigation", False, False),
            ("unsupported_task", "curved_lumen_navigation", False, False),
        )
        for canonical_task, nested_task, expects_geometry, expects_d1 in cases:
            with self.subTest(canonical_task=canonical_task, nested_task=nested_task), tempfile.TemporaryDirectory() as temp_dir:
                config = project_config(temp_dir)
                scenario, metadata = scenario_and_metadata(
                    config,
                    run_id=f"authority_{canonical_task or 'missing'}_{nested_task}",
                )
                if canonical_task is None:
                    nested_metadata = copy.deepcopy(metadata)
                    nested_metadata["reference_configuration"]["curved_scenario"]["task"] = nested_task
                    metadata = {
                        "requested_run_id": nested_metadata["requested_run_id"],
                        "metadata_override": nested_metadata,
                    }
                else:
                    metadata["reference_configuration"]["task"] = canonical_task
                    metadata["reference_configuration"]["curved_scenario"]["task"] = nested_task

                if canonical_task == "cylinder_navigation":
                    config["cylindrical_lumen"]["enabled"] = True
                recorder = make_recorder(config)
                geometry_calls = []
                d1_calls = []
                original_geometry = recorder_module.lumen_geometry_from_config
                original_d1 = recorder_module.compute_lumen_evaluation_metrics

                def geometry_spy(value):
                    geometry_calls.append(value)
                    return original_geometry(value)

                def d1_spy(**kwargs):
                    d1_calls.append(kwargs)
                    return original_d1(**kwargs)

                recorder_module.lumen_geometry_from_config = geometry_spy
                recorder_module.compute_lumen_evaluation_metrics = d1_spy
                try:
                    recorder.start(experiment_name="authority", metadata=metadata, monotonic_time=0.0)
                    if canonical_task == "curved_lumen_navigation":
                        add_curved_samples(recorder, scenario)
                    elif canonical_task == "cylinder_navigation":
                        add_cylinder_samples(recorder)
                    else:
                        add_simple_trajectory_samples(recorder)
                    result = recorder.stop(monotonic_time=1.0)
                finally:
                    recorder_module.lumen_geometry_from_config = original_geometry
                    recorder_module.compute_lumen_evaluation_metrics = original_d1

                summary = strict_json_load(result.run_dir / "summary.json")
                self.assertEqual(expects_geometry, bool(geometry_calls))
                self.assertEqual(expects_d1, bool(d1_calls))
                if expects_d1:
                    self.assertTrue(summary["lumen_evaluation"]["available"])
                else:
                    self.assertNotIn("lumen_evaluation", summary)
                if canonical_task == "cylinder_navigation":
                    self.assertIn("lumen_safety", summary)
                    self.assertTrue((result.run_dir / "cylinder_navigation.csv").is_file())

    def test_curved_identity_and_d1_are_still_required_for_authoritative_curved_task(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, run_id="curved_missing_task_identity")
            metadata.pop("task", None)
            metadata["reference_configuration"].pop("task", None)
            recorder = make_recorder(config)
            geometry_calls = []
            d1_calls = []
            original_geometry = recorder_module.lumen_geometry_from_config
            original_d1 = recorder_module.compute_lumen_evaluation_metrics
            recorder_module.lumen_geometry_from_config = lambda value: geometry_calls.append(value)
            recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: d1_calls.append(kwargs)
            try:
                recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                add_curved_samples(recorder, scenario)
                result = recorder.stop(monotonic_time=1.0)
            finally:
                recorder_module.lumen_geometry_from_config = original_geometry
                recorder_module.compute_lumen_evaluation_metrics = original_d1

            self.assertEqual([], geometry_calls)
            self.assertEqual([], d1_calls)
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertNotIn("lumen_evaluation", summary)

    def test_atomic_lumen_csv_failure_cleans_temp_and_preserves_existing_final_file(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config, run_id="atomic_csv_failure")
            recorder = make_recorder(config)
            original_writer = recorder_module.csv.writer
            writer_calls = {"count": 0}

            def failing_writer(handle):
                writer = original_writer(handle)
                if ".lumen_evaluation.csv." not in str(getattr(handle, "name", "")):
                    return writer

                class FailingWriter:
                    def writerow(self, row):
                        writer_calls["count"] += 1
                        if writer_calls["count"] >= 2:
                            raise OSError("controlled partial CSV failure")
                        return writer.writerow(row)

                return FailingWriter()

            recorder_module.csv.writer = failing_writer
            try:
                recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                add_curved_samples(recorder, scenario)
                result = recorder.stop(monotonic_time=1.0)
            finally:
                recorder_module.csv.writer = original_writer

            self.assertGreaterEqual(writer_calls["count"], 2)
            self.assertFalse((result.run_dir / "lumen_evaluation.csv").exists())
            self.assertEqual([], list(result.run_dir.glob(".lumen_evaluation.csv.*.tmp")))
            self.assertTrue((result.run_dir / "state.csv").is_file())
            self.assertTrue((result.run_dir / "backbone.csv").is_file())
            summary = strict_json_load(result.run_dir / "summary.json")
            self.assertFalse(summary["lumen_evaluation"]["available"])
            self.assertFalse(summary["lumen_evaluation"]["run_valid"])
            self.assertFalse(summary["navigation"]["navigation_success"])
            self.assertIsNone(summary["lumen_evaluation"]["physical_safety"]["minimum_physical_clearance_m"])

            existing = result.run_dir / "lumen_evaluation.csv"
            existing.write_text("old-complete-file\n", encoding="utf-8")
            writer_calls["count"] = 0
            recorder_module.csv.writer = failing_writer
            try:
                with self.assertRaises(OSError):
                    recorder._write_lumen_evaluation_csv(result.run_dir, ({"timestamp_s": 0.0},))
            finally:
                recorder_module.csv.writer = original_writer
            self.assertEqual("old-complete-file\n", existing.read_text(encoding="utf-8"))
            self.assertEqual([], list(result.run_dir.glob(".lumen_evaluation.csv.*.tmp")))

    def test_pre_gate_timestamp_and_backbone_validation_rejects_nonfinite_or_malformed_data(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config)
            recorder = make_recorder(config)
            valid_backbone = np.vstack([scenario.centerline_point, scenario.validated_target])
            invalid_cases = (
                ("nan_timestamp", [aligned_lumen_sample(float("nan"), valid_backbone, valid_backbone[-1])], "nonfinite_timestamp"),
                ("positive_inf_timestamp", [aligned_lumen_sample(float("inf"), valid_backbone, valid_backbone[-1])], "nonfinite_timestamp"),
                ("negative_inf_timestamp", [aligned_lumen_sample(float("-inf"), valid_backbone, valid_backbone[-1])], "nonfinite_timestamp"),
                ("decreasing_timestamp", [
                    aligned_lumen_sample(1.0, valid_backbone, valid_backbone[-1]),
                    aligned_lumen_sample(0.0, valid_backbone, valid_backbone[-1]),
                ], "nonmonotonic_timestamps"),
                ("rank_one", [aligned_lumen_sample(0.0, np.zeros(3), np.zeros(3))], "malformed_backbone_data"),
                ("shape_n2", [aligned_lumen_sample(0.0, np.zeros((2, 2)), np.zeros(3))], "malformed_backbone_data"),
                ("shape_n4", [aligned_lumen_sample(0.0, np.zeros((2, 4)), np.zeros(3))], "malformed_backbone_data"),
                ("empty", [aligned_lumen_sample(0.0, np.empty((0, 3)), np.zeros(3))], "malformed_backbone_data"),
                ("nan_backbone", [aligned_lumen_sample(0.0, np.asarray([[0.0, 0.0, 0.0], [np.nan, 0.0, 0.0]]), [np.nan, 0.0, 0.0])], "nonfinite_backbone_data"),
                ("positive_inf_backbone", [aligned_lumen_sample(0.0, np.asarray([[0.0, 0.0, 0.0], [np.inf, 0.0, 0.0]]), [np.inf, 0.0, 0.0])], "nonfinite_backbone_data"),
                ("negative_inf_backbone", [aligned_lumen_sample(0.0, np.asarray([[0.0, 0.0, 0.0], [-np.inf, 0.0, 0.0]]), [-np.inf, 0.0, 0.0])], "nonfinite_backbone_data"),
            )
            original_geometry = recorder_module.lumen_geometry_from_config
            original_d1 = recorder_module.compute_lumen_evaluation_metrics
            for name, samples, reason in invalid_cases:
                with self.subTest(case=name):
                    geometry_calls = []
                    d1_calls = []
                    recorder_module.lumen_geometry_from_config = lambda value: geometry_calls.append(value)
                    recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: d1_calls.append(kwargs)
                    try:
                        result = recorder._lumen_evaluation_result(
                            alignment=lumen_alignment(samples),
                            metadata=metadata,
                        )
                    finally:
                        recorder_module.lumen_geometry_from_config = original_geometry
                        recorder_module.compute_lumen_evaluation_metrics = original_d1
                    self.assertEqual([], geometry_calls)
                    self.assertEqual([], d1_calls)
                    self.assertFalse(result.section["available"])
                    self.assertIn(reason, result.section["unavailable_reasons"])

    def test_unsupported_curved_identity_is_rejected_before_geometry_and_d1(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config = project_config(temp_dir)
            scenario, metadata = scenario_and_metadata(config)
            backbone = np.vstack([scenario.centerline_point, scenario.validated_target])
            invalid_identity = (
                ("unsupported_type", "curved_lumen_type", "unsupported_curve"),
                ("wrong_reference_mode", "reference_mode", "trajectory"),
                ("malformed_target", "executed_target", [0.0, 0.0]),
            )
            original_geometry = recorder_module.lumen_geometry_from_config
            original_d1 = recorder_module.compute_lumen_evaluation_metrics
            for name, key, value in invalid_identity:
                with self.subTest(case=name):
                    changed = copy.deepcopy(metadata)
                    changed[key] = value
                    geometry_calls = []
                    d1_calls = []
                    recorder_module.lumen_geometry_from_config = lambda item: geometry_calls.append(item)
                    recorder_module.compute_lumen_evaluation_metrics = lambda **kwargs: d1_calls.append(kwargs)
                    try:
                        result = make_recorder(config)._lumen_evaluation_result(
                            alignment=lumen_alignment([
                                aligned_lumen_sample(0.0, backbone, backbone[-1]),
                            ]),
                            metadata=changed,
                        )
                    finally:
                        recorder_module.lumen_geometry_from_config = original_geometry
                        recorder_module.compute_lumen_evaluation_metrics = original_d1
                    self.assertEqual([], geometry_calls)
                    self.assertEqual([], d1_calls)
                    self.assertFalse(result.section["available"])
                    self.assertTrue(any(reason.startswith("invalid_curved_identity:") for reason in result.section["unavailable_reasons"]))

    def test_summary_serializes_all_d1_sections_and_centerline_rmse_policy(self):
        for scenario_id in (CENTERLINE_TARGET, LATERAL_OFFSET_TARGET, NEAR_SAFETY_BOUNDARY_TARGET):
            with self.subTest(scenario_id=scenario_id), tempfile.TemporaryDirectory() as temp_dir:
                config = project_config(temp_dir)
                scenario, metadata = scenario_and_metadata(config, scenario_id=scenario_id, run_id=scenario_id)
                recorder = make_recorder(config)
                captured = []
                original = recorder_module.compute_lumen_evaluation_metrics

                def capture(**kwargs):
                    result = original(**kwargs)
                    captured.append(result)
                    return result

                recorder_module.compute_lumen_evaluation_metrics = capture
                try:
                    recorder.start(experiment_name="curved", metadata=metadata, monotonic_time=0.0)
                    add_curved_samples(recorder, scenario)
                    result = recorder.stop(monotonic_time=1.0)
                finally:
                    recorder_module.compute_lumen_evaluation_metrics = original

                self.assertEqual(1, len(captured))
                metrics = captured[0]
                summary = strict_json_load(result.run_dir / "summary.json")
                lumen = summary["lumen_evaluation"]
                identity = lumen["identity"]
                self.assertEqual(scenario.target_mode, identity["target_mode"])
                self.assertEqual(scenario.scenario_id, identity["scenario_id"])
                self.assertEqual(scenario.centerline_fraction, identity["centerline_fraction"])
                self.assertEqual(scenario.centerline_arc_length, identity["centerline_arc_length"])
                self.assertEqual(scenario.radial_offset, identity["radial_offset"])
                safety = metrics.safety
                self.assertEqual(safety.physical_safety_pass, lumen["physical_safety"]["physical_safety_pass"])
                self.assertEqual(safety.physical_collision_detected, lumen["physical_safety"]["collision_detected"])
                self.assertEqual(safety.physical_collision_sample_count, lumen["physical_safety"]["collision_sample_count"])
                self.assertEqual(safety.physical_collision_event_count, lumen["physical_safety"]["collision_event_count"])
                self.assertEqual(safety.physical_collision_duration, lumen["physical_safety"]["collision_duration_s"])
                self.assertEqual(safety.first_physical_collision_time, lumen["physical_safety"]["first_collision_time_s"])
                self.assertEqual(safety.minimum_physical_clearance, lumen["physical_safety"]["minimum_physical_clearance_m"])
                self.assertEqual(safety.final_physical_clearance, lumen["physical_safety"]["final_physical_clearance_m"])
                self.assertEqual(safety.safety_margin_pass, lumen["safety_margin"]["safety_margin_pass"])
                self.assertEqual(safety.safety_margin_violation_detected, lumen["safety_margin"]["margin_violation_detected"])
                self.assertEqual(safety.safety_margin_violation_sample_count, lumen["safety_margin"]["violation_sample_count"])
                self.assertEqual(safety.safety_margin_violation_event_count, lumen["safety_margin"]["violation_event_count"])
                self.assertEqual(safety.safety_margin_violation_duration, lumen["safety_margin"]["violation_duration_s"])
                self.assertEqual(safety.minimum_safety_clearance, lumen["safety_margin"]["minimum_safety_clearance_m"])
                self.assertEqual(safety.final_safety_clearance, lumen["safety_margin"]["final_safety_clearance_m"])
                self.assertEqual({item.constraint_type for item in safety.per_constraint_breakdown}, set(lumen["constraints"]))
                for item in safety.per_constraint_breakdown:
                    serialized = lumen["constraints"][item.constraint_type]
                    self.assertEqual(item.physical_violation_sample_count, serialized["physical_violation_sample_count"])
                    self.assertEqual(item.physical_violation_event_count, serialized["physical_violation_event_count"])
                    self.assertEqual(item.physical_violation_duration, serialized["physical_violation_duration_s"])
                    self.assertEqual(item.maximum_penetration, serialized["maximum_penetration_m"])
                    self.assertEqual(item.minimum_physical_clearance, serialized["minimum_physical_clearance_m"])
                    self.assertEqual(item.worst_sample_index, serialized["worst_sample_index"])
                    self.assertEqual(item.worst_backbone_index, serialized["worst_backbone_index"])
                progress = metrics.progress
                serialized_progress = lumen["progress"]
                self.assertEqual(progress.initial_centerline_arc_length, serialized_progress["initial_centerline_arc_length_m"])
                self.assertEqual(progress.final_centerline_arc_length, serialized_progress["final_centerline_arc_length_m"])
                self.assertEqual(progress.maximum_normalized_progress, serialized_progress["maximum_normalized_progress"])
                self.assertEqual(progress.mean_tip_radial_offset, serialized_progress["mean_radial_offset_m"])
                self.assertEqual(progress.rms_tip_radial_offset, serialized_progress["rms_radial_offset_m"])
                self.assertEqual(progress.final_local_lumen_radius, serialized_progress["final_local_radius_m"])
                self.assertIsNotNone(serialized_progress["centerline_tracking_rmse_m"])
                self.assertEqual(
                    progress.rms_tip_radial_offset,
                    serialized_progress["centerline_tracking_rmse_m"],
                )
                self.assertEqual(
                    bool(summary["navigation"]["goal_success"] and summary["navigation"]["physical_safety_pass"]),
                    summary["navigation"]["navigation_success"],
                )

    def assert_no_numpy_objects(self, value):
        if isinstance(value, dict):
            for item in value.values():
                self.assert_no_numpy_objects(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                self.assert_no_numpy_objects(item)
        else:
            self.assertNotIsInstance(value, np.ndarray)
            self.assertNotIsInstance(value, np.generic)


if __name__ == "__main__":
    unittest.main()
