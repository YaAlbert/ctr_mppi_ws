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

from ctr_evaluation.compare_results import compare_result_dirs, write_json
from ctr_evaluation.metrics import compare_summaries


CURVED_TASK = "curved_lumen_navigation"


def curved_summary(*, curved_type="circular_arc", scenario_id="centerline_target", available=True, run_valid=True):
    identity = {
        "task": CURVED_TASK,
        "reference_mode": "fixed_target",
        "target_mode": "fixed_target",
        "curved_lumen_type": curved_type,
        "scenario_id": scenario_id,
        "scenario_policy_version": "curved_scenario_v1",
        "scenario_fingerprint": f"scenario-{scenario_id}-{curved_type}",
        "geometry_frame": "world",
        "geometry_fingerprint": f"geometry-{curved_type}",
        "expected_geometry_fingerprint": f"geometry-{curved_type}",
        "reconstructed_geometry_fingerprint": f"geometry-{curved_type}",
        "geometry_fingerprint_match": True,
        "shared_environment_hash": f"environment-{curved_type}",
        "derived_target": [0.01, 0.0, 0.1],
        "requested_target": [0.01, 0.0, 0.1],
        "executed_target": [0.01, 0.0, 0.1],
        "centerline_fraction": 0.7,
        "centerline_arc_length": 0.1,
        "radial_offset": 0.0,
        "override_used": False,
    }
    lumen = {
        "schema_version": "lumen_evaluation_v1",
        "available": available,
        "run_valid": run_valid,
        "unavailable_reasons": [] if available else ["lumen_evaluation_unavailable"],
        "identity": identity,
        "geometry": {
            "mode": "curved",
            "type": curved_type,
            "frame": "world",
            "fingerprint": f"geometry-{curved_type}",
            "ctr_outer_radius_m": 0.001,
            "safety_margin_m": 0.0002,
            "minimum_lumen_radius_m": 0.003,
            "maximum_lumen_radius_m": 0.004,
        },
        "physical_safety": {
            "physical_safety_pass": True,
            "collision_detected": False,
            "collision_sample_count": 0,
            "collision_event_count": 0,
            "collision_duration_s": 0.0,
            "first_collision_time_s": None,
            "minimum_physical_clearance_m": 0.002,
            "final_physical_clearance_m": 0.003,
        },
        "safety_margin": {
            "safety_margin_pass": True,
            "margin_violation_detected": False,
            "violation_sample_count": 0,
            "violation_event_count": 0,
            "violation_duration_s": 0.0,
            "first_violation_time_s": None,
            "minimum_safety_clearance_m": 0.001,
            "final_safety_clearance_m": 0.0015,
        },
        "progress": {
            "final_normalized_progress": 0.8,
            "maximum_normalized_progress": 0.9,
        },
    }
    return {
        "goal": {"final_goal_error": 0.01, "rmse": 0.02, "time_to_goal": 0.8},
        "navigation": {
            "goal_success": True,
            "physical_safety_pass": True,
            "safety_margin_pass": True,
            "navigation_success": True,
        },
        "lumen_evaluation": lumen,
    }


def curved_metadata(*, role="candidate", curved_type="circular_arc", seed=11, task=CURVED_TASK):
    target = [0.01, 0.0, 0.1]
    return {
        "task": task,
        "run_role": role,
        "shared_environment_hash": f"environment-{curved_type}",
        "orchestration_id": "orchestration-1",
        "reference_start_policy": "fixed_target_window_epoch",
        "reference_lead_duration_s": 1.0,
        "reference_phase_offset_s": 1.0,
        "reference_pre_epoch_behavior": "fixed_target_ready",
        "evaluation_window_duration_s": 10.0,
        "initial_state_q": [0.0] * 6,
        "initial_tip_position": [0.0, 0.0, 0.0],
        "baseline_nonzero_command_count": 0,
        "candidate_command_after_recording": True,
        "requested_target": target,
        "executed_target": target,
        "target_replaced": False,
        "target_identity_valid": True,
        "reference_matches_requested_target": True,
        "candidate_seed": seed,
        "configuration": {
            "trajectory_type": "circle",
            "trajectory_parameters_hash": "trajectory-hash",
            "cylindrical_lumen_hash": "",
            "goal_configuration_hash": "goal-hash",
            "frame_id": "world",
            "model_configuration_hash": "model-hash",
            "software_mode": "simulation",
            "configured_control_period": 0.01,
            "reference_sample_period": 0.01,
            "goal": {"tolerance": 0.001, "required_hold_duration": 0.5},
        },
    }


def legacy_summary():
    return {"tracking": {"rmse": 1.0}, "control": {"total_control_effort": 2.0}}


def legacy_metadata(task):
    return {
        "task": task,
        "configuration": {
            "trajectory_type": "circle",
            "trajectory_parameters_hash": "trajectory-hash",
            "cylindrical_lumen_hash": "",
            "goal_configuration_hash": "goal-hash",
            "frame_id": "world",
            "model_configuration_hash": "model-hash",
            "software_mode": "simulation",
            "configured_control_period": 0.01,
            "reference_sample_period": 0.01,
        },
    }


def compare(candidate_summary, baseline_summary, candidate_metadata=None, baseline_metadata=None):
    return compare_summaries(
        candidate_summary=candidate_summary,
        baseline_summary=baseline_summary,
        candidate_metadata=candidate_metadata or curved_metadata(),
        baseline_metadata=baseline_metadata or curved_metadata(role="baseline", seed=None),
        near_zero_epsilon=1.0e-12,
        duration_tolerance=0.1,
        initial_state_tolerance=1.0e-6,
    )


class CurvedLumenCompatibilityTest(unittest.TestCase):
    def test_matching_curved_pairs_are_compatible_for_both_types(self):
        for curved_type in ("circular_arc", "s_curve"):
            with self.subTest(curved_type=curved_type):
                candidate = curved_summary(curved_type=curved_type)
                baseline = curved_summary(curved_type=curved_type)
                candidate_metadata = curved_metadata(curved_type=curved_type)
                baseline_metadata = curved_metadata(role="baseline", curved_type=curved_type, seed=None)
                result = compare(candidate, baseline, candidate_metadata, baseline_metadata)
                self.assertTrue(result.compatibility_valid)
                self.assertTrue(result.pair_identity_compatible)
                self.assertTrue(result.comparison_valid)
                self.assertTrue(result.baseline_run_valid)
                self.assertTrue(result.candidate_run_valid)
                self.assertTrue(result.improvement_evaluated)
                self.assertEqual("curved_comparison_v1", result.comparison_schema_version)

    def test_role_seed_controller_profile_and_timing_differences_are_allowed(self):
        candidate_metadata = curved_metadata(seed=42)
        baseline_metadata = curved_metadata(role="baseline", seed=None)
        candidate_metadata["controller_configuration_hash"] = "candidate-controller"
        baseline_metadata["controller_configuration_hash"] = "baseline-controller"
        candidate_metadata["actual_evaluation_window_duration_s"] = 9.9
        baseline_metadata["actual_evaluation_window_duration_s"] = 10.0
        result = compare(curved_summary(), curved_summary(), candidate_metadata, baseline_metadata)
        self.assertTrue(result.comparison_valid)
        self.assertEqual([], result.compatibility_reasons)

    def test_identity_mismatches_are_deterministic(self):
        fields = (
            ("reference_mode", "trajectory", "reference_mode_mismatch"),
            ("target_mode", "trajectory", "target_mode_mismatch"),
            ("curved_lumen_type", "s_curve", "curved_lumen_type_mismatch"),
            ("scenario_id", "lateral_offset_target", "scenario_id_mismatch"),
            ("scenario_policy_version", "other_policy", "scenario_policy_version_mismatch"),
            ("scenario_fingerprint", "other_scenario", "scenario_fingerprint_mismatch"),
            ("geometry_frame", "base_link", "geometry_frame_mismatch"),
            ("expected_geometry_fingerprint", "other_geometry", "geometry_fingerprint_mismatch"),
            ("reconstructed_geometry_fingerprint", "other_geometry", "geometry_fingerprint_mismatch"),
            ("derived_target", [0.02, 0.0, 0.1], "derived_target_mismatch"),
            ("requested_target", [0.02, 0.0, 0.1], "requested_target_mismatch"),
            ("executed_target", [0.02, 0.0, 0.1], "executed_target_mismatch"),
            ("centerline_fraction", 0.8, "centerline_fraction_mismatch"),
            ("centerline_arc_length", 0.2, "centerline_arc_length_mismatch"),
            ("radial_offset", 0.001, "radial_offset_mismatch"),
            ("override_used", True, "override_state_mismatch"),
        )
        for field_name, value, code in fields:
            with self.subTest(field=field_name):
                candidate = curved_summary()
                candidate["lumen_evaluation"]["identity"][field_name] = value
                result = compare(candidate, curved_summary())
                self.assertFalse(result.comparison_valid)
                self.assertIn(code, result.compatibility_reasons)
                self.assertEqual([], result.metric_comparisons)
                self.assertFalse(result.improvement_evaluated)
                self.assertIsNone(result.improvement_pass)

    def test_task_mismatch_and_missing_canonical_curved_task_are_not_inferred(self):
        candidate_metadata = curved_metadata(task="trajectory")
        result = compare(curved_summary(), curved_summary(), candidate_metadata, curved_metadata(role="baseline"))
        self.assertFalse(result.pair_identity_compatible)
        self.assertIn("task_mismatch", result.compatibility_reasons)
        self.assertFalse(result.candidate_run_valid)
        self.assertEqual([], result.metric_comparisons)

        missing = curved_metadata(role="baseline")
        missing.pop("task")
        result = compare(curved_summary(), curved_summary(), curved_metadata(), missing)
        self.assertFalse(result.comparison_valid)
        self.assertIn("task_mismatch", result.compatibility_reasons)

    def test_invalid_runs_produce_no_metric_comparisons_or_improvement(self):
        cases = (
            (curved_summary(available=False), curved_summary(), "candidate"),
            (curved_summary(), curved_summary(run_valid=False), "baseline"),
            (curved_summary(available=False), curved_summary(available=False), "both"),
        )
        for candidate, baseline, label in cases:
            with self.subTest(case=label):
                result = compare(candidate, baseline)
                self.assertFalse(result.comparison_valid)
                self.assertFalse(result.improvement_evaluated)
                self.assertIsNone(result.improvement_pass)
                self.assertEqual([], result.metric_comparisons)
                self.assertEqual([], result.boolean_comparisons)

    def test_internal_fingerprint_failure_invalidates_run(self):
        candidate = curved_summary()
        candidate["lumen_evaluation"]["identity"]["geometry_fingerprint_match"] = False
        result = compare(candidate, curved_summary())
        self.assertFalse(result.candidate_run_valid)
        self.assertIn("geometry_fingerprint_not_valid", result.candidate_invalid_reasons)
        self.assertEqual([], result.metric_comparisons)

    def test_missing_reference_identity_evidence_invalidates_pair(self):
        candidate = curved_summary()
        baseline = curved_summary()
        candidate_metadata = curved_metadata()
        baseline_metadata = curved_metadata(role="baseline", seed=None)
        candidate_metadata.pop("reference_matches_requested_target")

        result = compare(candidate, baseline, candidate_metadata, baseline_metadata)

        self.assertTrue(result.candidate_run_valid)
        self.assertFalse(result.pair_identity_compatible)
        self.assertFalse(result.comparison_valid)
        self.assertIn("candidate published reference target does not match requested_target", result.compatibility_reasons)

    def test_authoritative_geometry_fingerprint_is_consistent_and_compared(self):
        inconsistent = curved_summary()
        inconsistent["lumen_evaluation"]["geometry"]["fingerprint"] = "different"
        result = compare(inconsistent, curved_summary())
        self.assertFalse(result.candidate_run_valid)
        self.assertIn("geometry_fingerprint_inconsistent", result.candidate_invalid_reasons)
        self.assertFalse(result.comparison_valid)

        candidate = curved_summary()
        candidate["lumen_evaluation"]["identity"]["geometry_fingerprint"] = "candidate-geometry"
        candidate["lumen_evaluation"]["identity"]["expected_geometry_fingerprint"] = "candidate-geometry"
        candidate["lumen_evaluation"]["identity"]["reconstructed_geometry_fingerprint"] = "candidate-geometry"
        candidate["lumen_evaluation"]["geometry"]["fingerprint"] = "candidate-geometry"
        result = compare(candidate, curved_summary())
        self.assertTrue(result.candidate_run_valid)
        self.assertFalse(result.pair_identity_compatible)
        self.assertIn("geometry_fingerprint_mismatch", result.compatibility_reasons)
        self.assertEqual([], result.metric_comparisons)

    def test_required_curved_identity_fields_cannot_be_missing(self):
        cases = (
            ("geometry_fingerprint", lambda summary, metadata: summary["lumen_evaluation"]["identity"].pop("geometry_fingerprint")),
            ("shared_environment_hash", lambda summary, metadata: summary["lumen_evaluation"]["identity"].pop("shared_environment_hash")),
            ("target_tolerance", lambda summary, metadata: metadata["configuration"]["goal"].pop("tolerance")),
            ("required_hold_duration", lambda summary, metadata: metadata["configuration"]["goal"].pop("required_hold_duration")),
            ("target_mode", lambda summary, metadata: summary["lumen_evaluation"]["identity"].pop("target_mode")),
            ("centerline_fraction", lambda summary, metadata: summary["lumen_evaluation"]["identity"].pop("centerline_fraction")),
            ("centerline_arc_length", lambda summary, metadata: summary["lumen_evaluation"]["identity"].pop("centerline_arc_length")),
            ("radial_offset", lambda summary, metadata: summary["lumen_evaluation"]["identity"].pop("radial_offset")),
            ("evaluation_window_duration_s", lambda summary, metadata: metadata.pop("evaluation_window_duration_s")),
            ("configuration.model_configuration_hash", lambda summary, metadata: metadata["configuration"].pop("model_configuration_hash")),
            ("geometry.ctr_outer_radius_m", lambda summary, metadata: summary["lumen_evaluation"]["geometry"].pop("ctr_outer_radius_m")),
            ("geometry.safety_margin_m", lambda summary, metadata: summary["lumen_evaluation"]["geometry"].pop("safety_margin_m")),
            ("geometry.minimum_lumen_radius_m", lambda summary, metadata: summary["lumen_evaluation"]["geometry"].pop("minimum_lumen_radius_m")),
            ("geometry.maximum_lumen_radius_m", lambda summary, metadata: summary["lumen_evaluation"]["geometry"].pop("maximum_lumen_radius_m")),
        )
        for field_name, remove_field in cases:
            with self.subTest(field=field_name):
                baseline = curved_summary()
                candidate = curved_summary()
                baseline_metadata = curved_metadata(role="baseline", seed=None)
                candidate_metadata = curved_metadata()
                remove_field(baseline, baseline_metadata)
                result = compare(candidate, baseline, candidate_metadata, baseline_metadata)
                self.assertFalse(result.baseline_run_valid)
                self.assertTrue(result.candidate_run_valid)
                self.assertFalse(result.pair_identity_compatible)
                self.assertFalse(result.comparison_valid)
                self.assertIn(f"required_curved_identity_missing:{field_name}", result.baseline_invalid_reasons)
                self.assertFalse(result.improvement_evaluated)
                self.assertEqual([], result.metric_comparisons)

                baseline = curved_summary()
                candidate = curved_summary()
                baseline_metadata = curved_metadata(role="baseline", seed=None)
                candidate_metadata = curved_metadata()
                remove_field(candidate, candidate_metadata)
                result = compare(candidate, baseline, candidate_metadata, baseline_metadata)
                self.assertTrue(result.baseline_run_valid)
                self.assertFalse(result.candidate_run_valid)
                self.assertFalse(result.pair_identity_compatible)
                self.assertIn(f"required_curved_identity_missing:{field_name}", result.candidate_invalid_reasons)

                baseline = curved_summary()
                candidate = curved_summary()
                baseline_metadata = curved_metadata(role="baseline", seed=None)
                candidate_metadata = curved_metadata()
                remove_field(baseline, baseline_metadata)
                remove_field(candidate, candidate_metadata)
                result = compare(candidate, baseline, candidate_metadata, baseline_metadata)
                self.assertFalse(result.baseline_run_valid)
                self.assertFalse(result.candidate_run_valid)
                self.assertFalse(result.pair_identity_compatible)
                self.assertFalse(result.comparison_valid)
                self.assertFalse(result.improvement_evaluated)
                self.assertEqual([], result.metric_comparisons)

    def test_required_identity_numeric_and_string_types_are_validated(self):
        cases = (
            ("target_tolerance", lambda summary, metadata, value: metadata["configuration"]["goal"].__setitem__("tolerance", value)),
            ("required_hold_duration", lambda summary, metadata, value: metadata["configuration"]["goal"].__setitem__("required_hold_duration", value)),
            ("ctr_outer_radius_m", lambda summary, metadata, value: summary["lumen_evaluation"]["geometry"].__setitem__("ctr_outer_radius_m", value)),
            ("shared_environment_hash", lambda summary, metadata, value: summary["lumen_evaluation"]["identity"].__setitem__("shared_environment_hash", value)),
        )
        for field_name, assign in cases:
            invalid_values = ("malformed", True, float("nan"), float("inf"), float("-inf"))
            if field_name == "shared_environment_hash":
                invalid_values = ("", None, True, [], {})
            for value in invalid_values:
                with self.subTest(field=field_name, value=repr(value)):
                    candidate = curved_summary()
                    metadata = curved_metadata()
                    assign(candidate, metadata, value)
                    result = compare(candidate, curved_summary(), metadata, curved_metadata(role="baseline", seed=None))
                    self.assertFalse(result.candidate_run_valid)
                    self.assertFalse(result.comparison_valid)
                    self.assertFalse(result.improvement_evaluated)

    def test_goal_tolerance_and_hold_duration_mismatches_are_incompatible(self):
        for field, key in (("target_tolerance_mismatch", "tolerance"), ("required_hold_duration_mismatch", "required_hold_duration")):
            with self.subTest(field=field):
                candidate_metadata = curved_metadata()
                baseline_metadata = curved_metadata(role="baseline", seed=None)
                candidate_metadata["configuration"]["goal"][key] += 0.001
                result = compare(curved_summary(), curved_summary(), candidate_metadata, baseline_metadata)
                self.assertFalse(result.comparison_valid)
                self.assertIn(field, result.compatibility_reasons)
                self.assertFalse(result.improvement_evaluated)
                self.assertEqual([], result.metric_comparisons)

    def test_required_metric_missing_or_nonfinite_invalidates_run(self):
        missing = curved_summary()
        del missing["lumen_evaluation"]["progress"]["final_normalized_progress"]
        result = compare(missing, curved_summary())
        self.assertFalse(result.candidate_run_valid)
        self.assertIn("required_metric_missing:curved_final_normalized_progress", result.candidate_invalid_reasons)

        nonfinite = curved_summary()
        nonfinite["goal"]["rmse"] = math.nan
        result = compare(nonfinite, curved_summary())
        self.assertFalse(result.candidate_run_valid)
        self.assertIn("required_metric_nonfinite:curved_rms_target_error", result.candidate_invalid_reasons)

    def test_zero_denominator_keeps_pair_valid_but_relative_change_unavailable(self):
        baseline = curved_summary()
        baseline["lumen_evaluation"]["physical_safety"]["minimum_physical_clearance_m"] = 0.0
        result = compare(curved_summary(), baseline)
        self.assertTrue(result.comparison_valid)
        item = next(item for item in result.metric_comparisons if item.metric == "curved_minimum_physical_clearance")
        self.assertFalse(item.comparison_valid)
        self.assertIsNone(item.relative_improvement_percent)
        self.assertIn("near zero", item.reason)

    def test_boolean_metrics_use_boolean_semantics(self):
        baseline = curved_summary()
        baseline["navigation"]["goal_success"] = False
        baseline["navigation"]["navigation_success"] = False
        candidate = curved_summary()
        result = compare(candidate, baseline)
        boolean = {item.metric: item for item in result.boolean_comparisons}
        self.assertTrue(boolean["goal_success"].improved)
        self.assertTrue(boolean["navigation_success"].improved)
        self.assertNotIn("goal_success", {item.metric for item in result.metric_comparisons})

    def test_comparison_output_is_deterministic_and_sanitized(self):
        first = compare(curved_summary(), curved_summary())
        second = compare(curved_summary(), curved_summary())
        self.assertEqual(first.to_dict(), second.to_dict())
        serialized = json.dumps(first.to_dict(), allow_nan=False)
        self.assertNotIn("NaN", serialized)
        self.assertNotIn("Infinity", serialized)
        for detail in first.compatibility_details.get("curved_mismatch_details", []):
            self.assertNotIn("/tmp", json.dumps(detail))

    def test_compare_result_dirs_writes_structured_failure_without_mutating_summaries(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            candidate_dir = root / "candidate"
            baseline_dir = root / "baseline"
            candidate_dir.mkdir()
            baseline_dir.mkdir()
            candidate_summary = curved_summary()
            write_json(candidate_dir / "summary.json", candidate_summary)
            candidate_bytes = (candidate_dir / "summary.json").read_bytes()
            (baseline_dir / "summary.json").write_bytes(b"not-json")
            (candidate_dir / "metadata.yaml").write_text(yaml.safe_dump(curved_metadata()), encoding="utf-8")
            (baseline_dir / "metadata.yaml").write_text(yaml.safe_dump(curved_metadata(role="baseline")), encoding="utf-8")
            result = compare_result_dirs(
                candidate_dir=candidate_dir,
                baseline_dir=baseline_dir,
                duration_tolerance=0.1,
                initial_state_tolerance=1.0e-6,
                near_zero_epsilon=1.0e-12,
            )
            self.assertFalse(result["comparison_valid"])
            self.assertIn("summary_malformed", result["compatibility_reasons"])
            self.assertTrue((candidate_dir / "comparison.json").is_file())
            self.assertEqual(candidate_bytes, (candidate_dir / "summary.json").read_bytes())
            self.assertEqual(b"not-json", (baseline_dir / "summary.json").read_bytes())

    def test_legacy_trajectory_and_cylinder_comparisons_remain_legacy(self):
        for task in ("trajectory", "cylinder_navigation"):
            with self.subTest(task=task):
                result = compare_summaries(
                    candidate_summary=legacy_summary(),
                    baseline_summary=legacy_summary(),
                    candidate_metadata=legacy_metadata(task),
                    baseline_metadata=legacy_metadata(task),
                    near_zero_epsilon=1.0e-12,
                    duration_tolerance=0.1,
                    initial_state_tolerance=1.0e-6,
                )
                self.assertTrue(result.compatibility_valid)
                self.assertTrue(result.comparison_valid)
                self.assertIsNone(result.baseline_run_valid)
                self.assertTrue(result.metric_comparisons)
                self.assertEqual([], result.boolean_comparisons)


if __name__ == "__main__":
    unittest.main()
