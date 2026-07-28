import math
import os
import signal
import sys
import tempfile
import time
import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_mppi_controller"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_sim"))

from ctr_evaluation.metrics import compare_summaries  # noqa: E402
from ctr_evaluation.run_evaluation import (  # noqa: E402
    CommandAudit,
    CommandEvent,
    EvaluationOrchestrator,
    OrchestrationError,
    OrchestrationSettings,
    ProcessManager,
    StateTipSample,
    build_controller_configuration_hash,
    build_base_simulation_command,
    build_run_id,
    build_shared_environment_hash,
    command_event_from_message,
    compute_initial_stability,
    default_config_paths,
    main,
    model_reachability_sanity,
    output_root_from_config,
    parse_args,
    parse_completed_result,
    parse_started_run_id,
    process_matches,
    resolve_result_dir,
    strict_json_file,
    unexpected_command_publishers,
    validate_experiment_group,
    write_orchestration_failure,
)
from ctr_bringup.parameter_validation import load_parameter_files  # noqa: E402


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


def settings(**overrides):
    values = {
        "startup_timeout": 1.0,
        "service_timeout": 1.0,
        "topic_ready_timeout": 1.0,
        "reference_ready_timeout": 1.0,
        "finalization_timeout": 1.0,
        "initial_stability_duration": 0.5,
        "initial_stability_samples": 10,
        "initial_q_stability_tolerance": 1.0e-6,
        "initial_tip_stability_tolerance": 1.0e-6,
        "baseline_candidate_q_tolerance": 1.0e-6,
        "baseline_candidate_tip_tolerance": 1.0e-6,
        "reference_lead_time": 1.0,
        "command_zero_tolerance": 1.0e-12,
        "shutdown_sigint_timeout": 0.2,
        "shutdown_sigterm_timeout": 0.2,
        "allow_sigkill_cleanup": True,
        "require_no_baseline_command": True,
        "require_recording_before_candidate_command": True,
    }
    values.update(overrides)
    return OrchestrationSettings(**values)


def stable_samples(count=10, *, q_step=0.0, tip_step=0.0, nonfinite=False):
    samples = []
    for index in range(count):
        q = [q_step * index, 0.0, 0.0, 0.0, 0.0, 0.0]
        tip = [tip_step * index, 0.0, 0.08]
        if nonfinite and index == count - 1:
            q[0] = math.nan
        samples.append(StateTipSample(timestamp=float(index), q=q, tip=tip, receive_time=0.1 * index))
    return samples


def summary(rmse=1.0):
    return {
        "tracking": {"rmse": rmse, "mean_error": rmse, "median_error": rmse, "p95_error": rmse, "max_error": rmse},
        "control": {"total_control_effort": 0.0},
        "timing": {"deadline_overrun_percentage": 0.0},
    }


def orchestrated_meta(**overrides):
    values = {
        "orchestration_id": "orch",
        "run_role": "candidate",
        "shared_environment_hash": "shared",
        "controller_configuration_hash": "controller",
        "reference_start_policy": "scheduled_time",
        "reference_lead_duration_s": 1.0,
        "reference_phase_offset_s": 1.0,
        "reference_pre_epoch_behavior": "first_trajectory_point",
        "requested_evaluation_duration_s": 12.0,
        "evaluation_window_duration_s": 12.0,
        "actual_duration": 13.0,
        "initial_state_q": [0.0] * 6,
        "initial_tip_position": [0.0192, 0.0, 0.08],
        "baseline_nonzero_command_count": 0,
        "candidate_command_after_recording": True,
        "baseline_candidate_tip_tolerance": 5.0e-5,
        "configuration": {
            "trajectory_type": "circle",
            "trajectory_parameters_hash": "traj",
            "frame_id": "base_link",
            "model_configuration_hash": "model",
            "software_mode": "simulation",
            "configured_control_period": 0.05,
            "reference_sample_period": 0.05,
            "configured_duration": 12.0,
        },
    }
    values.update(overrides)
    return values


def write_metadata(run_dir, *, orchestration_id="orch", run_role="baseline"):
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "metadata.yaml").write_text(
        yaml.safe_dump({"orchestration_id": orchestration_id, "run_role": run_role}),
        encoding="utf-8",
    )


class RunEvaluationHelpersTest(unittest.TestCase):
    def test_cli_argument_parsing(self):
        args = parse_args(
            [
                "--experiment-group",
                "m5d_circle_matched",
                "--trajectory",
                "circle",
                "--baseline",
                "zero_command",
                "--candidate",
                "mppi",
                "--duration",
                "12.0",
            ]
        )
        self.assertEqual("m5d_circle_matched", args.experiment_group)
        self.assertEqual("circle", args.trajectory)
        self.assertAlmostEqual(12.0, args.duration)

    def test_cylinder_cli_argument_parsing(self):
        args = parse_args(
            [
                "--experiment-group",
                "m6a_cylinder",
                "--task",
                "cylinder_navigation",
                "--target",
                "0.015",
                "0.005",
                "0.1",
                "--mppi-profile",
                "cylinder_fast",
                "--seed",
                "11",
                "--duration",
                "8.0",
            ]
        )
        self.assertEqual("cylinder_navigation", args.task)
        self.assertEqual([0.015, 0.005, 0.1], args.target)
        self.assertEqual("cylinder_fast", args.mppi_profile)
        self.assertEqual(11, args.seed)

    def test_default_cli_success_allows_worse_performance(self):
        original = patch_fake_orchestrator({"comparison": {"compatibility_valid": True}, "baseline_improvement_pass": False})
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(0, code)

    def test_require_improvement_returns_nonzero_for_valid_worse_performance(self):
        original = patch_fake_orchestrator({"comparison": {"compatibility_valid": True}, "baseline_improvement_pass": False})
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0", "--require-improvement"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(4, code)

    def test_invalid_comparison_returns_nonzero(self):
        original = patch_fake_orchestrator({"comparison": {"compatibility_valid": False}, "baseline_improvement_pass": True})
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(3, code)

    def test_main_returns_nonzero_on_orchestration_failure(self):
        original = patch_raising_orchestrator()
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(2, code)

    def test_baseline_command_publisher_rejection(self):
        self.assertEqual(
            {"/ctr/safe_command": 1},
            unexpected_command_publishers({"/ctr/mppi_command": 0, "/ctr/safe_command": 1}),
        )

    def test_valid_experiment_group_names(self):
        for value in ("m5d1_circle_pair", "circle-01", "experiment.v2", "mppi_baseline_2026"):
            with self.subTest(value=value):
                self.assertEqual(value, validate_experiment_group(value))

    def test_invalid_experiment_group_names(self):
        invalid_values = (
            "",
            " ",
            ".",
            "..",
            "../outside",
            "group/child",
            "group\\child",
            "/tmp/result",
            "./group",
            "group/../outside",
            " group",
            "group ",
            "a" * 129,
            "group\x00child",
        )
        for value in invalid_values:
            with self.subTest(value=repr(value)):
                with self.assertRaisesRegex(OrchestrationError, "experiment_group"):
                    validate_experiment_group(value)

    def test_invalid_group_fails_before_subprocess_started(self):
        import ctr_evaluation.run_evaluation as module

        calls = []
        original = module.ProcessManager.start
        module.ProcessManager.start = lambda *args, **kwargs: calls.append((args, kwargs))
        try:
            code = main(["--experiment-group", "../outside", "--duration", "12.0"])
        finally:
            module.ProcessManager.start = original
        self.assertEqual(2, code)
        self.assertEqual([], calls)

    def test_baseline_launch_command_omits_empty_baseline_argument(self):
        command = build_base_simulation_command(
            experiment_group="group",
            controller_label="zero_command",
            baseline_dir=None,
        )
        self.assertNotIn("evaluation_baseline_result_dir:=", command)
        self.assertTrue(all(not item.endswith(":=") for item in command))

    def test_cylinder_launch_command_includes_navigation_arguments(self):
        command = build_base_simulation_command(
            experiment_group="group",
            controller_label="mppi",
            baseline_dir=None,
            task="cylinder_navigation",
            target_position=[0.015, 0.005, 0.1],
            mppi_profile="cylinder_fast",
            random_seed=11,
            run_role="candidate",
        )
        self.assertIn("enable_cylindrical_lumen:=true", command)
        self.assertIn("cylinder_target_x:=0.015000000", command)
        self.assertIn("cylinder_target_y:=0.005000000", command)
        self.assertIn("cylinder_target_z:=0.100000000", command)
        self.assertIn("cylinder_profile:=cylinder_fast", command)
        self.assertIn("mppi_random_seed:=11", command)
        self.assertIn("run_role:=candidate", command)

    def test_cylinder_launch_command_rejects_missing_target(self):
        with self.assertRaises(OrchestrationError):
            build_base_simulation_command(
                experiment_group="group",
                controller_label="mppi",
                baseline_dir=None,
                task="cylinder_navigation",
                target_position=[],
            )

    def test_candidate_launch_command_includes_baseline_result_dir(self):
        command = build_base_simulation_command(
            experiment_group="group",
            controller_label="mppi",
            baseline_dir=Path("/tmp/baseline"),
        )
        self.assertIn("evaluation_baseline_result_dir:=/tmp/baseline", command)

    def test_launch_command_includes_output_root_override(self):
        command = build_base_simulation_command(
            experiment_group="group",
            controller_label="mppi",
            baseline_dir=None,
            output_root=Path("/tmp/results"),
        )
        self.assertIn("evaluation_output_root:=/tmp/results", command)

    def test_baseline_nonzero_command_rejection(self):
        audit = CommandAudit(
            events=[
                CommandEvent("/ctr/safe_command", 1.0, "command_message_timestamp", 1.0, [0.0] * 6),
                CommandEvent("/ctr/safe_command", 2.0, "command_message_timestamp", 2.0, [0.0, 1.0e-3, 0.0, 0.0, 0.0, 0.0]),
            ]
        )
        self.assertEqual(1, audit.nonzero_count(1.0e-12))

    def test_candidate_command_before_recording_rejects_comparison(self):
        candidate = orchestrated_meta(candidate_command_after_recording=False)
        baseline = orchestrated_meta(run_role="baseline", controller_configuration_hash="baseline")
        result = compare_summaries(
            candidate_summary=summary(1.0),
            baseline_summary=summary(2.0),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertTrue(any("candidate command" in reason for reason in result.compatibility_reasons))
        self.assertIsNone(result.metric_comparisons[0].relative_improvement_percent)

    def test_candidate_command_receive_time_fallback(self):
        event = command_event_from_message(
            "/ctr/mppi_command",
            fake_command_msg(sec=0, nanosec=0, q_dot=[0.1] * 6),
            receive_time=3.0,
            receive_timestamp=4.5,
        )
        self.assertEqual("command_receive_timestamp", event.timestamp_type)
        self.assertAlmostEqual(4.5, event.timestamp)

    def test_candidate_command_message_timestamp_is_used(self):
        event = command_event_from_message(
            "/ctr/mppi_command",
            fake_command_msg(sec=2, nanosec=250000000, q_dot=[0.0] * 6),
            receive_time=3.0,
            receive_timestamp=4.5,
        )
        self.assertEqual("command_message_timestamp", event.timestamp_type)
        self.assertAlmostEqual(2.25, event.timestamp)

    def test_initial_stability_pass(self):
        stats = compute_initial_stability(stable_samples(10), settings())
        self.assertTrue(stats.stable)
        self.assertEqual(10, stats.consecutive_stable_samples)

    def test_initial_stability_failure(self):
        stats = compute_initial_stability(stable_samples(10, q_step=1.0e-4), settings())
        self.assertFalse(stats.stable)
        self.assertIn("q variation", stats.reason)

    def test_insufficient_stability_samples(self):
        stats = compute_initial_stability(stable_samples(3), settings())
        self.assertFalse(stats.stable)
        self.assertIn("sample count", stats.reason)

    def test_nonfinite_state_tip_rejection(self):
        stats = compute_initial_stability(stable_samples(10, nonfinite=True), settings())
        self.assertFalse(stats.stable)
        self.assertIn("non-finite", stats.reason)

    def test_initial_q_incompatibility(self):
        candidate = orchestrated_meta(initial_state_q=[0.001] + [0.0] * 5)
        baseline = orchestrated_meta(run_role="baseline")
        result = compare_summaries(
            candidate_summary=summary(),
            baseline_summary=summary(),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertTrue(any("initial state differs" in reason for reason in result.compatibility_reasons))

    def test_initial_tip_incompatibility(self):
        candidate = orchestrated_meta(initial_tip_position=[0.1, 0.0, 0.08])
        baseline = orchestrated_meta(run_role="baseline")
        result = compare_summaries(
            candidate_summary=summary(),
            baseline_summary=summary(),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertTrue(any("initial tip" in reason for reason in result.compatibility_reasons))

    def test_shared_environment_hash_compatibility(self):
        candidate = orchestrated_meta(shared_environment_hash="a")
        baseline = orchestrated_meta(run_role="baseline", shared_environment_hash="b")
        result = compare_summaries(
            candidate_summary=summary(),
            baseline_summary=summary(),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertIn("incompatible shared_environment_hash", result.compatibility_reasons)

    def test_controller_hash_difference_remains_allowed(self):
        candidate = orchestrated_meta(controller_configuration_hash="mppi")
        baseline = orchestrated_meta(run_role="baseline", controller_configuration_hash="zero")
        result = compare_summaries(
            candidate_summary=summary(1.0),
            baseline_summary=summary(2.0),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertTrue(result.compatibility_valid)

    def test_reference_phase_mismatch_rejection(self):
        candidate = orchestrated_meta(reference_phase_offset_s=1.2)
        baseline = orchestrated_meta(run_role="baseline", reference_phase_offset_s=1.0)
        result = compare_summaries(
            candidate_summary=summary(),
            baseline_summary=summary(),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertTrue(any("reference_phase_offset" in reason for reason in result.compatibility_reasons))

    def test_evaluation_window_duration_mismatch(self):
        candidate = orchestrated_meta(evaluation_window_duration_s=13.0)
        baseline = orchestrated_meta(run_role="baseline", evaluation_window_duration_s=12.0)
        result = compare_summaries(
            candidate_summary=summary(),
            baseline_summary=summary(),
            candidate_metadata=candidate,
            baseline_metadata=baseline,
            near_zero_epsilon=1.0e-12,
            duration_tolerance=0.1,
            initial_state_tolerance=5.0e-5,
        )
        self.assertFalse(result.compatibility_valid)
        self.assertIn("evaluation-window duration differs beyond tolerance", result.compatibility_reasons)

    def test_service_response_parsing(self):
        self.assertEqual("run_1", parse_started_run_id("started evaluation run run_1"))
        self.assertEqual(("run_1", Path("/tmp/run_1")), parse_completed_result("completed evaluation run run_1: /tmp/run_1"))

    def test_exact_run_id_result_discovery(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            run_dir = root / "group" / "run_exact"
            write_metadata(run_dir, orchestration_id="orch", run_role="baseline")
            resolved = resolve_result_dir(
                response_message=f"completed evaluation run run_exact: {run_dir}",
                output_root=root,
                experiment_group="group",
                run_id="run_exact",
                orchestration_id="orch",
                run_role="baseline",
            )
            self.assertEqual(run_dir.resolve(), resolved)

    def test_valid_result_path_inside_exact_group_is_accepted(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            run_dir = root / "group" / "run_exact"
            write_metadata(run_dir, orchestration_id="orch", run_role="baseline")
            resolved = resolve_result_dir(
                response_message=f"completed evaluation run run_exact: {run_dir}",
                output_root=root,
                experiment_group="group",
                run_id="run_exact",
                orchestration_id="orch",
                run_role="baseline",
            )
            self.assertEqual(run_dir.resolve(), resolved)

    def test_result_path_outside_output_root_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            base = Path(temp_dir)
            root = base / "results"
            outside = base / "outside" / "run_exact"
            root.mkdir()
            write_metadata(outside, orchestration_id="orch", run_role="baseline")
            with self.assertRaises(OrchestrationError):
                resolve_result_dir(
                    response_message=f"completed evaluation run run_exact: {outside}",
                    output_root=root,
                    experiment_group="group",
                    run_id="run_exact",
                    orchestration_id="orch",
                    run_role="baseline",
                )

    def test_result_path_inside_output_root_but_outside_requested_group_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            run_dir = root / "other_group" / "run_exact"
            write_metadata(run_dir, orchestration_id="orch", run_role="baseline")
            with self.assertRaises(OrchestrationError):
                resolve_result_dir(
                    response_message=f"completed evaluation run run_exact: {run_dir}",
                    output_root=root,
                    experiment_group="group",
                    run_id="run_exact",
                    orchestration_id="orch",
                    run_role="baseline",
                )

    def test_sibling_prefix_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            base = Path(temp_dir)
            root = base / "results"
            sibling = base / "results-other" / "group" / "run_exact"
            root.mkdir()
            write_metadata(sibling, orchestration_id="orch", run_role="baseline")
            with self.assertRaises(OrchestrationError):
                resolve_result_dir(
                    response_message=f"completed evaluation run run_exact: {sibling}",
                    output_root=root,
                    experiment_group="group",
                    run_id="run_exact",
                    orchestration_id="orch",
                    run_role="baseline",
                )

    def test_symlink_result_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            base = Path(temp_dir)
            root = base / "results"
            group = root / "group"
            outside = base / "outside" / "run_exact"
            group.mkdir(parents=True)
            write_metadata(outside, orchestration_id="orch", run_role="baseline")
            link = group / "run_exact"
            try:
                link.symlink_to(outside, target_is_directory=True)
            except (OSError, NotImplementedError) as exc:
                self.skipTest(f"symlink creation is not supported: {exc}")
            with self.assertRaises(OrchestrationError):
                resolve_result_dir(
                    response_message=f"completed evaluation run run_exact: {link}",
                    output_root=root,
                    experiment_group="group",
                    run_id="run_exact",
                    orchestration_id="orch",
                    run_role="baseline",
                )

    def test_ambiguous_result_directory_rejection(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            run_dir = root / "group" / "run_exact"
            other = root / "other" / "run_exact"
            write_metadata(run_dir, orchestration_id="orch", run_role="baseline")
            write_metadata(other, orchestration_id="orch", run_role="baseline")
            with self.assertRaises(OrchestrationError):
                resolve_result_dir(
                    response_message=f"completed evaluation run run_exact: {other}",
                    output_root=root,
                    experiment_group="group",
                    run_id="run_exact",
                    orchestration_id="orch",
                    run_role="baseline",
                )

    def test_result_directory_metadata_mismatch_rejection(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            run_dir = root / "group" / "run_exact"
            write_metadata(run_dir, orchestration_id="other", run_role="baseline")
            with self.assertRaises(OrchestrationError):
                resolve_result_dir(
                    response_message=f"completed evaluation run run_exact: {run_dir}",
                    output_root=root,
                    experiment_group="group",
                    run_id="run_exact",
                    orchestration_id="orch",
                    run_role="baseline",
                )

    def test_strict_json_rejects_nonstandard_tokens(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.json"
            path.write_text('{"value": NaN}\n', encoding="utf-8")
            with self.assertRaises(OrchestrationError):
                strict_json_file(path)

    def test_strict_orchestration_failure_json(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = write_orchestration_failure(Path(temp_dir), "group", "orch", {"value": math.nan, "reason": "failed"})
            data = strict_json_file(path)
            self.assertIsNone(data["value"])
            self.assertEqual("failed", data["reason"])

    def test_hash_design_allows_controller_difference(self):
        config = load_parameter_files(CONFIG_FILES)
        shared_a = build_shared_environment_hash(config, trajectory="circle", duration=12.0, reference_lead_time=1.0)
        shared_b = build_shared_environment_hash(config, trajectory="circle", duration=12.0, reference_lead_time=1.0)
        controller_a = build_controller_configuration_hash(config, "zero_command")
        controller_b = build_controller_configuration_hash(config, "mppi")
        self.assertEqual(shared_a, shared_b)
        self.assertNotEqual(controller_a, controller_b)

    def test_cylinder_shared_hash_changes_with_goal(self):
        config = load_parameter_files(CONFIG_FILES)
        changed = yaml.safe_load(yaml.safe_dump(config))
        changed["goal"]["position"] = [0.010, 0.012, 0.095]
        shared_a = build_shared_environment_hash(
            config,
            task="cylinder_navigation",
            trajectory="circle",
            duration=12.0,
            reference_lead_time=1.0,
        )
        shared_b = build_shared_environment_hash(
            changed,
            task="cylinder_navigation",
            trajectory="circle",
            duration=12.0,
            reference_lead_time=1.0,
        )
        self.assertNotEqual(shared_a, shared_b)

    def test_model_reachability_sanity_reaches_default_cylinder_target(self):
        from ctr_model.approximate_model import ApproximateCTRModel

        config = load_parameter_files(CONFIG_FILES)
        result = model_reachability_sanity(
            model=ApproximateCTRModel(config),
            config=config,
            target=config["goal"]["position"],
            tolerance=float(config["goal"]["tolerance"]),
        )
        self.assertTrue(result["reachable"], result)
        self.assertGreater(result["evaluated_candidates"], 0)

    def test_run_ids_are_unique(self):
        first = build_run_id("orch", "baseline", "zero_command")
        second = build_run_id("orch", "baseline", "zero_command")
        self.assertNotEqual(first, second)
        self.assertIn("baseline", first)

    def test_ros_domain_ids_are_unique_within_orchestration(self):
        import ctr_evaluation.run_evaluation as module

        args = parse_args(["--experiment-group", "group", "--duration", "12.0"])
        orchestrator = EvaluationOrchestrator(args)
        original = module.fresh_ros_domain_id
        values = iter([150, 150, 151])
        module.fresh_ros_domain_id = lambda: next(values)
        try:
            self.assertEqual(150, orchestrator._fresh_domain_id())
            self.assertEqual(151, orchestrator._fresh_domain_id())
        finally:
            module.fresh_ros_domain_id = original

    def test_output_root_override(self):
        config = {"evaluation": {"output_root": "evaluation_results"}}
        self.assertEqual(Path("/tmp/custom").resolve(), output_root_from_config(config, "/tmp/custom"))

    def test_default_config_paths_from_explicit_list(self):
        self.assertEqual([str(path.resolve()) for path in CONFIG_FILES], default_config_paths([str(path) for path in CONFIG_FILES]))

    def test_command_audit_first_event(self):
        audit = CommandAudit(
            events=[
                CommandEvent("/ctr/safe_command", 2.0, "command_message_timestamp", 2.0, [0.0] * 6),
                CommandEvent("/ctr/safe_command", 1.0, "command_message_timestamp", 1.0, [0.0] * 6),
            ]
        )
        self.assertAlmostEqual(1.0, audit.first_event().timestamp)


class ProcessManagerTest(unittest.TestCase):
    def test_pid_pgid_ownership_verification(self):
        manager = ProcessManager(REPO_ROOT)
        record = manager.start(role="sleep", command=[sys.executable, "-c", "import time; time.sleep(30)"], env=dict(os.environ))
        try:
            self.assertTrue(process_matches(record.identity))
        finally:
            manager.shutdown_all([record], settings())

    def test_sigint_clean_shutdown(self):
        manager = ProcessManager(REPO_ROOT)
        record = manager.start(role="sleep", command=[sys.executable, "-c", "import time; time.sleep(30)"], env=dict(os.environ))
        manager.shutdown_all([record], settings())
        self.assertIsNotNone(record.exit_code)
        self.assertTrue(any(event["signal"] == "SIGINT" and event["sent"] for event in record.shutdown_events))

    def test_sigterm_escalation_for_owned_process(self):
        manager = ProcessManager(REPO_ROOT)
        command = [
            sys.executable,
            "-c",
            "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(30)",
        ]
        record = manager.start(role="ignore_int", command=command, env=dict(os.environ))
        time.sleep(0.2)
        manager.shutdown_all([record], settings(shutdown_sigint_timeout=0.05, shutdown_sigterm_timeout=0.2))
        self.assertTrue(any(event["signal"] == "SIGTERM" and event["sent"] for event in record.shutdown_events))

    def test_sigkill_fallback_policy_for_owned_process(self):
        manager = ProcessManager(REPO_ROOT)
        command = [
            sys.executable,
            "-c",
            "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)",
        ]
        record = manager.start(role="ignore_int_term", command=command, env=dict(os.environ))
        time.sleep(0.2)
        manager.shutdown_all([record], settings(shutdown_sigint_timeout=0.05, shutdown_sigterm_timeout=0.05, allow_sigkill_cleanup=True))
        self.assertTrue(any(event["signal"] == "SIGKILL" and event["sent"] for event in record.shutdown_events))

    def test_no_unrelated_process_is_signaled(self):
        manager = ProcessManager(REPO_ROOT)
        owned = manager.start(role="owned", command=[sys.executable, "-c", "import time; time.sleep(30)"], env=dict(os.environ))
        unrelated = manager.start(role="unrelated", command=[sys.executable, "-c", "import time; time.sleep(30)"], env=dict(os.environ))
        try:
            manager.shutdown_all([owned], settings())
            self.assertIsNone(unrelated.process.poll())
        finally:
            manager.shutdown_all([unrelated], settings())


def fake_command_msg(*, sec, nanosec, q_dot):
    class Stamp:
        pass

    class Header:
        pass

    class Message:
        pass

    stamp = Stamp()
    stamp.sec = sec
    stamp.nanosec = nanosec
    header = Header()
    header.stamp = stamp
    msg = Message()
    msg.header = header
    msg.q_dot = q_dot
    return msg


def patch_fake_orchestrator(result):
    import ctr_evaluation.run_evaluation as module

    original = module.EvaluationOrchestrator

    class FakeOrchestrator:
        def __init__(self, args):
            self.args = args

        def run_pair(self):
            return result

    module.EvaluationOrchestrator = FakeOrchestrator
    return original


def patch_raising_orchestrator():
    import ctr_evaluation.run_evaluation as module

    original = module.EvaluationOrchestrator

    class RaisingOrchestrator:
        def __init__(self, args):
            self.args = args

        def run_pair(self):
            raise OrchestrationError("boom")

    module.EvaluationOrchestrator = RaisingOrchestrator
    return original


def restore_orchestrator(original):
    import ctr_evaluation.run_evaluation as module

    module.EvaluationOrchestrator = original


if __name__ == "__main__":
    unittest.main()
