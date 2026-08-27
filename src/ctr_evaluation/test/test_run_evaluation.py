import contextlib
from dataclasses import asdict
import io
import json
import math
import os
import signal
import sys
import tempfile
import time
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

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
    RosRunMonitor,
    StabilityStats,
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
    reference_subscription_qos_for_target_source,
    reference_target_identity,
    resolve_result_dir,
    baseline_process_guard_required,
    settings_with_paper_diagnostics,
    strict_json_file,
    target_vectors_equal,
    validate_target_identity_metadata,
    validate_task_options,
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


def cylinder_orchestrated_meta(**overrides):
    target = [0.015, 0.005, 0.1]
    values = orchestrated_meta(
        reference_start_policy="fixed_target_window_epoch",
        reference_pre_epoch_behavior="fixed_target_ready",
        requested_target=list(target),
        executed_target=list(target),
        target_replaced=False,
        target_identity_valid=True,
        reference_matches_requested_target=True,
    )
    values["configuration"] = {
        **values["configuration"],
        "goal": {"position": list(target), "tolerance": 0.003, "required_hold_duration": 0.5},
        "cylindrical_lumen": {"enabled": True},
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
    def test_paper_diagnostics_extend_only_the_finalization_timeout(self):
        base = settings(finalization_timeout=20.0)
        assert settings_with_paper_diagnostics(base, enabled=False) is base
        diagnostic = settings_with_paper_diagnostics(base, enabled=True)
        assert diagnostic.finalization_timeout == 60.0
        assert {
            key: value for key, value in asdict(diagnostic).items()
            if key != "finalization_timeout"
        } == {
            key: value for key, value in asdict(base).items()
            if key != "finalization_timeout"
        }

    def run_one_with_fake_runtime(
        self,
        stop_actions,
        *,
        pre_stop_error=None,
        cleanup=None,
        metadata_write_error=False,
    ):
        import ctr_evaluation.run_evaluation as module

        class FakeRecord:
            def __init__(self, role):
                self.role = role
                self.identity = SimpleNamespace(pid=1234)

            def to_dict(self):
                return {"role": self.role, "pid": self.identity.pid}

        class FakeProcessManager:
            def __init__(self):
                self.start_calls = []
                self.shutdown_calls = 0
                self.audit_calls = 0

            def start(self, *, role, command, env):
                self.start_calls.append(role)
                return FakeRecord(role)

            def shutdown_all(self, records, settings):
                self.shutdown_calls += 1

            def audit_cleanup(self, records):
                self.audit_calls += 1
                return cleanup or {"clean": True, "records": []}

        class FakeMonitor:
            def __init__(self, actions):
                self.stop_actions = list(actions)
                self.stop_calls = 0
                self.events = []
                self.now_calls = 0

            def record_runner_event(self, event, **details):
                self.events.append((event, details))

            def set_diagnostic_settings(self, settings):
                pass

            def wait_for_services(self, timeout):
                pass

            def wait_for_state_tip(self, timeout):
                pass

            def command_publisher_counts(self):
                return {"/ctr/mppi_command": 0, "/ctr/safe_command": 0}

            def command_audit_since_now(self, publisher_counts):
                return CommandAudit(publisher_counts=publisher_counts)

            def spin_for(self, duration):
                pass

            def collect_stability_samples(self, *, duration_s, timeout_s, minimum_samples):
                return stable_samples(minimum_samples)

            def record_stability_result(self, samples, stability, entry_time):
                pass

            def command_events_since(self, receive_time):
                return []

            def now(self):
                self.now_calls += 1
                return 1.0 if self.now_calls == 1 else 3.0

            def start_experiment(self, *, experiment_name, metadata, timeout_s):
                return "run_1"

            def wait_for_reference(self, timeout, *, require_horizon):
                if pre_stop_error is not None:
                    raise pre_stop_error

            def verify_fixed_reference_target(self, target, atol):
                pass

            def spin_until_time(self, target):
                pass

            def wait_for_first_command(self, timeout):
                return CommandEvent("/ctr/mppi_command", 1.1, "fake", 1.1, [0.0] * 6)

            def stop_experiment(self, *, timeout_s):
                self.stop_calls += 1
                action = self.stop_actions.pop(0)
                if isinstance(action, Exception):
                    raise action
                if action == "__response__":
                    return f"completed evaluation run run_1: {self.response_path}"
                return action

            def readiness_diagnostics(self):
                return {"evaluated_sample_count": 10, "evaluated_receive_time_span_s": 0.5}

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            run_dir = root / "fake_group" / "run_1"
            write_metadata(run_dir, orchestration_id="orch", run_role="candidate")
            (run_dir / "summary.json").write_text(json.dumps(summary()), encoding="utf-8")
            (run_dir / "report.md").write_text("# report\n", encoding="utf-8")
            monitor = FakeMonitor(stop_actions)
            monitor.response_path = run_dir
            process_manager = FakeProcessManager()
            orchestrator = object.__new__(EvaluationOrchestrator)
            orchestrator.args = SimpleNamespace(
                task="cylinder_navigation",
                duration=1.0,
                trajectory="circle",
                baseline="zero_command",
                candidate="mppi",
                mppi_profile="",
                seed=11,
            )
            orchestrator.output_root = root
            orchestrator.experiment_group = "fake_group"
            orchestrator.orchestration_id = "orch"
            orchestrator.curved_lumen_type = "circular_arc"
            orchestrator.settings = settings()
            orchestrator.process_manager = process_manager
            orchestrator._fresh_domain_id = lambda: 7
            orchestrator._target_position_for_launch = lambda: [0.01, 0.0, 0.1]
            orchestrator._start_metadata = mock.Mock(return_value={})
            orchestrator._reference_command = mock.Mock(return_value=["reference"])
            orchestrator._controller_command = mock.Mock(return_value=["controller"])
            orchestrator._runtime_metadata = mock.Mock(
                return_value={
                    "reference_matches_requested_target": True,
                    "requested_target": [0.01, 0.0, 0.1],
                    "executed_target": [0.01, 0.0, 0.1],
                    "target_replaced": False,
                    "target_identity_valid": True,
                }
            )

            real_write_json = module.write_json

            def write_json_with_failure(path, data):
                if metadata_write_error and Path(path).name == "orchestration.json":
                    raise RuntimeError("metadata write failed")
                return real_write_json(path, data)

            stable = StabilityStats(
                stable=True,
                reason="stable",
                first_q=[0.0] * 6,
                first_tip=[0.0, 0.0, 0.08],
                mean_q_variation=0.0,
                max_q_variation=0.0,
                mean_tip_variation=0.0,
                max_tip_variation=0.0,
                sample_count=10,
                consecutive_stable_samples=10,
                duration_s=0.5,
            )
            with mock.patch.object(module, "run_environment", return_value={}), \
                mock.patch.object(module, "build_base_simulation_command", return_value=["base"]), \
                mock.patch.object(module, "RosRunMonitor", return_value=monitor), \
                mock.patch.object(module, "build_run_id", return_value="run_1"), \
                mock.patch.object(module, "process_name_running", return_value=False), \
                mock.patch.object(module, "simulator_command_timeout", return_value=0.0), \
                mock.patch.object(module.time, "sleep"), \
                mock.patch.object(module, "compute_initial_stability", return_value=stable), \
                mock.patch.object(module, "write_json", side_effect=write_json_with_failure):
                try:
                    result = orchestrator._run_one(role="candidate", controller_label="mppi", baseline_dir=None)
                    error = None
                except Exception as exc:
                    result = None
                    error = exc

            persisted = None
            metadata_path = run_dir / "orchestration.json"
            if metadata_path.is_file():
                persisted = json.loads(metadata_path.read_text(encoding="utf-8"))
            return result, error, persisted, monitor, process_manager

    def diagnostic_monitor(self):
        monitor = object.__new__(RosRunMonitor)
        monitor._diagnostic_settings = settings()
        monitor._diagnostics = {
            "state_callback_count": 13,
            "tip_callback_count": 13,
            "state_callback_count_before_collection": 3,
            "tip_callback_count_before_collection": 3,
            "state_callback_count_during_collection": 10,
            "tip_callback_count_during_collection": 10,
            "stability_collection_start_monotonic": 10.0,
            "stability_deadline_monotonic": 11.0,
            "stability_collection_end_monotonic": 10.7,
            "evaluated_samples": [],
            "evaluated_sample_count": 0,
            "first_evaluated_sample_receive_monotonic": None,
            "last_evaluated_sample_receive_monotonic": None,
            "evaluated_receive_time_span_s": None,
            "criteria": None,
            "readiness_result": None,
            "readiness_failure_reason": None,
        }
        monitor._state_callback_sequence = 0
        monitor._readiness_state_queue = []
        monitor._readiness_collection_active = False
        return monitor

    def collect_with_batches(self, batches, *, pre_window=False, duration=0.5, minimum_samples=10):
        monitor = self.diagnostic_monitor()
        monitor._state_callback_sequence = 5
        if pre_window:
            monitor._readiness_state_queue = [(5, stable_samples(1)[0])]
        batch_index = 0

        def spin_once(_timeout):
            nonlocal batch_index
            if batch_index < len(batches):
                for sample in batches[batch_index]:
                    monitor._state_callback_sequence += 1
                    monitor._readiness_state_queue.append((monitor._state_callback_sequence, sample))
                batch_index += 1

        monitor.spin_once = spin_once
        clock_value = 0.0

        def monotonic():
            nonlocal clock_value
            value = clock_value
            clock_value += 0.1
            return value

        with mock.patch("ctr_evaluation.run_evaluation.time.monotonic", side_effect=monotonic):
            return monitor, monitor.collect_stability_samples(
                duration_s=duration,
                timeout_s=10.0,
                minimum_samples=minimum_samples,
            )

    def test_collection_excludes_pre_window_state_and_counts_each_new_callback(self):
        batches = [[sample] for sample in stable_samples(8)]
        monitor, samples = self.collect_with_batches(batches, pre_window=True)
        self.assertEqual(8, len(samples))
        self.assertEqual(8, monitor.readiness_diagnostics()["evaluated_sample_count"])
        self.assertEqual(5, monitor.readiness_diagnostics()["stability_collection_state_sequence"])

    def test_collection_waits_for_sample_count_after_duration(self):
        samples = stable_samples(12)
        samples = [StateTipSample(item.timestamp, item.q, item.tip, 0.0 if i == 0 else 0.6 + i * 0.01) for i, item in enumerate(samples)]
        monitor, collected = self.collect_with_batches([[item] for item in samples])
        self.assertEqual(10, len(collected))
        self.assertTrue(monitor.readiness_diagnostics()["evaluated_receive_time_span_s"] >= 0.5)

    def test_collection_waits_for_duration_after_sample_count(self):
        samples = stable_samples(60)
        samples = [StateTipSample(item.timestamp, item.q, item.tip, i * 0.01) for i, item in enumerate(samples)]
        monitor, collected = self.collect_with_batches([[item] for item in samples])
        self.assertGreaterEqual(len(collected), 50)
        self.assertGreaterEqual(monitor.readiness_diagnostics()["evaluated_receive_time_span_s"], 0.5)

    def test_collection_passes_only_after_both_requirements(self):
        samples = stable_samples(10)
        samples = [StateTipSample(item.timestamp, item.q, item.tip, i * (0.5 / 9.0)) for i, item in enumerate(samples)]
        monitor, collected = self.collect_with_batches([[item] for item in samples])
        stats = compute_initial_stability(collected, settings())
        monitor.record_stability_result(collected, stats, 1.0)
        diagnostics = monitor.readiness_diagnostics()
        self.assertEqual(10, len(collected))
        self.assertEqual({"finite_values": True, "sample_count": True, "duration": True, "q_variation": True, "tip_variation": True}, diagnostics["criteria"])
        self.assertTrue(stats.stable)

    def test_collection_timeout_preserves_sample_and_duration_failures(self):
        samples = stable_samples(8)
        samples = [StateTipSample(item.timestamp, item.q, item.tip, i * 0.01) for i, item in enumerate(samples)]
        monitor, collected = self.collect_with_batches([[item] for item in samples])
        stats = compute_initial_stability(collected, settings())
        monitor.record_stability_result(collected, stats, 1.0)
        diagnostics = monitor.readiness_diagnostics()
        self.assertEqual(8, diagnostics["evaluated_sample_count"])
        self.assertFalse(diagnostics["criteria"]["sample_count"])
        self.assertFalse(diagnostics["criteria"]["duration"])
        self.assertIn("sample count", diagnostics["readiness_failure_reason"])
        self.assertIn("duration", diagnostics["readiness_failure_reason"])

    def test_readiness_diagnostics_report_below_sample_count(self):
        samples = stable_samples(3)
        stats = compute_initial_stability(samples, settings())
        monitor = self.diagnostic_monitor()
        monitor.record_stability_result(samples, stats, 10.1)
        diagnostics = monitor.readiness_diagnostics()
        self.assertEqual(13, diagnostics["state_callback_count"])
        self.assertEqual(3, diagnostics["state_callback_count_before_collection"])
        self.assertEqual(10, diagnostics["state_callback_count_during_collection"])
        self.assertEqual(3, diagnostics["evaluated_sample_count"])
        self.assertFalse(diagnostics["criteria"]["sample_count"])
        self.assertFalse(diagnostics["criteria"]["duration"])
        self.assertFalse(diagnostics["readiness_result"])
        self.assertIn("sample count", diagnostics["readiness_failure_reason"])

    def test_readiness_diagnostics_report_stable_success_without_changing_contract(self):
        samples = stable_samples(10)
        stats = compute_initial_stability(samples, settings())
        monitor = self.diagnostic_monitor()
        monitor.record_stability_result(samples, stats, 10.1)
        diagnostics = monitor.readiness_diagnostics()
        self.assertTrue(stats.stable)
        self.assertTrue(diagnostics["readiness_result"])
        self.assertEqual({"finite_values": True, "sample_count": True, "duration": True, "q_variation": True, "tip_variation": True}, diagnostics["criteria"])
        self.assertEqual(10, diagnostics["evaluated_sample_count"])

    def test_readiness_diagnostics_report_duration_failure(self):
        samples = stable_samples(10)
        samples = [StateTipSample(sample.timestamp, sample.q, sample.tip, index * 0.01) for index, sample in enumerate(samples)]
        stats = compute_initial_stability(samples, settings())
        monitor = self.diagnostic_monitor()
        monitor.record_stability_result(samples, stats, 10.1)
        diagnostics = monitor.readiness_diagnostics()
        self.assertFalse(stats.stable)
        self.assertFalse(diagnostics["criteria"]["duration"])
        self.assertTrue(diagnostics["criteria"]["sample_count"])
        self.assertIn("duration", diagnostics["readiness_failure_reason"])

    def test_readiness_diagnostics_report_nonfinite_failure(self):
        samples = stable_samples(10, nonfinite=True)
        stats = compute_initial_stability(samples, settings())
        monitor = self.diagnostic_monitor()
        monitor.record_stability_result(samples, stats, 10.1)
        diagnostics = monitor.readiness_diagnostics()
        self.assertFalse(stats.stable)
        self.assertFalse(diagnostics["criteria"]["finite_values"])
        self.assertIn("non-finite", diagnostics["readiness_failure_reason"])

    def test_readiness_diagnostics_are_strict_json_and_persistable_on_failure(self):
        samples = stable_samples(3)
        stats = compute_initial_stability(samples, settings())
        monitor = self.diagnostic_monitor()
        monitor.record_stability_result(samples, stats, 10.1)
        with tempfile.TemporaryDirectory() as temp_dir:
            path = write_orchestration_failure(
                Path(temp_dir),
                "group",
                "orch",
                {"orchestration_success": False, "readiness_diagnostics": monitor.readiness_diagnostics()},
            )
            data = strict_json_file(path)
        self.assertEqual(3, data["readiness_diagnostics"]["evaluated_sample_count"])
        encoded = json.dumps(data, allow_nan=False)
        self.assertNotIn("NaN", encoded)
        self.assertNotIn("Infinity", encoded)

    def test_stop_service_timeout_records_deadline_and_future_state(self):
        monitor = object.__new__(RosRunMonitor)
        monitor._diagnostics = {"runner_events": []}
        monitor.spin_once = lambda _timeout: None

        class Future:
            def done(self):
                return False

            def cancelled(self):
                return False

            def exception(self):
                return None

        class Client:
            def call_async(self, _request):
                return Future()

        clock = iter((0.0, 0.1, 2.0, 2.1))
        with mock.patch("ctr_evaluation.run_evaluation.time.monotonic", side_effect=lambda: next(clock)):
            with self.assertRaisesRegex(OrchestrationError, "StopExperiment timed out"):
                monitor._call_service(Client(), None, 1.0, "StopExperiment")
        timeout_event = [event for event in monitor._diagnostics["runner_events"] if event["event"] == "StopExperiment_timeout"][0]
        wait_event = [event for event in monitor._diagnostics["runner_events"] if event["event"] == "StopExperiment_wait_start"][0]
        self.assertEqual(1.0, wait_event["timeout_s"])
        self.assertEqual(2.1, timeout_event["timeout_monotonic"])
        self.assertFalse(timeout_event["future_done"])

    def test_stop_timeout_retries_and_returns_authoritative_response(self):
        orchestrator = object.__new__(EvaluationOrchestrator)
        orchestrator.settings = settings()

        class Monitor:
            def __init__(self):
                self.calls = 0
                self.events = []

            def stop_experiment(self, *, timeout_s):
                self.calls += 1
                if self.calls == 1:
                    raise OrchestrationError("StopExperiment timed out")
                return "completed evaluation run run_1: /tmp/results/group/run_1"

            def record_runner_event(self, event, **details):
                self.events.append((event, details))

        monitor = Monitor()
        response, recovered, recovery_error = orchestrator._stop_experiment_with_recovery(monitor)

        self.assertEqual("completed evaluation run run_1: /tmp/results/group/run_1", response)
        self.assertTrue(recovered)
        self.assertIsInstance(recovery_error, OrchestrationError)
        self.assertEqual(2, monitor.calls)
        self.assertEqual("stop_recovery", monitor.events[0][0])
        self.assertEqual("ok", monitor.events[0][1]["status"])

    def test_stop_timeout_retry_failure_does_not_fabricate_success(self):
        orchestrator = object.__new__(EvaluationOrchestrator)
        orchestrator.settings = settings()

        class Monitor:
            def __init__(self):
                self.calls = 0

            def stop_experiment(self, *, timeout_s):
                self.calls += 1
                raise OrchestrationError("StopExperiment timed out" if self.calls == 1 else "service rejected")

        monitor = Monitor()
        with self.assertRaisesRegex(OrchestrationError, "StopExperiment timed out"):
            orchestrator._stop_experiment_with_recovery(monitor)
        self.assertEqual(2, monitor.calls)

    def test_normal_stop_uses_first_response_without_recovery(self):
        orchestrator = object.__new__(EvaluationOrchestrator)
        orchestrator.settings = settings()

        class Monitor:
            def __init__(self):
                self.calls = 0
                self.events = []

            def stop_experiment(self, *, timeout_s):
                self.calls += 1
                return "completed evaluation run run_1: /tmp/results/group/run_1"

            def record_runner_event(self, event, **details):
                self.events.append(event)

        monitor = Monitor()
        response, recovered, recovery_error = orchestrator._stop_experiment_with_recovery(monitor)

        self.assertEqual("completed evaluation run run_1: /tmp/results/group/run_1", response)
        self.assertFalse(recovered)
        self.assertIsNone(recovery_error)
        self.assertEqual(1, monitor.calls)
        self.assertEqual([], monitor.events)

    def test_recovered_response_persists_runtime_metadata_once(self):
        import ctr_evaluation.run_evaluation as module

        orchestrator = object.__new__(EvaluationOrchestrator)
        orchestrator.settings = settings()
        orchestrator.experiment_group = "recovered_stop_test"
        orchestrator.orchestration_id = "orch"

        class ProcessManager:
            def __init__(self):
                self.shutdown_calls = 0

            def shutdown_all(self, records, settings):
                self.shutdown_calls += 1

            def audit_cleanup(self, records):
                return {"clean": True, "records": []}

        class Monitor:
            def __init__(self):
                self.events = []

            def readiness_diagnostics(self):
                return {"evaluated_sample_count": 10, "evaluated_receive_time_span_s": 0.5}

            def record_runner_event(self, event, **details):
                self.events.append((event, details))

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            orchestrator.output_root = root
            run_dir = root / "recovered_stop_test" / "run_1"
            write_metadata(run_dir, orchestration_id="orch", run_role="candidate")
            (run_dir / "summary.json").write_text(json.dumps(summary()), encoding="utf-8")
            (run_dir / "report.md").write_text("# report\n", encoding="utf-8")
            orchestrator.process_manager = ProcessManager()
            orchestrator._runtime_metadata = mock.Mock(
                return_value={
                    "orchestration_success": True,
                    "reference_matches_requested_target": True,
                    "requested_target": [0.01, 0.0, 0.1],
                    "executed_target": [0.01, 0.0, 0.1],
                    "target_replaced": False,
                    "target_identity_valid": True,
                    "first_observed_reference_target": [0.01, 0.0, 0.1],
                }
            )
            monitor = Monitor()
            write_json_mock = mock.patch.object(module, "write_json", wraps=module.write_json)
            with write_json_mock as write_json_spy:
                result = orchestrator._finalize_run(
                    role="candidate",
                    run_id="run_1",
                    domain_id=7,
                    monitor=monitor,
                    records=[],
                    stability=mock.Mock(),
                    stop_response=f"completed evaluation run run_1: {run_dir}",
                    cleanup_state={"attempted": False},
                    stop_recovered=True,
                    recovery_error=OrchestrationError("StopExperiment timed out"),
                )

            persisted = json.loads((run_dir / "orchestration.json").read_text(encoding="utf-8"))
            self.assertEqual("run_1", result.run_id)
            self.assertTrue(persisted["reference_matches_requested_target"])
            self.assertTrue(persisted["stop_recovered"])
            self.assertIn("StopExperiment timed out", persisted["stop_recovery_error"])
            self.assertEqual(10, persisted["readiness_diagnostics"]["evaluated_sample_count"])
            self.assertTrue(persisted["cleanup_audit"]["clean"])
            self.assertEqual(1, write_json_spy.call_count)
            self.assertEqual(1, orchestrator.process_manager.shutdown_calls)
            candidate_metadata = cylinder_orchestrated_meta()
            candidate_metadata.update(persisted)
            comparison = compare_summaries(
                candidate_summary=summary(1.0),
                baseline_summary=summary(2.0),
                candidate_metadata=candidate_metadata,
                baseline_metadata=cylinder_orchestrated_meta(
                    run_role="baseline",
                    requested_target=[0.01, 0.0, 0.1],
                    executed_target=[0.01, 0.0, 0.1],
                ),
                near_zero_epsilon=1.0e-12,
                duration_tolerance=0.1,
                initial_state_tolerance=5.0e-5,
            )
            self.assertTrue(comparison.compatibility_valid, comparison.compatibility_reasons)

    def test_run_one_preserves_pre_stop_failure_after_recovered_stop(self):
        original_error = OrchestrationError("reference startup failed")
        result, error, persisted, monitor, process_manager = self.run_one_with_fake_runtime(
            ["__response__"],
            pre_stop_error=original_error,
        )

        self.assertIsNone(result)
        self.assertIs(error, original_error)
        self.assertEqual(1, monitor.stop_calls)
        self.assertEqual(1, process_manager.shutdown_calls)
        self.assertEqual(1, process_manager.audit_calls)
        self.assertIsNotNone(persisted)
        self.assertFalse(persisted["orchestration_success"])
        self.assertEqual("failed", persisted["terminal_status"])
        self.assertEqual("reference startup failed", persisted["error"])
        self.assertTrue(persisted["reference_matches_requested_target"])
        self.assertTrue(persisted["cleanup_audit"]["clean"])

    def test_run_one_cleanup_failure_persists_failed_terminal_state(self):
        result, error, persisted, monitor, process_manager = self.run_one_with_fake_runtime(
            ["__response__"],
            cleanup={"clean": False, "reason": "controller process remained"},
        )

        self.assertIsNone(result)
        self.assertIn("process cleanup audit failed", str(error))
        self.assertEqual(1, monitor.stop_calls)
        self.assertEqual(1, process_manager.shutdown_calls)
        self.assertEqual(1, process_manager.audit_calls)
        self.assertFalse(persisted["orchestration_success"])
        self.assertEqual("failed", persisted["terminal_status"])
        self.assertEqual({"clean": False, "reason": "controller process remained"}, persisted["cleanup_audit"])

    def test_run_one_metadata_write_failure_does_not_retry_cleanup_or_return_success(self):
        result, error, persisted, monitor, process_manager = self.run_one_with_fake_runtime(
            ["__response__"],
            metadata_write_error=True,
        )

        self.assertIsNone(result)
        self.assertEqual("metadata write failed", str(error))
        self.assertIsNone(persisted)
        self.assertEqual(1, monitor.stop_calls)
        self.assertEqual(1, process_manager.shutdown_calls)
        self.assertEqual(1, process_manager.audit_calls)

    def test_run_one_normal_stop_remains_successful_and_does_not_retry(self):
        result, error, persisted, monitor, process_manager = self.run_one_with_fake_runtime(["__response__"])

        self.assertIsNone(error)
        self.assertIsNotNone(result)
        self.assertEqual(1, monitor.stop_calls)
        self.assertEqual(1, process_manager.shutdown_calls)
        self.assertEqual(1, process_manager.audit_calls)
        self.assertTrue(persisted["orchestration_success"])
        self.assertEqual("completed", persisted["terminal_status"])

    def test_run_one_stop_timeout_retry_succeeds_without_prior_failure(self):
        result, error, persisted, monitor, process_manager = self.run_one_with_fake_runtime(
            [OrchestrationError("StopExperiment timed out"), "__response__"]
        )

        self.assertIsNone(error)
        self.assertIsNotNone(result)
        self.assertEqual(2, monitor.stop_calls)
        self.assertEqual(1, process_manager.shutdown_calls)
        self.assertEqual(1, process_manager.audit_calls)
        self.assertTrue(persisted["orchestration_success"])
        self.assertEqual("completed", persisted["terminal_status"])
        self.assertTrue(persisted["stop_recovered"])
        self.assertIn("StopExperiment timed out", persisted["stop_recovery_error"])

    def test_run_one_failed_stop_retry_does_not_return_success(self):
        result, error, persisted, monitor, process_manager = self.run_one_with_fake_runtime(
            [OrchestrationError("StopExperiment timed out"), OrchestrationError("retry rejected")]
        )

        self.assertIsNone(result)
        self.assertIn("StopExperiment timed out", str(error))
        self.assertEqual(2, monitor.stop_calls)
        self.assertEqual(1, process_manager.shutdown_calls)
        self.assertEqual(1, process_manager.audit_calls)
        self.assertIsNone(persisted)

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

    def test_automated_rviz_target_arguments_are_development_only(self):
        args = parse_args(
            [
                "--development-simulation",
                "--experiment-group",
                "rviz_evaluation",
                "--task",
                "curved_lumen_navigation",
                "--runtime-mode",
                "simulation",
                "--duration",
                "5",
                "--target",
                "0.01924686842428271",
                "0.0",
                "0.08098413850007993",
                "--development-target-source",
                "rviz",
                "--development-raw-target",
                "0.01924686842428271",
                "0.03",
                "0.08098413850007993",
                "--development-target-frame",
                "world",
                "--development-target-projection-distance",
                "0.03",
            ]
        )
        validate_task_options(args)
        args.development_simulation = False
        with self.assertRaisesRegex(OrchestrationError, "development target overrides"):
            validate_task_options(args)

    def test_automated_rviz_base_and_candidate_commands_use_fixed_topic_only(self):
        raw = [0.01924686842428271, 0.03, 0.08098413850007993]
        command = build_base_simulation_command(
            experiment_group="rviz_evaluation",
            controller_label="mppi",
            baseline_dir=None,
            task="curved_lumen_navigation",
            target_position=[raw[0], 0.0, raw[2]],
            slice_7g_profile=True,
            development_simulation=True,
            development_target_source="rviz",
            development_raw_target=raw,
            development_target_frame="world",
        )
        self.assertIn("reference_mode:=external_target", command)
        self.assertNotIn("reference_mode:=fixed_target", command)
        self.assertEqual(
            1,
            len([value for value in command if value.startswith("reference_mode:=")]),
        )
        self.assertIn("target_source:=rviz", command)
        self.assertIn("wait_for_target:=true", command)
        self.assertFalse(any("target_point_candidate" in value for value in command))

        orchestrator = EvaluationOrchestrator.__new__(EvaluationOrchestrator)
        orchestrator.development_simulation = True
        orchestrator.development_target_source = "rviz"
        orchestrator.development_raw_target = raw
        orchestrator.development_target_frame = "world"
        candidate = orchestrator._rviz_candidate_command()
        self.assertEqual("/ctr/target_point_candidate", candidate[4])
        self.assertEqual("geometry_msgs/msg/PointStamped", candidate[5])
        self.assertIn("frame_id: 'world'", candidate[6])

    def test_automated_cli_base_command_has_one_external_reference_mode(self):
        command = build_base_simulation_command(
            experiment_group="cli_evaluation",
            controller_label="mppi",
            baseline_dir=None,
            task="curved_lumen_navigation",
            target_position=[0.021180966381970152, 0.0, 0.08471218663414842],
            slice_7g_profile=True,
            development_simulation=True,
            development_target_source="cli",
        )
        assert [value for value in command if value.startswith("reference_mode:=")] == [
            "reference_mode:=external_target"
        ]

    def test_development_straight_fixed_target_task_is_valid(self):
        args = parse_args(
            [
                "--development-simulation",
                "--experiment-group", "straight_evaluation",
                "--duration", "5",
                "--runtime-mode", "simulation",
                "--task", "cylinder_navigation",
                "--target", "0.0192", "0", "0.084",
            ]
        )
        validate_task_options(args)

    def test_target_selector_monitor_uses_late_joiner_reference_qos(self):
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

        assert reference_subscription_qos_for_target_source("profile") == 10
        for target_source in ("cli", "rviz"):
            qos = reference_subscription_qos_for_target_source(target_source)
            assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
            assert qos.reliability == ReliabilityPolicy.RELIABLE
            assert qos.depth == 1

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

    def test_require_sampled_reachable_cli_argument_parsing(self):
        args = parse_args(
            [
                "--experiment-group",
                "m6a1",
                "--task",
                "cylinder_navigation",
                "--duration",
                "8.0",
                "--require-sampled-reachable",
            ]
        )
        self.assertTrue(args.require_sampled_reachable)

    def test_valid_target_is_preserved_exactly(self):
        original = patch_reachability(
            {
                "reachable": True,
                "best_error": 0.0,
                "best_q": [0.0] * 6,
                "best_tip": [0.015, 0.005, 0.1],
                "tolerance": 0.003,
                "evaluated_candidates": 1,
                "random_sample_count": 1,
                "random_seed": 4,
            }
        )
        try:
            orchestrator = EvaluationOrchestrator(
                parse_args(
                    [
                        "--experiment-group",
                        "target_identity",
                        "--task",
                        "cylinder_navigation",
                        "--target",
                        "0.015",
                        "0.005",
                        "0.100",
                        "--duration",
                        "8.0",
                    ]
                )
            )
        finally:
            restore_reachability(original)
        self.assertEqual([0.015, 0.005, 0.1], orchestrator.cylinder_setup["requested_target"])
        self.assertEqual([0.015, 0.005, 0.1], orchestrator.cylinder_setup["executed_target"])
        self.assertTrue(target_vectors_equal(orchestrator.cylinder_setup["requested_target"], orchestrator.cylinder_setup["executed_target"]))
        self.assertFalse(orchestrator.cylinder_setup["target_replaced"])

    def test_axial_target_is_preserved_exactly_when_sampled_check_fails(self):
        target = [0.019, 0.0, 0.105]
        orchestrator, stderr = make_orchestrator_with_sampled_failure(target)
        self.assertEqual(target, orchestrator.cylinder_setup["requested_target"])
        self.assertEqual(target, orchestrator.cylinder_setup["executed_target"])
        self.assertEqual(target, orchestrator._target_position_for_launch())
        self.assertFalse(orchestrator.cylinder_setup["target_replaced"])
        self.assertIn("sampled reachability was not confirmed", stderr)

    def test_lateral_target_is_preserved_exactly_when_sampled_check_fails(self):
        target = [0.010, 0.012, 0.095]
        orchestrator, stderr = make_orchestrator_with_sampled_failure(target)
        self.assertEqual(target, orchestrator.cylinder_setup["requested_target"])
        self.assertEqual(target, orchestrator.cylinder_setup["executed_target"])
        self.assertEqual(target, orchestrator._target_position_for_launch())
        self.assertFalse(orchestrator.cylinder_setup["target_replaced"])
        self.assertIn("sampled reachability was not confirmed", stderr)

    def test_geometry_invalid_target_fails_before_subprocess_started(self):
        import ctr_evaluation.run_evaluation as module

        calls = []
        original_start = module.ProcessManager.start
        module.ProcessManager.start = lambda *args, **kwargs: calls.append((args, kwargs))
        try:
            code = main(
                [
                    "--experiment-group",
                    "invalid_target",
                    "--task",
                    "cylinder_navigation",
                    "--target",
                    "0.040",
                    "0.000",
                    "0.100",
                    "--duration",
                    "8.0",
                ]
            )
        finally:
            module.ProcessManager.start = original_start
        self.assertEqual(2, code)
        self.assertEqual([], calls)

    def test_sampled_check_failure_records_false_by_default(self):
        orchestrator, _stderr = make_orchestrator_with_sampled_failure([0.019, 0.0, 0.105])
        reachability = orchestrator.cylinder_setup["reachability"]
        self.assertFalse(reachability["sampled_reachability_confirmed"])
        self.assertEqual("deterministic_sampling", reachability["sampled_reachability_method"])
        self.assertEqual(4, reachability["sampled_reachability_seed"])
        self.assertEqual(2048, reachability["sampled_reachability_sample_count"])

    def test_require_sampled_reachable_rejects_before_launch_without_replacement(self):
        import ctr_evaluation.run_evaluation as module
        from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen

        calls = []
        original_start = module.ProcessManager.start
        original_nearest = CylindricalLumen.nearest_valid_target
        reachability = patch_reachability(
            {
                "reachable": False,
                "best_error": 0.004,
                "best_q": [0.0] * 6,
                "best_tip": [0.016, 0.0, 0.1],
                "tolerance": 0.003,
                "evaluated_candidates": 1,
                "random_sample_count": 2048,
                "random_seed": 4,
            }
        )
        module.ProcessManager.start = lambda *args, **kwargs: calls.append((args, kwargs))
        CylindricalLumen.nearest_valid_target = lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("replacement calculated"))
        try:
            with self.assertRaisesRegex(OrchestrationError, "sampled reachability"):
                EvaluationOrchestrator(
                    parse_args(
                        [
                            "--experiment-group",
                            "strict_target_identity",
                            "--task",
                            "cylinder_navigation",
                            "--target",
                            "0.019",
                            "0.000",
                            "0.105",
                            "--duration",
                            "8.0",
                            "--require-sampled-reachable",
                        ]
                    )
                )
        finally:
            restore_reachability(reachability)
            module.ProcessManager.start = original_start
            CylindricalLumen.nearest_valid_target = original_nearest
        self.assertEqual([], calls)

    def test_suggested_target_never_becomes_executed_target(self):
        orchestrator, _stderr = make_orchestrator_with_sampled_failure([0.019, 0.0, 0.105])
        suggestion = orchestrator.cylinder_setup["reachability"]["suggested_target"]
        self.assertIsNotNone(suggestion)
        self.assertNotEqual(suggestion, orchestrator.cylinder_setup["executed_target"])
        self.assertEqual(orchestrator.cylinder_setup["requested_target"], orchestrator.cylinder_setup["executed_target"])

    def test_baseline_and_candidate_receive_identical_targets(self):
        target = [0.010, 0.012, 0.095]
        baseline = build_base_simulation_command(
            experiment_group="g",
            controller_label="zero_command",
            baseline_dir=None,
            task="cylinder_navigation",
            target_position=target,
        )
        candidate = build_base_simulation_command(
            experiment_group="g",
            controller_label="mppi",
            baseline_dir=Path("/tmp/baseline"),
            task="cylinder_navigation",
            target_position=target,
        )
        for arg in ("cylinder_target_x:=0.010000000", "cylinder_target_y:=0.012000000", "cylinder_target_z:=0.095000000"):
            self.assertIn(arg, baseline)
            self.assertIn(arg, candidate)

    def test_target_identity_metadata_validation_passes(self):
        metadata = cylinder_orchestrated_meta(run_role="baseline")
        validate_target_identity_metadata(metadata, expected_target=[0.015, 0.005, 0.1], label="baseline")

    def test_reference_target_identity_match_and_mismatch(self):
        matching = reference_target_identity(
            expected_target=[0.015, 0.005, 0.1],
            observed_target=[0.015, 0.005, 0.1],
            observed_timestamp=12.5,
            atol=1.0e-9,
        )
        self.assertTrue(matching["reference_matches_requested_target"])
        self.assertEqual([0.015, 0.005, 0.1], matching["first_observed_reference_target"])
        mismatching = reference_target_identity(
            expected_target=[0.015, 0.005, 0.1],
            observed_target=[0.015, 0.005001, 0.1],
            observed_timestamp=12.5,
            atol=1.0e-9,
        )
        self.assertFalse(mismatching["reference_matches_requested_target"])

    def test_shared_hash_changes_when_requested_target_changes(self):
        from ctr_mppi_controller.cylindrical_lumen import config_with_cylinder_overrides

        config = load_parameter_files(CONFIG_FILES)
        first = config_with_cylinder_overrides(config, enabled=True, target_position=[0.015, 0.005, 0.1])
        second = config_with_cylinder_overrides(config, enabled=True, target_position=[0.010, 0.012, 0.095])
        hash_first = build_shared_environment_hash(
            first,
            task="cylinder_navigation",
            trajectory="circle",
            duration=8.0,
            reference_lead_time=1.0,
        )
        hash_second = build_shared_environment_hash(
            second,
            task="cylinder_navigation",
            trajectory="circle",
            duration=8.0,
            reference_lead_time=1.0,
        )
        self.assertNotEqual(hash_first, hash_second)

    def test_suggested_target_does_not_affect_shared_hash(self):
        orchestrator, _stderr = make_orchestrator_with_sampled_failure([0.019, 0.0, 0.105])
        first = build_shared_environment_hash(
            orchestrator.project_config,
            task="cylinder_navigation",
            trajectory="circle",
            duration=8.0,
            reference_lead_time=orchestrator.settings.reference_lead_time,
        )
        orchestrator.cylinder_setup["reachability"]["suggested_target"] = [0.0, 0.0, 0.1]
        second = build_shared_environment_hash(
            orchestrator.project_config,
            task="cylinder_navigation",
            trajectory="circle",
            duration=8.0,
            reference_lead_time=orchestrator.settings.reference_lead_time,
        )
        self.assertEqual(first, second)

    def test_default_cli_success_allows_worse_performance(self):
        original = patch_fake_orchestrator(
            {"comparison": {"compatibility_valid": True, "comparison_valid": True}, "baseline_improvement_pass": False}
        )
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(0, code)

    def test_require_improvement_returns_nonzero_for_valid_worse_performance(self):
        original = patch_fake_orchestrator(
            {
                "comparison": {"compatibility_valid": True, "comparison_valid": True},
                "baseline_improvement_pass": False,
                "timing_pass": False,
                "real_time_pass": False,
            }
        )
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0", "--require-improvement"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(4, code)

    def test_timing_diagnostics_do_not_change_cli_acceptance(self):
        result = {
            "comparison": {"compatibility_valid": True, "comparison_valid": True},
            "baseline_improvement_pass": True,
            "timing_pass": False,
            "real_time_pass": False,
        }
        original = patch_fake_orchestrator(result)
        try:
            self.assertEqual(0, main(["--experiment-group", "g", "--duration", "12.0"]))
            self.assertEqual(0, main(["--experiment-group", "g", "--duration", "12.0", "--require-improvement"]))
        finally:
            restore_orchestrator(original)

    def test_invalid_comparison_returns_nonzero(self):
        original = patch_fake_orchestrator(
            {"comparison": {"compatibility_valid": True, "comparison_valid": False}, "baseline_improvement_pass": True}
        )
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0"])
        finally:
            restore_orchestrator(original)
        self.assertEqual(3, code)

    def test_require_improvement_rejects_identity_compatible_invalid_comparison(self):
        original = patch_fake_orchestrator(
            {"comparison": {"compatibility_valid": True, "comparison_valid": False}, "baseline_improvement_pass": True}
        )
        try:
            code = main(["--experiment-group", "g", "--duration", "12.0", "--require-improvement"])
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

    def test_development_baseline_uses_domain_publisher_audit_not_host_process_names(self):
        self.assertFalse(
            baseline_process_guard_required(role="baseline", development_simulation=True)
        )
        self.assertTrue(
            baseline_process_guard_required(role="baseline", development_simulation=False)
        )
        self.assertFalse(
            baseline_process_guard_required(role="candidate", development_simulation=False)
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

    def test_baseline_candidate_target_mismatch_invalidates_comparison(self):
        candidate = cylinder_orchestrated_meta(requested_target=[0.010, 0.012, 0.095], executed_target=[0.010, 0.012, 0.095])
        baseline = cylinder_orchestrated_meta(run_role="baseline")
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
        self.assertTrue(any("requested_target differ" in reason for reason in result.compatibility_reasons))
        self.assertIsNone(result.metric_comparisons[0].relative_improvement_percent)

    def test_requested_executed_target_mismatch_invalidates_comparison(self):
        candidate = cylinder_orchestrated_meta(executed_target=[0.010, 0.012, 0.095])
        baseline = cylinder_orchestrated_meta(run_role="baseline")
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
        self.assertTrue(any("requested_target differs from executed_target" in reason for reason in result.compatibility_reasons))

    def test_target_replaced_invalidates_comparison(self):
        candidate = cylinder_orchestrated_meta(target_replaced=True)
        baseline = cylinder_orchestrated_meta(run_role="baseline")
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
        self.assertTrue(any("target was replaced" in reason for reason in result.compatibility_reasons))

    def test_missing_target_identity_invalidates_cylinder_comparison(self):
        candidate = cylinder_orchestrated_meta()
        baseline = cylinder_orchestrated_meta(run_role="baseline")
        for key in ("requested_target", "executed_target", "target_replaced", "target_identity_valid"):
            candidate.pop(key, None)
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
        self.assertTrue(any("target identity metadata missing" in reason for reason in result.compatibility_reasons))

    def test_reference_target_mismatch_invalidates_comparison(self):
        candidate = cylinder_orchestrated_meta(reference_matches_requested_target=False)
        baseline = cylinder_orchestrated_meta(run_role="baseline")
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
        self.assertTrue(any("published reference target" in reason for reason in result.compatibility_reasons))

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


def patch_reachability(result):
    import ctr_evaluation.run_evaluation as module

    original = module.model_reachability_sanity
    module.model_reachability_sanity = lambda **kwargs: dict(result)
    return original


def restore_reachability(original):
    import ctr_evaluation.run_evaluation as module

    module.model_reachability_sanity = original


def make_orchestrator_with_sampled_failure(target):
    reachability = patch_reachability(
        {
            "reachable": False,
            "best_error": 0.004,
            "best_q": [0.0] * 6,
            "best_tip": [0.016, -0.002, 0.104],
            "tolerance": 0.003,
            "evaluated_candidates": 1,
            "random_sample_count": 2048,
            "random_seed": 4,
        }
    )
    stderr = io.StringIO()
    try:
        with contextlib.redirect_stderr(stderr):
            orchestrator = EvaluationOrchestrator(
                parse_args(
                    [
                        "--experiment-group",
                        "sampled_identity",
                        "--task",
                        "cylinder_navigation",
                        "--target",
                        f"{target[0]:.9f}",
                        f"{target[1]:.9f}",
                        f"{target[2]:.9f}",
                        "--duration",
                        "8.0",
                    ]
                )
            )
    finally:
        restore_reachability(reachability)
    return orchestrator, stderr.getvalue()


def restore_orchestrator(original):
    import ctr_evaluation.run_evaluation as module

    module.EvaluationOrchestrator = original


if __name__ == "__main__":
    unittest.main()
