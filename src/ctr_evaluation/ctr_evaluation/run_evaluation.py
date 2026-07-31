"""Deterministic paired baseline/candidate evaluation orchestration."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
from typing import Any, Callable
import uuid

import numpy as np
import yaml

from ctr_bringup.parameter_validation import load_parameter_files, validate_config_paths, validate_or_raise
from ctr_evaluation.compare_results import compare_result_dirs, read_json, write_json
from ctr_evaluation.curved_lumen_scenarios import (
    CENTERLINE_TARGET,
    CURVED_LUMEN_SCENARIO_IDS,
    CurvedLumenScenario,
    resolve_curved_lumen_scenario,
)
from ctr_evaluation.metrics import sanitize_for_json, stable_hash
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.cylindrical_lumen import (
    CylindricalLumen,
    config_with_cylinder_overrides,
    goal_position_from_config,
    goal_tolerance_from_config,
)
from ctr_mppi_controller.lumen_factory import CURVED_LUMEN_TYPES, config_with_lumen_overrides
from ctr_sim.simulation_core import CTRSimulationCore


TASK_TRAJECTORY = "trajectory"
TASK_CYLINDER_NAVIGATION = "cylinder_navigation"
TASK_CURVED_LUMEN_NAVIGATION = "curved_lumen_navigation"
TASK_CHOICES = (TASK_TRAJECTORY, TASK_CYLINDER_NAVIGATION, TASK_CURVED_LUMEN_NAVIGATION)
FIXED_TARGET_TASKS = (TASK_CYLINDER_NAVIGATION, TASK_CURVED_LUMEN_NAVIGATION)
DEFAULT_CURVED_LUMEN_TYPE = "circular_arc"
DEFAULT_CURVED_SCENARIO = CENTERLINE_TARGET

CONFIG_NAMES = (
    "robot_params.yaml",
    "model_params.yaml",
    "mppi_params.yaml",
    "simulation_params.yaml",
    "evaluation_params.yaml",
    "safety_params.yaml",
    "tactile_params.yaml",
    "hardware_params.yaml",
)
COMMAND_TOPICS = ("/ctr/mppi_command", "/ctr/safe_command")
EXPERIMENT_GROUP_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
MAX_EXPERIMENT_GROUP_LENGTH = 128
TARGET_IDENTITY_ATOL = 1.0e-9
RUN_STARTED_RE = re.compile(r"started evaluation run (?P<run_id>[A-Za-z0-9_-]+)")
RUN_COMPLETED_RE = re.compile(r"completed evaluation run (?P<run_id>[A-Za-z0-9_-]+): (?P<path>.+)$")


@dataclass(frozen=True)
class OrchestrationSettings:
    startup_timeout: float
    service_timeout: float
    topic_ready_timeout: float
    reference_ready_timeout: float
    finalization_timeout: float
    initial_stability_duration: float
    initial_stability_samples: int
    initial_q_stability_tolerance: float
    initial_tip_stability_tolerance: float
    baseline_candidate_q_tolerance: float
    baseline_candidate_tip_tolerance: float
    reference_lead_time: float
    command_zero_tolerance: float
    shutdown_sigint_timeout: float
    shutdown_sigterm_timeout: float
    allow_sigkill_cleanup: bool
    require_no_baseline_command: bool
    require_recording_before_candidate_command: bool


@dataclass(frozen=True)
class StateTipSample:
    timestamp: float
    q: list[float]
    tip: list[float]
    receive_time: float


@dataclass(frozen=True)
class StabilityStats:
    stable: bool
    reason: str
    first_q: list[float]
    first_tip: list[float]
    mean_q_variation: float
    max_q_variation: float
    mean_tip_variation: float
    max_tip_variation: float
    sample_count: int
    consecutive_stable_samples: int
    duration_s: float


@dataclass(frozen=True)
class CommandEvent:
    topic: str
    timestamp: float
    timestamp_type: str
    receive_time: float
    command: list[float]

    @property
    def command_norm(self) -> float:
        return float(np.linalg.norm(np.asarray(self.command, dtype=float)))


@dataclass
class CommandAudit:
    events: list[CommandEvent] = field(default_factory=list)
    publisher_counts: dict[str, int] = field(default_factory=dict)
    started_receive_time: float = field(default_factory=time.monotonic)

    def nonzero_count(self, tolerance: float) -> int:
        return sum(1 for event in self.events if event.command_norm > tolerance)

    def first_event(self) -> CommandEvent | None:
        if not self.events:
            return None
        return min(self.events, key=lambda event: (event.receive_time, event.timestamp))

    def to_dict(self, tolerance: float) -> dict[str, Any]:
        first = self.first_event()
        return {
            "command_message_count": len(self.events),
            "nonzero_command_count": self.nonzero_count(tolerance),
            "publisher_counts": dict(self.publisher_counts),
            "first_command_timestamp": None if first is None else first.timestamp,
            "first_command_timestamp_type": "" if first is None else first.timestamp_type,
            "first_command_topic": "" if first is None else first.topic,
        }


@dataclass(frozen=True)
class ProcessIdentity:
    pid: int
    pgid: int
    start_time_ticks: int
    command_line: str


@dataclass
class ProcessRecord:
    role: str
    command: list[str]
    process: subprocess.Popen
    identity: ProcessIdentity
    start_wall_time: str
    exit_code: int | None = None
    shutdown_events: list[dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "role": self.role,
            "pid": self.identity.pid,
            "pgid": self.identity.pgid,
            "command": self.command,
            "command_line": self.identity.command_line,
            "start_time_ticks": self.identity.start_time_ticks,
            "started_at": self.start_wall_time,
            "exit_code": self.exit_code,
            "shutdown_events": self.shutdown_events,
        }


@dataclass
class RunResult:
    role: str
    run_id: str
    run_dir: Path
    metadata: dict[str, Any]
    summary: dict[str, Any]
    orchestration: dict[str, Any]


class OrchestrationError(RuntimeError):
    """Raised when a deterministic evaluation prerequisite fails."""


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a deterministic zero-command baseline and MPPI candidate evaluation pair.")
    parser.add_argument("--experiment-group", required=True)
    parser.add_argument("--trajectory", default="circle", choices=("circle", "ellipse", "helix"))
    parser.add_argument("--task", default=TASK_TRAJECTORY, choices=TASK_CHOICES)
    parser.add_argument("--target", nargs=3, type=float, default=None)
    parser.add_argument("--curved-lumen-type", choices=CURVED_LUMEN_TYPES, default=None)
    parser.add_argument("--scenario", choices=CURVED_LUMEN_SCENARIO_IDS, default=None)
    parser.add_argument("--mppi-profile", default="")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--baseline", default="zero_command")
    parser.add_argument("--candidate", default="mppi")
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--runtime-mode", default="simulation")
    parser.add_argument("--require-improvement", action="store_true")
    parser.add_argument("--require-sampled-reachable", action="store_true")
    parser.add_argument("--output-root", default="")
    parser.add_argument("--config-path", action="append", default=[])
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        args.experiment_group = validate_experiment_group(args.experiment_group)
        result = EvaluationOrchestrator(args).run_pair()
    except Exception as exc:
        trace = traceback.format_exc()
        failure = {"orchestration_success": False, "error": str(exc), "traceback": trace}
        try:
            experiment_group = validate_experiment_group(args.experiment_group)
            config_paths = default_config_paths(args.config_path)
            project_config = load_parameter_files(config_paths)
            output_root = output_root_from_config(project_config, args.output_root)
            write_orchestration_failure(output_root, experiment_group, "", failure)
        except Exception:
            pass
        print(f"ctr_run_evaluation failed: {exc}\n{trace}")
        return 2

    comparison_valid = bool(result["comparison"].get("comparison_valid", False))
    if not comparison_valid:
        print("ctr_run_evaluation failed: comparison is not valid")
        return 3
    if args.require_improvement and not result["baseline_improvement_pass"]:
        print("ctr_run_evaluation failed: --require-improvement was set and baseline_improvement_pass is false")
        return 4
    print(json.dumps(sanitize_for_json(result), indent=2, allow_nan=False))
    return 0


def is_fixed_target_task(task: str) -> bool:
    return task in FIXED_TARGET_TASKS


def is_curved_lumen_task(task: str) -> bool:
    return task == TASK_CURVED_LUMEN_NAVIGATION


def reference_mode_for_task(task: str) -> str:
    return "fixed_target" if is_fixed_target_task(task) else "trajectory"


def validate_task_options(args: argparse.Namespace) -> None:
    if not is_curved_lumen_task(args.task):
        if args.curved_lumen_type is not None:
            raise OrchestrationError("--curved-lumen-type is only valid with --task curved_lumen_navigation")
        if args.scenario is not None:
            raise OrchestrationError("--scenario is only valid with --task curved_lumen_navigation")


class EvaluationOrchestrator:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        if not math.isfinite(args.duration) or args.duration <= 0.0:
            raise OrchestrationError("--duration must be positive and finite")
        validate_task_options(args)
        self.experiment_group = validate_experiment_group(args.experiment_group)
        self.config_paths = default_config_paths(args.config_path)
        raw_config = load_parameter_files(self.config_paths)
        self.curved_lumen_type = args.curved_lumen_type or DEFAULT_CURVED_LUMEN_TYPE
        self.curved_scenario_id = args.scenario or DEFAULT_CURVED_SCENARIO
        self.curved_scenario: CurvedLumenScenario | None = None
        if is_curved_lumen_task(args.task):
            self.project_config = self._resolve_curved_project_config(raw_config)
        else:
            self.project_config = config_with_cylinder_overrides(
                raw_config,
                enabled=args.task == TASK_CYLINDER_NAVIGATION,
                target_position=args.target,
                mppi_profile=args.mppi_profile,
                random_seed="" if args.seed is None else args.seed,
            )
        validate_or_raise(self.project_config)
        self.output_root = output_root_from_config(self.project_config, args.output_root)
        self.settings = orchestration_settings_from_config(self.project_config)
        self.orchestration_id = f"m5d1_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{uuid.uuid4().hex[:8]}"
        self.process_manager = ProcessManager(Path.cwd())
        self.used_domain_ids: set[int] = set()
        self.cylinder_setup = self._validate_cylinder_setup() if args.task == TASK_CYLINDER_NAVIGATION else {}

    def _resolve_curved_project_config(self, raw_config: dict[str, Any]) -> dict[str, Any]:
        reference = raw_config.get("reference", {})
        configured_mode = str(reference.get("mode", "fixed_target")) if isinstance(reference, dict) else ""
        if configured_mode != "fixed_target":
            raise OrchestrationError(
                "curved_lumen_navigation requires reference.mode=fixed_target in the effective configuration"
            )
        effective_config = config_with_lumen_overrides(
            raw_config,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type=self.curved_lumen_type,
            cylinder_profile=self.args.mppi_profile,
            random_seed="" if self.args.seed is None else self.args.seed,
        )
        effective_config.setdefault("reference", {})["mode"] = "fixed_target"
        self.curved_scenario = resolve_curved_lumen_scenario(
            effective_config,
            self.curved_scenario_id,
            target_override=self.args.target,
            curved_lumen_type=self.curved_lumen_type,
        )
        effective_config = config_with_lumen_overrides(
            effective_config,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type=self.curved_lumen_type,
            cylinder_profile=self.args.mppi_profile,
            target=self.curved_scenario.validated_target,
            random_seed="" if self.args.seed is None else self.args.seed,
        )
        effective_config.setdefault("reference", {})["mode"] = "fixed_target"
        return effective_config

    def run_pair(self) -> dict[str, Any]:
        baseline = self._run_one(role="baseline", controller_label=self.args.baseline, baseline_dir=None)
        if is_fixed_target_task(self.args.task):
            validate_target_identity_metadata(
                {**baseline.metadata, **baseline.orchestration},
                expected_target=self._target_position_for_launch(),
                label="baseline",
            )
        candidate = self._run_one(role="candidate", controller_label=self.args.candidate, baseline_dir=baseline.run_dir)
        if is_fixed_target_task(self.args.task):
            validate_target_identity_metadata(
                {**candidate.metadata, **candidate.orchestration},
                expected_target=self._target_position_for_launch(),
                label="candidate",
            )
        comparison = compare_result_dirs(
            candidate_dir=candidate.run_dir,
            baseline_dir=baseline.run_dir,
            duration_tolerance=float(self.project_config["evaluation"]["duration_compatibility_tolerance"]),
            initial_state_tolerance=self.settings.baseline_candidate_q_tolerance,
            near_zero_epsilon=float(self.project_config["evaluation"]["near_zero_baseline_epsilon"]),
        )
        candidate.orchestration["comparison_valid"] = comparison.get("comparison_valid", False)
        write_json(candidate.run_dir / "orchestration.json", candidate.orchestration)
        candidate_summary = read_json(candidate.run_dir / "summary.json")
        baseline_improvement_pass = bool(candidate_summary.get("acceptance", {}).get("baseline_improvement_pass", False))
        return {
            "orchestration_success": True,
            "orchestration_id": self.orchestration_id,
            "baseline_dir": str(baseline.run_dir),
            "candidate_dir": str(candidate.run_dir),
            "comparison": comparison,
            "comparison_valid": bool(comparison.get("comparison_valid", False)),
            "baseline_improvement_pass": baseline_improvement_pass,
            "timing_pass": bool(candidate_summary.get("acceptance", {}).get("timing_pass", False)),
            "real_time_pass": bool(candidate_summary.get("acceptance", {}).get("real_time_pass", False)),
        }

    def _run_one(self, *, role: str, controller_label: str, baseline_dir: Path | None) -> RunResult:
        domain_id = self._fresh_domain_id()
        run_id = build_run_id(self.orchestration_id, role, controller_label)
        records: list[ProcessRecord] = []
        monitor: RosRunMonitor | None = None
        recording_active = False
        try:
            env = run_environment(domain_id)
            base_command = build_base_simulation_command(
                experiment_group=self.experiment_group,
                controller_label=controller_label,
                baseline_dir=baseline_dir,
                output_root=self.output_root,
                task=self.args.task,
                target_position=self._target_position_for_launch(),
                curved_lumen_type=self.curved_lumen_type,
                mppi_profile=self.args.mppi_profile,
                random_seed=self.args.seed,
                run_role=role,
            )
            records.append(self.process_manager.start(role=f"{role}_base", command=base_command, env=env))
            monitor = RosRunMonitor(domain_id=domain_id)
            monitor.wait_for_services(self.settings.service_timeout)
            monitor.wait_for_state_tip(self.settings.topic_ready_timeout)
            if role == "baseline" and process_name_running("mppi_controller_node"):
                raise OrchestrationError("MPPI controller process is running during baseline startup")
            publisher_counts = monitor.command_publisher_counts()
            audit = monitor.command_audit_since_now(publisher_counts)
            if unexpected_command_publishers(publisher_counts):
                raise OrchestrationError(f"unexpected command publisher exists: {publisher_counts}")
            time.sleep(simulator_command_timeout())
            monitor.spin_for(simulator_command_timeout())
            samples = monitor.collect_stability_samples(
                duration_s=self.settings.initial_stability_duration,
                timeout_s=self.settings.topic_ready_timeout,
            )
            stability = compute_initial_stability(samples, self.settings)
            audit.events.extend(monitor.command_events_since(audit_start_receive_time(audit)))
            if not stability.stable:
                raise OrchestrationError(f"initial state is not stable: {stability.reason}")
            if audit.nonzero_count(self.settings.command_zero_tolerance) > 0:
                raise OrchestrationError("nonzero command received before recording")

            recording_start_time = monitor.now()
            reference_epoch = recording_start_time + self.settings.reference_lead_time
            evaluation_window_end = reference_epoch + self.args.duration
            metadata = self._start_metadata(
                role=role,
                controller_label=controller_label,
                run_id=run_id,
                domain_id=domain_id,
                recording_start_time=recording_start_time,
                reference_epoch=reference_epoch,
                evaluation_window_end=evaluation_window_end,
                stability=stability,
                audit=audit,
                publisher_counts=publisher_counts,
                records=records,
            )
            started_run_id = monitor.start_experiment(
                experiment_name=controller_label,
                metadata=metadata,
                timeout_s=self.settings.service_timeout,
            )
            recording_active = True
            if started_run_id != run_id:
                raise OrchestrationError(f"evaluator started unexpected run ID {started_run_id}; expected {run_id}")

            reference_command = self._reference_command(reference_epoch)
            records.append(self.process_manager.start(role=f"{role}_reference", command=reference_command, env=env))
            monitor.wait_for_reference(self.settings.reference_ready_timeout, require_horizon=self.args.task == TASK_TRAJECTORY)
            if is_fixed_target_task(self.args.task):
                monitor.verify_fixed_reference_target(self._target_position_for_launch(), TARGET_IDENTITY_ATOL)
            if role == "baseline" and self.args.task == TASK_TRAJECTORY:
                monitor.verify_pre_epoch_reference(
                    reference_epoch,
                    self.settings.reference_ready_timeout,
                    expected_first_point=expected_first_reference_point(self.project_config, self.args.trajectory),
                )
            if role == "candidate":
                if is_fixed_target_task(self.args.task):
                    monitor.spin_until_time(reference_epoch)
                controller_command = self._controller_command()
                records.append(self.process_manager.start(role=f"{role}_controller", command=controller_command, env=env))
                first_command = monitor.wait_for_first_command(self.settings.startup_timeout)
                if self.settings.require_recording_before_candidate_command and first_command.timestamp < recording_start_time:
                    raise OrchestrationError("candidate command timestamp is before recording start")

            monitor.spin_until_time(evaluation_window_end)
            stop_response = monitor.stop_experiment(timeout_s=self.settings.finalization_timeout)
            recording_active = False
            run_dir = resolve_result_dir(
                response_message=stop_response,
                output_root=self.output_root,
                experiment_group=self.experiment_group,
                run_id=run_id,
                orchestration_id=self.orchestration_id,
                run_role=role,
            )
            summary = strict_json_file(run_dir / "summary.json")
            if not (run_dir / "report.md").is_file():
                raise OrchestrationError(f"missing report.md in {run_dir}")
            orchestration = self._runtime_metadata(
                role=role,
                run_id=run_id,
                domain_id=domain_id,
                monitor=monitor,
                records=records,
                stability=stability,
                run_dir=run_dir,
            )
            write_json(run_dir / "orchestration.json", orchestration)
            self.process_manager.shutdown_all(records, self.settings)
            cleanup = self.process_manager.audit_cleanup(records)
            orchestration["cleanup_audit"] = cleanup
            write_json(run_dir / "orchestration.json", orchestration)
            if not cleanup["clean"]:
                raise OrchestrationError(f"process cleanup audit failed for {role}: {cleanup}")
            return RunResult(
                role=role,
                run_id=run_id,
                run_dir=run_dir,
                metadata=read_yaml(run_dir / "metadata.yaml"),
                summary=summary,
                orchestration=orchestration,
            )
        except Exception as exc:
            if monitor is not None and recording_active:
                try:
                    monitor.stop_experiment(timeout_s=self.settings.finalization_timeout)
                except Exception:
                    pass
            failure = {
                "orchestration_success": False,
                "orchestration_id": self.orchestration_id,
                "run_role": role,
                "run_id": run_id,
                "error": str(exc),
                "processes": [record.to_dict() for record in records],
            }
            write_orchestration_failure(self.output_root, self.experiment_group, self.orchestration_id, failure)
            self.process_manager.shutdown_all(records, self.settings)
            raise
        finally:
            if monitor is not None:
                monitor.close()

    def _fresh_domain_id(self) -> int:
        for _ in range(1000):
            domain_id = fresh_ros_domain_id()
            if domain_id not in self.used_domain_ids:
                self.used_domain_ids.add(domain_id)
                return domain_id
        raise OrchestrationError("failed to allocate a unique ROS_DOMAIN_ID for this orchestration")

    def _start_metadata(
        self,
        *,
        role: str,
        controller_label: str,
        run_id: str,
        domain_id: int,
        recording_start_time: float,
        reference_epoch: float,
        evaluation_window_end: float,
        stability: StabilityStats,
        audit: CommandAudit,
        publisher_counts: dict[str, int],
        records: list[ProcessRecord],
    ) -> dict[str, Any]:
        reference_config = self.project_config["reference"]
        reference_mode = reference_mode_for_task(self.args.task)
        reference_start_policy = "fixed_target_window_epoch" if is_fixed_target_task(self.args.task) else "scheduled_time"
        reference_pre_epoch_behavior = "fixed_target_ready" if is_fixed_target_task(self.args.task) else "first_trajectory_point"
        policy_reference_start = "fixed_target_window_epoch" if is_curved_lumen_task(self.args.task) else "scheduled_time"
        policy_pre_epoch = "fixed_target_ready" if is_curved_lumen_task(self.args.task) else "first_trajectory_point"
        orchestration_policy = {
            "initial_stability_duration": self.settings.initial_stability_duration,
            "initial_stability_samples": self.settings.initial_stability_samples,
            "reference_lead_time": self.settings.reference_lead_time,
            "reference_start_policy": policy_reference_start,
            "reference_pre_epoch_behavior": policy_pre_epoch,
            "formal_window": "reference_epoch_to_reference_epoch_plus_duration",
        }
        shared_environment_hash = build_shared_environment_hash(
            self.project_config,
            task=self.args.task,
            trajectory=self.args.trajectory,
            duration=self.args.duration,
            reference_lead_time=self.settings.reference_lead_time,
            curved_scenario=self.curved_scenario,
        )
        controller_hash = build_controller_configuration_hash(self.project_config, controller_label)
        target_identity = self._target_identity_metadata()
        return {
            "requested_run_id": run_id,
            "orchestration_id": self.orchestration_id,
            "run_role": role,
            "experiment_group": self.experiment_group,
            "requested_evaluation_duration_s": self.args.duration,
            "pre_roll_duration_s": self.settings.reference_lead_time,
            "evaluation_window_start_time_s": reference_epoch,
            "evaluation_window_end_time_s": evaluation_window_end,
            "evaluation_window_duration_s": self.args.duration,
            "recording_start_time_s": recording_start_time,
            "reference_start_policy": reference_start_policy,
            "scheduled_reference_epoch_s": reference_epoch,
            "reference_lead_duration_s": self.settings.reference_lead_time,
            "reference_phase_offset_s": self.settings.reference_lead_time,
            "reference_pre_epoch_behavior": reference_pre_epoch_behavior,
            "shared_environment_hash": shared_environment_hash,
            "controller_configuration_hash": controller_hash,
            "orchestration_hash": stable_hash(orchestration_policy),
            "orchestration_policy": orchestration_policy,
            "baseline_candidate_tip_tolerance": self.settings.baseline_candidate_tip_tolerance,
            "command_zero_tolerance": self.settings.command_zero_tolerance,
            "initial_state_stability": asdict(stability),
            "initial_tip_stability": {
                "first_tip": stability.first_tip,
                "mean_variation": stability.mean_tip_variation,
                "max_variation": stability.max_tip_variation,
            },
            "baseline_command_publisher_count": sum(publisher_counts.values()),
            "baseline_mppi_command_publisher_count": publisher_counts.get("/ctr/mppi_command", 0),
            "baseline_safe_command_publisher_count": publisher_counts.get("/ctr/safe_command", 0),
            "pre_roll_command_message_count": len(audit.events),
            "pre_roll_nonzero_command_count": audit.nonzero_count(self.settings.command_zero_tolerance),
            "unexpected_command_publishers": unexpected_command_publishers(publisher_counts),
            **target_identity,
            "reference_configuration": {
                "task": self.args.task,
                "reference_mode": reference_mode,
                "trajectory_type": self.args.trajectory,
                "trajectory_parameters": reference_config.get(self.args.trajectory, {}),
                "sample_period": reference_config.get("sample_period"),
                "frame_id": reference_config.get("frame_id"),
                "loop": reference_config.get("loop"),
                "completion_behavior": reference_config.get("completion_behavior"),
                "goal_position": None if not is_fixed_target_task(self.args.task) else self._target_position_for_launch(),
                "cylinder_setup": self.cylinder_setup,
                "curved_scenario": None if self.curved_scenario is None else self._curved_scenario_identity_metadata(),
                "reference_window_policy": reference_start_policy,
            },
            "processes_at_start": [record.to_dict() for record in records],
            "ros_domain_id": str(domain_id),
        }

    def _target_position_for_launch(self) -> list[float]:
        if self.args.task == TASK_CYLINDER_NAVIGATION:
            return [float(value) for value in goal_position_from_config(self.project_config)]
        if is_curved_lumen_task(self.args.task):
            if self.curved_scenario is None:
                raise OrchestrationError("curved_lumen_navigation target requested before scenario resolution")
            return [float(value) for value in self.curved_scenario.validated_target]
        return []

    def _target_identity_metadata(self) -> dict[str, Any]:
        if is_curved_lumen_task(self.args.task):
            return self._curved_scenario_identity_metadata()
        if self.args.task != TASK_CYLINDER_NAVIGATION:
            return {}
        requested = self.cylinder_setup.get("requested_target", self._target_position_for_launch())
        executed = self.cylinder_setup.get("executed_target", self._target_position_for_launch())
        identity = {
            "requested_target": [float(value) for value in requested],
            "executed_target": [float(value) for value in executed],
            "target_replaced": bool(self.cylinder_setup.get("target_replaced", False)),
            "target_identity_valid": bool(self.cylinder_setup.get("target_identity_valid", False)),
            "target_identity_tolerance": TARGET_IDENTITY_ATOL,
        }
        reachability = self.cylinder_setup.get("reachability", {})
        for key in (
            "sampled_reachability_confirmed",
            "sampled_reachability_method",
            "sampled_reachability_seed",
            "sampled_reachability_sample_count",
            "suggested_target",
        ):
            if key in reachability:
                identity[key] = reachability[key]
        return identity

    def _curved_scenario_identity_metadata(self) -> dict[str, Any]:
        if self.curved_scenario is None:
            raise OrchestrationError("curved_lumen_navigation metadata requested before scenario resolution")
        scenario = self.curved_scenario
        requested = [float(value) for value in scenario.requested_target]
        executed = [float(value) for value in scenario.validated_target]
        return {
            "requested_target": requested,
            "executed_target": executed,
            "validated_target": list(executed),
            "derived_target": [float(value) for value in scenario.derived_target],
            "target_replaced": False,
            "target_identity_valid": True,
            "target_identity_tolerance": TARGET_IDENTITY_ATOL,
            "override_used": bool(scenario.override_used),
            "target_override_used": bool(scenario.override_used),
            "reference_mode": "fixed_target",
            "curved_lumen_type": scenario.curved_lumen_type,
            "scenario_id": scenario.scenario_id,
            "scenario_policy_version": scenario.policy_version,
            "scenario_fingerprint": scenario.scenario_fingerprint,
            "geometry_frame": scenario.geometry_frame,
            "geometry_fingerprint": scenario.geometry_fingerprint,
            "centerline_fraction": float(scenario.centerline_fraction),
            "centerline_arc_length": float(scenario.centerline_arc_length),
            "radial_offset": float(scenario.radial_offset),
            "local_radius": float(scenario.local_radius),
            "preferred_radius": float(scenario.preferred_radius),
            "boundary_guard": float(scenario.boundary_guard),
            "near_boundary": bool(scenario.near_boundary),
        }

    def _cylinder_launch_arguments(self) -> list[str]:
        target = self._target_position_for_launch()
        args = [
            "enable_cylindrical_lumen:=true",
            f"cylinder_target_x:={target[0]:.9f}",
            f"cylinder_target_y:={target[1]:.9f}",
            f"cylinder_target_z:={target[2]:.9f}",
        ]
        if self.args.mppi_profile:
            args.append(f"cylinder_profile:={self.args.mppi_profile}")
        if self.args.seed is not None:
            args.append(f"mppi_random_seed:={self.args.seed}")
        return args

    def _fixed_target_launch_arguments(self) -> list[str]:
        if self.args.task == TASK_CYLINDER_NAVIGATION:
            return self._cylinder_launch_arguments()
        if not is_curved_lumen_task(self.args.task):
            raise OrchestrationError(f"{self.args.task} is not a fixed-target task")
        target = self._target_position_for_launch()
        args = [
            "enable_cylindrical_lumen:=false",
            "enable_curved_lumen:=true",
            f"curved_lumen_type:={self.curved_lumen_type}",
            f"cylinder_target_x:={target[0]:.9f}",
            f"cylinder_target_y:={target[1]:.9f}",
            f"cylinder_target_z:={target[2]:.9f}",
        ]
        if self.args.mppi_profile:
            args.append(f"cylinder_profile:={self.args.mppi_profile}")
        if self.args.seed is not None:
            args.append(f"mppi_random_seed:={self.args.seed}")
        return args

    def _reference_command(self, reference_epoch: float) -> list[str]:
        reference_mode = reference_mode_for_task(self.args.task)
        command = [
            "ros2",
            "launch",
            "ctr_bringup",
            "evaluation_reference.launch.py",
            "runtime_mode:=simulation",
            f"reference_mode:={reference_mode}",
            f"reference_type:={self.args.trajectory}",
        ]
        if self.args.task == TASK_TRAJECTORY:
            command.extend(
                [
                    "trajectory_start_policy:=scheduled_time",
                    f"scheduled_reference_epoch:={reference_epoch:.9f}",
                ]
            )
        else:
            command.extend(self._fixed_target_launch_arguments())
        return command

    def _controller_command(self) -> list[str]:
        reference_mode = reference_mode_for_task(self.args.task)
        command = [
            "ros2",
            "launch",
            "ctr_bringup",
            "evaluation_mppi_controller.launch.py",
            "runtime_mode:=simulation",
            f"reference_mode:={reference_mode}",
            f"reference_type:={self.args.trajectory}",
            "publish_safe_command_for_simulation:=true",
        ]
        if is_fixed_target_task(self.args.task):
            command.extend(self._fixed_target_launch_arguments())
        return command

    def _validate_cylinder_setup(self) -> dict[str, Any]:
        lumen = CylindricalLumen.from_config(self.project_config)
        model = ApproximateCTRModel(self.project_config)
        simulation = CTRSimulationCore(self.project_config)
        initial = model.forward_kinematics(simulation.q)
        initial_clearance = lumen.backbone_clearance(initial.backbone_points)
        if not initial_clearance.collision_free:
            raise OrchestrationError("initial model backbone is outside the cylindrical lumen")
        target = goal_position_from_config(self.project_config)
        validation = lumen.validate_target(target, frame_id=self.project_config["goal"].get("frame_id"))
        if not validation.valid:
            suggestion = lumen.nearest_valid_target(target)
            raise OrchestrationError(
                f"configured cylinder target is invalid: {validation.reasons}; nearest valid suggestion: {suggestion.tolist()}"
            )
        reachability = model_reachability_sanity(
            model=model,
            config=self.project_config,
            target=target,
            tolerance=goal_tolerance_from_config(self.project_config),
        )
        reachability["sampled_reachability_confirmed"] = bool(reachability["reachable"])
        reachability["sampled_reachability_method"] = "deterministic_sampling"
        reachability["sampled_reachability_seed"] = reachability.get("random_seed")
        reachability["sampled_reachability_sample_count"] = reachability.get("random_sample_count")
        if not reachability["reachable"]:
            if self.args.require_sampled_reachable:
                raise OrchestrationError(
                    "sampled reachability was not confirmed for the requested cylinder target; "
                    "strict mode refuses to launch without replacing the target"
                )
            suggested = lumen.nearest_valid_target(reachability["best_tip"]) if reachability.get("best_tip") else target
            reachability["suggested_target"] = [float(value) for value in suggested]
            print(
                "WARNING: sampled reachability was not confirmed for the requested cylinder target; "
                "continuing with the exact requested target because sampled reachability is a sanity check only.",
                file=sys.stderr,
            )
        else:
            reachability["suggested_target"] = None
        reachability["requested_target_replaced"] = False
        reachability["replacement_target"] = None
        target_list = [float(value) for value in target]
        return {
            "initial_backbone_minimum_clearance": initial_clearance.minimum_radial_clearance,
            "initial_tip": [float(value) for value in initial.tip_position],
            "requested_target": target_list,
            "executed_target": list(target_list),
            "target_replaced": False,
            "target_identity_valid": True,
            "target_validation": {"valid": validation.valid, "reasons": validation.reasons},
            "reachability": reachability,
        }

    def _runtime_metadata(
        self,
        *,
        role: str,
        run_id: str,
        domain_id: int,
        monitor: "RosRunMonitor",
        records: list[ProcessRecord],
        stability: StabilityStats,
        run_dir: Path,
    ) -> dict[str, Any]:
        command_audit = monitor.command_audit()
        first = command_audit.first_event()
        reference_epoch = None
        metadata_path = run_dir / "metadata.yaml"
        if metadata_path.is_file():
            metadata = read_yaml(metadata_path)
            reference_epoch = metadata.get("scheduled_reference_epoch_s")
        runtime_metadata = {
            "orchestration_success": True,
            "orchestration_id": self.orchestration_id,
            "run_id": run_id,
            "run_role": role,
            "ros_domain_id": str(domain_id),
            "initial_state_stability": asdict(stability),
            "command_audit": command_audit.to_dict(self.settings.command_zero_tolerance),
            "baseline_nonzero_command_count": command_audit.nonzero_count(self.settings.command_zero_tolerance) if role == "baseline" else None,
            "candidate_first_command_timestamp": None if first is None else first.timestamp,
            "candidate_first_command_timestamp_type": "" if first is None else first.timestamp_type,
            "candidate_command_after_recording": None if role != "candidate" else candidate_after_recording(run_dir, first),
            "candidate_command_before_reference_epoch": (
                None
                if role != "candidate" or first is None or reference_epoch is None
                else first.timestamp < float(reference_epoch)
            ),
            "candidate_command_at_or_after_reference_epoch": (
                None
                if role != "candidate" or first is None or reference_epoch is None
                else first.timestamp >= float(reference_epoch)
            ),
            "processes": [record.to_dict() for record in records],
        }
        runtime_metadata.update(self._target_identity_metadata())
        if is_fixed_target_task(self.args.task):
            runtime_metadata.update(monitor.fixed_reference_target_identity(self._target_position_for_launch(), TARGET_IDENTITY_ATOL))
        return runtime_metadata


class RosRunMonitor:
    def __init__(self, *, domain_id: int):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from ctr_interfaces.msg import CtrJointCommand, CtrState
        from ctr_interfaces.srv import StartExperiment, StopExperiment
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as NavPath

        self.rclpy = rclpy
        self.context = Context()
        try:
            rclpy.init(args=None, context=self.context, domain_id=domain_id)
        except TypeError:
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            rclpy.init(args=None, context=self.context)
        self.node = rclpy.create_node("ctr_run_evaluation_monitor", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.StartExperiment = StartExperiment
        self.StopExperiment = StopExperiment
        self.latest_state: StateTipSample | None = None
        self.latest_tip: tuple[float, list[float], float] | None = None
        self.reference_tip_seen = False
        self.reference_horizon_seen = False
        self.reference_path_seen = False
        self.latest_reference_tip_position: list[float] | None = None
        self.latest_reference_tip_timestamp: float | None = None
        self.first_reference_tip_position: list[float] | None = None
        self.first_reference_tip_timestamp: float | None = None
        self.latest_reference_horizon_first: list[float] | None = None
        self.command_events: list[CommandEvent] = []
        self.state_sub = self.node.create_subscription(CtrState, "/ctr/state", self._on_state, 10)
        self.tip_sub = self.node.create_subscription(PoseStamped, "/ctr/tip", self._on_tip, 10)
        self.ref_tip_sub = self.node.create_subscription(PoseStamped, "/ctr/reference/tip", lambda msg: self._on_reference("tip", msg), 10)
        self.ref_horizon_sub = self.node.create_subscription(NavPath, "/ctr/reference/horizon", lambda msg: self._on_reference("horizon", msg), 10)
        self.ref_path_sub = self.node.create_subscription(NavPath, "/ctr/reference/path", lambda msg: self._on_reference("path", msg), 10)
        self.command_subs = [
            self.node.create_subscription(CtrJointCommand, topic, lambda msg, topic=topic: self._on_command(topic, msg), 10)
            for topic in COMMAND_TOPICS
        ]
        self.start_client = self.node.create_client(StartExperiment, "/ctr/start_experiment")
        self.stop_client = self.node.create_client(StopExperiment, "/ctr/stop_experiment")

    def close(self) -> None:
        try:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        finally:
            self.executor.shutdown()
            if self.rclpy.ok(context=self.context):
                self.rclpy.shutdown(context=self.context)

    def now(self) -> float:
        return float(self.node.get_clock().now().nanoseconds) * 1.0e-9

    def wait_for_services(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            if self.start_client.service_is_ready() and self.stop_client.service_is_ready():
                return
        raise OrchestrationError("timed out waiting for evaluator Start/Stop services")

    def wait_for_state_tip(self, timeout_s: float) -> None:
        self._spin_until(lambda: self.latest_state is not None and self.latest_tip is not None, timeout_s, "state/tip readiness")

    def wait_for_reference(self, timeout_s: float, *, require_horizon: bool = True) -> None:
        self._spin_until(
            lambda: (
                self.reference_tip_seen
                and self.reference_path_seen
                and (self.reference_horizon_seen or not require_horizon)
            ),
            timeout_s,
            "reference readiness",
        )

    def verify_pre_epoch_reference(self, reference_epoch: float, timeout_s: float, *, expected_first_point: list[float]) -> None:
        deadline = min(time.monotonic() + timeout_s, time.monotonic() + max(0.0, reference_epoch - self.now()))
        while time.monotonic() < deadline and self.now() < reference_epoch:
            self.spin_once(0.05)
        expected = np.asarray(expected_first_point, dtype=float)
        for label, value in (
            ("reference tip", self.latest_reference_tip_position),
            ("reference horizon first point", self.latest_reference_horizon_first),
        ):
            if value is None:
                raise OrchestrationError(f"missing pre-epoch {label}")
            if not np.allclose(np.asarray(value, dtype=float), expected, atol=1.0e-12, rtol=0.0):
                raise OrchestrationError(f"pre-epoch {label} does not match the first trajectory point")

    def fixed_reference_target_identity(self, expected_target: Any, atol: float) -> dict[str, Any]:
        return reference_target_identity(
            expected_target=expected_target,
            observed_target=self.first_reference_tip_position,
            observed_timestamp=self.first_reference_tip_timestamp,
            atol=atol,
        )

    def verify_fixed_reference_target(self, expected_target: Any, atol: float) -> None:
        identity = self.fixed_reference_target_identity(expected_target, atol)
        if identity["reference_matches_requested_target"] is not True:
            raise OrchestrationError(
                "published fixed reference target does not match requested target: "
                f"{identity}"
            )

    def collect_stability_samples(self, *, duration_s: float, timeout_s: float) -> list[StateTipSample]:
        samples: list[StateTipSample] = []
        start = time.monotonic()
        deadline = start + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(0.02)
            if self.latest_state is not None and (not samples or samples[-1].receive_time != self.latest_state.receive_time):
                samples.append(self.latest_state)
                if samples[-1].receive_time - samples[0].receive_time >= duration_s:
                    return samples
        return samples

    def command_publisher_counts(self) -> dict[str, int]:
        return {topic: len(self.node.get_publishers_info_by_topic(topic)) for topic in COMMAND_TOPICS}

    def command_audit_since_now(self, publisher_counts: dict[str, int]) -> CommandAudit:
        return CommandAudit(events=[], publisher_counts=publisher_counts, started_receive_time=time.monotonic())

    def command_audit(self) -> CommandAudit:
        return CommandAudit(events=list(self.command_events), publisher_counts=self.command_publisher_counts())

    def command_events_since(self, receive_time: float) -> list[CommandEvent]:
        return [event for event in self.command_events if event.receive_time >= receive_time]

    def start_experiment(self, *, experiment_name: str, metadata: dict[str, Any], timeout_s: float) -> str:
        request = self.StartExperiment.Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.experiment_name = experiment_name
        request.metadata = json.dumps(sanitize_for_json(metadata), allow_nan=False)
        response = self._call_service(self.start_client, request, timeout_s, "StartExperiment")
        if not response.accepted:
            raise OrchestrationError(f"StartExperiment rejected: {response.message}")
        run_id = parse_started_run_id(response.message)
        if run_id is None:
            raise OrchestrationError(f"StartExperiment response did not include run ID: {response.message}")
        return run_id

    def stop_experiment(self, *, timeout_s: float) -> str:
        request = self.StopExperiment.Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        response = self._call_service(self.stop_client, request, timeout_s, "StopExperiment")
        if not response.accepted:
            raise OrchestrationError(f"StopExperiment rejected: {response.message}")
        return str(response.message)

    def wait_for_first_command(self, timeout_s: float) -> CommandEvent:
        start_count = len(self.command_events)
        self._spin_until(lambda: len(self.command_events) > start_count, timeout_s, "first command")
        return self.command_events[start_count]

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self.spin_once(0.05)

    def spin_until_time(self, target_time_s: float) -> None:
        while self.now() < target_time_s:
            self.spin_once(0.05)

    def spin_once(self, timeout_s: float) -> None:
        self.executor.spin_once(timeout_sec=timeout_s)

    def _spin_until(self, predicate: Callable[[], bool], timeout_s: float, label: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            if predicate():
                return
        raise OrchestrationError(f"timed out waiting for {label}")

    def _call_service(self, client, request, timeout_s: float, label: str):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            if future.done():
                return future.result()
        raise OrchestrationError(f"{label} timed out")

    def _on_state(self, msg) -> None:
        try:
            q = [float(value) for value in msg.q]
            tip = [
                float(msg.tip_pose.position.x),
                float(msg.tip_pose.position.y),
                float(msg.tip_pose.position.z),
            ]
            timestamp = stamp_seconds(msg.header.stamp)
            sample = StateTipSample(timestamp=timestamp, q=q, tip=tip, receive_time=time.monotonic())
            if len(q) == 6:
                self.latest_state = sample
        except (TypeError, ValueError):
            return

    def _on_tip(self, msg) -> None:
        try:
            self.latest_tip = (
                stamp_seconds(msg.header.stamp),
                [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)],
                time.monotonic(),
            )
        except (TypeError, ValueError):
            return

    def _on_reference(self, kind: str, msg) -> None:
        if kind == "tip":
            self.reference_tip_seen = True
            position = [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ]
            timestamp = stamp_seconds(msg.header.stamp)
            self.latest_reference_tip_position = position
            self.latest_reference_tip_timestamp = timestamp
            if self.first_reference_tip_position is None:
                self.first_reference_tip_position = list(position)
                self.first_reference_tip_timestamp = timestamp
        elif kind == "horizon":
            self.reference_horizon_seen = True
            if msg.poses:
                first = msg.poses[0].pose.position
                self.latest_reference_horizon_first = [float(first.x), float(first.y), float(first.z)]
        elif kind == "path":
            self.reference_path_seen = True

    def _on_command(self, topic: str, msg) -> None:
        self.command_events.append(command_event_from_message(topic, msg, receive_time=time.monotonic(), receive_timestamp=self.now()))


class ProcessManager:
    def __init__(self, workspace: Path):
        self.workspace = workspace

    def start(self, *, role: str, command: list[str], env: dict[str, str]) -> ProcessRecord:
        process = subprocess.Popen(
            command,
            cwd=self.workspace,
            env=env,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(0.05)
        identity = process_identity(process.pid)
        return ProcessRecord(
            role=role,
            command=list(command),
            process=process,
            identity=identity,
            start_wall_time=datetime.now(timezone.utc).isoformat(),
        )

    def shutdown_all(self, records: list[ProcessRecord], settings: OrchestrationSettings) -> None:
        for record in reversed(records):
            self._shutdown(record, signal.SIGINT, settings.shutdown_sigint_timeout, "SIGINT")
        for record in reversed(records):
            if self._alive(record):
                self._shutdown(record, signal.SIGTERM, settings.shutdown_sigterm_timeout, "SIGTERM")
        for record in reversed(records):
            if self._alive(record):
                if not settings.allow_sigkill_cleanup:
                    record.shutdown_events.append({"signal": "SIGKILL", "sent": False, "reason": "disabled by policy"})
                    continue
                self._shutdown(record, signal.SIGKILL, 1.0, "SIGKILL")

    def audit_cleanup(self, records: list[ProcessRecord]) -> dict[str, Any]:
        remaining = []
        zombies = []
        for record in records:
            for proc in list_process_group(record.identity.pgid):
                remaining.append(proc)
                if "Z" in proc.get("stat", ""):
                    zombies.append(proc)
        return {"clean": not remaining and not zombies, "remaining": remaining, "zombies": zombies}

    def _shutdown(self, record: ProcessRecord, sig: signal.Signals, timeout_s: float, label: str) -> None:
        if not self._alive(record):
            record.exit_code = record.process.poll()
            return
        if not process_matches(record.identity):
            record.shutdown_events.append({"signal": label, "sent": False, "reason": "identity mismatch"})
            return
        try:
            os.killpg(record.identity.pgid, sig)
            record.shutdown_events.append({"signal": label, "sent": True, "pgid": record.identity.pgid})
        except ProcessLookupError:
            record.exit_code = record.process.poll()
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            code = record.process.poll()
            if code is not None:
                record.exit_code = code
                return
            time.sleep(0.05)
        record.exit_code = record.process.poll()

    def _alive(self, record: ProcessRecord) -> bool:
        return record.process.poll() is None


def default_config_paths(extra_paths: list[str] | None = None) -> list[str]:
    if extra_paths:
        return validate_config_paths(extra_paths)
    try:
        from ament_index_python.packages import get_package_share_directory

        config_dir = Path(get_package_share_directory("ctr_bringup")) / "config"
    except Exception:
        config_dir = Path(__file__).resolve().parents[3] / "config"
    return validate_config_paths([str(config_dir / name) for name in CONFIG_NAMES])


def output_root_from_config(config: dict[str, Any], override: str = "") -> Path:
    value = override or str(config["evaluation"]["output_root"])
    return Path(value).expanduser().resolve()


def orchestration_settings_from_config(config: dict[str, Any]) -> OrchestrationSettings:
    values = dict(config["evaluation"].get("orchestration", {}))
    return OrchestrationSettings(
        startup_timeout=float(values["startup_timeout"]),
        service_timeout=float(values["service_timeout"]),
        topic_ready_timeout=float(values["topic_ready_timeout"]),
        reference_ready_timeout=float(values["reference_ready_timeout"]),
        finalization_timeout=float(values["finalization_timeout"]),
        initial_stability_duration=float(values["initial_stability_duration"]),
        initial_stability_samples=int(values["initial_stability_samples"]),
        initial_q_stability_tolerance=float(values["initial_q_stability_tolerance"]),
        initial_tip_stability_tolerance=float(values["initial_tip_stability_tolerance"]),
        baseline_candidate_q_tolerance=float(values["baseline_candidate_q_tolerance"]),
        baseline_candidate_tip_tolerance=float(values["baseline_candidate_tip_tolerance"]),
        reference_lead_time=float(values["reference_lead_time"]),
        command_zero_tolerance=float(values["command_zero_tolerance"]),
        shutdown_sigint_timeout=float(values["shutdown_sigint_timeout"]),
        shutdown_sigterm_timeout=float(values["shutdown_sigterm_timeout"]),
        allow_sigkill_cleanup=bool(values["allow_sigkill_cleanup"]),
        require_no_baseline_command=bool(values["require_no_baseline_command"]),
        require_recording_before_candidate_command=bool(values["require_recording_before_candidate_command"]),
    )


def compute_initial_stability(samples: list[StateTipSample], settings: OrchestrationSettings) -> StabilityStats:
    if not samples:
        return empty_stability("no samples")
    q = np.asarray([sample.q for sample in samples], dtype=float)
    tip = np.asarray([sample.tip for sample in samples], dtype=float)
    if q.shape[1:] != (6,) or tip.shape[1:] != (3,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(tip)):
        return empty_stability("non-finite or malformed state/tip sample")
    duration = samples[-1].receive_time - samples[0].receive_time
    q_variation = np.linalg.norm(q - q[0], axis=1)
    tip_variation = np.linalg.norm(tip - tip[0], axis=1)
    stable = (
        duration >= settings.initial_stability_duration
        and len(samples) >= settings.initial_stability_samples
        and float(np.max(q_variation)) <= settings.initial_q_stability_tolerance
        and float(np.max(tip_variation)) <= settings.initial_tip_stability_tolerance
    )
    reasons = []
    if duration < settings.initial_stability_duration:
        reasons.append("duration below threshold")
    if len(samples) < settings.initial_stability_samples:
        reasons.append("sample count below threshold")
    if float(np.max(q_variation)) > settings.initial_q_stability_tolerance:
        reasons.append("q variation above threshold")
    if float(np.max(tip_variation)) > settings.initial_tip_stability_tolerance:
        reasons.append("tip variation above threshold")
    return StabilityStats(
        stable=stable,
        reason="ok" if stable else "; ".join(reasons),
        first_q=[float(value) for value in q[0]],
        first_tip=[float(value) for value in tip[0]],
        mean_q_variation=float(np.mean(q_variation)),
        max_q_variation=float(np.max(q_variation)),
        mean_tip_variation=float(np.mean(tip_variation)),
        max_tip_variation=float(np.max(tip_variation)),
        sample_count=len(samples),
        consecutive_stable_samples=len(samples) if stable else 0,
        duration_s=float(duration),
    )


def empty_stability(reason: str) -> StabilityStats:
    return StabilityStats(False, reason, [], [], math.nan, math.nan, math.nan, math.nan, 0, 0, 0.0)


def parse_started_run_id(message: str) -> str | None:
    match = RUN_STARTED_RE.search(message)
    return None if match is None else match.group("run_id")


def parse_completed_result(message: str) -> tuple[str, Path] | None:
    match = RUN_COMPLETED_RE.search(message)
    if match is None:
        return None
    return match.group("run_id"), Path(match.group("path").strip())


def resolve_result_dir(
    *,
    response_message: str,
    output_root: Path,
    experiment_group: str,
    run_id: str,
    orchestration_id: str,
    run_role: str,
) -> Path:
    group_name = validate_experiment_group(experiment_group)
    root = output_root.expanduser().resolve()
    expected_parent = (root / group_name).resolve()
    if expected_parent == root or not path_is_relative_to(expected_parent, root):
        raise OrchestrationError(f"experiment_group resolves outside output_root: {experiment_group}")
    parsed = parse_completed_result(response_message)
    candidates: list[Path] = []
    if parsed is not None:
        parsed_run_id, parsed_path = parsed
        if parsed_run_id != run_id:
            raise OrchestrationError(f"StopExperiment returned run ID {parsed_run_id}; expected {run_id}")
        candidates.append(parsed_path)
    exact = expected_parent / run_id
    if exact.exists():
        candidates.append(exact)
    unique = []
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if resolved not in unique:
            unique.append(resolved)
    if len(unique) != 1:
        raise OrchestrationError(f"ambiguous result directory resolution for {run_id}: {unique}")
    result = unique[0]
    if (
        result.name != run_id
        or result.parent != expected_parent
        or not path_is_relative_to(result, expected_parent)
    ):
        raise OrchestrationError(f"result directory is outside expected group: {result}")
    metadata = read_yaml(result / "metadata.yaml")
    if metadata.get("orchestration_id") != orchestration_id:
        raise OrchestrationError("result metadata orchestration_id mismatch")
    if metadata.get("run_role") != run_role:
        raise OrchestrationError("result metadata run_role mismatch")
    return result


def unexpected_command_publishers(publisher_counts: dict[str, int]) -> dict[str, int]:
    return {topic: count for topic, count in publisher_counts.items() if count > 0}


def audit_start_receive_time(audit: CommandAudit) -> float:
    return audit.started_receive_time


def candidate_after_recording(run_dir: Path, first: CommandEvent | None) -> bool:
    if first is None:
        return False
    metadata = read_yaml(run_dir / "metadata.yaml")
    start = float(metadata.get("recording_start_time_s", math.inf))
    return first.timestamp >= start


def target_vector(value: Any, label: str) -> np.ndarray:
    target = np.asarray(value, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise OrchestrationError(f"{label} must be a finite 3D target")
    return target


def target_vectors_equal(first: Any, second: Any, *, atol: float = TARGET_IDENTITY_ATOL) -> bool:
    return bool(np.allclose(target_vector(first, "first target"), target_vector(second, "second target"), atol=atol, rtol=0.0))


def reference_target_identity(*, expected_target: Any, observed_target: Any, observed_timestamp: float | None, atol: float) -> dict[str, Any]:
    expected = target_vector(expected_target, "expected target")
    if observed_target is None:
        return {
            "first_observed_reference_target": None,
            "reference_target_timestamp": None,
            "reference_matches_requested_target": False,
            "reference_target_difference": math.inf,
            "reference_target_identity_tolerance": float(atol),
            "reference_target_identity_reason": "missing reference target",
        }
    observed = target_vector(observed_target, "observed reference target")
    difference = float(np.linalg.norm(observed - expected))
    matches = bool(np.allclose(observed, expected, atol=float(atol), rtol=0.0))
    return {
        "first_observed_reference_target": [float(value) for value in observed],
        "reference_target_timestamp": None if observed_timestamp is None else float(observed_timestamp),
        "reference_matches_requested_target": matches,
        "reference_target_difference": difference,
        "reference_target_identity_tolerance": float(atol),
        "reference_target_identity_reason": "ok" if matches else "reference target differs from requested target",
    }


def validate_target_identity_metadata(metadata: dict[str, Any], *, expected_target: Any, label: str) -> None:
    requested = target_identity_value(metadata, "requested_target")
    executed = target_identity_value(metadata, "executed_target")
    if requested is None or executed is None:
        raise OrchestrationError(f"{label} target identity metadata is missing")
    if target_identity_value(metadata, "target_replaced") is not False:
        raise OrchestrationError(f"{label} target identity reports target_replaced=true")
    if target_identity_value(metadata, "target_identity_valid") is not True:
        raise OrchestrationError(f"{label} target identity is not valid")
    if not target_vectors_equal(requested, executed):
        raise OrchestrationError(f"{label} requested_target differs from executed_target")
    if not target_vectors_equal(requested, expected_target):
        raise OrchestrationError(f"{label} requested_target differs from the orchestrator target")
    reference_matches = target_identity_value(metadata, "reference_matches_requested_target")
    if reference_matches is not None and reference_matches is not True:
        raise OrchestrationError(f"{label} published reference target did not match requested_target")


def target_identity_value(metadata: dict[str, Any], key: str) -> Any:
    if key in metadata:
        return metadata[key]
    override = metadata.get("metadata_override", {})
    if isinstance(override, dict) and key in override:
        return override[key]
    runtime = metadata.get("orchestration_runtime", {})
    if isinstance(runtime, dict) and key in runtime:
        return runtime[key]
    return None


def strict_json_file(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    for token in ("NaN", "Infinity", "-Infinity"):
        if token in text:
            raise OrchestrationError(f"non-strict JSON token {token} found in {path}")

    def reject_constant(value):
        raise OrchestrationError(f"non-strict JSON constant {value} found in {path}")

    return json.loads(text, parse_constant=reject_constant)


def read_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise OrchestrationError(f"YAML file must contain a map: {path}")
    return data


def write_orchestration_failure(output_root: Path, experiment_group: str, orchestration_id: str, data: dict[str, Any]) -> Path:
    group_name = validate_experiment_group(experiment_group)
    root = output_root.expanduser().resolve()
    group_dir = (root / group_name).resolve()
    if group_dir == root or not path_is_relative_to(group_dir, root):
        raise OrchestrationError(f"experiment_group resolves outside output_root: {experiment_group}")
    group_dir.mkdir(parents=True, exist_ok=True)
    prefix = orchestration_id or f"m5d1_failure_{uuid.uuid4().hex[:8]}"
    path = group_dir / f"{prefix}_orchestration_failure.json"
    write_json(path, data)
    return path


def validate_experiment_group(value: Any) -> str:
    if not isinstance(value, str):
        raise OrchestrationError("invalid experiment_group: value must be a string")
    if "\x00" in value:
        raise OrchestrationError("invalid experiment_group: NUL characters are not allowed")
    if not value or value.strip() == "":
        raise OrchestrationError("invalid experiment_group: value must be non-empty")
    if value != value.strip():
        raise OrchestrationError("invalid experiment_group: leading or trailing whitespace is not allowed")
    if len(value) > MAX_EXPERIMENT_GROUP_LENGTH:
        raise OrchestrationError(
            f"invalid experiment_group: value must be at most {MAX_EXPERIMENT_GROUP_LENGTH} characters"
        )
    if value in {".", ".."}:
        raise OrchestrationError("invalid experiment_group: traversal components are not allowed")
    if "/" in value or "\\" in value:
        raise OrchestrationError("invalid experiment_group: path separators are not allowed")
    path = Path(value)
    if path.is_absolute():
        raise OrchestrationError("invalid experiment_group: absolute paths are not allowed")
    if len(path.parts) != 1 or path.parts[0] != value:
        raise OrchestrationError("invalid experiment_group: value must be one path component")
    if any(part in {"", ".", ".."} for part in path.parts):
        raise OrchestrationError("invalid experiment_group: traversal components are not allowed")
    if not EXPERIMENT_GROUP_RE.fullmatch(value):
        raise OrchestrationError(
            "invalid experiment_group: expected ASCII format ^[A-Za-z0-9][A-Za-z0-9._-]*$"
        )
    return value


def path_is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def build_shared_environment_hash(
    config: dict[str, Any],
    *,
    task: str = "trajectory",
    trajectory: str,
    duration: float,
    reference_lead_time: float,
    curved_scenario: CurvedLumenScenario | None = None,
) -> str:
    reference = config["reference"]
    robot = config["robot"]
    if is_curved_lumen_task(task):
        if curved_scenario is None:
            raise OrchestrationError("curved_lumen_navigation shared environment hash requires a resolved scenario")
        return stable_hash(
            {
                "task": task,
                "model": config["model"],
                "simulation": config["simulation"],
                "reference": {
                    "mode": "fixed_target",
                    "sample_period": reference["sample_period"],
                    "frame_id": reference["frame_id"],
                    "loop": reference["loop"],
                    "completion_behavior": reference["completion_behavior"],
                    "duration": reference["duration"],
                    "reference_lead_time": reference_lead_time,
                },
                "curved_lumen": config.get("curved_lumen", {}),
                "goal": config.get("goal", {}),
                "scenario": curved_scenario_hash_payload(curved_scenario),
                "frames": robot["frames"],
                "software_mode": "simulation",
                "evaluation_window_duration": duration,
            }
        )
    cylinder = config.get("cylindrical_lumen", {}) if task == "cylinder_navigation" else None
    goal = config.get("goal", {}) if task == "cylinder_navigation" else None
    return stable_hash(
        {
            "task": task,
            "model": config["model"],
            "simulation": config["simulation"],
            "reference": {
                "mode": "fixed_target" if task == "cylinder_navigation" else "trajectory",
                "trajectory_type": trajectory,
                "trajectory_parameters": reference[trajectory],
                "sample_period": reference["sample_period"],
                "frame_id": reference["frame_id"],
                "loop": reference["loop"],
                "completion_behavior": reference["completion_behavior"],
                "duration": reference["duration"],
                "reference_lead_time": reference_lead_time,
            },
            "cylindrical_lumen": cylinder,
            "goal": goal,
            "frames": robot["frames"],
            "software_mode": "simulation",
            "evaluation_window_duration": duration,
        }
    )


def curved_scenario_hash_payload(scenario: CurvedLumenScenario) -> dict[str, Any]:
    return {
        "policy_version": scenario.policy_version,
        "scenario_id": scenario.scenario_id,
        "curved_lumen_type": scenario.curved_lumen_type,
        "geometry_frame": scenario.geometry_frame,
        "geometry_fingerprint": scenario.geometry_fingerprint,
        "scenario_fingerprint": scenario.scenario_fingerprint,
        "centerline_fraction": float(scenario.centerline_fraction),
        "derived_target": [float(value) for value in scenario.derived_target],
        "requested_target": [float(value) for value in scenario.requested_target],
        "validated_target": [float(value) for value in scenario.validated_target],
        "override_used": bool(scenario.override_used),
    }


def build_controller_configuration_hash(config: dict[str, Any], controller_label: str) -> str:
    return stable_hash({"controller_label": controller_label, "mppi": config["mppi"]})


def expected_first_reference_point(config: dict[str, Any], trajectory_type: str) -> list[float]:
    from ctr_mppi_controller.nodes.reference_manager_node import build_reference_trajectory, reference_settings_from_config

    settings = reference_settings_from_config(config, mode_override="trajectory", type_override=trajectory_type)
    trajectory = build_reference_trajectory(config, settings=settings)
    return [float(value) for value in trajectory.points[0]]


def build_run_id(orchestration_id: str, role: str, controller_label: str) -> str:
    safe = "".join(char if char.isalnum() or char in {"-", "_"} else "_" for char in controller_label)
    return f"{orchestration_id}_{role}_{safe}_{uuid.uuid4().hex[:8]}"


def build_base_simulation_command(
    *,
    experiment_group: str,
    controller_label: str,
    baseline_dir: Path | None,
    output_root: Path | None = None,
    task: str = "trajectory",
    target_position: list[float] | None = None,
    curved_lumen_type: str = "",
    mppi_profile: str = "",
    random_seed: int | None = None,
    run_role: str = "",
) -> list[str]:
    command = [
        "ros2",
        "launch",
        "ctr_bringup",
        "simulation.launch.py",
        "runtime_mode:=simulation",
        "start_evaluation:=true",
        "start_mppi_controller:=false",
        "start_reference_manager:=false",
        "start_manual_command_publisher:=false",
        "mppi_publish_safe_for_simulation:=false",
        f"evaluation_experiment_group:={experiment_group}",
        f"evaluation_controller_label:={controller_label}",
    ]
    if output_root is not None:
        command.append(f"evaluation_output_root:={output_root}")
    if task == TASK_CYLINDER_NAVIGATION:
        target = target_position or []
        if len(target) != 3:
            raise OrchestrationError("cylinder_navigation launch requires a 3D target position")
        command.extend(
            [
                "enable_cylindrical_lumen:=true",
                f"cylinder_target_x:={float(target[0]):.9f}",
                f"cylinder_target_y:={float(target[1]):.9f}",
                f"cylinder_target_z:={float(target[2]):.9f}",
            ]
        )
        if mppi_profile:
            command.append(f"cylinder_profile:={mppi_profile}")
        if random_seed is not None:
            command.append(f"mppi_random_seed:={int(random_seed)}")
    elif is_curved_lumen_task(task):
        lumen_type = curved_lumen_type or DEFAULT_CURVED_LUMEN_TYPE
        if lumen_type not in CURVED_LUMEN_TYPES:
            raise OrchestrationError(f"curved_lumen_navigation launch requires a supported curved lumen type: {lumen_type}")
        target = target_position or []
        if len(target) != 3:
            raise OrchestrationError("curved_lumen_navigation launch requires a 3D target position")
        command.extend(
            [
                "enable_cylindrical_lumen:=false",
                "enable_curved_lumen:=true",
                f"curved_lumen_type:={lumen_type}",
                "reference_mode:=fixed_target",
                f"cylinder_target_x:={float(target[0]):.9f}",
                f"cylinder_target_y:={float(target[1]):.9f}",
                f"cylinder_target_z:={float(target[2]):.9f}",
            ]
        )
        if mppi_profile:
            command.append(f"cylinder_profile:={mppi_profile}")
        if random_seed is not None:
            command.append(f"mppi_random_seed:={int(random_seed)}")
    if run_role:
        command.append(f"run_role:={run_role}")
    if baseline_dir is not None:
        command.append(f"evaluation_baseline_result_dir:={baseline_dir}")
    return command


def model_reachability_sanity(
    *,
    model: ApproximateCTRModel,
    config: dict[str, Any],
    target: Any,
    tolerance: float,
) -> dict[str, Any]:
    """Cheap deterministic reachability sanity check for the approximate model."""

    target_array = np.asarray(target, dtype=float)
    if target_array.shape != (3,) or not np.all(np.isfinite(target_array)):
        raise OrchestrationError("target must be finite with shape (3,)")
    limits = config["robot"]["limits"]
    insertion_min = np.asarray(limits["insertion_min"], dtype=float)
    insertion_max = np.asarray(limits["insertion_max"], dtype=float)
    rotation_min = np.asarray(limits["rotation_min"], dtype=float)
    rotation_max = np.asarray(limits["rotation_max"], dtype=float)
    if insertion_min.shape != (3,) or insertion_max.shape != (3,) or rotation_min.shape != (3,) or rotation_max.shape != (3,):
        raise OrchestrationError("joint limits must contain 3 insertion and 3 rotation bounds")

    insertion_values = [
        insertion_min,
        insertion_max,
        0.5 * (insertion_min + insertion_max),
        np.array([insertion_max[0], insertion_min[1], insertion_max[2]], dtype=float),
        np.array([insertion_min[0], insertion_max[1], insertion_max[2]], dtype=float),
    ]
    rotation_values = [
        np.zeros(3, dtype=float),
        rotation_min,
        rotation_max,
        0.5 * (rotation_min + rotation_max),
    ]
    goal_config = config.get("goal", {})
    random_count = int(goal_config.get("reachability_samples", 0) or 0)
    random_seed = int(goal_config.get("reachability_seed", 0) or 0)
    if random_count > 0:
        rng = np.random.default_rng(random_seed)
        random_insertions = rng.uniform(insertion_min, insertion_max, size=(random_count, 3))
        random_rotations = rng.uniform(rotation_min, rotation_max, size=(random_count, 3))
    else:
        random_insertions = np.empty((0, 3), dtype=float)
        random_rotations = np.empty((0, 3), dtype=float)

    best_error = math.inf
    best_q: list[float] = []
    best_tip: list[float] = []
    evaluated = 0
    for insertion in insertion_values:
        for rotation in rotation_values:
            q = np.concatenate([insertion, rotation])
            try:
                tip = model.forward_kinematics(q).tip_position
            except Exception:
                continue
            if not np.all(np.isfinite(tip)):
                continue
            evaluated += 1
            error = float(np.linalg.norm(tip - target_array))
            if error < best_error:
                best_error = error
                best_q = [float(value) for value in q]
                best_tip = [float(value) for value in tip]
    for insertion, rotation in zip(random_insertions, random_rotations):
        q = np.concatenate([insertion, rotation])
        try:
            tip = model.forward_kinematics(q).tip_position
        except Exception:
            continue
        if not np.all(np.isfinite(tip)):
            continue
        evaluated += 1
        error = float(np.linalg.norm(tip - target_array))
        if error < best_error:
            best_error = error
            best_q = [float(value) for value in q]
            best_tip = [float(value) for value in tip]
    return {
        "reachable": bool(best_error <= float(tolerance)),
        "best_error": float(best_error),
        "best_q": best_q,
        "best_tip": best_tip,
        "tolerance": float(tolerance),
        "evaluated_candidates": evaluated,
        "random_sample_count": random_count,
        "random_seed": random_seed,
    }


def fresh_ros_domain_id() -> int:
    return 100 + (uuid.uuid4().int % 100)


def run_environment(domain_id: int) -> dict[str, str]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    log_dir = Path(os.environ.get("ROS_LOG_DIR", f"/tmp/ctr_mppi_ros_log_{domain_id}"))
    log_dir.mkdir(parents=True, exist_ok=True)
    env["ROS_LOG_DIR"] = str(log_dir)
    return env


def simulator_command_timeout() -> float:
    return 0.25


def stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def command_event_from_message(topic: str, msg: Any, *, receive_time: float, receive_timestamp: float) -> CommandEvent:
    timestamp = stamp_seconds(msg.header.stamp)
    timestamp_type = "command_message_timestamp" if timestamp > 0.0 and math.isfinite(timestamp) else "command_receive_timestamp"
    if timestamp_type == "command_receive_timestamp":
        timestamp = float(receive_timestamp)
    try:
        command = [float(value) for value in msg.q_dot]
    except (TypeError, ValueError):
        command = [math.nan] * 6
    return CommandEvent(
        topic=topic,
        timestamp=timestamp,
        timestamp_type=timestamp_type,
        receive_time=float(receive_time),
        command=command,
    )


def process_name_running(name: str) -> bool:
    for proc in list_processes():
        if name in proc.get("args", ""):
            return True
    return False


def process_identity(pid: int) -> ProcessIdentity:
    return ProcessIdentity(
        pid=int(pid),
        pgid=os.getpgid(pid),
        start_time_ticks=process_start_time_ticks(pid),
        command_line=process_command_line(pid),
    )


def process_matches(identity: ProcessIdentity) -> bool:
    try:
        return (
            os.getpgid(identity.pid) == identity.pgid
            and process_start_time_ticks(identity.pid) == identity.start_time_ticks
            and process_command_line(identity.pid) == identity.command_line
        )
    except (OSError, ProcessLookupError, FileNotFoundError):
        return False


def process_start_time_ticks(pid: int) -> int:
    text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    after_comm = text.rsplit(")", 1)[1].strip()
    fields = after_comm.split()
    return int(fields[19])


def process_command_line(pid: int) -> str:
    data = Path(f"/proc/{pid}/cmdline").read_bytes()
    return data.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()


def list_process_group(pgid: int) -> list[dict[str, Any]]:
    return [proc for proc in list_processes() if proc.get("pgid") == pgid]


def list_processes() -> list[dict[str, Any]]:
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid=,pgid=,stat=,args="],
            check=True,
            text=True,
            capture_output=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    processes = []
    for line in result.stdout.splitlines():
        parts = line.strip().split(maxsplit=3)
        if len(parts) < 4:
            continue
        try:
            pid = int(parts[0])
            pgid = int(parts[1])
        except ValueError:
            continue
        processes.append({"pid": pid, "pgid": pgid, "stat": parts[2], "args": parts[3]})
    return processes


if __name__ == "__main__":
    raise SystemExit(main())
