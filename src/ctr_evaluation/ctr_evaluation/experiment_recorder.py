"""Experiment lifecycle, data recording, and result-file writing."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import uuid
from typing import Any

import numpy as np
import yaml

from ctr_evaluation.metrics import (
    AcceptanceResults,
    ControlMetrics,
    DataQualityMetrics,
    EvaluationSummary,
    EvaluationThresholds,
    NumericalSafetyMetrics,
    TrackingMetrics,
    TimingMetrics,
    aggregate_trial_summaries,
    compute_acceptance,
    compute_control_metrics,
    compute_timing_metrics,
    compute_tracking_metrics,
    dataclass_to_plain,
    sanitize_for_json,
    stable_hash,
)
from ctr_evaluation.time_alignment import (
    AlignmentConfig,
    AlignmentResult,
    TimedCommand,
    TimedReference,
    TimedSolve,
    TimedState,
    align_samples,
    aligned_arrays,
    command_sample,
    reference_sample,
    solve_sample,
    state_sample,
)


STATE_IDLE = "IDLE"
STATE_RECORDING = "RECORDING"
STATE_FINALIZING = "FINALIZING"
STATE_COMPLETED = "COMPLETED"


@dataclass(frozen=True)
class EvaluationRecorderConfig:
    enabled: bool
    output_root: Path
    experiment_group: str
    controller_label: str
    baseline_label: str
    baseline_result_dir: str
    configured_duration: float
    auto_finalize_on_shutdown: bool
    max_samples_per_topic: int
    alignment: AlignmentConfig
    thresholds: EvaluationThresholds
    duration_compatibility_tolerance: float
    initial_state_compatibility_tolerance: float
    plot_generation: bool
    report_generation: bool
    physical_validation: bool
    hardware_validation: bool
    software_mode: str
    trajectory_type: str
    trajectory_parameters: dict[str, Any]
    frame_id: str
    reference_sample_period: float
    mppi_parameters: dict[str, Any]
    model_parameters: dict[str, Any]
    random_seed: int | None
    command_limits: np.ndarray
    state_min: np.ndarray
    state_max: np.ndarray

    @classmethod
    def from_project_config(
        cls,
        project_config: dict[str, Any],
        *,
        overrides: dict[str, Any] | None = None,
    ) -> "EvaluationRecorderConfig":
        overrides = overrides or {}
        evaluation = project_config.get("evaluation")
        if not isinstance(evaluation, dict):
            raise ValueError("project configuration must contain an `evaluation` section")
        mppi = project_config["mppi"]
        reference = project_config["reference"]
        robot = project_config["robot"]
        trajectory_type = str(reference["trajectory_type"])
        trajectory_parameters = reference.get(trajectory_type, {})
        output_root = Path(str(_override(overrides, "output_root", evaluation["output_root"])))
        configured_duration = _positive_number(evaluation["configured_duration"], "evaluation.configured_duration")
        control_frequency = _positive_number(mppi["control_frequency"], "mppi.control_frequency")
        limits = robot["limits"]
        command_limits = np.asarray(
            list(limits["insertion_velocity_max"]) + list(limits["rotation_velocity_max"]),
            dtype=float,
        )
        state_min = np.asarray(list(limits["insertion_min"]) + list(limits["rotation_min"]), dtype=float)
        state_max = np.asarray(list(limits["insertion_max"]) + list(limits["rotation_max"]), dtype=float)
        return cls(
            enabled=_bool(evaluation["enabled"], "evaluation.enabled"),
            output_root=output_root,
            experiment_group=str(_override(overrides, "experiment_group", evaluation["experiment_group"])),
            controller_label=str(_override(overrides, "controller_label", evaluation["controller_label"])),
            baseline_label=str(evaluation["baseline_label"]),
            baseline_result_dir=str(_override(overrides, "baseline_result_dir", evaluation["baseline_result_dir"])),
            configured_duration=configured_duration,
            auto_finalize_on_shutdown=_bool(
                evaluation["auto_finalize_on_shutdown"],
                "evaluation.auto_finalize_on_shutdown",
            ),
            max_samples_per_topic=_positive_int(evaluation["max_samples_per_topic"], "evaluation.max_samples_per_topic"),
            alignment=AlignmentConfig(
                maximum_reference_gap=_positive_number(
                    evaluation["maximum_reference_alignment_gap"],
                    "evaluation.maximum_reference_alignment_gap",
                ),
                maximum_command_gap=_positive_number(
                    evaluation["maximum_command_alignment_gap"],
                    "evaluation.maximum_command_alignment_gap",
                ),
                maximum_solve_gap=_positive_number(
                    evaluation["maximum_solve_alignment_gap"],
                    "evaluation.maximum_solve_alignment_gap",
                ),
                require_command=_bool(
                    evaluation["require_command_for_alignment"],
                    "evaluation.require_command_for_alignment",
                ),
            ),
            thresholds=EvaluationThresholds(
                configured_duration=configured_duration,
                configured_control_frequency=control_frequency,
                tracking_tolerance=_positive_number(evaluation["tracking_tolerance"], "evaluation.tracking_tolerance"),
                transient_stable_cycles=_positive_int(
                    evaluation["transient_stable_cycles"],
                    "evaluation.transient_stable_cycles",
                ),
                steady_state_window=_nonnegative_number(
                    evaluation["steady_state_window"],
                    "evaluation.steady_state_window",
                ),
                steady_state_fraction=_fraction(evaluation["steady_state_fraction"], "evaluation.steady_state_fraction"),
                minimum_valid_sample_count=_positive_int(
                    evaluation["minimum_valid_sample_count"],
                    "evaluation.minimum_valid_sample_count",
                ),
                maximum_invalid_sample_percentage=_percentage(
                    evaluation["maximum_invalid_sample_percentage"],
                    "evaluation.maximum_invalid_sample_percentage",
                ),
                maximum_saturation_percentage=_percentage(
                    evaluation["maximum_saturation_percentage"],
                    "evaluation.maximum_saturation_percentage",
                ),
                maximum_deadline_overrun_percentage=_percentage(
                    evaluation["maximum_deadline_overrun_percentage"],
                    "evaluation.maximum_deadline_overrun_percentage",
                ),
                required_minimum_baseline_improvement=float(evaluation["required_minimum_baseline_improvement"]),
                near_zero_baseline_epsilon=_positive_number(
                    evaluation["near_zero_baseline_epsilon"],
                    "evaluation.near_zero_baseline_epsilon",
                ),
            ),
            duration_compatibility_tolerance=_nonnegative_number(
                evaluation["duration_compatibility_tolerance"],
                "evaluation.duration_compatibility_tolerance",
            ),
            initial_state_compatibility_tolerance=_nonnegative_number(
                evaluation["initial_state_compatibility_tolerance"],
                "evaluation.initial_state_compatibility_tolerance",
            ),
            plot_generation=_bool(evaluation["plot_generation"], "evaluation.plot_generation"),
            report_generation=_bool(evaluation["report_generation"], "evaluation.report_generation"),
            physical_validation=_bool(evaluation["physical_validation"], "evaluation.physical_validation"),
            hardware_validation=_bool(evaluation["hardware_validation"], "evaluation.hardware_validation"),
            software_mode=str(project_config.get("runtime", {}).get("mode", "software_simulation")),
            trajectory_type=trajectory_type,
            trajectory_parameters=dict(trajectory_parameters),
            frame_id=str(reference["frame_id"]),
            reference_sample_period=_positive_number(reference["sample_period"], "reference.sample_period"),
            mppi_parameters=dict(mppi),
            model_parameters=dict(project_config["model"]),
            random_seed=_optional_int(mppi.get("random_seed")),
            command_limits=command_limits,
            state_min=state_min,
            state_max=state_max,
        )


@dataclass(frozen=True)
class FinalizationResult:
    run_id: str
    run_dir: Path
    summary: dict[str, Any]
    metadata: dict[str, Any]
    comparison: dict[str, Any] | None
    output_files: list[Path]


class ExperimentRecorder:
    """Record experiment samples and finalize them into an evaluation run."""

    def __init__(self, *, config: EvaluationRecorderConfig, project_config: dict[str, Any]):
        self.config = config
        self.project_config = project_config
        self.lifecycle_state = STATE_IDLE
        self.run_id: str | None = None
        self.experiment_name = ""
        self.start_wall_time: datetime | None = None
        self.stop_wall_time: datetime | None = None
        self.start_monotonic_time: float | None = None
        self.stop_monotonic_time: float | None = None
        self.metadata_override: dict[str, Any] = {}
        self.finalization_result: FinalizationResult | None = None
        self.reset_buffers()

    def reset_buffers(self) -> None:
        self.states: list[TimedState] = []
        self.tip_records: list[dict[str, float]] = []
        self.references: list[TimedReference] = []
        self.raw_commands: list[TimedCommand] = []
        self.safe_commands: list[TimedCommand] = []
        self.solves: list[TimedSolve] = []
        self.topic_counts: dict[str, int] = {}
        self.invalid_counts: dict[str, int] = {
            "state": 0,
            "reference": 0,
            "command": 0,
            "solve": 0,
            "dimension": 0,
            "command_limit": 0,
            "state_limit": 0,
        }
        self.horizon_records: list[dict[str, Any]] = []
        self.path_records: list[dict[str, Any]] = []
        self.initial_state_q: list[float] | None = None

    def start(
        self,
        *,
        experiment_name: str,
        metadata: dict[str, Any] | None = None,
        monotonic_time: float = 0.0,
    ) -> str:
        if self.lifecycle_state == STATE_RECORDING:
            raise RuntimeError("an experiment is already recording")
        if self.lifecycle_state == STATE_FINALIZING:
            raise RuntimeError("an experiment is finalizing")
        if self.lifecycle_state not in {STATE_IDLE, STATE_COMPLETED}:
            raise RuntimeError(f"cannot start experiment from lifecycle state {self.lifecycle_state}")
        self.reset_buffers()
        self.finalization_result = None
        safe_name = sanitize_name(experiment_name or self.config.controller_label)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.run_id = f"{timestamp}_{safe_name}_{uuid.uuid4().hex[:8]}"
        self.experiment_name = experiment_name or safe_name
        self.start_wall_time = datetime.now(timezone.utc)
        self.stop_wall_time = None
        self.start_monotonic_time = float(monotonic_time)
        self.stop_monotonic_time = None
        self.metadata_override = dict(metadata or {})
        self.lifecycle_state = STATE_RECORDING
        return self.run_id

    def stop(self, *, monotonic_time: float, interrupted: bool = False) -> FinalizationResult:
        if self.lifecycle_state == STATE_COMPLETED:
            if self.finalization_result is not None:
                return self.finalization_result
            raise RuntimeError("experiment is completed without a finalization result")
        if self.lifecycle_state == STATE_FINALIZING:
            raise RuntimeError("experiment finalization is already in progress")
        if self.lifecycle_state != STATE_RECORDING:
            raise RuntimeError("no experiment is recording")
        self.stop_monotonic_time = float(monotonic_time)
        self.stop_wall_time = datetime.now(timezone.utc)
        return self.finalize(interrupted=interrupted)

    def finalize(self, *, interrupted: bool = False) -> FinalizationResult:
        if self.lifecycle_state == STATE_COMPLETED:
            if self.finalization_result is not None:
                return self.finalization_result
            raise RuntimeError("experiment is completed without a finalization result")
        if self.lifecycle_state == STATE_FINALIZING:
            raise RuntimeError("experiment finalization is already in progress")
        if self.lifecycle_state != STATE_RECORDING:
            raise RuntimeError("cannot finalize unless an experiment is recording")
        if self.run_id is None:
            raise RuntimeError("cannot finalize before start")
        self.lifecycle_state = STATE_FINALIZING
        group_dir = self.config.output_root / self.config.experiment_group
        partial_dir = group_dir / f"{self.run_id}.partial"
        final_dir = group_dir / self.run_id
        if partial_dir.exists():
            raise FileExistsError(f"partial result directory already exists: {partial_dir}")
        if final_dir.exists():
            raise FileExistsError(f"final result directory already exists: {final_dir}")
        partial_dir.mkdir(parents=True)
        try:
            selected_commands = self.safe_commands if self.safe_commands else self.raw_commands
            alignment = align_samples(
                states=self.states,
                references=self.references,
                commands=selected_commands,
                solves=self.solves,
                config=self.config.alignment,
            )
            metadata = self._metadata(interrupted=interrupted)
            metadata["initial_state_q"] = self.initial_state_q
            self._write_raw_files(partial_dir)
            summary = self._summary(alignment=alignment, metadata=metadata)
            write_yaml(partial_dir / "metadata.yaml", metadata)
            write_json(partial_dir / "summary.json", summary)
            write_aligned_csv(partial_dir / "aligned_samples.csv", alignment)

            comparison = None
            if self.config.report_generation or self.config.plot_generation:
                from ctr_evaluation.report_generator import generate_plots, generate_report

                plot_paths: list[Path] = []
                if self.config.plot_generation:
                    plot_paths = generate_plots(partial_dir, alignment.samples)
                if self.config.report_generation:
                    generate_report(
                        run_dir=partial_dir,
                        metadata=metadata,
                        summary=summary,
                        comparison=None,
                        plot_paths=plot_paths,
                    )

            if self.config.baseline_result_dir:
                from ctr_evaluation.compare_results import compare_result_dirs

                comparison = compare_result_dirs(
                    candidate_dir=partial_dir,
                    baseline_dir=Path(self.config.baseline_result_dir),
                    duration_tolerance=self.config.duration_compatibility_tolerance,
                    initial_state_tolerance=self.config.initial_state_compatibility_tolerance,
                    near_zero_epsilon=self.config.thresholds.near_zero_baseline_epsilon,
                )
                summary = self._apply_baseline_acceptance(summary, comparison)
                write_json(partial_dir / "summary.json", summary)
                if self.config.report_generation:
                    from ctr_evaluation.report_generator import generate_report

                    generate_report(
                        run_dir=partial_dir,
                        metadata=metadata,
                        summary=summary,
                        comparison=comparison,
                        plot_paths=[path for path in partial_dir.glob("*.png")],
                    )

            partial_dir.replace(final_dir)
            self._write_aggregate(group_dir)
            self.lifecycle_state = STATE_COMPLETED
            self.finalization_result = FinalizationResult(
                run_id=self.run_id,
                run_dir=final_dir,
                summary=summary,
                metadata=metadata,
                comparison=comparison,
                output_files=sorted(path for path in final_dir.iterdir() if path.is_file()),
            )
            return self.finalization_result
        except Exception as exc:
            self._write_finalization_error(partial_dir, exc)
            raise

    def record_state(self, *, timestamp: float, q: Any, q_dot: Any, tip_position: Any) -> None:
        if not self._accept_sample("/ctr/state"):
            return
        try:
            sample = state_sample(timestamp, q, q_dot, tip_position)
            self.states.append(sample)
            if self.initial_state_q is None:
                self.initial_state_q = [float(value) for value in sample.q]
            if np.any(sample.q < self.config.state_min) or np.any(sample.q > self.config.state_max):
                self.invalid_counts["state_limit"] += 1
        except ValueError:
            self.invalid_counts["state"] += 1

    def record_tip(self, *, timestamp: float, position: Any) -> None:
        if not self._accept_sample("/ctr/tip"):
            return
        try:
            point = np.asarray(position, dtype=float)
            if point.shape != (3,) or not np.all(np.isfinite(point)):
                raise ValueError("tip position must be finite with shape (3,)")
            self.tip_records.append(
                {
                    "timestamp": float(timestamp),
                    "x": float(point[0]),
                    "y": float(point[1]),
                    "z": float(point[2]),
                }
            )
        except (TypeError, ValueError):
            self.invalid_counts["state"] += 1

    def record_reference(self, *, timestamp: float, position: Any, progress: float | None = None) -> None:
        if not self._accept_sample("/ctr/reference/tip"):
            return
        try:
            self.references.append(reference_sample(timestamp, position, progress))
        except ValueError:
            self.invalid_counts["reference"] += 1

    def record_command(self, *, timestamp: float, command: Any, saturated: bool, source: str) -> None:
        topic = "/ctr/safe_command" if source == "safe_command" else "/ctr/mppi_command"
        if not self._accept_sample(topic):
            return
        try:
            sample = command_sample(timestamp, command, saturated=saturated, source=source)
            if np.any(np.abs(sample.command) > self.config.command_limits + 1.0e-12):
                self.invalid_counts["command_limit"] += 1
            if source == "safe_command":
                self.safe_commands.append(sample)
            else:
                self.raw_commands.append(sample)
        except ValueError:
            self.invalid_counts["command"] += 1

    def record_solve_timing(self, *, timestamp: float, solve_time: Any, saturated: bool) -> None:
        if not self._accept_sample("/ctr/controller/metrics"):
            return
        try:
            self.solves.append(solve_sample(timestamp, solve_time, saturated=saturated))
        except ValueError:
            self.invalid_counts["solve"] += 1

    def record_horizon(self, *, timestamp: float, count: int, first_point: Any, final_point: Any) -> None:
        if not self._accept_sample("/ctr/reference/horizon"):
            return
        try:
            first = np.asarray(first_point, dtype=float)
            final = np.asarray(final_point, dtype=float)
            if first.shape != (3,) or final.shape != (3,) or not np.all(np.isfinite(first)) or not np.all(np.isfinite(final)):
                raise ValueError("malformed horizon points")
            self.horizon_records.append(
                {
                    "timestamp": float(timestamp),
                    "count": int(count),
                    "first_x": float(first[0]),
                    "first_y": float(first[1]),
                    "first_z": float(first[2]),
                    "final_x": float(final[0]),
                    "final_y": float(final[1]),
                    "final_z": float(final[2]),
                }
            )
        except (TypeError, ValueError):
            self.invalid_counts["dimension"] += 1

    def record_path(self, *, timestamp: float, count: int) -> None:
        if not self._accept_sample("/ctr/reference/path"):
            return
        self.path_records.append({"timestamp": float(timestamp), "count": int(count)})

    def record_topic(self, topic: str) -> None:
        self.topic_counts[topic] = self.topic_counts.get(topic, 0) + 1

    def _accept_sample(self, topic: str) -> bool:
        if self.lifecycle_state != STATE_RECORDING:
            return False
        self.record_topic(topic)
        return self.topic_counts[topic] <= self.config.max_samples_per_topic

    def _metadata(self, *, interrupted: bool) -> dict[str, Any]:
        actual_duration = 0.0
        if self.start_monotonic_time is not None and self.stop_monotonic_time is not None:
            actual_duration = max(0.0, self.stop_monotonic_time - self.start_monotonic_time)
        git = git_metadata(Path.cwd())
        return {
            "run_id": self.run_id,
            "experiment_group": self.config.experiment_group,
            "experiment_name": self.experiment_name,
            "controller_label": self.config.controller_label,
            "baseline_label": self.config.baseline_label,
            "started_at": self.start_wall_time.isoformat() if self.start_wall_time else "",
            "stopped_at": self.stop_wall_time.isoformat() if self.stop_wall_time else "",
            "configured_duration": self.config.configured_duration,
            "actual_duration": actual_duration,
            "interrupted": bool(interrupted),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "git": git,
            "hostname": socket.gethostname(),
            "software_only": self.config.software_mode != "hardware",
            "configuration": {
                "trajectory_type": self.config.trajectory_type,
                "trajectory_parameters": self.config.trajectory_parameters,
                "trajectory_parameters_hash": stable_hash(self.config.trajectory_parameters),
                "frame_id": self.config.frame_id,
                "mppi_parameters": self.config.mppi_parameters,
                "mppi_parameters_hash": stable_hash(self.config.mppi_parameters),
                "model_parameters": self.config.model_parameters,
                "model_configuration_hash": stable_hash(self.config.model_parameters),
                "random_seed": self.config.random_seed,
                "configured_duration": self.config.configured_duration,
                "configured_control_period": self.config.thresholds.control_period,
                "reference_sample_period": self.config.reference_sample_period,
                "software_mode": self.config.software_mode,
            },
            "metadata_override": self.metadata_override,
            "timestamp_limitations": [
                "Evaluation uses state timestamps and interpolated immediate references.",
                "Command stamps are publication times, not guaranteed command-application times.",
                "Controller metrics do not expose solve input state/reference timestamps.",
                "Horizon Path messages carry one header stamp, not per-horizon-point timestamps.",
            ],
            "topics": self._topic_status(),
        }

    def _summary(self, *, alignment: AlignmentResult, metadata: dict[str, Any]) -> dict[str, Any]:
        arrays = aligned_arrays(alignment.samples)
        timestamps = arrays["timestamps"]
        progress = arrays["reference_progress"]
        progress_arg = None if progress.size == 0 or np.all(np.isnan(progress)) else progress
        if timestamps.size:
            valid_duration = float(max(0.0, timestamps[-1] - timestamps[0]))
        else:
            valid_duration = 0.0
        tracking = compute_tracking_metrics(
            times=timestamps,
            tip_positions=arrays["tip_positions"],
            reference_positions=arrays["reference_positions"],
            tolerance=self.config.thresholds.tracking_tolerance,
            stable_cycles=self.config.thresholds.transient_stable_cycles,
            steady_state_window=self.config.thresholds.steady_state_window,
            steady_state_fraction=self.config.thresholds.steady_state_fraction,
            path_progress=progress_arg,
        )
        control = compute_control_metrics(
            times=timestamps,
            commands=arrays["commands"],
            saturation_flags=arrays["saturation_flags"],
            missing_command_flags=arrays["missing_command_flags"],
        )
        solve_samples = sorted(self.solves, key=lambda sample: sample.timestamp)
        state_samples = sorted(self.states, key=lambda sample: sample.timestamp)
        reference_samples = sorted(self.references, key=lambda sample: sample.timestamp)
        command_samples = sorted(self.safe_commands or self.raw_commands, key=lambda sample: sample.timestamp)
        solve_times = np.asarray([sample.solve_time for sample in solve_samples], dtype=float)
        solve_stamps = np.asarray([sample.timestamp for sample in solve_samples], dtype=float)
        state_stamps = np.asarray([sample.timestamp for sample in state_samples], dtype=float)
        reference_stamps = np.asarray([sample.timestamp for sample in reference_samples], dtype=float)
        command_stamps = np.asarray([sample.timestamp for sample in command_samples], dtype=float)
        timing = compute_timing_metrics(
            solve_times=solve_times,
            solve_timestamps=solve_stamps,
            state_timestamps=state_stamps,
            reference_timestamps=reference_stamps,
            command_timestamps=command_stamps,
            configured_control_frequency=self.config.thresholds.configured_control_frequency,
            experiment_wall_duration=float(metadata["actual_duration"]),
            valid_aligned_evaluation_duration=valid_duration,
        )
        missing_topic_count = sum(1 for topic in required_topics() if self.topic_counts.get(topic, 0) == 0)
        numerical = NumericalSafetyMetrics(
            nonfinite_state_samples=self.invalid_counts["state"],
            nonfinite_reference_samples=self.invalid_counts["reference"],
            nonfinite_command_samples=self.invalid_counts["command"],
            malformed_dimension_count=self.invalid_counts["dimension"],
            command_limit_violation_count=self.invalid_counts["command_limit"],
            state_limit_violation_count=self.invalid_counts["state_limit"],
            saturation_count=control.saturation_count,
            missing_required_topic_count=missing_topic_count,
        )
        data_quality = DataQualityMetrics(
            raw_state_sample_count=alignment.diagnostics.raw_state_sample_count,
            raw_reference_sample_count=alignment.diagnostics.raw_reference_sample_count,
            raw_command_sample_count=alignment.diagnostics.raw_command_sample_count,
            valid_aligned_sample_count=alignment.diagnostics.valid_aligned_sample_count,
            rejected_aligned_sample_count=alignment.diagnostics.rejected_aligned_sample_count,
            invalid_nonfinite_sample_count=alignment.diagnostics.invalid_nonfinite_sample_count,
            mean_alignment_gap=alignment.diagnostics.mean_alignment_gap,
            maximum_alignment_gap=alignment.diagnostics.maximum_alignment_gap,
            reference_interpolation_count=alignment.diagnostics.reference_interpolation_count,
            nearest_reference_fallback_count=alignment.diagnostics.nearest_reference_fallback_count,
            missing_command_count=alignment.diagnostics.missing_command_count,
            missing_topic_count=missing_topic_count,
        )
        acceptance = compute_acceptance(
            tracking=tracking,
            control=control,
            timing=timing,
            numerical_safety=numerical,
            data_quality=data_quality,
            thresholds=self.config.thresholds,
            baseline_improvement_valid=not self.config.baseline_result_dir,
            physical_validation=self.config.physical_validation,
            hardware_validation=self.config.hardware_validation,
        )
        summary = EvaluationSummary(
            tracking=tracking,
            control=control,
            timing=timing,
            numerical_safety=numerical,
            data_quality=data_quality,
            acceptance=acceptance,
        ).to_dict()
        summary["alignment_rejection_reasons"] = alignment.diagnostics.rejection_reasons
        summary["topic_status"] = self._topic_status()
        return summary

    def _apply_baseline_acceptance(self, summary: dict[str, Any], comparison: dict[str, Any]) -> dict[str, Any]:
        updated = dict(summary)
        acceptance = dict(updated["acceptance"])
        reasons = list(acceptance.get("reasons", []))
        passed, reason = _baseline_rmse_improvement_pass(
            comparison,
            required_improvement=self.config.thresholds.required_minimum_baseline_improvement,
        )
        acceptance["baseline_improvement_pass"] = passed
        if not passed and reason not in reasons:
            reasons.append(reason)
        acceptance["reasons"] = reasons
        updated["acceptance"] = acceptance
        return updated

    def _write_raw_files(self, run_dir: Path) -> None:
        write_rows(
            run_dir / "state.csv",
            ["timestamp", "q0", "q1", "q2", "q3", "q4", "q5", "q_dot0", "q_dot1", "q_dot2", "q_dot3", "q_dot4", "q_dot5", "tip_x", "tip_y", "tip_z"],
            [
                [sample.timestamp, *sample.q.tolist(), *sample.q_dot.tolist(), *sample.tip_position.tolist()]
                for sample in self.states
            ],
        )
        write_rows(
            run_dir / "tip.csv",
            ["timestamp", "x", "y", "z"],
            self.tip_records,
        )
        write_rows(
            run_dir / "reference.csv",
            ["timestamp", "x", "y", "z", "progress"],
            [
                [
                    sample.timestamp,
                    *sample.position.tolist(),
                    "" if sample.progress is None else sample.progress,
                ]
                for sample in self.references
            ],
        )
        selected_commands = self.safe_commands if self.safe_commands else self.raw_commands
        write_rows(
            run_dir / "command.csv",
            ["timestamp", "source", "u0", "u1", "u2", "u3", "u4", "u5", "saturated"],
            [
                [sample.timestamp, sample.source, *sample.command.tolist(), sample.saturated]
                for sample in selected_commands
            ],
        )
        write_rows(
            run_dir / "solve_timing.csv",
            ["timestamp", "solve_time", "saturated"],
            [[sample.timestamp, sample.solve_time, sample.saturated] for sample in self.solves],
        )
        write_rows(
            run_dir / "horizon.csv",
            ["timestamp", "count", "first_x", "first_y", "first_z", "final_x", "final_y", "final_z"],
            self.horizon_records,
        )
        write_rows(
            run_dir / "reference_path.csv",
            ["timestamp", "count"],
            self.path_records,
        )

    def _write_aggregate(self, group_dir: Path) -> None:
        summaries = []
        for summary_path in group_dir.glob("*/summary.json"):
            try:
                summaries.append(json.loads(summary_path.read_text(encoding="utf-8")))
            except (OSError, json.JSONDecodeError):
                continue
        aggregate = aggregate_trial_summaries(summaries)
        write_json(group_dir / "aggregate_summary.json", aggregate)
        report = ["# Aggregate Evaluation Summary", "", f"Run count: {aggregate['count']}", ""]
        for key, values in aggregate.get("metrics", {}).items():
            report.append(
                f"- {key}: mean={values['mean']:.6g}, median={values['median']:.6g}, "
                f"min={values['minimum']:.6g}, max={values['maximum']:.6g}"
            )
        (group_dir / "aggregate_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    def _write_finalization_error(self, partial_dir: Path, exc: Exception) -> None:
        try:
            if partial_dir.exists():
                write_json(
                    partial_dir / "finalization_error.json",
                    {
                        "error": str(exc),
                        "state": self.lifecycle_state,
                        "run_id": self.run_id,
                        "partial_dir": str(partial_dir),
                    },
                )
        except Exception:
            pass

    def _topic_status(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for topic in observed_topics():
            result[topic] = {
                "count": self.topic_counts.get(topic, 0),
                "received": self.topic_counts.get(topic, 0) > 0,
                "required": topic in required_topics(),
                "optional": topic not in required_topics(),
            }
        return result


def write_aligned_csv(path: Path, alignment: AlignmentResult) -> None:
    rows = []
    for sample in alignment.samples:
        rows.append(
            [
                sample.timestamp,
                *sample.q.tolist(),
                *sample.q_dot.tolist(),
                *sample.tip_position.tolist(),
                *sample.reference_position.tolist(),
                *sample.command.tolist(),
                sample.solve_time,
                sample.command_saturated,
                sample.missing_command,
                sample.reference_gap,
                sample.command_gap,
                sample.solve_gap,
                sample.used_reference_interpolation,
                sample.used_nearest_reference,
                "" if sample.reference_progress is None else sample.reference_progress,
            ]
        )
    write_rows(
        path,
        [
            "timestamp",
            "q0",
            "q1",
            "q2",
            "q3",
            "q4",
            "q5",
            "q_dot0",
            "q_dot1",
            "q_dot2",
            "q_dot3",
            "q_dot4",
            "q_dot5",
            "tip_x",
            "tip_y",
            "tip_z",
            "ref_x",
            "ref_y",
            "ref_z",
            "u0",
            "u1",
            "u2",
            "u3",
            "u4",
            "u5",
            "solve_time",
            "command_saturated",
            "missing_command",
            "reference_gap",
            "command_gap",
            "solve_gap",
            "reference_interpolated",
            "nearest_reference",
            "reference_progress",
        ],
        rows,
    )


def write_rows(path: Path, fieldnames: list[str], rows: Any) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fieldnames)
        for row in rows:
            if isinstance(row, dict):
                writer.writerow([row.get(field, "") for field in fieldnames])
            else:
                writer.writerow(row)


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(sanitize_for_json(data), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_yaml(path: Path, data: Any) -> None:
    path.write_text(yaml.safe_dump(dataclass_to_plain(data), sort_keys=False), encoding="utf-8")


def observed_topics() -> tuple[str, ...]:
    return (
        "/ctr/state",
        "/ctr/tip",
        "/ctr/reference/tip",
        "/ctr/reference/horizon",
        "/ctr/reference/path",
        "/ctr/mppi_command",
        "/ctr/safe_command",
        "/ctr/controller/metrics",
        "/ctr/controller/trajectory_metrics",
        "/diagnostics",
    )


def required_topics() -> tuple[str, ...]:
    return ("/ctr/state", "/ctr/reference/tip")


def sanitize_name(value: str) -> str:
    cleaned = "".join(char if char.isalnum() or char in {"-", "_"} else "_" for char in value.strip())
    return cleaned or "experiment"


def _baseline_rmse_improvement_pass(
    comparison: dict[str, Any],
    *,
    required_improvement: float,
) -> tuple[bool, str]:
    if not comparison.get("compatibility_valid", False):
        return False, "baseline comparison is incompatible"
    for item in comparison.get("metric_comparisons", []):
        if item.get("metric") != "rmse":
            continue
        if not item.get("comparison_valid", False):
            return False, f"RMSE baseline comparison is invalid: {item.get('reason', 'unknown reason')}"
        improvement = item.get("relative_improvement_percent")
        if improvement is None:
            return False, "RMSE baseline improvement is unavailable"
        if float(improvement) < required_improvement:
            return False, "RMSE baseline improvement is below threshold"
        return True, "ok"
    return False, "RMSE baseline comparison is missing"


def git_metadata(workspace: Path) -> dict[str, Any]:
    return {
        "commit": _git(["rev-parse", "HEAD"], workspace),
        "short_commit": _git(["rev-parse", "--short", "HEAD"], workspace),
        "branch": _git(["branch", "--show-current"], workspace),
        "dirty": bool(_git(["status", "--short"], workspace)),
    }


def _git(args: list[str], workspace: Path) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=workspace,
            check=True,
            text=True,
            capture_output=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return result.stdout.strip()


def _override(overrides: dict[str, Any], key: str, default: Any) -> Any:
    value = overrides.get(key)
    if value is None or value == "":
        return default
    return value


def _bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be a boolean")
    return value


def _positive_number(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _nonnegative_number(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric < 0.0:
        raise ValueError(f"{label} must be non-negative")
    return numeric


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_number(value, label)
    integer = int(numeric)
    if integer != numeric:
        raise ValueError(f"{label} must be an integer")
    return integer


def _percentage(value: Any, label: str) -> float:
    numeric = _nonnegative_number(value, label)
    if numeric > 100.0:
        raise ValueError(f"{label} must be in [0, 100]")
    return numeric


def _fraction(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric <= 0.0 or numeric > 1.0:
        raise ValueError(f"{label} must be in (0, 1]")
    return numeric


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        numeric = int(value)
    except (TypeError, ValueError):
        return None
    return numeric
