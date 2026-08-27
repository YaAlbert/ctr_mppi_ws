"""Experiment lifecycle, data recording, and result-file writing."""

from __future__ import annotations

import csv
import cProfile
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
import json
import math
import os
import pstats
import io
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time
import uuid
from typing import Any, Callable, Mapping

import numpy as np
import yaml

from ctr_evaluation.curved_lumen_scenarios import (
    CURVED_LUMEN_SCENARIO_IDS,
    CURVED_SCENARIO_POLICY_VERSION,
)
from ctr_evaluation.lumen_metrics import LumenEvaluationMetrics, compute_lumen_evaluation_metrics
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
    compute_goal_metrics,
    compute_lumen_safety_metrics,
    compute_motion_metrics,
    compute_timing_metrics,
    compute_tracking_metrics,
    dataclass_to_plain,
    sanitize_for_json,
    stable_hash,
)
from ctr_mppi_controller.cylindrical_lumen import (
    CylindricalLumen,
    goal_hold_duration_from_config,
    goal_position_from_config,
    goal_tolerance_from_config,
)
from ctr_mppi_controller.lumen_factory import (
    CURVED_LUMEN_TYPES,
    config_with_lumen_overrides,
    lumen_geometry_fingerprint,
    lumen_geometry_fingerprint_payload,
    lumen_geometry_from_config,
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
from ctr_evaluation.publication_model import (
    Applicability,
    ArtifactRecord,
    ArtifactSpec,
    LayerASnapshot,
    PrePromotionLedger,
    PublicationStatus,
    build_artifact_inventory,
    prepromotion_failure_record,
    prepromotion_not_applicable_record,
    prepromotion_staged_record,
    validate_artifact_specs,
)


STATE_IDLE = "IDLE"
STATE_RECORDING = "RECORDING"
STATE_FINALIZING = "FINALIZING"
STATE_COMPLETED = "COMPLETED"

TASK_CURVED_LUMEN_NAVIGATION = "curved_lumen_navigation"
REFERENCE_MODE_FIXED_TARGET = "fixed_target"
LUMEN_EVALUATION_SCHEMA_VERSION = "lumen_evaluation_v1"
TIP_BACKBONE_CONSISTENCY_TOLERANCE = 1.0e-9
LUMEN_EVALUATION_CSV_FIELDS = [
    "timestamp_s",
    "physical_clearance_m",
    "safety_clearance_m",
    "physical_collision",
    "safety_margin_violation",
    "selected_constraint_type",
    "closest_backbone_index",
    "wall_penetration_m",
    "inlet_penetration_m",
    "outlet_penetration_m",
    "tip_centerline_x",
    "tip_centerline_y",
    "tip_centerline_z",
    "tip_centerline_segment_index",
    "tip_centerline_interpolation_fraction",
    "centerline_arc_length_m",
    "normalized_progress",
    "tip_progress_out_of_extent",
    "radial_offset_m",
    "local_radius_m",
]
MPPI_COST_TERM_NAMES = (
    "stage_tip_target",
    "terminal_target",
    "control_effort",
    "control_rate",
    "safety_margin",
    "wall_collision",
    "end_cap_collision",
    "terminal_lumen",
    "tactile",
)
MPPI_COST_CSV_FIELDS = [
    "timestamp_s",
    "minimum_cost",
    "mean_cost",
    "effective_sample_weight",
    *[f"raw.{name}" for name in MPPI_COST_TERM_NAMES],
    *[f"weight.{name}" for name in MPPI_COST_TERM_NAMES],
    *[f"weighted.{name}" for name in MPPI_COST_TERM_NAMES],
    *[f"weighted_mean.{name}" for name in MPPI_COST_TERM_NAMES],
]
MPPI_TIMING_CSV_FIELDS = [
    "timestamp_s",
    "timing.sampling_s",
    "timing.rollout_propagation_s",
    "timing.target_control_cost_s",
    "timing.lumen_cost_s",
    "timing.tactile_cost_s",
    "timing.weight_normalization_s",
    "timing.control_update_s",
    "ros_message_conversion_s",
    "timing.solve_total_s",
]
TACTILE_SAFETY_CSV_FIELDS = [
    "timestamp_s",
    "event_type",
    "received_timestamp_s",
    "data_age_s",
    "frame_id",
    "frame_valid",
    "source",
    "simulation_source",
    "raw_force_n",
    "filtered_force_n",
    "force_magnitude_n",
    "clearance_m",
    "contact",
    "warning",
    "stop",
    "valid",
    "region",
    "contact_on_n",
    "contact_off_n",
    "warning_on_n",
    "warning_off_n",
    "stop_on_n",
    "stop_off_n",
    *[f"commanded_u{index}" for index in range(6)],
    *[f"safe_u{index}" for index in range(6)],
    "commanded_norm",
    "safe_norm",
    "applied_scale",
    "command_gated",
    "safety_state",
    "safety_state_name",
    "safety_command_allowed",
    "safety_emergency_stop",
    "safety_fault",
    "safety_valid",
    "safety_reason",
]


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
    enable_finalization_profiling: bool
    diagnostic_data_collection: bool
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
    cylindrical_lumen: CylindricalLumen | None
    goal_position: np.ndarray | None
    goal_tolerance: float | None
    goal_required_hold_duration: float | None

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
            enable_finalization_profiling=_bool(
                evaluation.get("enable_finalization_profiling", False),
                "evaluation.enable_finalization_profiling",
            ),
            diagnostic_data_collection=_bool(
                evaluation.get("diagnostic_data_collection", False),
                "evaluation.diagnostic_data_collection",
            ),
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
            cylindrical_lumen=(
                CylindricalLumen.from_config(project_config)
                if bool(project_config.get("cylindrical_lumen", {}).get("enabled", False))
                else None
            ),
            goal_position=(
                goal_position_from_config(project_config)
                if isinstance(project_config.get("goal"), dict)
                else None
            ),
            goal_tolerance=(
                goal_tolerance_from_config(project_config)
                if isinstance(project_config.get("goal"), dict)
                else None
            ),
            goal_required_hold_duration=(
                goal_hold_duration_from_config(project_config)
                if isinstance(project_config.get("goal"), dict)
                else None
            ),
        )


@dataclass(frozen=True)
class FinalizationResult:
    run_id: str
    run_dir: Path
    summary: dict[str, Any]
    metadata: dict[str, Any]
    comparison: dict[str, Any] | None
    output_files: list[Path]
    promotion: PromotionResult | None = None


@dataclass(frozen=True)
class LumenBackboneData:
    timestamps: np.ndarray
    backbones: tuple[np.ndarray, ...]
    tip_points: np.ndarray
    data_quality: dict[str, Any]


@dataclass(frozen=True)
class LumenRecorderResult:
    required: bool
    section: dict[str, Any] | None = None
    csv_rows: tuple[dict[str, Any], ...] = ()


class StagingSetupError(RuntimeError):
    """The disconnected coordinator could not exclusively acquire staging."""


class ProducerRenderError(RuntimeError):
    """A producer failed while rendering its artifact content."""


class ProducerStagingError(RuntimeError):
    """A producer failed while writing its artifact to staging."""


class PromotionStatus(str, Enum):
    """Filesystem-only outcome of the Slice 3 promotion boundary."""

    PRE_PROMOTION_FAILED = "PRE_PROMOTION_FAILED"
    PROMOTION_REFUSED = "PROMOTION_REFUSED"
    PROMOTION_FAILED = "PROMOTION_FAILED"
    PROMOTED_AND_OBSERVED = "PROMOTED_AND_OBSERVED"
    PROMOTED_OBSERVATION_FAILED = "PROMOTED_OBSERVATION_FAILED"


@dataclass(frozen=True)
class PromotionResult:
    """Non-authoritative evidence about staging and final-path observation."""

    status: PromotionStatus
    staging_dir: Path
    final_dir: Path
    reason: str | None = None
    observed_paths: tuple[Path, ...] = ()
    failure_evidence_dir: Path | None = None


def _prepromotion_order(
    specs: tuple[ArtifactSpec, ...],
) -> tuple[ArtifactSpec, ...]:
    by_name = {item.logical_name: item for item in specs}
    indegree = {item.logical_name: 0 for item in specs}
    children = {item.logical_name: [] for item in specs}
    for item in specs:
        for dependency in item.dependencies:
            indegree[item.logical_name] += 1
            children[dependency].append(item.logical_name)
    ready = sorted(name for name, count in indegree.items() if count == 0)
    ordered: list[ArtifactSpec] = []
    while ready:
        name = ready.pop(0)
        ordered.append(by_name[name])
        for child in sorted(children[name]):
            indegree[child] -= 1
            if indegree[child] == 0:
                ready.append(child)
                ready.sort()
    if len(ordered) != len(specs):
        raise ValueError(
            "artifact dependency graph did not produce a complete order"
        )
    return tuple(ordered)


def _dependency_failure_reason(name: str, failed: list[str]) -> str:
    joined = ", ".join(sorted(failed))
    return f"applicable dependency failed for {name}: {joined}"


def prepare_prepromotion_ledger(
    *,
    layer_a: LayerASnapshot,
    inventory: tuple[ArtifactSpec, ...] | list[ArtifactSpec],
    staging_root: Path,
    applicability: Mapping[str, Applicability | bool] | None,
    producer_registry: Mapping[str, Callable[[Path], Any]],
    report_producer: Callable[[Path], Any] | None,
    orchestration_producer: Callable[[Path], Any] | None = None,
    comparison_producer: Callable[[Path], Any] | None = None,
    acquire_staging: bool = True,
) -> PrePromotionLedger:
    """Run disconnected artifact attempts and return a pre-promotion ledger.

    Producers receive their exact target path inside the caller-owned staging
    directory and must create that regular file. No final directory is
    inspected and this function never renames or publishes the staging root.
    """
    if not isinstance(layer_a, LayerASnapshot):
        raise TypeError("layer_a must be a LayerASnapshot")
    specs = tuple(inventory)
    validate_artifact_specs(specs)
    if not isinstance(staging_root, Path):
        raise TypeError("staging_root must be a Path")
    if (
        staging_root.name in {"", ".", ".."}
        or not staging_root.name.endswith(".partial")
    ):
        raise ValueError(
            "staging_root must be an explicitly named .partial directory"
        )
    if acquire_staging:
        if staging_root.exists():
            raise StagingSetupError(f"staging root already exists: {staging_root}")
        try:
            staging_root.mkdir(parents=True, exist_ok=False)
        except OSError as exc:
            raise StagingSetupError(
                f"could not acquire staging root: {staging_root}"
            ) from exc
    elif staging_root.is_symlink() or not staging_root.is_dir():
        raise StagingSetupError(
            f"pre-acquired staging root is not a directory: {staging_root}"
        )

    applicability = dict(applicability or {})
    unknown_applicability = {
        name for name in applicability
    } - {spec.logical_name for spec in specs}
    if unknown_applicability:
        raise ValueError(
            "unknown applicability entries: "
            f"{', '.join(sorted(unknown_applicability))}"
        )
    execution_applicability: dict[str, Applicability] = {}
    for spec in specs:
        selected = applicability.get(spec.logical_name, spec.applicability)
        if isinstance(selected, bool):
            selected = (
                Applicability.APPLICABLE
                if selected else Applicability.NOT_APPLICABLE
            )
        if not isinstance(selected, Applicability):
            raise TypeError(
                f"applicability for {spec.logical_name} must be "
                "an Applicability"
            )
        execution_applicability[spec.logical_name] = selected

    records: dict[str, ArtifactRecord] = {}
    for spec in specs:
        if (
            execution_applicability[spec.logical_name]
            is Applicability.NOT_APPLICABLE
        ):
            records[spec.logical_name] = prepromotion_not_applicable_record(
                spec, layer_a
            )

    ordered = _prepromotion_order(specs)

    def attempt(
        spec: ArtifactSpec, producer: Callable[[Path], Any] | None
    ) -> None:
        target = staging_root / spec.relative_path

        def cleanup_target() -> None:
            if target.is_file() and not target.is_symlink():
                try:
                    target.unlink()
                except OSError:
                    pass

        try:
            target.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.STAGE_FAILED,
                f"{type(exc).__name__}: {exc}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        if producer is None and spec.logical_name != "finalization_trace":
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.DEPENDENCY_FAILED,
                f"external producer callback absent: {spec.logical_name}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        try:
            if producer is None:
                try:
                    target.write_text(
                        json.dumps(
                            {
                                "schema_version": "finalization_trace_v1",
                                "events": [],
                            }
                        ) + "\n",
                        encoding="utf-8",
                    )
                except Exception as exc:
                    raise ProducerStagingError(str(exc)) from exc
            else:
                producer(target)
        except ProducerRenderError as exc:
            cleanup_target()
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.RENDER_FAILED,
                f"{type(exc).__name__}: {exc}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        except ProducerStagingError as exc:
            cleanup_target()
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.STAGE_FAILED,
                f"{type(exc).__name__}: {exc}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        except Exception as exc:
            cleanup_target()
            # Unexpected producer errors default to render failure. Residue is
            # never used to infer the failed protocol stage.
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.RENDER_FAILED,
                f"{type(exc).__name__}: {exc}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        try:
            valid_target = target.is_file() and not target.is_symlink()
        except Exception as exc:
            cleanup_target()
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.STAGE_FAILED,
                f"{type(exc).__name__}: {exc}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        if not valid_target:
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.STAGE_FAILED,
                "producer did not create expected regular file: "
                f"{spec.relative_path}",
                run_applicability=execution_applicability[spec.logical_name],
            )
            return
        records[spec.logical_name] = prepromotion_staged_record(
            spec,
            layer_a,
            run_applicability=execution_applicability[spec.logical_name],
        )

    report_spec = next(
        (spec for spec in specs if spec.logical_name == "report"), None
    )
    for spec in ordered:
        if spec.logical_name == "report":
            continue
        if (
            execution_applicability[spec.logical_name]
            is Applicability.NOT_APPLICABLE
        ):
            continue
        failed = [
            dependency for dependency in spec.dependencies
            if records.get(dependency) is not None
            and records[dependency].publication_status not in {
                None,
                PublicationStatus.NOT_APPLICABLE,
            }
        ]
        if failed:
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.DEPENDENCY_FAILED,
                _dependency_failure_reason(spec.logical_name, failed),
                run_applicability=execution_applicability[spec.logical_name],
            )
            continue
        if spec.logical_name == "orchestration":
            attempt(spec, orchestration_producer)
        elif (
            spec.logical_name == "comparison"
            and comparison_producer is not None
        ):
            attempt(spec, comparison_producer)
        else:
            attempt(spec, producer_registry.get(spec.logical_name))

    if (
        report_spec is not None
        and execution_applicability[report_spec.logical_name]
        is Applicability.APPLICABLE
    ):
        failed = [
            dependency for dependency in report_spec.dependencies
            if records.get(dependency) is not None
            and records[dependency].publication_status not in {
                None,
                PublicationStatus.NOT_APPLICABLE,
            }
        ]
        if failed:
            records["report"] = prepromotion_failure_record(
                report_spec, layer_a, PublicationStatus.DEPENDENCY_FAILED,
                _dependency_failure_reason("report", failed),
                run_applicability=execution_applicability[
                    report_spec.logical_name
                ],
            )
        else:
            attempt(report_spec, report_producer)

    for spec in specs:
        if spec.logical_name not in records:
            records[spec.logical_name] = prepromotion_failure_record(
                spec, layer_a, PublicationStatus.DEPENDENCY_FAILED,
                f"producer execution did not terminalize: {spec.logical_name}",
                run_applicability=execution_applicability[spec.logical_name],
            )
    ordered_records = tuple(records[spec.logical_name] for spec in specs)
    return PrePromotionLedger(ordered_records, specs, staging_root)


def _owned_path(root: Path, relative_path: str) -> Path:
    root_resolved = root.resolve()
    candidate = root / relative_path
    candidate_resolved = candidate.resolve(strict=False)
    if root_resolved not in candidate_resolved.parents:
        raise ValueError(
            f"artifact path escapes staging root: {relative_path}"
        )
    return candidate


def _promotion_eligibility(ledger: PrePromotionLedger) -> tuple[str, ...]:
    if not isinstance(ledger, PrePromotionLedger):
        return ("promotion requires a PrePromotionLedger",)
    root = ledger.staging_root
    if root.is_symlink() or not root.is_dir():
        return (f"staging directory is absent: {root}",)
    reasons: list[str] = []
    for record in ledger.records:
        if record.run_applicability is Applicability.NOT_APPLICABLE:
            continue
        try:
            target = _owned_path(root, record.relative_path)
        except ValueError as exc:
            reasons.append(str(exc))
            continue
        if record.publication_status is None:
            if target.is_symlink() or not target.is_file():
                reasons.append(
                    f"staged artifact is missing or not a regular file: "
                    f"{record.logical_name}"
                )
        elif record.required:
            reasons.append(
                f"required applicable artifact failed: {record.logical_name}"
            )
    return tuple(reasons)


def promote_prepromotion_ledger(
    ledger: PrePromotionLedger,
    final_dir: Path,
) -> PromotionResult:
    """Promote staging and observe paths without publication authority."""
    if not isinstance(final_dir, Path):
        raise TypeError("final_dir must be a Path")
    if not isinstance(ledger, PrePromotionLedger):
        return PromotionResult(
            PromotionStatus.PRE_PROMOTION_FAILED,
            Path("."),
            final_dir,
            "promotion requires a PrePromotionLedger",
        )
    staging_dir = ledger.staging_root
    try:
        reasons = list(_promotion_eligibility(ledger))
        if staging_dir.name != f"{final_dir.name}.partial":
            reasons.append("staging and final paths are not the same run")
        if staging_dir.parent != final_dir.parent:
            reasons.append("staging and final paths do not share a parent")
        if final_dir.exists():
            reasons.append(f"final directory already exists: {final_dir}")
        if reasons:
            return PromotionResult(
                PromotionStatus.PROMOTION_REFUSED,
                staging_dir,
                final_dir,
                "; ".join(reasons),
                failure_evidence_dir=staging_dir,
            )
        try:
            staging_dir.replace(final_dir)
        except OSError as exc:
            return PromotionResult(
                PromotionStatus.PROMOTION_FAILED,
                staging_dir,
                final_dir,
                f"atomic promotion failed: {exc}",
                failure_evidence_dir=staging_dir,
            )
        expected: set[Path] = set()
        missing: list[str] = []
        for record in ledger.records:
            if (
                record.run_applicability is Applicability.APPLICABLE
                and record.publication_status is None
            ):
                try:
                    path = _owned_path(final_dir, record.relative_path)
                except ValueError as exc:
                    missing.append(str(exc))
                    continue
                expected.add(path.relative_to(final_dir.resolve()))
                if path.is_symlink() or not path.is_file():
                    missing.append(record.logical_name)
        observed_entries = {
            path.relative_to(final_dir.resolve())
            for path in final_dir.rglob("*")
        }
        unexpected = sorted(observed_entries - expected, key=str)
        if missing or unexpected:
            reason_parts = []
            if missing:
                reason_parts.append(
                    "missing final artifacts: " + ", ".join(sorted(missing))
                )
            if unexpected:
                reason_parts.append(
                    "unexpected final files: "
                    + ", ".join(map(str, unexpected))
                )
            return PromotionResult(
                PromotionStatus.PROMOTED_OBSERVATION_FAILED,
                staging_dir,
                final_dir,
                "; ".join(reason_parts),
                failure_evidence_dir=final_dir,
            )
        observed = tuple(sorted((final_dir / path for path in expected), key=str))
        return PromotionResult(
            PromotionStatus.PROMOTED_AND_OBSERVED,
            staging_dir,
            final_dir,
            observed_paths=observed,
        )
    except (OSError, ValueError) as exc:
        return PromotionResult(
            PromotionStatus.PROMOTION_REFUSED,
            staging_dir,
            final_dir,
            str(exc),
            failure_evidence_dir=staging_dir,
        )


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
        self._finalization_trace: list[dict[str, Any]] = []
        self._finalization_trace_path: Path | None = None
        self._metric_trace_start_ns: int | None = None
        self.reset_buffers()

    def prepare_prepromotion_ledger(self, **kwargs: Any) -> PrePromotionLedger:
        """Expose the disconnected coordinator without changing finalization."""
        return prepare_prepromotion_ledger(**kwargs)

    @staticmethod
    def _existing_artifact_producer(path: Path) -> None:
        if path.is_symlink() or not path.is_file():
            raise ProducerStagingError(
                f"expected rendered artifact is unavailable: {path.name}"
            )

    def _record_optional_diagnostic(
        self,
        stage: str,
        *,
        phase: str,
        status: str,
        **details: Any,
    ) -> None:
        try:
            callback = getattr(self, "record_diagnostic_event", None)
            if callable(callback):
                callback(stage, phase=phase, status=status, **details)
        except Exception:
            pass

    def _record_optional_metric_stage_end(
        self,
        stage: str,
        start_ns: int,
        *,
        status: str,
        details: dict[str, Any] | None = None,
    ) -> None:
        try:
            callback = getattr(self, "_record_metric_stage_end", None)
            if callable(callback):
                callback(stage, start_ns, status=status, details=details)
        except Exception:
            pass

    def _start_optional_profile(self) -> Any | None:
        try:
            enabled = bool(
                getattr(self.config, "enable_finalization_profiling", False)
            )
            profile_factory = getattr(cProfile, "Profile", None)
            if not enabled or not callable(profile_factory):
                return None
            profile = profile_factory()
            profile.enable()
            return profile
        except Exception:
            return None

    def _finish_optional_profile(self, profile: Any | None) -> None:
        try:
            if profile is not None:
                profile.disable()
            callback = getattr(self, "_record_metric_profile", None)
            if callable(callback):
                callback(profile)
        except Exception:
            pass

    def _set_optional_trace_path(self, path: Path) -> None:
        try:
            state = object.__getattribute__(self, "__dict__")
            if "_finalization_trace_path" not in state:
                return
            object.__setattr__(self, "_finalization_trace_path", path)
        except Exception:
            pass

    def _orchestration_artifact_producer(
        self,
        path: Path,
        metadata: dict[str, Any],
    ) -> None:
        orchestration = metadata.get("orchestration_runtime")
        if not isinstance(orchestration, dict):
            orchestration = {
                "schema_version": "orchestration_v1",
                "run_id": self.run_id,
                "source": "experiment_recorder",
            }
        write_json(path, orchestration)

    def _finalization_layer_a(
        self,
        metadata: dict[str, Any],
        comparison: dict[str, Any] | None,
        interrupted: bool,
    ) -> LayerASnapshot:
        return LayerASnapshot(
            snapshot_id=stable_hash(
                {"run_id": self.run_id, "metadata": metadata}
            ),
            operational_reason=metadata.get("operational_reason"),
            workflow_classification=(
                "INTERRUPTED" if interrupted else "COMPLETED"
            ),
            workflow_exit_code=0,
            comparison_valid=(
                None
                if comparison is None
                else bool(comparison.get("comparison_valid", False))
            ),
            timeout_status="DESCRIPTIVE_ONLY",
            cancellation_evidence=metadata.get("cancellation_evidence", ()),
            delivery_classification="RECORDER_FINALIZATION",
            compatibility_valid=(
                None
                if comparison is None
                else bool(comparison.get("compatibility_valid", False))
            ),
            timing_data=metadata.get("timing", {}),
        )

    def _prepare_finalization_ledger(
        self,
        *,
        partial_dir: Path,
        metadata: dict[str, Any],
        summary: dict[str, Any],
        comparison: dict[str, Any] | None,
        alignment: AlignmentResult,
        interrupted: bool,
        preparation_failures: dict[str, str] | None = None,
        comparison_applicable: bool | None = None,
    ) -> PrePromotionLedger:
        include_lumen = _has_curved_lumen_metadata(metadata)
        include_cylinder = (
            self.config.cylindrical_lumen is not None
            and self.config.goal_position is not None
        )
        inventory = build_artifact_inventory(
            include_lumen=include_lumen,
            include_cylinder=include_cylinder,
            include_plots=self.config.plot_generation,
            include_comparison=(
                comparison is not None
                if comparison_applicable is None
                else comparison_applicable
            ),
            include_diagnostics=self.config.diagnostic_data_collection,
        )
        existing = self._existing_artifact_producer
        producers: dict[str, Callable[[Path], Any]] = {
            spec.logical_name: existing for spec in inventory
        }
        for name, reason in (preparation_failures or {}).items():
            def failed_producer(
                _target: Path,
                reason: str = reason,
            ) -> None:
                raise ProducerStagingError(reason)

            producers[name] = failed_producer
        from ctr_evaluation.report_generator import (
            generate_report,
            plot_producer_registry,
        )

        plot_producers = plot_producer_registry(
            partial_dir,
            alignment.samples,
            metadata,
            include_cylinder_plots=include_cylinder,
            include_lumen_plots=include_lumen,
            include_diagnostic_plots=self.config.diagnostic_data_collection,
        )
        for name, producer in plot_producers.items():
            producers[name] = lambda _target, producer=producer: producer(
                partial_dir
            )

        def report(path: Path) -> None:
            self._record_optional_diagnostic(
                "report_generation", phase="start", status="started"
            )
            try:
                generate_report(
                    run_dir=partial_dir,
                    metadata=metadata,
                    summary=summary,
                    comparison=comparison,
                    plot_paths=[path for path in partial_dir.glob("*.png")],
                )
                if not path.is_file():
                    raise ProducerStagingError(
                        f"report producer did not create {path.name}"
                    )
            except Exception as exc:
                self._record_optional_diagnostic(
                    "report_generation",
                    phase="end",
                    status="error",
                    error_type=type(exc).__name__,
                    error_message=str(exc),
                )
                raise
            self._record_optional_diagnostic(
                "report_generation", phase="end", status="ok"
            )

        def orchestration(path: Path) -> None:
            self._orchestration_artifact_producer(path, metadata)

        layer_a = self._finalization_layer_a(metadata, comparison, interrupted)
        return prepare_prepromotion_ledger(
            layer_a=layer_a,
            inventory=inventory,
            staging_root=partial_dir,
            applicability=None,
            producer_registry=producers,
            report_producer=report,
            orchestration_producer=orchestration,
            comparison_producer=existing,
            acquire_staging=False,
        )

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
        self.backbone_records: list[dict[str, Any]] = []
        self.tactile_evidence_records: list[dict[str, Any]] = []
        self.safety_evidence_records: list[dict[str, Any]] = []
        self.mppi_diagnostic_records: list[dict[str, Any]] = []
        self.invalid_mppi_diagnostic_count = 0
        self.initial_state_q: list[float] | None = None
        self.initial_tip_position: list[float] | None = None
        self.slice_7g_safety_fault_count = 0
        self.slice_7g_tactile_invalid_count = 0

    def _reject_existing_run_id(self, run_id: str) -> None:
        group_dir = self.config.output_root / self.config.experiment_group
        partial_dir = group_dir / f"{run_id}.partial"
        final_dir = group_dir / run_id
        if partial_dir.exists():
            raise FileExistsError(f"partial result directory already exists: {partial_dir}")
        if final_dir.exists():
            raise FileExistsError(f"final result directory already exists: {final_dir}")

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
        self._finalization_trace = []
        self._finalization_trace_path = None
        self._metric_trace_start_ns = None
        safe_name = sanitize_name(experiment_name or self.config.controller_label)
        requested_run_id = requested_run_id_from_metadata(metadata or {})
        if requested_run_id:
            self.run_id = requested_run_id
            self._reject_existing_run_id(self.run_id)
        else:
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
        self._record_optional_diagnostic(
            "data_collection_disabled", phase="end", status="ok"
        )
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
        self._set_optional_trace_path(partial_dir / "finalization_trace.json")
        self._record_optional_diagnostic(
            "finalization", phase="start", status="started"
        )
        try:
            self._record_optional_diagnostic(
                "data_snapshot", phase="start", status="started"
            )
            metadata = self._metadata(interrupted=interrupted)
            metadata["initial_state_q"] = self.initial_state_q
            metadata["initial_tip_position"] = self.initial_tip_position
            selected_commands = self.safe_commands if self.safe_commands else self.raw_commands
            alignment_states = self._states_for_evaluation_window(metadata)
            alignment = align_samples(
                states=alignment_states,
                references=self.references,
                commands=selected_commands,
                solves=self.solves,
                config=self.config.alignment,
            )
            self._record_optional_diagnostic(
                "data_snapshot", phase="end", status="ok"
            )
            metadata["alignment_window"] = {
                "state_samples_recorded": len(self.states),
                "state_samples_evaluated": len(alignment_states),
                "reference_samples_recorded": len(self.references),
                "command_samples_recorded": len(selected_commands),
            }
            metadata["actual_evaluation_window_duration_s"] = (
                max(0.0, alignment.samples[-1].timestamp - alignment.samples[0].timestamp)
                if alignment.samples
                else 0.0
            )
            alignment_start_ns = time.monotonic_ns()
            self._record_optional_metric_stage_end(
                "alignment",
                alignment_start_ns,
                status="ok",
                details={"aligned_count": len(alignment.samples)},
            )
            preparation_failures: dict[str, str] = {}
            raw_artifacts = (
                "raw_state",
                "raw_tip",
                "raw_reference",
                "raw_command",
                "solve_timing",
                "horizon",
                "reference_path",
                "backbone",
                "cylinder_navigation",
                "tactile_safety_evidence",
                "mppi_cost_terms",
                "mppi_computation",
            )
            raw_start_ns = time.monotonic_ns()
            self._record_optional_diagnostic(
                "metric_calculation.raw_data_write",
                phase="start",
                status="started",
            )
            try:
                self._write_raw_files(partial_dir)
            except Exception as exc:
                reason = f"raw artifact write failed: {type(exc).__name__}: {exc}"
                for logical_name in raw_artifacts:
                    preparation_failures[logical_name] = reason
            self._record_optional_metric_stage_end(
                "raw_data_write",
                raw_start_ns,
                status="ok",
                details={"state_count": len(self.states)},
            )
            lumen_profile = self._start_optional_profile()
            lumen_start_ns = time.monotonic_ns()
            self._record_optional_diagnostic(
                "metric_calculation.lumen_evaluation",
                phase="start",
                status="started",
            )
            lumen_result = None
            lumen_status = "ok"
            try:
                lumen_result = self._lumen_evaluation_result(alignment=alignment, metadata=metadata)
            except BaseException as exc:
                lumen_status = "error"
                preparation_failures["lumen_evaluation"] = (
                    f"lumen evaluation failed: {type(exc).__name__}: {exc}"
                )
                lumen_result = LumenRecorderResult(required=True)
            finally:
                self._record_optional_metric_stage_end(
                    "lumen_evaluation",
                    lumen_start_ns,
                    status=lumen_status,
                    details={"aligned_count": len(alignment.samples)},
                )
                self._finish_optional_profile(lumen_profile)
            lumen_result = self._write_lumen_evaluation_csv_if_available(
                partial_dir, lumen_result
            )
            if (
                lumen_result.required
                and not (partial_dir / "lumen_evaluation.csv").is_file()
            ):
                preparation_failures.setdefault(
                    "lumen_evaluation",
                    "lumen evaluation did not produce lumen_evaluation.csv",
                )
            try:
                summary = self._summary(
                    alignment=alignment,
                    metadata=metadata,
                    lumen_result=lumen_result,
                )
            except Exception as exc:
                summary = {}
                preparation_failures["summary"] = (
                    f"summary write failed: {type(exc).__name__}: {exc}"
                )
            summary_start_ns = time.monotonic_ns()
            self._record_optional_metric_stage_end(
                "summary",
                summary_start_ns,
                status="ok",
                details={"summary_keys": len(summary)},
            )
            try:
                write_yaml(partial_dir / "metadata.yaml", metadata)
            except Exception as exc:
                preparation_failures["metadata"] = (
                    f"metadata write failed: {type(exc).__name__}: {exc}"
                )
            try:
                write_json(partial_dir / "summary.json", summary)
            except Exception as exc:
                preparation_failures["summary"] = (
                    f"summary write failed: {type(exc).__name__}: {exc}"
                )
            try:
                write_aligned_csv(partial_dir / "aligned_samples.csv", alignment)
            except Exception as exc:
                preparation_failures["aligned_samples"] = (
                    f"aligned sample write failed: {type(exc).__name__}: {exc}"
                )

            comparison = None

            comparison_applicable = bool(self.config.baseline_result_dir)
            if comparison_applicable:
                self._record_optional_diagnostic(
                    "comparison_metadata_update",
                    phase="start",
                    status="started",
                )
                from ctr_evaluation.compare_results import compare_result_dirs

                try:
                    comparison = compare_result_dirs(
                        candidate_dir=partial_dir,
                        baseline_dir=Path(self.config.baseline_result_dir),
                        duration_tolerance=self.config.duration_compatibility_tolerance,
                        initial_state_tolerance=self.config.initial_state_compatibility_tolerance,
                        near_zero_epsilon=self.config.thresholds.near_zero_baseline_epsilon,
                    )
                    summary = self._apply_baseline_acceptance(summary, comparison)
                    write_json(partial_dir / "summary.json", summary)
                except Exception as exc:
                    comparison = None
                    preparation_failures["comparison"] = (
                        f"comparison failed: {type(exc).__name__}: {exc}"
                    )
                    preparation_failures["comparison_report"] = (
                        f"comparison failed: {type(exc).__name__}: {exc}"
                    )
                self._record_optional_diagnostic(
                    "comparison_metadata_update",
                    phase="end",
                    status="ok",
                )

            try:
                ledger = self._prepare_finalization_ledger(
                    partial_dir=partial_dir,
                    metadata=metadata,
                    summary=summary,
                    comparison=comparison,
                    alignment=alignment,
                    interrupted=interrupted,
                    preparation_failures=preparation_failures,
                    comparison_applicable=comparison_applicable,
                )
            except (StagingSetupError, TypeError, ValueError) as exc:
                promotion = PromotionResult(
                    PromotionStatus.PRE_PROMOTION_FAILED,
                    partial_dir,
                    final_dir,
                    str(exc),
                    failure_evidence_dir=partial_dir,
                )
                try:
                    self._write_finalization_error(partial_dir, exc)
                except Exception:
                    pass
                self.lifecycle_state = STATE_COMPLETED
                self.finalization_result = FinalizationResult(
                    run_id=self.run_id,
                    run_dir=partial_dir,
                    summary=summary,
                    metadata=metadata,
                    comparison=comparison,
                    output_files=sorted(
                        path
                        for path in partial_dir.iterdir()
                        if path.is_file()
                    ),
                    promotion=promotion,
                )
                return self.finalization_result
            promotion = promote_prepromotion_ledger(ledger, final_dir)
            if promotion.status is not PromotionStatus.PROMOTED_AND_OBSERVED:
                evidence_dir = promotion.failure_evidence_dir or partial_dir
                try:
                    self._write_finalization_error(
                        evidence_dir,
                        RuntimeError(promotion.reason or promotion.status.value),
                    )
                except Exception:
                    pass
                if promotion.final_dir.is_dir():
                    self._record_optional_diagnostic(
                        "final_path_observation",
                        phase="error",
                        status=promotion.status.value,
                        error_message=promotion.reason,
                    )
                self.lifecycle_state = STATE_COMPLETED
                self.finalization_result = FinalizationResult(
                    run_id=self.run_id,
                    run_dir=(
                        promotion.final_dir
                        if promotion.final_dir.is_dir()
                        else partial_dir
                    ),
                    summary=summary,
                    metadata=metadata,
                    comparison=comparison,
                    output_files=sorted(
                        path
                        for path in (
                            promotion.final_dir
                            if promotion.final_dir.is_dir()
                            else partial_dir
                        ).iterdir()
                        if path.is_file()
                    ),
                    promotion=promotion,
                )
                return self.finalization_result
            self._set_optional_trace_path(
                final_dir / "finalization_trace.json"
            )
            self._record_optional_diagnostic(
                "final_artifact_rename", phase="end", status="ok"
            )
            self._write_aggregate(group_dir)
            self._record_optional_diagnostic(
                "aggregate_write", phase="end", status="ok"
            )
            self.lifecycle_state = STATE_COMPLETED
            self.finalization_result = FinalizationResult(
                run_id=self.run_id,
                run_dir=final_dir,
                summary=summary,
                metadata=metadata,
                comparison=comparison,
                output_files=sorted(
                    path for path in final_dir.iterdir() if path.is_file()
                ),
                promotion=promotion,
            )
            self._record_optional_diagnostic(
                "finalization", phase="end", status="ok"
            )
            return self.finalization_result
        except Exception as exc:
            self._record_optional_diagnostic(
                "finalization",
                phase="error",
                status="error",
                error_type=type(exc).__name__,
                error_message=str(exc),
            )
            self._write_finalization_error(partial_dir, exc)
            raise

    def record_diagnostic_event(
        self,
        stage: str,
        *,
        phase: str,
        status: str,
        error_type: str | None = None,
        error_message: str | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        event = {
            "monotonic_ns": time.monotonic_ns(),
            "utc": datetime.now(timezone.utc).isoformat(),
            "pid": os.getpid(),
            "thread_id": threading.get_ident(),
            "run_id": self.run_id,
            "experiment_group": self.config.experiment_group,
            "run_role": self.metadata_override.get("run_role"),
            "stage": stage,
            "phase": phase,
            "status": status,
            "error_type": error_type,
            "error_message": error_message,
        }
        if details:
            event["details"] = sanitize_for_json(details)
        self._finalization_trace.append(event)
        if self._finalization_trace_path is not None:
            write_json(
                self._finalization_trace_path,
                {"schema_version": "finalization_trace_v1", "events": self._finalization_trace},
            )

    def _record_metric_stage_end(
        self,
        stage: str,
        start_ns: int,
        *,
        status: str,
        details: dict[str, Any] | None = None,
    ) -> None:
        payload = dict(details or {})
        end_ns = time.monotonic_ns()
        payload.update(
            {
                "elapsed_s": (end_ns - start_ns) / 1.0e9,
                "call_count": 1,
                "cumulative_s": (
                    (end_ns - self._metric_trace_start_ns) / 1.0e9
                    if self._metric_trace_start_ns is not None
                    else None
                ),
            }
        )
        self.record_diagnostic_event(
            f"metric_calculation.{stage}",
            phase="end",
            status=status,
            details=payload,
        )

    def _lumen_geometry_point_count(self, result: Any) -> int | None:
        if not result or not result.section:
            return None
        geometry = result.section.get("geometry", {})
        points = geometry.get("centerline_points") if isinstance(geometry, dict) else None
        return len(points) if isinstance(points, list) else None

    def _record_metric_profile(self, profile: cProfile.Profile | None) -> None:
        if profile is None:
            self.record_diagnostic_event(
                "metric_calculation.profile",
                phase="end",
                status="disabled",
                details={"enabled": False, "function_count": 0, "top_functions": []},
            )
            return
        stream = io.StringIO()
        stats = pstats.Stats(profile, stream=stream).sort_stats("cumulative")
        rows = []
        for function in stats.fcn_list[:20]:
            values = stats.stats[function]
            primitive, calls, total, cumulative, _callers = values
            rows.append(
                {
                    "function": f"{function[0]}:{function[1]}:{function[2]}",
                    "calls": calls,
                    "primitive_calls": primitive,
                    "self_s": total,
                    "cumulative_s": cumulative,
                }
            )
        self.record_diagnostic_event(
            "metric_calculation.profile",
            phase="end",
            status="ok",
            details={"enabled": True, "function_count": len(stats.stats), "top_functions": rows},
        )

    def record_state(self, *, timestamp: float, q: Any, q_dot: Any, tip_position: Any, backbone_points: Any | None = None) -> None:
        if not self._accept_sample("/ctr/state"):
            return
        try:
            sample = state_sample(timestamp, q, q_dot, tip_position, backbone_points=backbone_points)
            self.states.append(sample)
            if sample.backbone_points is not None:
                for index, point in enumerate(sample.backbone_points):
                    self.backbone_records.append(
                        {
                            "timestamp": sample.timestamp,
                            "index": index,
                            "x": float(point[0]),
                            "y": float(point[1]),
                            "z": float(point[2]),
                        }
                    )
            if self.initial_state_q is None:
                self.initial_state_q = [float(value) for value in sample.q]
            if self.initial_tip_position is None:
                self.initial_tip_position = [float(value) for value in sample.tip_position]
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

    def record_slice_7g_tactile(self, *, valid: bool, source: str) -> None:
        """Retain Slice 7G tactile topic presence and invalid-sample accounting."""

        self.record_topic("/ctr/tactile/state")
        if self.lifecycle_state != STATE_RECORDING:
            return
        if type(valid) is not bool or type(source) is not str or not valid or source != "simulated":
            self.slice_7g_tactile_invalid_count += 1

    def record_slice_7g_safety(self, *, valid: bool, fault: bool, emergency_stop: bool) -> None:
        """Retain supervisor status; only recording-window faults count toward acceptance."""

        self.record_topic("/ctr/safety/status")
        if self.lifecycle_state != STATE_RECORDING:
            return
        if (
            type(valid) is not bool
            or type(fault) is not bool
            or type(emergency_stop) is not bool
            or not valid
            or fault
            or emergency_stop
        ):
            self.slice_7g_safety_fault_count += 1

    def record_tactile_evidence(self, **values: Any) -> None:
        if not self.config.diagnostic_data_collection or self.lifecycle_state != STATE_RECORDING:
            return
        try:
            timestamp = _number(values["timestamp"], "tactile timestamp")
            received = _number(values["received_timestamp"], "tactile received timestamp")
            raw = np.asarray(values["raw_values"], dtype=float)
            filtered = np.asarray(values["filtered_values"], dtype=float)
            if raw.ndim != 1 or filtered.ndim != 1 or not np.all(np.isfinite(raw)) or not np.all(np.isfinite(filtered)):
                raise ValueError("tactile arrays must be finite one-dimensional values")
            thresholds = self.project_config["tactile"]["thresholds"]
            self.tactile_evidence_records.append(
                {
                    "timestamp_s": timestamp,
                    "received_timestamp_s": received,
                    "data_age_s": max(0.0, received - timestamp),
                    "frame_id": str(values["frame_id"]),
                    "frame_valid": str(values["frame_id"]) == self.config.frame_id,
                    "source": str(values["source"]),
                    "simulation_source": str(values["source"]) == "simulated",
                    "raw_force_n": float(raw[0]) if raw.size else float(values["force_magnitude"]),
                    "filtered_force_n": float(filtered[0]) if filtered.size else float(values["force_magnitude"]),
                    "force_magnitude_n": _number(values["force_magnitude"], "force magnitude"),
                    "clearance_m": _number(values["clearance_m"], "tactile clearance"),
                    "contact": bool(values["contact"]),
                    "warning": bool(values["warning"]),
                    "stop": bool(values["stop"]),
                    "valid": bool(values["valid"]),
                    "region": int(values["region"]),
                    "contact_on_n": float(thresholds["contact"]),
                    "contact_off_n": float(thresholds["contact_off"]),
                    "warning_on_n": float(thresholds["warning"]),
                    "warning_off_n": float(thresholds["warning_off"]),
                    "stop_on_n": float(thresholds["stop"]),
                    "stop_off_n": float(thresholds["stop_off"]),
                }
            )
        except (KeyError, TypeError, ValueError):
            self.slice_7g_tactile_invalid_count += 1

    def record_safety_evidence(self, **values: Any) -> None:
        if not self.config.diagnostic_data_collection or self.lifecycle_state != STATE_RECORDING:
            return
        try:
            self.safety_evidence_records.append(
                {
                    "timestamp_s": _number(values["timestamp"], "safety timestamp"),
                    "state": int(values["state"]),
                    "state_name": str(values["state_name"]),
                    "command_allowed": bool(values["command_allowed"]),
                    "emergency_stop": bool(values["emergency_stop"]),
                    "fault": bool(values["fault"]),
                    "valid": bool(values["valid"]),
                    "diagnostic_status": str(values["diagnostic_status"]),
                }
            )
        except (KeyError, TypeError, ValueError):
            self.slice_7g_safety_fault_count += 1

    def record_mppi_diagnostic(self, *, timestamp: Any, values: Mapping[str, str]) -> None:
        if not self.config.diagnostic_data_collection or self.lifecycle_state != STATE_RECORDING:
            return
        try:
            if values.get("schema_version") != "ctr_mppi_evaluation_iteration_v1" or values.get("valid") != "true":
                raise ValueError("unexpected MPPI diagnostic schema")
            record: dict[str, Any] = {"timestamp_s": _number(timestamp, "MPPI diagnostic timestamp")}
            for key, value in values.items():
                if key in {"schema_version", "valid"}:
                    continue
                record[key] = _number(value, f"MPPI diagnostic {key}")
            self.mppi_diagnostic_records.append(record)
            self.record_topic("/ctr/evaluation/mppi_diagnostics")
        except (TypeError, ValueError):
            self.invalid_mppi_diagnostic_count += 1

    def record_invalid_mppi_diagnostic(self) -> None:
        if self.config.diagnostic_data_collection and self.lifecycle_state == STATE_RECORDING:
            self.invalid_mppi_diagnostic_count += 1

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
        metadata: dict[str, Any] = {
            "run_id": self.run_id,
            "experiment_group": self.config.experiment_group,
            "experiment_name": self.experiment_name,
            "controller_label": self.config.controller_label,
            "baseline_label": self.config.baseline_label,
            "started_at": self.start_wall_time.isoformat() if self.start_wall_time else "",
            "stopped_at": self.stop_wall_time.isoformat() if self.stop_wall_time else "",
            "configured_duration": self.config.configured_duration,
            "actual_duration": actual_duration,
            "total_recording_duration_s": actual_duration,
            "recording_stop_time_s": self.stop_monotonic_time,
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
                "mppi_profile": self.config.mppi_parameters.get("active_profile", ""),
                "cylindrical_lumen": (
                    dataclass_to_plain(self.config.cylindrical_lumen)
                    if self.config.cylindrical_lumen is not None
                    else None
                ),
                "cylindrical_lumen_hash": (
                    stable_hash(dataclass_to_plain(self.config.cylindrical_lumen))
                    if self.config.cylindrical_lumen is not None
                    else ""
                ),
                "goal": {
                    "position": None if self.config.goal_position is None else self.config.goal_position.tolist(),
                    "tolerance": self.config.goal_tolerance,
                    "required_hold_duration": self.config.goal_required_hold_duration,
                },
                "goal_configuration_hash": stable_hash(
                    {
                        "position": None if self.config.goal_position is None else self.config.goal_position.tolist(),
                        "tolerance": self.config.goal_tolerance,
                        "required_hold_duration": self.config.goal_required_hold_duration,
                    }
                ),
                "configured_duration": self.config.configured_duration,
                "configured_control_period": self.config.thresholds.control_period,
                "reference_sample_period": self.config.reference_sample_period,
                "software_mode": self.config.software_mode,
                "diagnostic_data_collection": self.config.diagnostic_data_collection,
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
        metadata.update(self._configuration_hash_metadata())
        metadata.update(promoted_orchestration_metadata(self.metadata_override))
        metadata.update(self._command_guard_metadata(metadata))
        return metadata

    def _configuration_hash_metadata(self) -> dict[str, Any]:
        reference = self.project_config.get("reference", {})
        robot = self.project_config.get("robot", {})
        shared_environment = {
            "model_parameters": self.config.model_parameters,
            "simulation_parameters": self.project_config.get("simulation", {}),
            "cylindrical_lumen": (
                dataclass_to_plain(self.config.cylindrical_lumen)
                if self.config.cylindrical_lumen is not None
                else None
            ),
            "goal": {
                "position": None if self.config.goal_position is None else self.config.goal_position.tolist(),
                "tolerance": self.config.goal_tolerance,
                "required_hold_duration": self.config.goal_required_hold_duration,
            },
            "reference": {
                "trajectory_type": self.config.trajectory_type,
                "trajectory_parameters": self.config.trajectory_parameters,
                "frame_id": self.config.frame_id,
                "sample_period": self.config.reference_sample_period,
                "loop": reference.get("loop"),
                "completion_behavior": reference.get("completion_behavior"),
                "duration": reference.get("duration"),
            },
            "frames": robot.get("frames", {}),
            "software_mode": self.config.software_mode,
        }
        controller_configuration = {
            "controller_label": self.config.controller_label,
            "mppi_parameters": self.config.mppi_parameters,
        }
        orchestration_policy = self.metadata_override.get("orchestration_policy", {})
        return {
            "shared_environment_hash": self.metadata_override.get(
                "shared_environment_hash",
                stable_hash(shared_environment),
            ),
            "controller_configuration_hash": self.metadata_override.get(
                "controller_configuration_hash",
                stable_hash(controller_configuration),
            ),
            "orchestration_hash": self.metadata_override.get(
                "orchestration_hash",
                stable_hash(orchestration_policy),
            ),
        }

    def _command_guard_metadata(self, metadata: dict[str, Any]) -> dict[str, Any]:
        commands = sorted(self.safe_commands or self.raw_commands, key=lambda sample: sample.timestamp)
        command_zero_tolerance = float(metadata.get("command_zero_tolerance", 0.0) or 0.0)
        nonzero_count = sum(1 for sample in commands if float(np.linalg.norm(sample.command)) > command_zero_tolerance)
        result: dict[str, Any] = {
            "recorded_command_message_count": len(commands),
            "recorded_nonzero_command_count": nonzero_count,
        }
        run_role = metadata.get("run_role")
        if run_role == "baseline":
            result["baseline_command_message_count"] = len(commands)
            result["baseline_nonzero_command_count"] = nonzero_count
        if run_role == "candidate":
            first = commands[0] if commands else None
            recording_start = _optional_float(metadata.get("recording_start_time_s"))
            reference_epoch = _optional_float(metadata.get("scheduled_reference_epoch_s"))
            first_timestamp = None if first is None else float(first.timestamp)
            result.update(
                {
                    "candidate_first_command_timestamp": first_timestamp,
                    "candidate_first_command_timestamp_type": "command_message_timestamp" if first is not None else "",
                    "candidate_command_after_recording": (
                        False
                        if first_timestamp is None or recording_start is None
                        else first_timestamp >= recording_start
                    ),
                    "candidate_command_before_reference_epoch": (
                        False
                        if first_timestamp is None or reference_epoch is None
                        else first_timestamp < reference_epoch
                    ),
                    "candidate_command_at_or_after_reference_epoch": (
                        False
                        if first_timestamp is None or reference_epoch is None
                        else first_timestamp >= reference_epoch
                    ),
                }
            )
        return result

    def _states_for_evaluation_window(self, metadata: dict[str, Any]) -> list[TimedState]:
        window_start = _optional_float(metadata.get("evaluation_window_start_time_s"))
        window_end = _optional_float(metadata.get("evaluation_window_end_time_s"))
        if window_start is None or window_end is None:
            return list(self.states)
        if window_end < window_start:
            return []
        return [sample for sample in self.states if window_start <= sample.timestamp <= window_end]

    def _summary(
        self,
        *,
        alignment: AlignmentResult,
        metadata: dict[str, Any],
        lumen_result: LumenRecorderResult | None = None,
    ) -> dict[str, Any]:
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
        slice_7g_profile = self.project_config.get("runtime", {}).get("slice_7g_profile") is True
        missing_topic_count = sum(
            1 for topic in required_topics(slice_7g_profile=slice_7g_profile)
            if self.topic_counts.get(topic, 0) == 0
        )
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
            missing_backbone_sample_count=sum(1 for sample in alignment.samples if sample.backbone_points is None),
        )
        goal_metrics = None
        lumen_safety = None
        motion = None
        if self.config.cylindrical_lumen is not None and self.config.goal_position is not None:
            goal_metrics = compute_goal_metrics(
                times=timestamps,
                tip_positions=arrays["tip_positions"],
                goal_position=self.config.goal_position,
                tolerance=float(self.config.goal_tolerance),
                required_hold_duration=float(self.config.goal_required_hold_duration),
            )
            lumen_safety = compute_lumen_safety_metrics(
                times=timestamps,
                backbone_points=[sample.backbone_points for sample in alignment.samples],
                lumen=self.config.cylindrical_lumen,
            )
            motion = compute_motion_metrics(
                times=timestamps,
                tip_positions=arrays["tip_positions"],
                q_values=arrays["q"],
                goal_position=self.config.goal_position,
                control=control,
            )
        elif _is_curved_lumen_run(metadata):
            curved_goal = _curved_executed_target(metadata)
            if curved_goal is not None:
                goal_metrics = compute_goal_metrics(
                    times=timestamps,
                    tip_positions=arrays["tip_positions"],
                    goal_position=curved_goal,
                    tolerance=_positive_number(self.project_config["goal"]["tolerance"], "goal.tolerance"),
                    required_hold_duration=_nonnegative_number(
                        self.project_config["goal"]["required_hold_duration"],
                        "goal.required_hold_duration",
                    ),
                )
                motion = compute_motion_metrics(
                    times=timestamps,
                    tip_positions=arrays["tip_positions"],
                    q_values=arrays["q"],
                    goal_position=curved_goal,
                    control=control,
                )
        acceptance = compute_acceptance(
            tracking=tracking,
            control=control,
            timing=timing,
            numerical_safety=numerical,
            data_quality=data_quality,
            thresholds=self.config.thresholds,
            baseline_improvement_valid=not self.config.baseline_result_dir,
            goal=goal_metrics,
            lumen_safety=lumen_safety,
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
        interrupted = bool(metadata.get("interrupted", False))
        summary["run_status"] = {
            "status": "incomplete" if interrupted else "completed",
            "interrupted": interrupted,
            "completed_evaluation_window": not interrupted,
        }
        fixed_target_reference = (
            _metadata_value(metadata, "target_mode") == REFERENCE_MODE_FIXED_TARGET
        )
        target_selection = _metadata_value(metadata, "development_target_selection")
        summary["metric_semantics"] = {
            "tracking_rmse_name": (
                "tip_to_target_rmse_m" if fixed_target_reference else "reference_tracking_rmse_m"
            ),
            "tracking_rmse_formula": "sqrt(mean(||tip_i-reference_i||_2^2))",
            "tracking_rmse_units": "m",
            "reference_pose_count": (
                target_selection.get("reference_pose_count")
                if isinstance(target_selection, dict)
                else None
            ),
        }
        if goal_metrics is not None:
            summary["goal"] = dataclass_to_plain(goal_metrics)
            summary["goal"]["tip_to_target_rmse_m"] = float(goal_metrics.rmse)
        if lumen_safety is not None:
            summary["lumen_safety"] = dataclass_to_plain(lumen_safety)
        if motion is not None:
            summary["motion"] = dataclass_to_plain(motion)
        summary["alignment_rejection_reasons"] = alignment.diagnostics.rejection_reasons
        summary["topic_status"] = self._topic_status()
        if slice_7g_profile:
            summary["slice_7g_safety"] = {
                "fault_count": int(self.slice_7g_safety_fault_count),
            }
            summary["slice_7g_tactile"] = {
                "invalid_sample_count": int(self.slice_7g_tactile_invalid_count),
            }
        if lumen_result is not None and lumen_result.required and lumen_result.section is not None:
            summary["lumen_evaluation"] = lumen_result.section
            summary["navigation"] = _curved_navigation_summary(
                lumen_result.section,
                goal_metrics,
                completed=not interrupted,
            )
            summary["acceptance"] = _acceptance_with_curved_lumen(
                summary["acceptance"],
                lumen_result.section,
                goal_metrics,
                completed=not interrupted,
            )
        elif interrupted:
            acceptance = dict(summary["acceptance"])
            reasons = list(acceptance.get("reasons", []))
            acceptance["functional_pass"] = False
            _append_reason_once(
                reasons,
                "evaluation was interrupted before the configured window completed",
            )
            acceptance["reasons"] = reasons
            summary["acceptance"] = acceptance
        summary["paper_metrics"] = self._paper_metrics(
            alignment=alignment,
            metadata=metadata,
            lumen_result=lumen_result,
        )
        return summary

    def _paper_metrics(
        self,
        *,
        alignment: AlignmentResult,
        metadata: dict[str, Any],
        lumen_result: LumenRecorderResult | None,
    ) -> dict[str, Any]:
        samples = alignment.samples
        tip = np.asarray([sample.tip_position for sample in samples], dtype=float)
        path_length = (
            float(np.sum(np.linalg.norm(np.diff(tip, axis=0), axis=1)))
            if tip.shape[0] > 1
            else 0.0
        )
        selected = sorted(self.safe_commands or self.raw_commands, key=lambda item: item.timestamp)
        commands = np.asarray([sample.command for sample in selected], dtype=float)
        insertion_variation = (
            float(np.sum(np.linalg.norm(np.diff(commands[:, :3], axis=0), axis=1)))
            if commands.shape[0] > 1
            else 0.0
        )
        rotation_variation = (
            float(np.sum(np.linalg.norm(np.diff(commands[:, 3:], axis=0), axis=1)))
            if commands.shape[0] > 1
            else 0.0
        )
        if commands.size:
            saturated = np.isclose(np.abs(commands), self.config.command_limits[None, :], rtol=0.0, atol=1.0e-12)
            insertion_saturation = float(100.0 * np.mean(np.any(saturated[:, :3], axis=1)))
            rotation_saturation = float(100.0 * np.mean(np.any(saturated[:, 3:], axis=1)))
        else:
            insertion_saturation = rotation_saturation = 0.0
        result: dict[str, Any] = {
            "schema_version": "ctr_final_system_run_metrics_v1",
            "cartesian_path_length_m": path_length,
            "cumulative_control_effort_formula": "sum_i(||u_i||_2^2 * dt_i)",
            "cumulative_control_effort_units": "mixed_command_units_squared_second",
            "insertion_total_variation_m_per_s": insertion_variation,
            "rotation_total_variation_rad_per_s": rotation_variation,
            "insertion_saturation_percentage": insertion_saturation,
            "rotation_saturation_percentage": rotation_saturation,
            "tip_sample_count": len(self.tip_records),
            "solve_sample_count": len(self.solves),
            "aligned_sample_count": len(samples),
            "requested_runtime_s": metadata.get("requested_evaluation_duration_s"),
            "configured_runtime_s": metadata.get("configured_duration"),
            "actual_runtime_s": metadata.get("actual_duration"),
            "diagnostic_data_collection": self.config.diagnostic_data_collection,
            "invalid_mppi_diagnostic_count": self.invalid_mppi_diagnostic_count,
        }
        if lumen_result is not None and lumen_result.csv_rows:
            rows = list(lumen_result.csv_rows)
            clearance = np.asarray([float(row["physical_clearance_m"]) for row in rows])
            radial = np.asarray([float(row["radial_offset_m"]) for row in rows])
            stamps = np.asarray([float(row["timestamp_s"]) for row in rows])
            margin = np.asarray([bool(row["safety_margin_violation"]) for row in rows])
            collision = np.asarray([bool(row["physical_collision"]) for row in rows])
            minimum_index = int(np.argmin(clearance))
            maximum_index = int(np.argmax(radial))
            result.update(
                {
                    "maximum_centerline_distance_m": float(radial[maximum_index]),
                    "maximum_centerline_distance_timestamp_s": float(stamps[maximum_index]),
                    "minimum_whole_backbone_clearance_m": float(clearance[minimum_index]),
                    "minimum_whole_backbone_clearance_timestamp_s": float(stamps[minimum_index]),
                    "safety_margin_crossing_count": _transition_count(margin),
                    "safety_margin_violation_duration_s": _flag_duration(stamps, margin),
                    "collision_transition_count": _transition_count(collision),
                }
            )
        if self.config.diagnostic_data_collection:
            tactile_rows = sorted(self.tactile_evidence_records, key=lambda row: row["timestamp_s"])
            safety_rows = sorted(self.safety_evidence_records, key=lambda row: row["timestamp_s"])
            for name in ("contact", "warning", "stop"):
                flags = np.asarray([bool(row[name]) for row in tactile_rows])
                stamps = np.asarray([float(row["timestamp_s"]) for row in tactile_rows])
                result[f"tactile_{name}_event_count"] = _transition_count(flags)
                result[f"tactile_{name}_duration_s"] = _flag_duration(stamps, flags)
            for name, predicate in (
                ("invalid", lambda row: not bool(row["valid"])),
                ("stale", lambda row: float(row["data_age_s"]) > float(self.project_config["safety"]["tactile_timeout"])),
            ):
                flags = np.asarray([predicate(row) for row in tactile_rows])
                stamps = np.asarray([float(row["timestamp_s"]) for row in tactile_rows])
                result[f"tactile_{name}_event_count"] = _transition_count(flags)
                result[f"tactile_{name}_duration_s"] = _flag_duration(stamps, flags)
            safety_stamps = np.asarray([float(row["timestamp_s"]) for row in safety_rows])
            for name, predicate in (
                ("scaling", lambda row: row["state_name"] == "warning"),
                ("stop", lambda row: bool(row["emergency_stop"])),
                ("latch", lambda row: bool(row["fault"])),
            ):
                flags = np.asarray([predicate(row) for row in safety_rows])
                result[f"safety_{name}_event_count"] = _transition_count(flags)
                result[f"safety_{name}_duration_s"] = _flag_duration(safety_stamps, flags)
        return result

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

    def _lumen_evaluation_result(self, *, alignment: AlignmentResult, metadata: dict[str, Any]) -> LumenRecorderResult:
        if not _has_curved_lumen_metadata(metadata):
            return LumenRecorderResult(required=False)

        identity, identity_reasons = _curved_lumen_identity(metadata)
        data_quality = _base_lumen_data_quality(alignment.samples)
        if identity_reasons:
            return LumenRecorderResult(
                required=True,
                section=_unavailable_lumen_section(identity=identity, data_quality=data_quality, reasons=identity_reasons),
            )

        backbone_data, backbone_reasons = _curved_lumen_backbone_data(alignment.samples)
        data_quality = backbone_data.data_quality
        if backbone_reasons:
            return LumenRecorderResult(
                required=True,
                section=_unavailable_lumen_section(identity=identity, data_quality=data_quality, reasons=backbone_reasons),
            )

        try:
            geometry, geometry_payload, reconstructed_fingerprint = self._reconstruct_lumen_geometry(identity)
        except Exception as exc:
            return LumenRecorderResult(
                required=True,
                section=_unavailable_lumen_section(
                    identity=identity,
                    data_quality=data_quality,
                    reasons=[f"geometry_construction_failed: {exc}"],
                ),
            )

        expected_fingerprint = str(identity["geometry_fingerprint"])
        geometry_frame = str(getattr(geometry, "frame_id", ""))
        if geometry_frame != identity["geometry_frame"]:
            updated = dict(identity)
            updated["reconstructed_geometry_fingerprint"] = reconstructed_fingerprint
            updated["geometry_fingerprint_match"] = False
            return LumenRecorderResult(
                required=True,
                section=_unavailable_lumen_section(
                    identity=updated,
                    data_quality={**data_quality, "geometry_fingerprint_match": False},
                    reasons=[
                        "geometry_fingerprint_mismatch: reconstructed geometry frame does not match metadata"
                    ],
                ),
            )
        if reconstructed_fingerprint != expected_fingerprint:
            updated = dict(identity)
            updated["reconstructed_geometry_fingerprint"] = reconstructed_fingerprint
            updated["geometry_fingerprint_match"] = False
            return LumenRecorderResult(
                required=True,
                section=_unavailable_lumen_section(
                    identity=updated,
                    data_quality={**data_quality, "geometry_fingerprint_match": False},
                    reasons=[
                        "geometry_fingerprint_mismatch: reconstructed geometry fingerprint does not match metadata"
                    ],
                ),
            )

        clearance_start_ns = time.monotonic_ns()
        self.record_diagnostic_event(
            "metric_calculation.lumen_clearance_and_progress",
            phase="start",
            status="started",
            details={
                "aligned_count": len(backbone_data.backbones),
                "backbone_point_count_total": sum(len(points) for points in backbone_data.backbones),
                "backbone_point_count_max": max((len(points) for points in backbone_data.backbones), default=0),
                "geometry_point_count": len(getattr(geometry, "centerline_points", [])),
            },
        )
        clearance_status = "ok"
        try:
            metrics = compute_lumen_evaluation_metrics(
                geometry=geometry,
                times=backbone_data.timestamps,
                backbone_points=backbone_data.backbones,
                tip_points=backbone_data.tip_points,
                # This is a descriptive geometric metric for every lumen run; it is
                # independent of whether the selected target lies on the centerline.
                compute_centerline_tracking_rmse=True,
                tip_backbone_tolerance=TIP_BACKBONE_CONSISTENCY_TOLERANCE,
            )
        except Exception as exc:
            clearance_status = "error"
            self._record_metric_stage_end(
                "lumen_clearance_and_progress",
                clearance_start_ns,
                status=clearance_status,
                details={
                    "aligned_count": len(backbone_data.backbones),
                    "backbone_point_count_total": sum(len(points) for points in backbone_data.backbones),
                    "backbone_point_count_max": max((len(points) for points in backbone_data.backbones), default=0),
                    "geometry_point_count": len(getattr(geometry, "centerline_points", [])),
                },
            )
            updated = dict(identity)
            updated["reconstructed_geometry_fingerprint"] = reconstructed_fingerprint
            updated["geometry_fingerprint_match"] = True
            return LumenRecorderResult(
                required=True,
                section=_unavailable_lumen_section(
                    identity=updated,
                    data_quality={
                        **data_quality,
                        "geometry_fingerprint_match": True,
                        "metric_computation_success": False,
                    },
                    reasons=[f"lumen_metric_computation_failed: {exc}"],
                ),
            )
        except BaseException:
            clearance_status = "error"
            self._record_metric_stage_end(
                "lumen_clearance_and_progress",
                clearance_start_ns,
                status=clearance_status,
                details={
                    "aligned_count": len(backbone_data.backbones),
                    "backbone_point_count_total": sum(len(points) for points in backbone_data.backbones),
                    "backbone_point_count_max": max((len(points) for points in backbone_data.backbones), default=0),
                    "geometry_point_count": len(getattr(geometry, "centerline_points", [])),
                },
            )
            raise

        self._record_metric_stage_end(
            "lumen_clearance_and_progress",
            clearance_start_ns,
            status=clearance_status,
            details={
                "aligned_count": len(backbone_data.backbones),
                "backbone_point_count_total": sum(len(points) for points in backbone_data.backbones),
                "backbone_point_count_max": max((len(points) for points in backbone_data.backbones), default=0),
                "geometry_point_count": len(getattr(geometry, "centerline_points", [])),
            },
        )

        updated_identity = dict(identity)
        updated_identity["reconstructed_geometry_fingerprint"] = reconstructed_fingerprint
        updated_identity["geometry_fingerprint_match"] = True
        section = _available_lumen_section(
            identity=updated_identity,
            geometry=geometry,
            geometry_payload=geometry_payload,
            metrics=metrics,
            data_quality={
                **data_quality,
                "geometry_fingerprint_match": True,
                "metric_computation_success": True,
            },
        )
        return LumenRecorderResult(
            required=True,
            section=section,
            csv_rows=tuple(_lumen_sample_csv_rows(metrics)),
        )

    def _reconstruct_lumen_geometry(
        self,
        identity: dict[str, Any],
    ) -> tuple[Any, dict[str, Any], str]:
        effective_config = config_with_lumen_overrides(
            self.project_config,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type=str(identity["curved_lumen_type"]),
            target=identity["executed_target"],
        )
        effective_config.setdefault("reference", {})["mode"] = REFERENCE_MODE_FIXED_TARGET
        geometry = lumen_geometry_from_config(effective_config)
        if geometry is None:
            raise ValueError("effective curved configuration did not produce a lumen geometry")
        payload = lumen_geometry_fingerprint_payload(geometry)
        fingerprint = lumen_geometry_fingerprint(geometry)
        return geometry, payload, fingerprint

    def _write_lumen_evaluation_csv_if_available(
        self,
        run_dir: Path,
        result: LumenRecorderResult,
    ) -> LumenRecorderResult:
        if not result.required or result.section is None or not result.csv_rows:
            return result
        try:
            self._write_lumen_evaluation_csv(run_dir, result.csv_rows)
        except Exception as exc:
            identity = dict(result.section.get("identity", {}))
            data_quality = dict(result.section.get("data_quality", {}))
            data_quality["metric_computation_success"] = False
            data_quality["lumen_csv_written"] = False
            section = _unavailable_lumen_section(
                identity=identity,
                data_quality=data_quality,
                reasons=[f"lumen_csv_write_failed: {exc}"],
            )
            return LumenRecorderResult(required=True, section=section)
        data_quality = dict(result.section.get("data_quality", {}))
        data_quality["lumen_csv_written"] = True
        section = dict(result.section)
        section["data_quality"] = data_quality
        return LumenRecorderResult(required=True, section=section, csv_rows=result.csv_rows)

    def _write_lumen_evaluation_csv(self, run_dir: Path, rows: tuple[dict[str, Any], ...]) -> None:
        destination = run_dir / "lumen_evaluation.csv"
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                newline="",
                dir=run_dir,
                prefix=f".{destination.name}.",
                suffix=".tmp",
                delete=False,
            ) as handle:
                temporary_path = Path(handle.name)
                writer = csv.writer(handle)
                writer.writerow(LUMEN_EVALUATION_CSV_FIELDS)
                for row in rows:
                    writer.writerow([row.get(field, "") for field in LUMEN_EVALUATION_CSV_FIELDS])
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, destination)
            temporary_path = None
        finally:
            if temporary_path is not None:
                try:
                    temporary_path.unlink()
                except FileNotFoundError:
                    pass

    def _write_raw_files(self, run_dir: Path) -> None:
        write_rows(
            run_dir / "state.csv",
            ["timestamp", "q0", "q1", "q2", "q3", "q4", "q5", "q_dot0", "q_dot1", "q_dot2", "q_dot3", "q_dot4", "q_dot5", "tip_x", "tip_y", "tip_z"],
            [
                [sample.timestamp, *sample.q.tolist(), *sample.q_dot.tolist(), *sample.tip_position.tolist()]
                for sample in sorted(self.states, key=lambda item: item.timestamp)
            ],
        )
        write_rows(
            run_dir / "tip.csv",
            ["timestamp", "x", "y", "z"],
            sorted(self.tip_records, key=lambda item: item["timestamp"]),
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
                for sample in sorted(self.references, key=lambda item: item.timestamp)
            ],
        )
        selected_commands = self.safe_commands if self.safe_commands else self.raw_commands
        write_rows(
            run_dir / "command.csv",
            ["timestamp", "source", "u0", "u1", "u2", "u3", "u4", "u5", "saturated"],
            [
                [sample.timestamp, sample.source, *sample.command.tolist(), sample.saturated]
                for sample in sorted(selected_commands, key=lambda item: item.timestamp)
            ],
        )
        write_rows(
            run_dir / "solve_timing.csv",
            ["timestamp", "solve_time", "saturated"],
            [
                [sample.timestamp, sample.solve_time, sample.saturated]
                for sample in sorted(self.solves, key=lambda item: item.timestamp)
            ],
        )
        write_rows(
            run_dir / "horizon.csv",
            ["timestamp", "count", "first_x", "first_y", "first_z", "final_x", "final_y", "final_z"],
            sorted(self.horizon_records, key=lambda item: item["timestamp"]),
        )
        write_rows(
            run_dir / "reference_path.csv",
            ["timestamp", "count"],
            sorted(self.path_records, key=lambda item: item["timestamp"]),
        )
        write_rows(
            run_dir / "backbone.csv",
            ["timestamp", "index", "x", "y", "z"],
            sorted(
                self.backbone_records,
                key=lambda item: (item["timestamp"], item["index"]),
            ),
        )
        if self.config.cylindrical_lumen is not None and self.config.goal_position is not None:
            write_rows(
                run_dir / "cylinder_navigation.csv",
                [
                    "timestamp",
                    "tip_to_goal_error",
                    "minimum_backbone_clearance",
                    "minimum_axial_end_cap_clearance",
                    "collision",
                    "safety_margin_violation",
                    "closest_backbone_point_index",
                ],
                self._cylinder_rows(),
            )
        if self.config.diagnostic_data_collection:
            tactile_safety_rows = self._tactile_safety_rows()
            write_rows(
                run_dir / "tactile_safety.csv",
                TACTILE_SAFETY_CSV_FIELDS,
                ([row.get(field, "") for field in TACTILE_SAFETY_CSV_FIELDS] for row in tactile_safety_rows),
            )
            write_rows(
                run_dir / "mppi_cost_terms.csv",
                MPPI_COST_CSV_FIELDS,
                ([row.get(field, "") for field in MPPI_COST_CSV_FIELDS] for row in self.mppi_diagnostic_records),
            )
            write_rows(
                run_dir / "mppi_computation.csv",
                MPPI_TIMING_CSV_FIELDS,
                ([row.get(field, "") for field in MPPI_TIMING_CSV_FIELDS] for row in self.mppi_diagnostic_records),
            )

    def _tactile_safety_rows(self) -> list[dict[str, Any]]:
        events = [
            (float(row["timestamp_s"]), "tactile", row)
            for row in self.tactile_evidence_records
        ] + [
            (float(row["timestamp_s"]), "safety", row)
            for row in self.safety_evidence_records
        ]
        events.sort(key=lambda item: (item[0], item[1]))
        tactile: dict[str, Any] = {}
        safety: dict[str, Any] = {}
        rows: list[dict[str, Any]] = []
        raw_commands = sorted(self.raw_commands, key=lambda item: item.timestamp)
        safe_commands = sorted(self.safe_commands, key=lambda item: item.timestamp)
        for timestamp, kind, event in events:
            if kind == "tactile":
                tactile = event
            else:
                safety = event
            raw = _latest_command_at_or_before(raw_commands, timestamp)
            safe = _latest_command_at_or_before(safe_commands, timestamp)
            raw_values = np.zeros(6, dtype=float) if raw is None else raw.command
            safe_values = np.zeros(6, dtype=float) if safe is None else safe.command
            raw_norm = float(np.linalg.norm(raw_values))
            safe_norm = float(np.linalg.norm(safe_values))
            scale = 0.0 if raw_norm <= 1.0e-15 else min(1.0, safe_norm / raw_norm)
            row: dict[str, Any] = {
                "timestamp_s": timestamp,
                "event_type": kind,
                **tactile,
                **{f"commanded_u{index}": float(value) for index, value in enumerate(raw_values)},
                **{f"safe_u{index}": float(value) for index, value in enumerate(safe_values)},
                "commanded_norm": raw_norm,
                "safe_norm": safe_norm,
                "applied_scale": scale,
                "command_gated": bool(safety) and not bool(safety.get("command_allowed", False)),
                "safety_state": safety.get("state", ""),
                "safety_state_name": safety.get("state_name", ""),
                "safety_command_allowed": safety.get("command_allowed", ""),
                "safety_emergency_stop": safety.get("emergency_stop", ""),
                "safety_fault": safety.get("fault", ""),
                "safety_valid": safety.get("valid", ""),
                "safety_reason": safety.get("diagnostic_status", ""),
            }
            rows.append(row)
        return rows

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

    def _cylinder_rows(self) -> list[list[Any]]:
        if self.config.cylindrical_lumen is None or self.config.goal_position is None:
            return []
        rows: list[list[Any]] = []
        for sample in self.states:
            if sample.backbone_points is None:
                rows.append([sample.timestamp, "", "", "", True, True, -1])
                continue
            clearance = self.config.cylindrical_lumen.backbone_clearance(sample.backbone_points)
            tip_error = float(np.linalg.norm(sample.tip_position - self.config.goal_position))
            rows.append(
                [
                    sample.timestamp,
                    tip_error,
                    clearance.minimum_radial_clearance,
                    clearance.minimum_axial_clearance,
                    clearance.collision_count > 0,
                    clearance.safety_margin_violation_count > 0,
                    clearance.closest_backbone_point_index,
                ]
            )
        return rows


def _has_curved_lumen_metadata(metadata: dict[str, Any]) -> bool:
    return _canonical_orchestration_task(metadata) == TASK_CURVED_LUMEN_NAVIGATION


def _is_curved_lumen_run(metadata: dict[str, Any]) -> bool:
    return _canonical_orchestration_task(metadata) == TASK_CURVED_LUMEN_NAVIGATION


def _canonical_orchestration_task(metadata: dict[str, Any]) -> Any:
    return metadata.get("task")


def _curved_lumen_identity(metadata: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    reasons: list[str] = []
    identity: dict[str, Any] = {
        "task": _canonical_orchestration_task(metadata),
        "reference_mode": _metadata_value(metadata, "reference_mode"),
        "target_mode": _metadata_value(metadata, "target_mode"),
        "curved_lumen_type": _metadata_value(metadata, "curved_lumen_type"),
        "scenario_id": _metadata_value(metadata, "scenario_id"),
        "scenario_policy_version": _metadata_value(metadata, "scenario_policy_version"),
        "scenario_fingerprint": _metadata_value(metadata, "scenario_fingerprint"),
        "geometry_frame": _metadata_value(metadata, "geometry_frame"),
        "geometry_fingerprint": _metadata_value(metadata, "geometry_fingerprint"),
        "expected_geometry_fingerprint": _metadata_value(metadata, "geometry_fingerprint"),
        "reconstructed_geometry_fingerprint": None,
        "geometry_fingerprint_match": False,
        "shared_environment_hash": _metadata_value(metadata, "shared_environment_hash"),
        "run_role": _metadata_value(metadata, "run_role"),
        "development_simulation": _metadata_value(metadata, "development_simulation"),
        "production_promotion_evidence": _metadata_value(
            metadata, "production_promotion_evidence"
        ),
        "development_disclaimer": _metadata_value(metadata, "development_disclaimer"),
        "development_target_selection": _metadata_value(
            metadata, "development_target_selection"
        ),
        "derived_target": None,
        "requested_target": None,
        "executed_target": None,
        "validated_target": None,
        "centerline_fraction": None,
        "centerline_arc_length": None,
        "radial_offset": None,
        "override_used": None,
    }
    for key in (
        "task",
        "reference_mode",
        "target_mode",
        "curved_lumen_type",
        "scenario_id",
        "scenario_policy_version",
        "scenario_fingerprint",
        "geometry_frame",
        "geometry_fingerprint",
    ):
        value = identity[key]
        if value is None or str(value) == "":
            reasons.append(f"missing_curved_identity:{key}")
            identity[key] = None
        else:
            identity[key] = str(value)

    if identity["task"] is not None and identity["task"] != TASK_CURVED_LUMEN_NAVIGATION:
        reasons.append("invalid_curved_identity:task")
    if identity["reference_mode"] is not None and identity["reference_mode"] != REFERENCE_MODE_FIXED_TARGET:
        reasons.append("invalid_curved_identity:reference_mode")
    if identity["target_mode"] is not None and identity["target_mode"] != REFERENCE_MODE_FIXED_TARGET:
        reasons.append("invalid_curved_identity:target_mode")
    if identity["curved_lumen_type"] is not None and identity["curved_lumen_type"] not in CURVED_LUMEN_TYPES:
        reasons.append("invalid_curved_identity:curved_lumen_type")
    if identity["scenario_id"] is not None and identity["scenario_id"] not in CURVED_LUMEN_SCENARIO_IDS:
        reasons.append("invalid_curved_identity:scenario_id")
    if (
        identity["scenario_policy_version"] is not None
        and identity["scenario_policy_version"] != CURVED_SCENARIO_POLICY_VERSION
    ):
        reasons.append("invalid_curved_identity:scenario_policy_version")

    for key in ("derived_target", "requested_target", "executed_target", "validated_target"):
        raw_value = _metadata_value(metadata, key)
        if raw_value is None:
            reasons.append(f"missing_curved_identity:{key}")
            continue
        try:
            identity[key] = _vector3_payload(raw_value, key)
        except ValueError:
            reasons.append(f"invalid_curved_identity:{key}")

    override_value = _metadata_value(metadata, "override_used")
    if override_value is None:
        override_value = _metadata_value(metadata, "target_override_used")
    if override_value is None:
        reasons.append("missing_curved_identity:override_used")
    elif not isinstance(override_value, bool):
            reasons.append("invalid_curved_identity:override_used")
    else:
        identity["override_used"] = bool(override_value)

    for key in ("centerline_fraction", "centerline_arc_length", "radial_offset"):
        raw_value = _metadata_value(metadata, key)
        if raw_value is None:
            reasons.append(f"missing_curved_identity:{key}")
            continue
        if isinstance(raw_value, bool):
            reasons.append(f"invalid_curved_identity:{key}")
            continue
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            reasons.append(f"invalid_curved_identity:{key}")
            continue
        if not math.isfinite(value):
            reasons.append(f"invalid_curved_identity:{key}")
            continue
        identity[key] = value

    if identity["executed_target"] is not None and identity["validated_target"] is not None:
        if not _vectors_close(identity["executed_target"], identity["validated_target"], tolerance=1.0e-12):
            reasons.append("invalid_curved_identity:executed_validated_target_mismatch")
    return identity, reasons


def _curved_executed_target(metadata: dict[str, Any]) -> np.ndarray | None:
    raw_value = _metadata_value(metadata, "executed_target")
    if raw_value is None:
        raw_value = _metadata_value(metadata, "validated_target")
    try:
        return np.asarray(_vector3_payload(raw_value, "executed_target"), dtype=float)
    except (TypeError, ValueError):
        return None


def _metadata_value(metadata: dict[str, Any], key: str) -> Any:
    if key in metadata:
        return metadata[key]
    override = metadata.get("metadata_override")
    if isinstance(override, dict):
        if key in override:
            return override[key]
        nested = _metadata_value(override, key)
        if nested is not None:
            return nested
    reference = metadata.get("reference_configuration")
    if isinstance(reference, dict):
        if key in reference:
            return reference[key]
        scenario = reference.get("curved_scenario")
        if isinstance(scenario, dict) and key in scenario:
            return scenario[key]
    scenario = metadata.get("curved_scenario")
    if isinstance(scenario, dict) and key in scenario:
        return scenario[key]
    runtime = metadata.get("orchestration_runtime")
    if isinstance(runtime, dict) and key in runtime:
        return runtime[key]
    return None


def _vector3_payload(value: Any, label: str) -> list[float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must be finite with shape (3,)")
    return [float(item) for item in array]


def _vectors_close(left: Any, right: Any, *, tolerance: float) -> bool:
    return bool(np.allclose(np.asarray(left, dtype=float), np.asarray(right, dtype=float), atol=tolerance, rtol=0.0))


def _base_lumen_data_quality(samples: list[Any]) -> dict[str, Any]:
    return {
        "aligned_sample_count": len(samples),
        "backbone_sample_count": 0,
        "missing_backbone_count": 0,
        "malformed_backbone_count": 0,
        "nonfinite_backbone_count": 0,
        "minimum_backbone_points": None,
        "maximum_backbone_points": None,
        "timestamps_monotonic": True,
        "duplicate_timestamp_count": 0,
        "tip_backbone_consistent": True,
        "tip_backbone_mismatch_count": 0,
        "geometry_identity_available": False,
        "geometry_fingerprint_match": False,
        "target_identity_available": False,
        "metric_computation_success": False,
        "lumen_csv_written": False,
    }


def _curved_lumen_backbone_data(samples: list[Any]) -> tuple[LumenBackboneData, list[str]]:
    reasons: list[str] = []
    data_quality = _base_lumen_data_quality(samples)
    timestamps: list[float] = []
    backbones: list[np.ndarray] = []
    tips: list[np.ndarray] = []
    point_counts: list[int] = []

    if not samples:
        reasons.append("missing_backbone_data:no_aligned_samples")

    previous_timestamp: float | None = None
    for index, sample in enumerate(samples):
        timestamp = float(sample.timestamp)
        timestamps.append(timestamp)
        if not math.isfinite(timestamp):
            reasons.append("nonfinite_timestamp")
        if previous_timestamp is not None:
            if timestamp < previous_timestamp:
                data_quality["timestamps_monotonic"] = False
                reasons.append("nonmonotonic_timestamps")
            elif timestamp == previous_timestamp:
                data_quality["duplicate_timestamp_count"] += 1
        previous_timestamp = timestamp

        points = sample.backbone_points
        if points is None:
            data_quality["missing_backbone_count"] += 1
            reasons.append("missing_backbone_data")
            continue
        try:
            backbone = np.asarray(points, dtype=float)
        except (TypeError, ValueError):
            data_quality["malformed_backbone_count"] += 1
            reasons.append("malformed_backbone_data")
            continue
        if backbone.ndim != 2 or backbone.shape[1] != 3 or backbone.shape[0] < 1:
            data_quality["malformed_backbone_count"] += 1
            reasons.append("malformed_backbone_data")
            continue
        if not np.all(np.isfinite(backbone)):
            data_quality["nonfinite_backbone_count"] += 1
            reasons.append("nonfinite_backbone_data")
            continue
        try:
            tip = np.asarray(sample.tip_position, dtype=float)
        except (TypeError, ValueError):
            data_quality["malformed_backbone_count"] += 1
            reasons.append("malformed_tip_data")
            continue
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            data_quality["malformed_backbone_count"] += 1
            reasons.append("malformed_tip_data")
            continue
        if not np.allclose(tip, backbone[-1], atol=TIP_BACKBONE_CONSISTENCY_TOLERANCE, rtol=0.0):
            data_quality["tip_backbone_consistent"] = False
            data_quality["tip_backbone_mismatch_count"] += 1
            reasons.append("tip_backbone_mismatch")
        backbones.append(backbone.astype(float, copy=True))
        tips.append(tip.astype(float, copy=True))
        point_counts.append(int(backbone.shape[0]))

    if len(timestamps) != len(backbones):
        reasons.append("missing_backbone_data:timestamp_backbone_count_mismatch")
    if point_counts:
        data_quality["backbone_sample_count"] = len(backbones)
        data_quality["minimum_backbone_points"] = min(point_counts)
        data_quality["maximum_backbone_points"] = max(point_counts)
    if not backbones and "missing_backbone_data:no_aligned_samples" not in reasons:
        reasons.append("missing_backbone_data")

    data_quality["geometry_identity_available"] = True
    data_quality["target_identity_available"] = True
    unique_reasons = _unique_reasons(reasons)
    return (
        LumenBackboneData(
            timestamps=np.asarray(timestamps, dtype=float),
            backbones=tuple(backbones),
            tip_points=np.asarray(tips, dtype=float),
            data_quality=data_quality,
        ),
        unique_reasons,
    )


def _available_lumen_section(
    *,
    identity: dict[str, Any],
    geometry: Any,
    geometry_payload: dict[str, Any],
    metrics: LumenEvaluationMetrics,
    data_quality: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": LUMEN_EVALUATION_SCHEMA_VERSION,
        "available": True,
        "run_valid": True,
        "unavailable_reasons": [],
        "identity": _lumen_identity_section(identity),
        "geometry": _lumen_geometry_section(identity, geometry, geometry_payload),
        "data_quality": data_quality,
        "physical_safety": _physical_safety_section(metrics),
        "safety_margin": _safety_margin_section(metrics),
        "constraints": _constraints_section(metrics),
        "progress": _progress_section(metrics),
    }


def _unavailable_lumen_section(
    *,
    identity: dict[str, Any],
    data_quality: dict[str, Any],
    reasons: list[str],
) -> dict[str, Any]:
    clean_reasons = _unique_reasons(reasons)
    section_identity = _lumen_identity_section(identity)
    section_identity.setdefault("geometry_fingerprint_match", False)
    return {
        "schema_version": LUMEN_EVALUATION_SCHEMA_VERSION,
        "available": False,
        "run_valid": False,
        "unavailable_reasons": clean_reasons,
        "identity": section_identity,
        "geometry": _unavailable_lumen_geometry_section(identity),
        "data_quality": {
            **_base_lumen_data_quality([]),
            **data_quality,
            "metric_computation_success": False,
        },
        "physical_safety": _unavailable_physical_safety_section(),
        "safety_margin": _unavailable_safety_margin_section(),
        "constraints": {
            "wall": _unavailable_constraint_section("wall"),
            "inlet": _unavailable_constraint_section("inlet"),
            "outlet": _unavailable_constraint_section("outlet"),
        },
        "progress": _unavailable_progress_section(),
    }


def _lumen_identity_section(identity: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "task",
        "reference_mode",
        "target_mode",
        "curved_lumen_type",
        "scenario_id",
        "scenario_policy_version",
        "scenario_fingerprint",
        "geometry_frame",
        "geometry_fingerprint",
        "expected_geometry_fingerprint",
        "reconstructed_geometry_fingerprint",
        "geometry_fingerprint_match",
        "derived_target",
        "requested_target",
        "executed_target",
        "validated_target",
        "centerline_fraction",
        "centerline_arc_length",
        "radial_offset",
        "development_simulation",
        "production_promotion_evidence",
        "development_disclaimer",
        "development_target_selection",
        "override_used",
        "shared_environment_hash",
        "run_role",
    )
    return {key: identity.get(key) for key in keys}


def _lumen_geometry_section(identity: dict[str, Any], geometry: Any, payload: dict[str, Any]) -> dict[str, Any]:
    radius_profile = np.asarray(getattr(geometry, "radius_profile", []), dtype=float)
    return {
        "mode": "curved",
        "type": identity.get("curved_lumen_type"),
        "frame": getattr(geometry, "frame_id", None),
        "fingerprint": identity.get("reconstructed_geometry_fingerprint"),
        "fingerprint_payload": sanitize_for_json(payload),
        "ctr_outer_radius_m": _finite_float_or_none(getattr(geometry, "ctr_outer_radius", None)),
        "safety_margin_m": _finite_float_or_none(getattr(geometry, "safety_margin", None)),
        "minimum_lumen_radius_m": (
            float(np.min(radius_profile)) if radius_profile.size and np.all(np.isfinite(radius_profile)) else None
        ),
        "maximum_lumen_radius_m": (
            float(np.max(radius_profile)) if radius_profile.size and np.all(np.isfinite(radius_profile)) else None
        ),
    }


def _unavailable_lumen_geometry_section(identity: dict[str, Any]) -> dict[str, Any]:
    return {
        "mode": "curved",
        "type": identity.get("curved_lumen_type"),
        "frame": identity.get("geometry_frame"),
        "fingerprint": identity.get("reconstructed_geometry_fingerprint"),
        "fingerprint_payload": None,
        "ctr_outer_radius_m": None,
        "safety_margin_m": None,
        "minimum_lumen_radius_m": None,
        "maximum_lumen_radius_m": None,
    }


def _physical_safety_section(metrics: LumenEvaluationMetrics) -> dict[str, Any]:
    safety = metrics.safety
    return {
        "physical_safety_pass": bool(safety.physical_safety_pass),
        "collision_detected": bool(safety.physical_collision_detected),
        "collision_sample_count": int(safety.physical_collision_sample_count),
        "collision_event_count": int(safety.physical_collision_event_count),
        "collision_duration_s": float(safety.physical_collision_duration),
        "first_collision_time_s": safety.first_physical_collision_time,
        "minimum_physical_clearance_m": float(safety.minimum_physical_clearance),
        "final_physical_clearance_m": float(safety.final_physical_clearance),
        "worst_constraint": str(safety.worst_physical_constraint),
        "worst_sample_index": int(safety.worst_physical_sample_index),
        "worst_backbone_index": int(safety.worst_physical_backbone_index),
    }


def _safety_margin_section(metrics: LumenEvaluationMetrics) -> dict[str, Any]:
    safety = metrics.safety
    return {
        "safety_margin_pass": bool(safety.safety_margin_pass),
        "margin_violation_detected": bool(safety.safety_margin_violation_detected),
        "violation_sample_count": int(safety.safety_margin_violation_sample_count),
        "violation_event_count": int(safety.safety_margin_violation_event_count),
        "violation_duration_s": float(safety.safety_margin_violation_duration),
        "first_violation_time_s": safety.first_safety_margin_violation_time,
        "minimum_safety_clearance_m": float(safety.minimum_safety_clearance),
        "final_safety_clearance_m": float(safety.final_safety_clearance),
        "worst_constraint": str(safety.worst_safety_constraint),
        "worst_sample_index": int(safety.worst_safety_sample_index),
        "worst_backbone_index": int(safety.worst_safety_backbone_index),
    }


def _constraints_section(metrics: LumenEvaluationMetrics) -> dict[str, Any]:
    return {
        str(item.constraint_type): {
            "physical_violation_sample_count": int(item.physical_violation_sample_count),
            "physical_violation_event_count": int(item.physical_violation_event_count),
            "physical_violation_duration_s": float(item.physical_violation_duration),
            "first_physical_violation_time_s": item.first_physical_violation_time,
            "maximum_penetration_m": float(item.maximum_penetration),
            "minimum_physical_clearance_m": float(item.minimum_physical_clearance),
            "worst_sample_index": int(item.worst_sample_index),
            "worst_backbone_index": int(item.worst_backbone_index),
        }
        for item in metrics.safety.per_constraint_breakdown
    }


def _progress_section(metrics: LumenEvaluationMetrics) -> dict[str, Any]:
    progress = metrics.progress
    samples = metrics.samples
    out_of_extent_count = sum(1 for sample in samples if sample.tip_progress_out_of_extent)
    return {
        "initial_centerline_arc_length_m": float(progress.initial_centerline_arc_length),
        "final_centerline_arc_length_m": float(progress.final_centerline_arc_length),
        "minimum_centerline_arc_length_m": float(progress.minimum_centerline_arc_length),
        "maximum_centerline_arc_length_m": float(progress.maximum_centerline_arc_length),
        "initial_normalized_progress": float(progress.initial_normalized_progress),
        "final_normalized_progress": float(progress.final_normalized_progress),
        "maximum_normalized_progress": float(progress.maximum_normalized_progress),
        "tip_progress_out_of_extent_count": int(out_of_extent_count),
        "initial_radial_offset_m": float(samples[0].tip_radial_offset) if samples else None,
        "final_radial_offset_m": float(progress.final_tip_radial_offset),
        "mean_radial_offset_m": float(progress.mean_tip_radial_offset),
        "rms_radial_offset_m": float(progress.rms_tip_radial_offset),
        "maximum_radial_offset_m": float(progress.maximum_tip_radial_offset),
        "mean_local_radius_m": float(progress.mean_local_lumen_radius),
        "final_local_radius_m": float(progress.final_local_lumen_radius),
        "centerline_tracking_rmse_m": progress.centerline_tracking_rmse,
    }


def _unavailable_physical_safety_section() -> dict[str, Any]:
    return {
        "physical_safety_pass": False,
        "collision_detected": None,
        "collision_sample_count": None,
        "collision_event_count": None,
        "collision_duration_s": None,
        "first_collision_time_s": None,
        "minimum_physical_clearance_m": None,
        "final_physical_clearance_m": None,
        "worst_constraint": None,
        "worst_sample_index": None,
        "worst_backbone_index": None,
    }


def _unavailable_safety_margin_section() -> dict[str, Any]:
    return {
        "safety_margin_pass": False,
        "margin_violation_detected": None,
        "violation_sample_count": None,
        "violation_event_count": None,
        "violation_duration_s": None,
        "first_violation_time_s": None,
        "minimum_safety_clearance_m": None,
        "final_safety_clearance_m": None,
        "worst_constraint": None,
        "worst_sample_index": None,
        "worst_backbone_index": None,
    }


def _unavailable_constraint_section(constraint: str) -> dict[str, Any]:
    return {
        "constraint_type": constraint,
        "physical_violation_sample_count": None,
        "physical_violation_event_count": None,
        "physical_violation_duration_s": None,
        "first_physical_violation_time_s": None,
        "maximum_penetration_m": None,
        "minimum_physical_clearance_m": None,
        "worst_sample_index": None,
        "worst_backbone_index": None,
    }


def _unavailable_progress_section() -> dict[str, Any]:
    return {
        "initial_centerline_arc_length_m": None,
        "final_centerline_arc_length_m": None,
        "minimum_centerline_arc_length_m": None,
        "maximum_centerline_arc_length_m": None,
        "initial_normalized_progress": None,
        "final_normalized_progress": None,
        "maximum_normalized_progress": None,
        "tip_progress_out_of_extent_count": None,
        "initial_radial_offset_m": None,
        "final_radial_offset_m": None,
        "mean_radial_offset_m": None,
        "rms_radial_offset_m": None,
        "maximum_radial_offset_m": None,
        "mean_local_radius_m": None,
        "final_local_radius_m": None,
        "centerline_tracking_rmse_m": None,
    }


def _lumen_sample_csv_rows(metrics: LumenEvaluationMetrics) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for sample in metrics.samples:
        point = np.asarray(sample.tip_centerline_point, dtype=float)
        rows.append(
            {
                "timestamp_s": float(sample.timestamp),
                "physical_clearance_m": float(sample.physical_clearance),
                "safety_clearance_m": float(sample.safety_clearance),
                "physical_collision": bool(sample.physical_collision),
                "safety_margin_violation": bool(sample.safety_margin_violation),
                "selected_constraint_type": str(sample.selected_constraint_type),
                "closest_backbone_index": int(sample.closest_backbone_index),
                "wall_penetration_m": float(sample.wall_penetration),
                "inlet_penetration_m": float(sample.inlet_penetration),
                "outlet_penetration_m": float(sample.outlet_penetration),
                "tip_centerline_x": float(point[0]),
                "tip_centerline_y": float(point[1]),
                "tip_centerline_z": float(point[2]),
                "tip_centerline_segment_index": int(sample.tip_centerline_segment_index),
                "tip_centerline_interpolation_fraction": float(sample.tip_centerline_interpolation_fraction),
                "centerline_arc_length_m": float(sample.tip_centerline_arc_length),
                "normalized_progress": float(sample.normalized_tip_progress),
                "tip_progress_out_of_extent": bool(sample.tip_progress_out_of_extent),
                "radial_offset_m": float(sample.tip_radial_offset),
                "local_radius_m": float(sample.local_lumen_radius),
            }
        )
    return rows


def _curved_navigation_summary(
    section: dict[str, Any],
    goal_metrics: Any | None,
    *,
    completed: bool = True,
) -> dict[str, Any]:
    goal_success = False if goal_metrics is None else bool(goal_metrics.goal_reached)
    physical_pass = bool(section.get("physical_safety", {}).get("physical_safety_pass", False))
    safety_margin_pass = bool(section.get("safety_margin", {}).get("safety_margin_pass", False))
    run_valid = bool(section.get("run_valid", False))
    return {
        "run_valid": run_valid,
        "goal_success": goal_success,
        "physical_safety_pass": physical_pass,
        "safety_margin_pass": safety_margin_pass,
        "navigation_success": bool(completed and run_valid and goal_success and physical_pass),
        "completed_evaluation_window": bool(completed),
    }


def _acceptance_with_curved_lumen(
    acceptance: dict[str, Any],
    section: dict[str, Any],
    goal_metrics: Any | None,
    *,
    completed: bool = True,
) -> dict[str, Any]:
    updated = dict(acceptance)
    reasons = list(updated.get("reasons", []))
    goal_success = False if goal_metrics is None else bool(goal_metrics.goal_reached)
    physical_pass = bool(section.get("physical_safety", {}).get("physical_safety_pass", False))
    safety_margin_pass = bool(section.get("safety_margin", {}).get("safety_margin_pass", False))
    updated["goal_reached_pass"] = goal_success
    updated["collision_free_pass"] = physical_pass
    updated["safety_margin_pass"] = safety_margin_pass
    if not goal_success:
        _append_reason_once(reasons, "goal tolerance hold requirement was not met")
    if not physical_pass:
        _append_reason_once(reasons, "generic lumen physical safety failed or was unavailable")
    if not safety_margin_pass:
        _append_reason_once(reasons, "generic lumen safety margin failed or was unavailable")
    if not completed:
        updated["functional_pass"] = False
        _append_reason_once(reasons, "evaluation was interrupted before the configured window completed")
    updated["reasons"] = reasons
    return updated


def _append_reason_once(reasons: list[str], reason: str) -> None:
    if reason not in reasons:
        reasons.append(reason)


def _finite_float_or_none(value: Any) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


def _unique_reasons(reasons: list[str]) -> list[str]:
    result: list[str] = []
    for reason in reasons:
        if reason not in result:
            result.append(reason)
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
                _finite_csv_value(sample.solve_time),
                sample.command_saturated,
                sample.missing_command,
                _finite_csv_value(sample.reference_gap),
                _finite_csv_value(sample.command_gap),
                _finite_csv_value(sample.solve_gap),
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


def _latest_command_at_or_before(
    commands: list[TimedCommand], timestamp: float
) -> TimedCommand | None:
    latest = None
    for command in commands:
        if command.timestamp > timestamp:
            break
        latest = command
    return latest


def _transition_count(flags: np.ndarray) -> int:
    values = np.asarray(flags, dtype=bool)
    if values.size == 0:
        return 0
    return int(values[0]) + int(np.sum(values[1:] & ~values[:-1]))


def _flag_duration(timestamps: np.ndarray, flags: np.ndarray) -> float:
    times = np.asarray(timestamps, dtype=float)
    values = np.asarray(flags, dtype=bool)
    if times.size < 2 or values.size != times.size:
        return 0.0
    dt = np.maximum(0.0, np.diff(times, append=times[-1]))
    return float(np.sum(dt[values]))


def _finite_csv_value(value: Any) -> Any:
    """Serialize unavailable optional numerics as an empty CSV field, never NaN."""
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return ""
    return value


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
        "/ctr/evaluation/mppi_diagnostics",
        "/ctr/tactile/state",
        "/ctr/safety/status",
        "/diagnostics",
    )


def required_topics(*, slice_7g_profile: bool = False) -> tuple[str, ...]:
    topics = ("/ctr/state", "/ctr/reference/tip")
    if slice_7g_profile:
        topics += ("/ctr/tip", "/ctr/tactile/state", "/ctr/safety/status", "/ctr/safe_command")
    return topics


def sanitize_name(value: str) -> str:
    cleaned = "".join(char if char.isalnum() or char in {"-", "_"} else "_" for char in value.strip())
    return cleaned or "experiment"


def requested_run_id_from_metadata(metadata: dict[str, Any]) -> str | None:
    for key in ("requested_run_id", "run_id"):
        value = metadata.get(key)
        if value is None or value == "":
            continue
        text = str(value)
        cleaned = sanitize_name(text)
        if cleaned != text:
            raise ValueError("requested run ID may contain only alphanumeric characters, dashes, and underscores")
        return cleaned
    return None


def promoted_orchestration_metadata(metadata: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "orchestration_id",
        "run_role",
        "development_simulation",
        "production_promotion_evidence",
        "development_disclaimer",
        "development_target_selection",
        "requested_evaluation_duration_s",
        "total_recording_duration_s",
        "pre_roll_duration_s",
        "evaluation_window_start_time_s",
        "evaluation_window_end_time_s",
        "evaluation_window_duration_s",
        "recording_start_time_s",
        "recording_stop_time_s",
        "reference_start_policy",
        "scheduled_reference_epoch_s",
        "reference_lead_duration_s",
        "reference_phase_offset_s",
        "reference_pre_epoch_behavior",
        "shared_environment_hash",
        "controller_configuration_hash",
        "orchestration_hash",
        "initial_state_stability",
        "initial_tip_stability",
        "baseline_command_publisher_count",
        "baseline_safe_command_publisher_count",
        "baseline_mppi_command_publisher_count",
        "pre_roll_command_message_count",
        "pre_roll_nonzero_command_count",
        "unexpected_command_publishers",
        "command_zero_tolerance",
        "requested_target",
        "executed_target",
        "target_replaced",
        "target_identity_valid",
        "target_identity_tolerance",
        "sampled_reachability_confirmed",
        "sampled_reachability_method",
        "sampled_reachability_seed",
        "sampled_reachability_sample_count",
        "suggested_target",
        "validated_target",
        "derived_target",
        "override_used",
        "target_override_used",
        "reference_mode",
        "target_mode",
        "target_tolerance",
        "required_hold_duration",
        "curved_lumen_type",
        "scenario_id",
        "scenario_policy_version",
        "scenario_fingerprint",
        "geometry_frame",
        "geometry_fingerprint",
        "centerline_fraction",
        "centerline_arc_length",
        "radial_offset",
    )
    result = {key: metadata[key] for key in keys if key in metadata}
    reference = metadata.get("reference_configuration")
    if isinstance(reference, dict):
        for key in ("task", "reference_mode", "reference_transport"):
            if key in reference and key not in result:
                result[key] = reference[key]
        scenario = reference.get("curved_scenario")
        if isinstance(scenario, dict):
            for key in (
                "requested_target",
                "executed_target",
                "validated_target",
                "derived_target",
                "override_used",
                "target_override_used",
                "reference_mode",
                "target_mode",
                "target_tolerance",
                "required_hold_duration",
                "curved_lumen_type",
                "scenario_id",
                "scenario_policy_version",
                "scenario_fingerprint",
                "geometry_frame",
                "geometry_fingerprint",
                "centerline_fraction",
                "centerline_arc_length",
                "radial_offset",
            ):
                if key in scenario and key not in result:
                    result[key] = scenario[key]
    return result


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


def _optional_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


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
