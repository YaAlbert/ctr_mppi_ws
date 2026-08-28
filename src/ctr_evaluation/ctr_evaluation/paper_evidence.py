"""Controlled final-system experiment matrix and paper-artifact exporter."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
from types import SimpleNamespace
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

from ctr_bringup.development_physical_evidence import (
    SIMULATOR_PAPER_EVALUATION_FRESHNESS_TIMEOUT_S,
)
from ctr_bringup.parameter_validation import load_parameter_files
from ctr_evaluation.run_evaluation import (
    EvaluationOrchestrator,
    default_config_paths,
    parse_args as parse_evaluation_args,
)
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.lumen_factory import lumen_geometry_from_config
from ctr_safety.nodes.safety_supervisor_node import (
    SafetySupervisorNode,
    TactileSnapshot as SafetyTactileSnapshot,
)
from ctr_sim.nodes.development_target_selector_node import (
    build_sampled_reachability_cloud,
    sampled_reachability_predicate,
    select_development_target,
)
from ctr_tactile.tactile_processing import TactileProcessingParameters, TactileProcessor


LEGACY_COMPARISON_TARGET = (0.021180966381970152, 0.0, 0.08471218663414842)
TESTED_TARGET = (0.0166457424, 0.00397477634, 0.102231139)
TARGET_IDENTITY_TOLERANCE_M = 1.0e-12
SEEDS = (11, 22, 33)
PAPER_FIGURES = (
    "repeatability_metrics.png",
    "repeatability_convergence.png",
    "target_source_comparison.png",
    "target_difficulty_comparison.png",
    "lumen_geometry_comparison.png",
    "controller_configuration_comparison.png",
    "tactile_safety_response.png",
    "cost_term_breakdown.png",
    "deadline_analysis.png",
    "mppi_computation_breakdown.png",
    "rviz_navigation.png",
)
PAPER_TABLES = (
    "reference_run.csv",
    "repeatability.csv",
    "target_source.csv",
    "target_difficulty.csv",
    "lumen_geometry.csv",
    "controller_configuration.csv",
    "tactile_safety.csv",
    "robustness.csv",
)
FORBIDDEN_PRESENTATION_TEXT = (
    "slice 7g", "slice_7g", "feature/", "pre-merge", "post-merge",
    "not production evidence", "development stage", "development-stage",
)
FORMAL_WINDOW_EVIDENCE_VALIDATOR_SCHEMA = (
    "ctr-formal-window-evidence-validation-v1"
)
FORMAL_WINDOW_BOUNDARY_CONVENTION = "inclusive_start_inclusive_end"
FORMAL_WINDOW_DURATION_S = 25.0
REQUIRED_RUN_ARTIFACTS = frozenset(
    {
        "state.csv",
        "tip.csv",
        "reference.csv",
        "command.csv",
        "solve_timing.csv",
        "horizon.csv",
        "reference_path.csv",
        "backbone.csv",
        "metadata.yaml",
        "summary.json",
        "aligned_samples.csv",
        "tactile_safety.csv",
        "mppi_cost_terms.csv",
        "mppi_computation.csv",
        "tracking_error.png",
        "trajectory_xy.png",
        "trajectory_3d.png",
        "tip_trajectory.png",
        "command_history.png",
        "solve_time.png",
        "cumulative_control_effort.png",
        "curved_wall_clearance.png",
        "centerline_tracking_error.png",
        "curved_lumen_trajectory_3d.png",
        "tactile_safety_response.png",
        "cost_term_breakdown.png",
        "deadline_analysis.png",
        "mppi_computation_breakdown.png",
    }
)
_PROHIBITED_AUTHORITATIVE_SAFETY_REASON_TOKENS = (
    "authentication",
    "disconnected",
    "duplicate_sequence_changed",
    "future_dated",
    "integrity",
    "invalid",
    "producer",
    "rollback",
    "service",
    "stale",
    "timeout",
    "torn_read",
    "unavailable",
)
METRIC_DEFINITIONS = {
    "readiness_time_s": {
        "formula": "t(readiness_complete)-t(orchestrator_start)", "units": "s",
        "window": "startup readiness interval", "nan_policy": "unavailable when readiness is not reached",
    },
    "final_target_error_m": {
        "formula": "||tip_N-target||_2", "units": "m", "window": "final aligned tip sample",
        "nan_policy": "unavailable when no aligned sample exists",
    },
    "tip_to_target_rmse_m": {
        "formula": "sqrt(mean_i(||tip_i-target||_2^2))", "units": "m",
        "window": "all valid aligned tip samples", "nan_policy": "unavailable for an empty aligned set",
    },
    "centerline_tracking_rmse_m": {
        "formula": "sqrt(mean_i(d(tip_i, analytic_centerline)^2))", "units": "m",
        "window": "all valid lumen-evaluation tip samples", "nan_policy": "unavailable without lumen samples",
    },
    "minimum_clearance_m": {
        "formula": "min_i,min_backbone(radius-local_radial_distance-ctr_outer_radius)", "units": "m",
        "window": "all evaluated backbones", "nan_policy": "unavailable without authenticated backbone samples",
    },
    "effective_solve_frequency_hz": {
        "formula": "(N-1)/(t_last-t_first)", "units": "Hz", "window": "controller-metric timestamps",
        "nan_policy": "unavailable with fewer than two solve samples",
    },
    "cumulative_control_effort": {
        "formula": "sum_i(||u_i||_2^2*dt_i)", "units": "mixed command units squared second",
        "window": "valid aligned applied safe-command samples", "nan_policy": "zero for an empty command set",
    },
    "maximum_centerline_distance_m": {
        "formula": "max_i d(tip_i, analytic_centerline)", "units": "m",
        "window": "all valid lumen samples", "nan_policy": "unavailable without lumen samples",
    },
    "safety_margin_crossing_count": {
        "formula": "count of false-to-true safety-margin transitions", "units": "count",
        "window": "all lumen samples", "nan_policy": "zero for no crossings",
    },
    "safety_margin_violation_duration_s": {
        "formula": "sum of timestamp intervals whose terminal sample violates the margin", "units": "s",
        "window": "all lumen samples", "nan_policy": "zero for no violations",
    },
    "collision_count": {
        "formula": "count of false-to-true physical-collision transitions", "units": "count",
        "window": "all whole-backbone lumen samples", "nan_policy": "zero for no collisions",
    },
    "requested_runtime_s": {
        "formula": "runner CLI duration", "units": "s", "window": "run request",
        "nan_policy": "unavailable when request metadata is absent",
    },
    "configured_runtime_s": {
        "formula": "validated evaluation duration", "units": "s", "window": "effective configuration",
        "nan_policy": "unavailable when configuration metadata is absent",
    },
    "actual_runtime_s": {
        "formula": "monotonic experiment-stop time minus experiment-start time", "units": "s",
        "window": "recording lifecycle", "nan_policy": "unavailable if lifecycle did not start",
    },
    "solve_time_statistics_s": {
        "formula": "median, percentile_95, and max of finite solve durations", "units": "s",
        "window": "controller iterations in the recording window", "nan_policy": "unavailable for no solves",
    },
    "deadline_miss": {
        "formula": "count and 100*count/N where solve_time > configured_control_period", "units": "count, %",
        "window": "controller iterations in the recording window", "nan_policy": "zero count for no solves",
    },
    "cartesian_path_length_m": {
        "formula": "sum_i(||tip_i-tip_(i-1)||_2)", "units": "m",
        "window": "valid aligned tip samples", "nan_policy": "zero for fewer than two samples",
    },
    "command_total_variation": {
        "formula": "sum_i(||u_i-u_(i-1)||_2), insertion and rotation groups separately",
        "units": "m/s and rad/s", "window": "timestamped applied safe commands",
        "nan_policy": "zero for fewer than two commands",
    },
    "command_saturation_percentage": {
        "formula": "100*N(any group component equals validated limit)/N", "units": "%",
        "window": "timestamped applied safe commands", "nan_policy": "zero for no commands",
    },
    "tactile_safety_events": {
        "formula": "false-to-true event count and interval duration for each recorded state", "units": "count, s",
        "window": "diagnostic recording window", "nan_policy": "zero when a state is absent",
    },
    "sample_counts": {
        "formula": "number of raw tip, solve, and valid aligned records", "units": "count",
        "window": "recording window", "nan_policy": "zero when absent",
    },
    "completion_status": {
        "formula": "recorder lifecycle terminal state plus finalization result", "units": "enum",
        "window": "complete run lifecycle", "nan_policy": "incomplete/aborted is never successful",
    },
}


@dataclass(frozen=True)
class RunSpec:
    test_id: str
    experiment: str
    geometry: str
    target_case: str
    target_source: str
    controller_profile: str
    seed: int
    scenario: str
    target: tuple[float, float, float] | None = None

    @property
    def group(self) -> str:
        return "__".join(
            (self.test_id.lower(), self.geometry, self.target_case, self.target_source,
             self.controller_profile, f"seed_{self.seed}")
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run and package the final CTR simulation evidence matrix.")
    parser.add_argument("--output-root", default="")
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--matrix", choices=("all", "reference", "repeatability", "target_source", "target_difficulty", "geometry", "controller"), default="all")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--aggregate-only", action="store_true")
    return parser.parse_args(argv)


def matrix_specs() -> tuple[RunSpec, ...]:
    specs: list[RunSpec] = [
        RunSpec("E1-reference", "reference", "circular_arc", "nominal", "profile", "cylinder_fast", 11, "centerline_target")
    ]
    specs.extend(
        RunSpec(f"E2-repeat-{seed}", "repeatability", "circular_arc", "nominal", "profile", "cylinder_fast", seed, "centerline_target")
        for seed in (11, 22, 33, 44, 55)
    )
    for source in ("profile", "cli", "rviz"):
        for seed in SEEDS:
            specs.append(RunSpec(f"E3-{source}-{seed}", "target_source", "circular_arc", "nominal", source, "cylinder_fast", seed, "centerline_target", TESTED_TARGET))
    for case, scenario in (
        ("nominal", "centerline_target"),
        ("lateral_offset", "lateral_offset_target"),
        ("near_safety_margin", "near_safety_boundary_target"),
    ):
        for seed in SEEDS:
            specs.append(RunSpec(f"E4-{case}-{seed}", "target_difficulty", "circular_arc", case, "profile", "cylinder_fast", seed, scenario))
    for seed in SEEDS:
        specs.append(RunSpec(f"E5-straight-{seed}", "lumen_geometry", "straight", "normalized_nominal", "profile", "cylinder_fast", seed, "", (0.0192, 0.0, 0.084)))
        specs.append(RunSpec(f"E5-circular-{seed}", "lumen_geometry", "circular_arc", "normalized_nominal", "profile", "cylinder_fast", seed, "centerline_target"))
        specs.append(RunSpec(f"E5-s_curve-{seed}", "lumen_geometry", "s_curve", "normalized_nominal", "profile", "cylinder_fast", seed, "s_curve_middle_target"))
    for profile in ("paper_economy", "cylinder_fast", "paper_extended"):
        for seed in SEEDS:
            specs.append(RunSpec(f"E6-{profile}-{seed}", "controller_configuration", "circular_arc", "nominal", "profile", profile, seed, "centerline_target"))
    return tuple(specs)


def select_specs(name: str) -> tuple[RunSpec, ...]:
    mapping = {
        "reference": "reference", "repeatability": "repeatability", "target_source": "target_source",
        "target_difficulty": "target_difficulty", "geometry": "lumen_geometry", "controller": "controller_configuration",
    }
    return matrix_specs() if name == "all" else tuple(spec for spec in matrix_specs() if spec.experiment == mapping[name])


def run_spec(root: Path, spec: RunSpec, duration: float) -> dict[str, Any]:
    argv = [
        "--development-simulation", "--paper-diagnostics",
        "--physical-evidence-transport", "authenticated_shared_memory",
        "--simulator-paper-evaluation-profile",
        "--experiment-group", spec.group,
        "--mppi-profile", spec.controller_profile, "--seed", str(spec.seed), "--duration", format(duration, ".17g"),
        "--runtime-mode", "simulation", "--output-root", str(root),
    ]
    if spec.geometry == "straight":
        argv.extend(("--task", "cylinder_navigation", "--target", *(format(value, ".17g") for value in spec.target or ())))
    else:
        argv.extend(("--task", "curved_lumen_navigation", "--curved-lumen-type", spec.geometry, "--scenario", spec.scenario))
        if spec.target is not None:
            argv.extend(("--target", *(format(value, ".17g") for value in spec.target)))
    argv.extend(("--development-target-source", spec.target_source))
    if spec.target_source == "rviz":
        raw = spec.target or TESTED_TARGET
        argv.extend(("--development-raw-target", *(format(value, ".17g") for value in raw),
                     "--development-target-frame", "base_link", "--development-target-projection-distance", "0"))
    started = datetime.now(timezone.utc).isoformat()
    try:
        orchestrator = EvaluationOrchestrator(parse_evaluation_args(list(argv)))
        block_reason = target_source_block_reason(spec, orchestrator.project_config)
        if block_reason is not None:
            row = {
                **asdict(spec),
                "matrix_status": "blocked",
                "failure_reason": block_reason,
                "started_at": started,
                "candidate_dir": "",
            }
            append_jsonl(root / "matrix_progress.jsonl", row)
            return row
        result = orchestrator.run_pair()
        candidate = Path(result["candidate_dir"])
        row = extract_run_row(spec, candidate)
        row.update({"matrix_status": "completed", "started_at": started, "candidate_dir": str(candidate)})
    except Exception as exc:
        row = {**asdict(spec), "matrix_status": "failed", "failure_reason": f"{type(exc).__name__}: {exc}", "started_at": started, "candidate_dir": ""}
    append_jsonl(root / "matrix_progress.jsonl", row)
    return row


def target_source_block_reason(
    spec: RunSpec,
    config: dict[str, Any],
) -> str | None:
    """Use the final validator to preflight coordinate-identical E3 targets."""

    if spec.experiment != "target_source":
        return None
    target = spec.target or TESTED_TARGET
    geometry = lumen_geometry_from_config(config)
    reachability = sampled_reachability_predicate(
        build_sampled_reachability_cloud(ApproximateCTRModel(config), config),
        float(config["goal"]["tolerance"]),
    )
    selection = select_development_target(
        target,
        input_frame="base_link",
        target_source=spec.target_source,
        geometry=geometry,
        controller_frame=str(config["reference"]["frame_id"]),
        world_frame=str(config["robot"]["frames"]["world"]),
        projection_limit=float(
            config["simulation"]["development_target_selection"]["projection_limit"]
        ),
        reachable=reachability,
        accepted_target_timestamp=0.0,
        seed=spec.seed,
    )
    if selection.accepted and selection.validated_target is not None:
        accepted = np.asarray(selection.validated_target, dtype=np.float64)
        if np.allclose(
            accepted,
            np.asarray(target),
            rtol=0.0,
            atol=TARGET_IDENTITY_TOLERANCE_M,
        ):
            return None
    return (
        "coordinate-identical target-source comparison blocked by final target "
        f"validator: {selection.status}"
    )


def extract_run_row(spec: RunSpec, run_dir: Path) -> dict[str, Any]:
    ensure_standard_plot_names(run_dir)
    summary = read_json(run_dir / "summary.json")
    metadata = read_yaml(run_dir / "metadata.yaml")
    goal = summary.get("goal", {})
    lumen = summary.get("lumen_evaluation", {})
    timing = summary.get("timing", {})
    control = summary.get("control", {})
    paper = summary.get("paper_metrics", {})
    selection = metadata.get("development_target_selection", {})
    target = selection.get("validated_target") or metadata.get("executed_target") or metadata.get("validated_target")
    row = {
        **asdict(spec),
        "run_id": metadata.get("run_id"),
        "completion_status": summary.get("run_status", {}).get("status"),
        "navigation_success": summary.get("navigation", {}).get("navigation_success", goal.get("goal_reached")),
        "accepted_target": json.dumps(target, separators=(",", ":")),
        "accepted_target_timestamp_s": selection.get("accepted_target_timestamp_s"),
        "readiness_time_s": metadata.get("orchestration_runtime", {}).get("readiness_diagnostics", {}).get("readiness_elapsed_s"),
        "final_target_error_m": goal.get("final_goal_error"),
        "tip_to_target_rmse_m": goal.get("tip_to_target_rmse_m"),
        "centerline_tracking_rmse_m": lumen.get("progress", {}).get("centerline_tracking_rmse_m"),
        "maximum_centerline_distance_m": paper.get("maximum_centerline_distance_m"),
        "minimum_clearance_m": lumen.get("physical_safety", {}).get("minimum_physical_clearance_m", summary.get("lumen_safety", {}).get("minimum_clearance")),
        "collision_count": lumen.get("physical_safety", {}).get("collision_event_count", summary.get("lumen_safety", {}).get("collision_count", 0)),
        "actual_runtime_s": timing.get("experiment_wall_duration"),
        "effective_solve_frequency_hz": timing.get("effective_solve_frequency"),
        "solve_median_s": timing.get("median_solve_time"),
        "solve_p95_s": timing.get("p95_solve_time"),
        "solve_max_s": timing.get("max_solve_time"),
        "deadline_miss_count": timing.get("deadline_overrun_count"),
        "deadline_miss_percentage": timing.get("deadline_overrun_percentage"),
        "cartesian_path_length_m": paper.get("cartesian_path_length_m"),
        "cumulative_control_effort": control.get("total_control_effort"),
        "insertion_total_variation": paper.get("insertion_total_variation_m_per_s"),
        "rotation_total_variation": paper.get("rotation_total_variation_rad_per_s"),
        "insertion_saturation_percentage": paper.get("insertion_saturation_percentage"),
        "rotation_saturation_percentage": paper.get("rotation_saturation_percentage"),
        "tip_sample_count": paper.get("tip_sample_count"),
        "solve_sample_count": paper.get("solve_sample_count"),
        "aligned_sample_count": paper.get("aligned_sample_count"),
        "git_commit": metadata.get("git", {}).get("commit"),
        "source_dirty": metadata.get("git", {}).get("dirty"),
        "configuration_hash": metadata.get("controller_configuration_hash"),
    }
    if spec.geometry == "straight":
        derived = read_csv(run_dir / "lumen_evaluation.csv")
        if derived:
            radial = np.asarray([float(item["radial_offset_m"]) for item in derived])
            clearance = np.asarray([float(item["physical_clearance_m"]) for item in derived])
            collision_flags = np.asarray([item["physical_collision"].lower() == "true" for item in derived])
            row["centerline_tracking_rmse_m"] = float(np.sqrt(np.mean(radial**2)))
            row["maximum_centerline_distance_m"] = float(np.max(radial))
            row["minimum_clearance_m"] = float(np.min(clearance))
            row["collision_count"] = _transition_count(collision_flags)
    return row


def build_tactile_stress_table(config: dict[str, Any]) -> list[dict[str, Any]]:
    parameters = TactileProcessingParameters.from_mapping(config)
    processor = TactileProcessor(parameters)
    scale = float(config["safety"]["soft_contact"]["velocity_scale"])
    safety = _safety_shell(config)
    sequence: list[tuple[str, float]] = []
    for scenario, force, count in (
        ("no_contact", 0.0, 4), ("contact_threshold", 0.12, 12),
        ("contact_release", 0.0, 12), ("warning_threshold", 0.35, 24),
        ("warning_release", 0.0, 24), ("stop_threshold", 0.55, 36),
        ("stop_release", 0.0, 36),
    ):
        sequence.extend((scenario, force) for _ in range(count))
    rows: list[dict[str, Any]] = []
    for index, (scenario, force) in enumerate(sequence):
        sample = processor.process(
            [force], clearance_m=0.01, geometric_contact=force >= parameters.contact_on_n,
            timestamp_s=0.1 * index,
        )
        safety._test_now_ns = 1_000_000_000 + index * 100_000_000
        safety._test_now_mono = 0.01 + index * 0.1
        safety._state.header.stamp.sec = safety._test_now_ns // 1_000_000_000
        safety._state.header.stamp.nanosec = safety._test_now_ns % 1_000_000_000
        safety._state_received_mono = safety._test_now_mono
        safety._raw_command_received_mono = safety._test_now_mono
        safety._tactile_received_mono = safety._test_now_mono
        safety._tactile_status = "eligible_stop" if sample.stop else "eligible_warning" if sample.warning else "eligible_no_contact"
        safety._tactile = SafetyTactileSnapshot(
            stamp_ns=safety._test_now_ns, frame_id="base_link", valid=sample.valid,
            clearance_m=sample.clearance_m, force_magnitude=sample.force_n,
            contact=sample.contact, warning=sample.warning, stop=sample.stop,
            region=sample.region,
        )
        if sample.stop:
            safety._stop_latched = True
            safety._fault_latched = True
            safety._latched_fault_reason = "tactile_stop"
        decision = safety._decision()
        applied_scale = (
            float(decision.command[0] / safety._raw_command.q_dot[0])
            if safety._raw_command.q_dot[0] and decision.allowed else 0.0
        )
        rows.append({
            "test_id": f"E7-{scenario}", "scenario": scenario, "timestamp_s": 0.1 * index,
            "raw_simulated_force_n": sample.raw_signal,
            "filtered_simulated_force_n": sample.force_n,
            "contact": sample.contact, "warning": sample.warning, "stop": sample.stop,
            "region": sample.region, "command_scale": applied_scale,
            "contact_on_n": parameters.contact_on_n, "contact_off_n": parameters.contact_off_n,
            "warning_on_n": parameters.warning_on_n, "warning_off_n": parameters.warning_off_n,
            "stop_on_n": parameters.stop_on_n, "stop_off_n": parameters.stop_off_n,
            "safety_state": decision.state_name, "safety_fault": decision.fault,
            "safety_emergency_stop": decision.emergency_stop,
            "latched_fault_expected": safety._fault_latched,
            "evidence_class": "diagnostic_stress_test",
        })
    safety._tactile_status = "tactile_invalid"
    invalid = safety._decision()
    rows.append({
        "test_id": "E7-invalid", "scenario": "invalid evidence",
        "timestamp_s": safety._test_now_mono, "valid": False, "command_scale": 0.0,
        "safety_state": invalid.state_name, "safety_fault": invalid.fault,
        "evidence_class": "diagnostic_stress_test",
    })
    safety._tactile_status = "eligible_no_contact"
    safety._test_now_mono += safety.tactile_timeout + 0.01
    stale = safety._decision()
    rows.append({
        "test_id": "E7-stale", "scenario": "stale evidence",
        "timestamp_s": safety._test_now_mono, "valid": False, "command_scale": 0.0,
        "safety_state": stale.state_name, "safety_fault": stale.fault,
        "evidence_class": "diagnostic_stress_test",
    })
    safety._test_now_mono += 0.01
    safety._state_received_mono = safety._test_now_mono
    safety._raw_command_received_mono = safety._test_now_mono
    safety._tactile_received_mono = safety._test_now_mono
    safety._tactile_status = "eligible_no_contact"
    safety._tactile = SafetyTactileSnapshot(
        stamp_ns=safety._test_now_ns, frame_id="base_link", valid=True,
        clearance_m=0.01, force_magnitude=0.0, contact=False,
        warning=False, stop=False, region=0,
    )
    response = safety._on_clear_fault(SimpleNamespace(), SimpleNamespace(accepted=False, message=""))
    cleared = safety._decision()
    rows.append({
        "test_id": "E7-authorized-clear", "scenario": "authorized clear after safe evidence",
        "timestamp_s": safety._test_now_mono, "valid": True,
        "command_scale": 1.0 if cleared.allowed else 0.0,
        "safety_state": cleared.state_name, "safety_fault": cleared.fault,
        "clear_accepted": response.accepted, "evidence_class": "diagnostic_stress_test",
    })
    return rows


class _AlwaysSafeGeometry:
    def check_backbone(self, _points: Any) -> tuple[bool, str, None]:
        return True, "geometry_clear", None


def _safety_shell(config: dict[str, Any]) -> SafetySupervisorNode:
    node = SafetySupervisorNode.__new__(SafetySupervisorNode)
    node._lock = threading.RLock()
    node._start_mono = 0.0
    node._last_tactile_stamp_ns = 0
    node._tactile = None
    node._tactile_received_mono = None
    node._tactile_status = "startup_unavailable"
    node._stop_latched = False
    node._fault_latched = False
    node._latched_fault_reason = ""
    node._last_reason = "startup_unavailable"
    node._last_safe_command = (0.0,) * 6
    stamp = SimpleNamespace(sec=1, nanosec=0)
    header = SimpleNamespace(stamp=stamp, frame_id="base_link")
    node._raw_command = SimpleNamespace(q_dot=[0.01] * 6, valid=True)
    node._raw_command_received_mono = 0.0
    node._state = SimpleNamespace(header=header, valid=True, backbone=[])
    node._state_received_mono = 0.0
    node.frame_id = "base_link"
    node.safety_enabled = True
    node.tactile_enabled = True
    safety = config["safety"]
    node.state_timeout = float(safety["state_timeout"])
    node.command_timeout = float(safety["command_timeout"])
    node.tactile_timeout = float(safety["tactile_timeout"])
    node.tactile_startup_grace = float(safety["tactile_startup_grace_s"])
    node.tactile_future_skew = float(safety["tactile_future_skew_s"])
    node.soft_contact_velocity_scale = float(safety["soft_contact"]["velocity_scale"])
    node.geometry = _AlwaysSafeGeometry()
    node._test_now_ns = 1_000_000_000
    node._test_now_mono = 0.01
    node._now_ns = lambda: node._test_now_ns
    node._monotonic = lambda: node._test_now_mono
    return node


def robustness_rows() -> list[dict[str, Any]]:
    return [
        {"test_id": "E8-invalid-cli", "case": "invalid CLI target", "expected": "rejected", "test": "src/ctr_sim/test/test_development_target_selector.py::test_cli_point_outside_wall_is_rejected_without_projection"},
        {"test_id": "E8-invalid-rviz", "case": "invalid RViz candidate", "expected": "rejected", "test": "src/ctr_sim/test/test_development_target_selector.py::test_excessive_projection_distance_is_rejected"},
        {"test_id": "E8-projection", "case": "projection beyond 0.035 m", "expected": "rejected", "test": "src/ctr_sim/test/test_development_target_selector.py::test_excessive_projection_distance_is_rejected"},
        {"test_id": "E8-one-shot", "case": "target update after motion", "expected": "rejected", "test": "src/ctr_sim/test/test_development_target_selector.py::test_target_update_policy_accepts_only_before_motion_starts"},
        {"test_id": "E8-interrupted", "case": "interrupted run", "expected": "incomplete", "test": "src/ctr_evaluation/test/test_experiment_recorder.py::ExperimentRecorderTests::test_interrupted_run_is_explicitly_incomplete_and_not_successful"},
        {"test_id": "E8-missing-artifact", "case": "missing required artifact", "expected": "not successful", "test": "src/ctr_evaluation/test/test_development_simulation.py::test_functional_result_rejects_incomplete_evaluation_window"},
    ]


def validate_matrix_contract(rows: list[dict[str, Any]], expected: tuple[RunSpec, ...]) -> list[str]:
    """Return deterministic matrix-completeness and target-equivalence failures."""

    failures: list[str] = []
    expected_ids = {spec.test_id for spec in expected}
    actual_ids = {str(row.get("test_id")) for row in rows}
    if actual_ids != expected_ids:
        failures.append(
            f"matrix IDs differ: missing={sorted(expected_ids - actual_ids)}, "
            f"unexpected={sorted(actual_ids - expected_ids)}"
        )
    for row in rows:
        if row.get("test_id") not in expected_ids:
            continue
        if row.get("matrix_status") != "completed" or row.get("completion_status") != "completed":
            failures.append(f"{row.get('test_id')}: not completed")
    for row in rows:
        if row.get("experiment") != "target_source" or row.get("matrix_status") != "completed":
            continue
        try:
            target = np.asarray(json.loads(str(row.get("accepted_target"))), dtype=np.float64)
        except (TypeError, ValueError, json.JSONDecodeError):
            failures.append(f"{row.get('test_id')}: accepted target is not parseable")
            continue
        if target.shape != (3,) or not np.isfinite(target).all():
            failures.append(f"{row.get('test_id')}: accepted target is not one finite 3-vector")
        elif not np.allclose(target, np.asarray(TESTED_TARGET), rtol=0.0, atol=1.0e-12):
            failures.append(f"{row.get('test_id')}: accepted target differs from comparison target")
    return failures


def aggregate(root: Path, rows: list[dict[str, Any]], config_path: Path) -> dict[str, Any]:
    figures = root / "paper_figures"
    tables = root / "paper_tables"
    export = root / "overleaf_upload"
    for path in (figures, tables, export):
        path.mkdir(parents=True, exist_ok=True)
    successful = [row for row in rows if row.get("matrix_status") == "completed" and row.get("completion_status") == "completed"]
    table_groups = {
        "reference_run.csv": ("reference",), "repeatability.csv": ("repeatability",),
        "target_source.csv": ("target_source",), "target_difficulty.csv": ("target_difficulty",),
        "lumen_geometry.csv": ("lumen_geometry",), "controller_configuration.csv": ("controller_configuration",),
    }
    for filename, experiments in table_groups.items():
        write_csv(
            tables / filename,
            [publication_row(row) for row in rows if row.get("experiment") in experiments],
        )
    del config_path
    config = load_parameter_files(default_config_paths())
    tactile = build_tactile_stress_table(config)
    write_csv(tables / "tactile_safety.csv", tactile)
    write_csv(tables / "robustness.csv", robustness_rows())
    write_csv(root / "comparison.csv", rows)
    write_json(root / "comparison.json", {"runs": rows, "metric_definitions": METRIC_DEFINITIONS})
    write_csv(root / "experiment_matrix.csv", [{**asdict(spec), "group": spec.group} for spec in matrix_specs()])
    plot_index = generate_figures(figures, successful, tactile)
    write_json(root / "plot_index.json", plot_index)
    validation, validation_failures = validate_artifacts(root, rows)
    (root / "artifact_validation.md").write_text(validation, encoding="utf-8")
    results = paper_results(rows, plot_index)
    (root / "paper_results.md").write_text(results, encoding="utf-8")
    for name in PAPER_FIGURES:
        shutil.copy2(figures / name, export / name)
    for name in PAPER_TABLES:
        shutil.copy2(tables / name, export / name)
    shutil.copy2(root / "paper_results.md", export / "paper_results.md")
    shutil.copy2(root / "plot_index.json", export / "plot_index.json")
    manifest = build_manifest(root)
    write_json(root / "manifest.json", manifest)
    return {
        "completed": len(successful), "total": len(rows),
        "artifact_validation_failures": validation_failures,
        "validation": validation, "export": str(export),
    }


def generate_figures(figures: Path, rows: list[dict[str, Any]], tactile: list[dict[str, Any]]) -> dict[str, Any]:
    from cycler import cycler

    plt.rcParams.update({
        "savefig.dpi": 300, "font.size": 8.5, "axes.titlesize": 9.5,
        "legend.fontsize": 7.5,
        "axes.prop_cycle": cycler(color=("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9", "#000000")),
    })
    index: dict[str, Any] = {}
    groups = {
        "repeatability_metrics.png": ("repeatability", "Seed Repeatability", "seed"),
        "target_source_comparison.png": ("target_source", "Target-Source Comparison", "target_source"),
        "target_difficulty_comparison.png": ("target_difficulty", "Target-Difficulty Comparison", "target_case"),
        "lumen_geometry_comparison.png": ("lumen_geometry", "Lumen-Geometry Comparison", "geometry"),
        "controller_configuration_comparison.png": ("controller_configuration", "Controller Configuration Comparison", "controller_profile"),
    }
    for filename, (experiment, title, category) in groups.items():
        source = [row for row in rows if row.get("experiment") == experiment]
        _metric_comparison_plot(figures / filename, source, title, category)
        index[filename] = figure_record(filename, title, source, "mean and sample range", "nominal_navigation")
    repeat = [row for row in rows if row.get("experiment") == "repeatability"]
    _convergence_plot(figures / "repeatability_convergence.png", repeat)
    index["repeatability_convergence.png"] = figure_record("repeatability_convergence.png", "Repeatability Convergence", repeat, "raw trace by seed", "nominal_navigation")
    _tactile_plot(figures / "tactile_safety_response.png", tactile)
    index["tactile_safety_response.png"] = figure_record("tactile_safety_response.png", "Tactile and Safety Response", tactile, "deterministic threshold stress sequence", "diagnostic_stress_test")
    reference = next((row for row in rows if row.get("experiment") == "reference"), None)
    for filename, source_name, title in (
        ("cost_term_breakdown.png", "cost_term_breakdown.png", "MPPI Cost-Term Breakdown"),
        ("deadline_analysis.png", "deadline_analysis.png", "Deadline Analysis"),
        ("mppi_computation_breakdown.png", "mppi_computation_breakdown.png", "MPPI Computation-Time Breakdown"),
    ):
        if reference and Path(reference["candidate_dir"]).joinpath(source_name).is_file():
            shutil.copy2(Path(reference["candidate_dir"]) / source_name, figures / filename)
        else:
            _blocked_figure(figures / filename, title, "reference-run evidence unavailable")
        index[filename] = figure_record(filename, title, [] if reference is None else [reference], "raw per-iteration trace", "nominal_navigation")
    _rviz_navigation_plot(figures / "rviz_navigation.png", reference)
    index["rviz_navigation.png"] = figure_record("rviz_navigation.png", "Autonomous Navigation in a Curved Lumen", [] if reference is None else [reference], "raw recorded trajectory and centerline", "nominal_navigation")
    return index


def _metric_comparison_plot(path: Path, rows: list[dict[str, Any]], title: str, category: str) -> None:
    metrics = (("final_target_error_m", "final target error [m]"), ("centerline_tracking_rmse_m", "centerline RMSE [m]"), ("minimum_clearance_m", "minimum clearance [m]"), ("effective_solve_frequency_hz", "solve frequency [Hz]"))
    fig, axes = plt.subplots(2, 2, figsize=(7.1, 5.6))
    categories = sorted({str(row.get(category)) for row in rows})
    for ax, (field, label) in zip(axes.flat, metrics):
        for index, name in enumerate(categories):
            values = finite_values(row.get(field) for row in rows if str(row.get(category)) == name)
            if values.size:
                ax.scatter(np.full(values.size, index), values, s=16, alpha=0.65)
                ax.errorbar(index, np.mean(values), yerr=[[np.mean(values)-np.min(values)], [np.max(values)-np.mean(values)]], fmt="D", color="black", capsize=3)
        ax.set_xticks(range(len(categories)), categories, rotation=20, ha="right")
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.25)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _convergence_plot(path: Path, rows: list[dict[str, Any]]) -> None:
    fig, ax = plt.subplots(figsize=(7.1, 4.2))
    plotted = False
    for row in rows:
        csv_path = Path(row["candidate_dir"]) / "aligned_samples.csv"
        data = read_csv(csv_path)
        if not data:
            continue
        t = np.asarray([float(item["timestamp"]) for item in data])
        tip = np.asarray([[float(item[key]) for key in ("tip_x", "tip_y", "tip_z")] for item in data])
        ref = np.asarray([[float(item[key]) for key in ("ref_x", "ref_y", "ref_z")] for item in data])
        ax.plot(t-t[0], np.linalg.norm(tip-ref, axis=1), alpha=0.7, label=f"seed {row['seed']}")
        plotted = True
    if not plotted:
        ax.text(0.5, 0.5, "repeatability evidence unavailable", transform=ax.transAxes, ha="center")
    ax.set_title("Repeatability Convergence")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("tip-to-target error [m]")
    ax.grid(True, alpha=0.25)
    if plotted:
        ax.legend(ncol=3)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def _tactile_plot(path: Path, rows: list[dict[str, Any]]) -> None:
    times = np.asarray([float(row.get("timestamp_s", math.nan)) for row in rows])
    force = np.asarray([float(row.get("filtered_simulated_force_n", math.nan)) for row in rows])
    scale = np.asarray([float(row.get("command_scale", math.nan)) for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(7.1, 5.0), sharex=True)
    axes[0].plot(times, force, marker="o", color="#0072B2", label="filtered simulated force")
    axes[0].set_title("Tactile and Safety Response"); axes[0].set_ylabel("force [N]"); axes[0].grid(True, alpha=0.25)
    axes[1].step(times, scale, where="post", color="#D55E00", label="applied command scale")
    axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("scale [-]"); axes[1].set_ylim(-0.05, 1.05); axes[1].grid(True, alpha=0.25)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def _rviz_navigation_plot(path: Path, row: dict[str, Any] | None) -> None:
    if row is None:
        _blocked_figure(path, "Autonomous Navigation in a Curved Lumen", "reference-run evidence unavailable")
        return
    run = Path(row["candidate_dir"])
    data = read_csv(run / "aligned_samples.csv")
    lumen = read_csv(run / "lumen_evaluation.csv")
    if not data or not lumen:
        _blocked_figure(path, "Autonomous Navigation in a Curved Lumen", "trajectory evidence unavailable")
        return
    tip = np.asarray([[float(item[key]) for key in ("tip_x", "tip_y", "tip_z")] for item in data])
    center = np.asarray([[float(item[key]) for key in ("tip_centerline_x", "tip_centerline_y", "tip_centerline_z")] for item in lumen])
    fig = plt.figure(figsize=(7.1, 5.2)); ax = fig.add_subplot(111, projection="3d")
    ax.plot(center[:,0], center[:,1], center[:,2], color="#56B4E9", label="closest centerline")
    ax.plot(tip[:,0], tip[:,1], tip[:,2], color="#009E73", label="executed tip trajectory")
    target = json.loads(row["accepted_target"]) if row.get("accepted_target") else None
    if target:
        ax.scatter(*target, color="#E69F00", s=35, label="accepted target")
    ax.set_title("Autonomous Navigation in a Curved Lumen"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]"); ax.legend()
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def _blocked_figure(path: Path, title: str, reason: str) -> None:
    fig, ax = plt.subplots(figsize=(7.1, 3.5)); ax.set_title(title); ax.text(0.5, 0.5, reason, ha="center", va="center"); ax.set_axis_off(); fig.tight_layout(); fig.savefig(path); plt.close(fig)


def figure_record(filename: str, title: str, rows: list[dict[str, Any]], aggregation: str, evidence_class: str) -> dict[str, Any]:
    commit = git_value("rev-parse", "HEAD")
    return {
        "figure": f"paper_figures/{filename}", "title": title, "caption": f"{title}. {aggregation}.",
        "source_test_ids": [row.get("test_id") for row in rows], "source_run_directories": [row.get("candidate_dir") for row in rows],
        "metric_definitions": METRIC_DEFINITIONS, "aggregation_and_uncertainty": aggregation,
        "tested_commit": commit, "dirty_state": bool(git_value("status", "--porcelain", "--untracked-files=no")),
        "configuration_hashes": sorted({str(row.get("configuration_hash")) for row in rows if row.get("configuration_hash")}),
        "generated_at": datetime.now(timezone.utc).isoformat(), "plotting_script": "src/ctr_evaluation/ctr_evaluation/paper_evidence.py",
        "plotting_script_commit": commit, "evidence_class": evidence_class,
        "runtime_scheduling": "non-real-time Ubuntu host",
        "controller_realtime_claim": False,
        "simulator_physical_evidence_freshness_timeout_s": 0.20,
        "production_hardware_freshness_timeout_s": 0.10,
        "simulator_watchdog_is_production_validation": False,
    }


def validate_formal_window_evidence(run_dir: Path) -> dict[str, Any]:
    """Validate simulator evidence in the committed formal wall-clock window.

    The recorder's state-window convention is inclusive at both boundaries, so
    this validator uses ``start <= source_stamp <= end``. Safety status rows in
    ``tactile_safety.csv`` inherit the latest tactile row's generic timestamp;
    the authenticated ``safety_source_stamp_s`` is therefore the authoritative
    wall-clock field used to select safety observations. ROS delivery fields
    remain diagnostic and never substitute for ``safety_queued_age_s``.
    """

    failures: list[str] = []
    metadata: dict[str, Any] = {}
    summary: dict[str, Any] = {}
    try:
        metadata = read_yaml(run_dir / "metadata.yaml")
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        failures.append(f"metadata unreadable: {type(exc).__name__}")
    try:
        summary = read_json(run_dir / "summary.json")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        failures.append(f"summary unreadable: {type(exc).__name__}")

    start = _finite_number(metadata.get("evaluation_window_start_time_s"))
    end = _finite_number(metadata.get("evaluation_window_end_time_s"))
    duration = _finite_number(metadata.get("evaluation_window_duration_s"))
    if start is None:
        failures.append("formal window start is missing or nonfinite")
    if end is None:
        failures.append("formal window end is missing or nonfinite")
    if duration is None:
        failures.append("formal window duration is missing or nonfinite")
    if start is not None and end is not None and end <= start:
        failures.append("formal window is empty or reversed")
    if duration is not None and not math.isclose(
        duration, FORMAL_WINDOW_DURATION_S, rel_tol=0.0, abs_tol=1.0e-9
    ):
        failures.append("formal window duration is not exactly 25 seconds")
    if start is not None and end is not None and duration is not None and not math.isclose(
        end - start, duration, rel_tol=0.0, abs_tol=1.0e-9
    ):
        failures.append("formal window endpoints and duration are inconsistent")

    override = metadata.get("metadata_override", {})
    if isinstance(override, dict):
        for key, authoritative in (
            ("evaluation_window_start_time_s", start),
            ("evaluation_window_end_time_s", end),
            ("evaluation_window_duration_s", duration),
        ):
            if key not in override:
                continue
            duplicate = _finite_number(override.get(key))
            if (
                duplicate is None
                or authoritative is None
                or not math.isclose(
                    duplicate, authoritative, rel_tol=0.0, abs_tol=1.0e-9
                )
            ):
                failures.append(f"formal window metadata disagrees for {key}")

    recording_start = _finite_number(metadata.get("recording_start_time_s"))
    recording_stop = _finite_number(metadata.get("recording_stop_time_s"))
    if start is not None and recording_start is not None and recording_start > start:
        failures.append("recording starts after the formal window")
    if end is not None and recording_stop is not None and recording_stop < end:
        failures.append("recording stops before the formal window")

    paper_metrics = summary.get("paper_metrics", {})
    requested_duration = _finite_number(
        paper_metrics.get("requested_runtime_s")
        if isinstance(paper_metrics, dict)
        else None
    )
    if requested_duration is not None and duration is not None and not math.isclose(
        requested_duration, duration, rel_tol=0.0, abs_tol=1.0e-9
    ):
        failures.append("summary runtime disagrees with the formal window")

    run_status = summary.get("run_status", {})
    navigation = summary.get("navigation", {})
    safety_summary = summary.get("slice_7g_safety", {})
    if not isinstance(run_status, dict) or run_status.get("status") != "completed":
        failures.append("terminal run status is not completed")
    if not isinstance(run_status, dict) or run_status.get("interrupted") is not False:
        failures.append("terminal run is interrupted or lacks interruption status")
    if (
        not isinstance(run_status, dict)
        or run_status.get("completed_evaluation_window") is not True
    ):
        failures.append("terminal run did not complete the formal window")
    for key in (
        "run_valid",
        "physical_safety_pass",
        "safety_margin_pass",
        "completed_evaluation_window",
    ):
        if not isinstance(navigation, dict) or navigation.get(key) is not True:
            failures.append(f"navigation terminal field is not true: {key}")
    if (
        not isinstance(safety_summary, dict)
        or _finite_number(safety_summary.get("fault_count")) != 0.0
    ):
        failures.append("summary reports a safety fault")

    configuration = metadata.get("configuration", {})
    profile_values = (
        metadata.get("simulator_paper_evaluation_profile"),
        configuration.get("simulator_paper_evaluation_profile")
        if isinstance(configuration, dict)
        else None,
        override.get("simulator_paper_evaluation_profile")
        if isinstance(override, dict)
        else None,
    )
    if True not in profile_values:
        failures.append("simulator paper-evaluation profile is not enabled")

    tactile_safety_rows = read_csv(run_dir / "tactile_safety.csv")
    tactile_rows = [
        row for row in tactile_safety_rows if row.get("event_type") == "tactile"
    ]
    safety_rows = [
        row for row in tactile_safety_rows if row.get("event_type") == "safety"
    ]
    if not tactile_safety_rows:
        failures.append("tactile_safety.csv is missing or empty")

    formal_tactile: list[dict[str, str]] = []
    formal_safety: list[dict[str, str]] = []
    if start is not None and end is not None and end > start:
        formal_tactile = [
            row
            for row in tactile_rows
            if _inside_inclusive_window(row.get("timestamp_s"), start, end)
        ]
        formal_safety = [
            row
            for row in safety_rows
            if _inside_inclusive_window(row.get("safety_source_stamp_s"), start, end)
        ]
    if not formal_safety:
        failures.append("formal window has no authoritative safety evidence")

    full_safety_diagnostics = _safety_diagnostics(safety_rows)
    formal_safety_diagnostics = _safety_diagnostics(formal_safety)
    full_ros_diagnostics = _ros_delivery_diagnostics(tactile_rows)
    formal_ros_diagnostics = _ros_delivery_diagnostics(formal_tactile)

    for index, row in enumerate(formal_safety):
        age = _finite_number(row.get("safety_queued_age_s"))
        if age is None or age < 0.0:
            failures.append(
                f"formal safety row {index} has invalid safety_direct_age_s"
            )
        elif age >= SIMULATOR_PAPER_EVALUATION_FRESHNESS_TIMEOUT_S:
            failures.append(
                f"formal safety row {index} safety_direct_age_s is outside [0,0.20)"
            )
        valid = _csv_bool(row.get("safety_valid"))
        fault = _csv_bool(row.get("safety_fault"))
        emergency_stop = _csv_bool(row.get("safety_emergency_stop"))
        if valid is not True:
            failures.append(f"formal safety row {index} is invalid")
        if fault is not False:
            failures.append(f"formal safety row {index} reports a fault")
        if emergency_stop is not False:
            failures.append(f"formal safety row {index} reports emergency stop")
        reason = str(row.get("safety_reason", "")).split("|", 1)[0]
        if not reason:
            failures.append(f"formal safety row {index} lacks a safety reason")
        elif any(
            token in reason
            for token in _PROHIBITED_AUTHORITATIVE_SAFETY_REASON_TOKENS
        ):
            failures.append(
                f"formal safety row {index} has fail-closed reason: {reason}"
            )
        if _positive_integer(row.get("safety_source_sequence")) is None:
            failures.append(
                f"formal safety row {index} has invalid authoritative sequence"
            )
        if _finite_number(row.get("safety_source_stamp_s")) is None:
            failures.append(
                f"formal safety row {index} has invalid authoritative source stamp"
            )
        if _csv_bool(row.get("safety_out_of_order_sequence")) is True:
            failures.append(
                f"formal safety row {index} reports out-of-order evidence"
            )

    formal_sequence = formal_safety_diagnostics["sequence"]
    if formal_sequence["rollback_count"]:
        failures.append("formal authoritative safety sequence rolls back")
    if formal_sequence["source_stamp_rollback_count"]:
        failures.append("formal authoritative safety source timestamp rolls back")
    if formal_sequence["same_sequence_mutation_count"]:
        failures.append("formal authoritative same sequence changes timestamp")

    result = {
        "schema_version": FORMAL_WINDOW_EVIDENCE_VALIDATOR_SCHEMA,
        "eligible": False,
        "failures": sorted(set(failures)),
        "formal_window": {
            "boundary_convention": FORMAL_WINDOW_BOUNDARY_CONVENTION,
            "start_time_s": start,
            "end_time_s": end,
            "duration_s": duration,
            "tactile_row_count": len(formal_tactile),
            "authoritative_safety_row_count": len(formal_safety),
        },
        "authoritative_safety": {
            "age_field": "safety_queued_age_s",
            "reported_name": "safety_direct_age_s",
            "freshness_rule": "0 <= safety_direct_age_s < 0.20",
            "freshness_timeout_s": (
                SIMULATOR_PAPER_EVALUATION_FRESHNESS_TIMEOUT_S
            ),
            "full_recording": full_safety_diagnostics,
            "formal_window": formal_safety_diagnostics,
        },
        "ros_tactile_delivery": {
            "age_field": "data_age_s",
            "reported_name": "ros_tactile_delivery_age_s",
            "scientific_freshness_authority": False,
            "latest_sample_forward_gaps_allowed": True,
            "source_mailbox_overwrites_are_diagnostic": True,
            "full_recording": full_ros_diagnostics,
            "formal_window": formal_ros_diagnostics,
        },
    }
    result["eligible"] = not result["failures"]
    return result


def format_formal_window_evidence_validation(result: dict[str, Any]) -> str:
    """Return deterministic Markdown with explicit ROS/direct-safety labels."""

    safety = result["authoritative_safety"]["formal_window"]
    ros_full = result["ros_tactile_delivery"]["full_recording"]
    ros_formal = result["ros_tactile_delivery"]["formal_window"]
    return "; ".join(
        (
            f"schema={result['schema_version']}",
            f"eligibility={'PASS' if result['eligible'] else 'FAIL'}",
            f"boundary={result['formal_window']['boundary_convention']}",
            "safety_direct_age_s(formal)="
            f"{json.dumps(safety['safety_direct_age_s'], sort_keys=True, allow_nan=False)}",
            "ros_tactile_delivery_age_s(full)="
            f"{json.dumps(ros_full['ros_tactile_delivery_age_s'], sort_keys=True, allow_nan=False)}",
            "ros_tactile_delivery_age_s(formal)="
            f"{json.dumps(ros_formal['ros_tactile_delivery_age_s'], sort_keys=True, allow_nan=False)}",
            "ros_forward_gap_count(full/formal)="
            f"{ros_full['sequence']['forward_gap_count']}/"
            f"{ros_formal['sequence']['forward_gap_count']}",
            "source_mailbox_overwrite_total(full/formal)="
            f"{ros_full['source_mailbox_overwrites']['total']}/"
            f"{ros_formal['source_mailbox_overwrites']['total']}",
            f"failures={json.dumps(result['failures'], sort_keys=True, allow_nan=False)}",
        )
    )


def _finite_number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _positive_integer(value: Any) -> int | None:
    numeric = _finite_number(value)
    if numeric is None or numeric <= 0.0 or not numeric.is_integer():
        return None
    return int(numeric)


def _csv_bool(value: Any) -> bool | None:
    if type(value) is bool:
        return value
    if type(value) is not str:
        return None
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    return None


def _inside_inclusive_window(value: Any, start: float, end: float) -> bool:
    stamp = _finite_number(value)
    return stamp is not None and start <= stamp <= end


def _numeric_diagnostics(rows: list[dict[str, str]], field: str) -> dict[str, Any]:
    values = [
        numeric
        for row in rows
        if (numeric := _finite_number(row.get(field))) is not None
    ]
    if not values:
        return {
            "count": 0,
            "invalid_count": len(rows),
            "min": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": len(values),
        "invalid_count": len(rows) - len(values),
        "min": float(np.min(array)),
        "p50": float(np.percentile(array, 50.0)),
        "p95": float(np.percentile(array, 95.0)),
        "p99": float(np.percentile(array, 99.0)),
        "max": float(np.max(array)),
    }


def _sequence_diagnostics(
    rows: list[dict[str, str]],
    *,
    sequence_field: str,
    stamp_field: str,
) -> dict[str, Any]:
    forward_gaps: list[dict[str, Any]] = []
    rollback_count = 0
    stamp_rollback_count = 0
    same_sequence_mutation_count = 0
    invalid_count = 0
    previous: tuple[int, float] | None = None
    for row in rows:
        sequence = _positive_integer(row.get(sequence_field))
        stamp = _finite_number(row.get(stamp_field))
        if sequence is None or stamp is None:
            invalid_count += 1
            continue
        if previous is not None:
            previous_sequence, previous_stamp = previous
            delta = sequence - previous_sequence
            if delta > 1:
                forward_gaps.append(
                    {
                        "previous_sequence": previous_sequence,
                        "current_sequence": sequence,
                        "missing_sequence_count": delta - 1,
                        "previous_source_stamp_s": previous_stamp,
                        "current_source_stamp_s": stamp,
                    }
                )
            elif delta < 0:
                rollback_count += 1
            elif delta == 0 and stamp != previous_stamp:
                same_sequence_mutation_count += 1
            if stamp < previous_stamp:
                stamp_rollback_count += 1
        previous = (sequence, stamp)
    return {
        "observation_count": len(rows),
        "invalid_count": invalid_count,
        "forward_gap_count": len(forward_gaps),
        "missing_sequence_count": sum(
            item["missing_sequence_count"] for item in forward_gaps
        ),
        "forward_gaps": forward_gaps,
        "rollback_count": rollback_count,
        "source_stamp_rollback_count": stamp_rollback_count,
        "same_sequence_mutation_count": same_sequence_mutation_count,
    }


def _safety_diagnostics(rows: list[dict[str, str]]) -> dict[str, Any]:
    return {
        "row_count": len(rows),
        "safety_direct_age_s": _numeric_diagnostics(
            rows, "safety_queued_age_s"
        ),
        "safety_receipt_gap_s": _numeric_diagnostics(
            rows, "safety_receipt_gap_s"
        ),
        "safety_source_stamp_gap_s": _numeric_diagnostics(
            rows, "safety_source_stamp_gap_s"
        ),
        "invalid_status_count": sum(
            _csv_bool(row.get("safety_valid")) is not True for row in rows
        ),
        "fault_status_count": sum(
            _csv_bool(row.get("safety_fault")) is not False for row in rows
        ),
        "emergency_stop_status_count": sum(
            _csv_bool(row.get("safety_emergency_stop")) is not False
            for row in rows
        ),
        "sequence": _sequence_diagnostics(
            rows,
            sequence_field="safety_source_sequence",
            stamp_field="safety_source_stamp_s",
        ),
    }


def _ros_delivery_diagnostics(rows: list[dict[str, str]]) -> dict[str, Any]:
    overwrite_values = [
        numeric
        for row in rows
        if (numeric := _finite_number(row.get("source_mailbox_overwrites")))
        is not None
    ]
    return {
        "row_count": len(rows),
        "ros_tactile_delivery_age_s": _numeric_diagnostics(rows, "data_age_s"),
        "evaluator_receipt_gap_s": _numeric_diagnostics(
            rows, "evaluator_receipt_gap_s"
        ),
        "source_mailbox_overwrites": {
            "count": len(overwrite_values),
            "invalid_count": len(rows) - len(overwrite_values),
            "nonzero_row_count": sum(value > 0.0 for value in overwrite_values),
            "total": float(sum(overwrite_values)),
            "max": float(max(overwrite_values, default=0.0)),
        },
        "sequence": _sequence_diagnostics(
            rows,
            sequence_field="source_sequence",
            stamp_field="timestamp_s",
        ),
    }


def _invalid_pngs(run: Path, required: Iterable[str]) -> list[str]:
    invalid: list[str] = []
    for name in sorted(item for item in required if item.endswith(".png")):
        path = run / name
        if not path.is_file() or path.stat().st_size == 0:
            continue
        try:
            decoded = plt.imread(path)
        except (OSError, SyntaxError, ValueError):
            invalid.append(name)
            continue
        if np.asarray(decoded).size == 0:
            invalid.append(name)
    return invalid


def validate_artifacts(root: Path, rows: list[dict[str, Any]]) -> tuple[str, int]:
    del root
    lines = ["# Artifact Validation", "", "Validation requires finite structured data and nonempty required artifacts.", ""]
    failures = 0
    for row in rows:
        if row.get("matrix_status") != "completed":
            reason = neutral_publication_text(str(row.get("failure_reason", "")))
            lines.append(f"- {row.get('test_id')}: `{row.get('matrix_status')}` — {reason}")
            continue
        run = Path(row["candidate_dir"])
        missing = sorted(
            name
            for name in REQUIRED_RUN_ARTIFACTS
            if not (run / name).is_file() or (run / name).stat().st_size == 0
        )
        if row.get("geometry") != "straight" and not (run / "lumen_evaluation.csv").is_file():
            missing.append("lumen_evaluation.csv")
        invalid_pngs = _invalid_pngs(run, REQUIRED_RUN_ARTIFACTS)
        recomputed, differences = independently_recompute_metrics(run, row)
        failed_differences = {
            key: value for key, value in differences.items() if value > 1.0e-12
        }
        evidence = validate_formal_window_evidence(run)
        evidence_markdown = format_formal_window_evidence_validation(evidence)
        failures += bool(
            missing or invalid_pngs or failed_differences or evidence["failures"]
        )
        lines.append(
            f"- {row.get('test_id')}: "
            f"{'PASS' if not missing and not invalid_pngs and not failed_differences and evidence['eligible'] else 'FAIL'}; "
            f"missing={missing}; invalid_pngs={invalid_pngs}; "
            f"recomputed={recomputed}; differences={differences}; "
            f"formal_window_evidence=({evidence_markdown})"
        )
    lines.extend(("", f"Completed rows with artifact failures: {failures}", ""))
    return "\n".join(lines), failures


def independently_recompute_metrics(
    run_dir: Path,
    reported: dict[str, Any],
) -> tuple[dict[str, float], dict[str, float]]:
    """Recompute paper metrics directly from persisted CSV bytes."""

    aligned = read_csv(run_dir / "aligned_samples.csv")
    lumen = read_csv(run_dir / "lumen_evaluation.csv")
    solves = read_csv(run_dir / "solve_timing.csv")
    result: dict[str, float] = {}
    if aligned:
        tip = np.asarray([[float(row[key]) for key in ("tip_x", "tip_y", "tip_z")] for row in aligned])
        target = np.asarray([[float(row[key]) for key in ("ref_x", "ref_y", "ref_z")] for row in aligned])
        errors = np.linalg.norm(tip - target, axis=1)
        result["final_target_error_m"] = float(errors[-1])
        result["tip_to_target_rmse_m"] = float(np.sqrt(np.mean(errors**2)))
        result["cartesian_path_length_m"] = float(np.sum(np.linalg.norm(np.diff(tip, axis=0), axis=1)))
        stamps = np.asarray([float(row["timestamp"]) for row in aligned])
        commands = np.asarray([[float(row[f"u{index}"]) for index in range(6)] for row in aligned])
        dt = np.concatenate(([0.0], np.maximum(np.diff(stamps), 0.0)))
        result["cumulative_control_effort"] = float(np.sum(np.sum(commands**2, axis=1) * dt))
    if lumen:
        radial = np.asarray([float(row["radial_offset_m"]) for row in lumen])
        clearance = np.asarray([float(row["physical_clearance_m"]) for row in lumen])
        collision = np.asarray([row["physical_collision"].lower() == "true" for row in lumen])
        result["centerline_tracking_rmse_m"] = float(np.sqrt(np.mean(radial**2)))
        result["minimum_clearance_m"] = float(np.min(clearance))
        result["collision_count"] = float(_transition_count(collision))
    if solves:
        stamps = np.asarray([float(row["timestamp"]) for row in solves])
        durations = np.asarray([float(row["solve_time"]) for row in solves])
        result["effective_solve_frequency_hz"] = (
            float((stamps.size - 1) / (stamps[-1] - stamps[0]))
            if stamps.size > 1 and stamps[-1] > stamps[0] else 0.0
        )
        result["solve_median_s"] = float(np.median(durations))
        result["solve_p95_s"] = float(np.percentile(durations, 95.0))
        result["solve_max_s"] = float(np.max(durations))
    differences: dict[str, float] = {}
    for key, value in result.items():
        expected = reported.get(key)
        if expected is None:
            differences[key] = math.inf
        else:
            differences[key] = abs(float(expected) - value)
    return result, differences


def _transition_count(flags: np.ndarray) -> int:
    previous = False
    count = 0
    for flag in flags:
        current = bool(flag)
        if current and not previous:
            count += 1
        previous = current
    return count


def paper_results(rows: list[dict[str, Any]], plot_index: dict[str, Any]) -> str:
    lines = ["# Final CTR Simulation Evidence", "", "All distances use metres, times use seconds, and frequencies use hertz.", ""]
    for experiment, heading in (
        ("reference", "Reference Run"), ("repeatability", "Seed Repeatability"), ("target_source", "Target-Source Comparison"),
        ("target_difficulty", "Target-Difficulty Comparison"), ("lumen_geometry", "Lumen-Geometry Comparison"),
        ("controller_configuration", "Controller Configuration Comparison"),
    ):
        source = [row for row in rows if row.get("experiment") == experiment and row.get("matrix_status") == "completed"]
        lines.extend((
            f"## {heading}", "",
            f"Source test IDs: {', '.join(str(row['test_id']) for row in source) or 'none'}", "",
            "Source run directories:", "",
            *(f"- `{row['candidate_dir']}`" for row in source), "",
            "The comparison panels report final target error, centerline-tracking RMSE, "
            "minimum whole-backbone clearance, and full-ROS effective solve frequency. "
            "Points are individual deterministic runs; diamonds are arithmetic means "
            "and whiskers are the observed sample range.", "",
        ))
        summary_parts: list[str] = []
        for metric in ("final_target_error_m", "tip_to_target_rmse_m", "centerline_tracking_rmse_m", "minimum_clearance_m", "effective_solve_frequency_hz"):
            values = finite_values(row.get(metric) for row in source)
            if values.size:
                lines.append(f"- {metric}: mean={np.mean(values):.9g}, std={np.std(values):.9g}, median={np.median(values):.9g}, range=[{np.min(values):.9g}, {np.max(values):.9g}] (n={values.size})")
                summary_parts.append(f"{metric} ranged from {np.min(values):.4g} to {np.max(values):.4g}")
        ieee = (
            f"Across {len(source)} completed simulator runs in the {heading.lower()} experiment, "
            + ("; ".join(summary_parts) if summary_parts else "the requested metrics were unavailable")
            + ". These deterministic comparisons characterize observed sensitivity without "
            "claiming statistical significance, hardware validity, or real-time performance."
        )
        lines.extend(("", f"Main quantitative conclusion: {ieee}", "", f"IEEE-style text: {ieee}", ""))
    lines.extend(("## Metric Semantics", ""))
    for name, definition in METRIC_DEFINITIONS.items():
        lines.append(f"- `{name}` [{definition['units']}]: {definition['formula']}; window: {definition['window']}; missing data: {definition['nan_policy']}.")
    lines.extend((
        "", "## Runtime and Limitations", "",
        "- All evidence is simulator-only and was collected on a non-real-time Ubuntu host.",
        "- The evaluated simulator physical-evidence watchdog is 0.20 s and remains fail-closed at that boundary.",
        "- The production/hardware freshness contract remains 0.10 s; the simulator watchdog is not production validation.",
        "- The controller and host are not claimed to provide real-time execution.",
        "- Five seeds do not justify statistical-significance claims.",
        "- Timing is host- and load-dependent and is not hardware certification.",
        "- Tactile stress evidence uses simulated force and production-equivalent thresholds; it is not physical sensor validation.",
        "",
    ))
    return "\n".join(lines)


def ensure_standard_plot_names(run_dir: Path) -> None:
    """Provide legacy paper-standard lumen filenames for a straight-lumen run."""

    ensure_straight_lumen_evidence(run_dir)
    clearance = run_dir / "curved_wall_clearance.png"
    trajectory = run_dir / "curved_lumen_trajectory_3d.png"
    centerline = run_dir / "centerline_tracking_error.png"
    if not clearance.is_file() and (run_dir / "wall_clearance.png").is_file():
        shutil.copy2(run_dir / "wall_clearance.png", clearance)
    if not trajectory.is_file() and (run_dir / "cylinder_backbone_target_3d.png").is_file():
        shutil.copy2(run_dir / "cylinder_backbone_target_3d.png", trajectory)
    if centerline.is_file():
        return
    state = read_csv(run_dir / "aligned_samples.csv")
    metadata = read_yaml(run_dir / "metadata.yaml") if (run_dir / "metadata.yaml").is_file() else {}
    cylinder = metadata.get("cylindrical_lumen", {})
    axis = np.asarray(cylinder.get("axis", [0.0, 0.0, 1.0]), dtype=float)
    axis = axis / np.linalg.norm(axis) if axis.shape == (3,) and np.linalg.norm(axis) > 0 else np.asarray([0.0, 0.0, 1.0])
    origin = np.asarray(cylinder.get("origin", [0.0, 0.0, 0.0]), dtype=float)
    times: list[float] = []
    distances: list[float] = []
    for item in state:
        try:
            tip = np.asarray([float(item[key]) for key in ("tip_x", "tip_y", "tip_z")])
            delta = tip - origin
            distances.append(float(np.linalg.norm(delta - np.dot(delta, axis) * axis)))
            times.append(float(item["timestamp"]))
        except (KeyError, TypeError, ValueError):
            continue
    fig, ax = plt.subplots(figsize=(7.1, 3.8))
    if times:
        ax.plot(np.asarray(times) - times[0], distances, color="#0072B2")
    else:
        ax.text(0.5, 0.5, "centerline evidence unavailable", transform=ax.transAxes, ha="center")
    ax.set_title("Centerline Tracking Error"); ax.set_xlabel("time [s]"); ax.set_ylabel("centerline distance [m]"); ax.grid(True, alpha=0.25)
    fig.tight_layout(); fig.savefig(centerline, dpi=300); plt.close(fig)


def ensure_straight_lumen_evidence(run_dir: Path) -> None:
    """Derive the common centerline schema from persisted straight-lumen evidence."""

    target = run_dir / "lumen_evaluation.csv"
    if target.is_file() or not (run_dir / "cylinder_navigation.csv").is_file():
        return
    aligned = read_csv(run_dir / "aligned_samples.csv")
    cylinder_rows = read_csv(run_dir / "cylinder_navigation.csv")
    metadata = read_yaml(run_dir / "metadata.yaml")
    cylinder = metadata.get("configuration", {}).get("cylindrical_lumen", metadata.get("cylindrical_lumen", {}))
    origin = np.asarray(cylinder.get("axis_origin", [0.0, 0.0, 0.0]), dtype=float)
    axis = np.asarray(cylinder.get("axis_direction", [0.0, 0.0, 1.0]), dtype=float)
    axis /= np.linalg.norm(axis)
    radius = float(cylinder.get("radius", 0.03))
    rows: list[dict[str, Any]] = []
    for sample, clearance in zip(aligned, cylinder_rows):
        tip = np.asarray([float(sample[key]) for key in ("tip_x", "tip_y", "tip_z")])
        delta = tip - origin
        axial = float(np.dot(delta, axis))
        center = origin + axial * axis
        radial = float(np.linalg.norm(tip - center))
        rows.append({
            "timestamp_s": float(sample["timestamp"]),
            "physical_clearance_m": float(clearance["minimum_backbone_clearance"]),
            "safety_clearance_m": float(clearance["minimum_backbone_clearance"]),
            "physical_collision": clearance["collision"],
            "safety_margin_violation": clearance["safety_margin_violation"],
            "tip_centerline_x": float(center[0]), "tip_centerline_y": float(center[1]),
            "tip_centerline_z": float(center[2]), "radial_offset_m": radial,
            "local_radius_m": radius,
        })
    write_csv(target, rows)


def build_manifest(root: Path) -> dict[str, Any]:
    members = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and path_safe(item, root)):
        relative = path.relative_to(root).as_posix()
        if relative == "manifest.json":
            continue
        members.append({"path": relative, "size": path.stat().st_size, "sha256": sha256(path)})
    return {
        "schema_version": "ctr_final_system_evidence_manifest_v1",
        "tested_commit": git_value("rev-parse", "HEAD"),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "runtime_scheduling": "non-real-time Ubuntu host",
        "controller_realtime_claim": False,
        "simulator_physical_evidence_freshness_timeout_s": 0.20,
        "production_hardware_freshness_timeout_s": 0.10,
        "simulator_watchdog_is_production_validation": False,
        "members": members,
    }


def forbidden_presentation_findings(root: Path) -> list[str]:
    findings = []
    publication_paths = [root / "paper_results.md", root / "artifact_validation.md"]
    publication_paths.extend((root / "paper_tables").glob("*.csv"))
    publication_paths.extend(
        path for path in (root / "overleaf_upload").iterdir()
        if path.is_file() and path.suffix.lower() in {".md", ".csv"}
    )
    for path in publication_paths:
        text = path.read_text(encoding="utf-8").lower() if path.is_file() else ""
        findings.extend(f"{path.name}:{phrase}" for phrase in FORBIDDEN_PRESENTATION_TEXT if phrase in text)
    index = read_json(root / "plot_index.json") if (root / "plot_index.json").is_file() else {}
    for name, record in index.items():
        visible = f"{record.get('title', '')} {record.get('caption', '')}".lower()
        findings.extend(f"{name}:{phrase}" for phrase in FORBIDDEN_PRESENTATION_TEXT if phrase in visible)
    return findings


def neutral_publication_text(value: str) -> str:
    """Remove internal legacy labels from publication-facing diagnostics only."""

    replacements = {
        "Slice 7G ": "",
        "slice_7g_": "",
        "slice-7g-": "",
    }
    result = value
    for source, replacement in replacements.items():
        result = result.replace(source, replacement)
    return result


def publication_row(row: dict[str, Any]) -> dict[str, Any]:
    """Return a publication-safe copy while retaining raw matrix provenance."""

    result = dict(row)
    if "failure_reason" in result:
        result["failure_reason"] = neutral_publication_text(str(result["failure_reason"]))
    return result


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row}) or ["status"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def append_jsonl(path: Path, value: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(value, allow_nan=False, separators=(",", ":")) + "\n")


def read_progress(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False, sort_keys=True) + "\n", encoding="utf-8")


def finite_values(values: Iterable[Any]) -> np.ndarray:
    result = []
    for value in values:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(numeric):
            result.append(numeric)
    return np.asarray(result, dtype=float)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def path_safe(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return not path.is_symlink()
    except ValueError:
        return False


def git_value(*args: str) -> str:
    return subprocess.run(("git", *args), check=True, text=True, stdout=subprocess.PIPE).stdout.strip()


def default_root() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path.cwd() / "evaluation_results" / f"final_system_{stamp}"


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = Path(args.output_root).resolve() if args.output_root else default_root().resolve()
    expected_parent = (Path.cwd() / "evaluation_results").resolve()
    if root.parent != expected_parent or not root.name.startswith("final_system_"):
        raise ValueError("output root must be a direct evaluation_results/final_system_* directory")
    if root.exists() and not (args.resume or args.aggregate_only):
        raise FileExistsError(f"output root already exists: {root}")
    root.mkdir(parents=True, exist_ok=True)
    rows = read_progress(root / "matrix_progress.jsonl")
    completed_ids = {row.get("test_id") for row in rows}
    selected_specs = select_specs(args.matrix)
    if not args.aggregate_only:
        for spec in selected_specs:
            if args.resume and spec.test_id in completed_ids:
                continue
            row = run_spec(root, spec, args.duration)
            rows.append(row)
            print(json.dumps({"test_id": spec.test_id, "status": row.get("matrix_status"), "candidate_dir": row.get("candidate_dir", "")}, allow_nan=False), flush=True)
    result = aggregate(root, rows, Path.cwd() / "config" / "tactile_params.yaml")
    findings = forbidden_presentation_findings(root)
    matrix_failures = validate_matrix_contract(rows, selected_specs)
    print(json.dumps({
        "output_root": str(root), **result,
        "forbidden_presentation_findings": findings,
        "matrix_contract_failures": matrix_failures,
    }, indent=2, allow_nan=False))
    return 0 if (
        not findings
        and not matrix_failures
        and result["completed"]
        and result["artifact_validation_failures"] == 0
    ) else 2


if __name__ == "__main__":
    sys.exit(main())
