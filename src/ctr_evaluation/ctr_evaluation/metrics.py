"""ROS-independent quantitative evaluation metrics.

The formulas in this module are the offline evaluation definitions for the
project. Online controller diagnostics may publish a subset for runtime
inspection, but rigorous run reports should use timestamp-aligned samples and
these functions.
"""

from __future__ import annotations

import hashlib
import math
import os
from dataclasses import asdict, dataclass, field
from statistics import NormalDist
from typing import Any, Iterable

import numpy as np

from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen


NO_TRANSIENT_REACHED = -1.0
TARGET_IDENTITY_ATOL = 1.0e-9
METRIC_DIRECTIONS = {
    "rmse": "lower",
    "mean_error": "lower",
    "median_error": "lower",
    "p95_error": "lower",
    "max_error": "lower",
    "final_error": "lower",
    "steady_state_error": "lower",
    "time_to_first_tolerance_entry": "lower",
    "transient_duration": "lower",
    "time_inside_tolerance_percentage": "higher",
    "total_control_effort": "lower",
    "insertion_control_effort": "lower",
    "rotation_control_effort": "lower",
    "total_command_variation": "lower",
    "saturation_percentage": "lower",
    "mean_solve_time": "lower",
    "median_solve_time": "lower",
    "p95_solve_time": "lower",
    "max_solve_time": "lower",
    "effective_solve_frequency": "higher",
    "deadline_overrun_percentage": "lower",
    "valid_aligned_sample_count": "higher",
    "final_goal_error": "lower",
    "minimum_tip_error": "lower",
    "goal_hold_duration": "higher",
    "time_inside_goal_tolerance_percentage": "higher",
    "minimum_backbone_wall_clearance": "higher",
    "mean_minimum_backbone_clearance": "higher",
    "p05_clearance": "higher",
    "safety_margin_violation_duration": "lower",
    "radial_collision_count": "lower",
    "radial_collision_duration": "lower",
    "inlet_violation_count": "lower",
    "outlet_violation_count": "lower",
    "maximum_penetration_depth": "lower",
    "tip_path_length": "lower",
    "joint_space_path_length": "lower",
    "path_efficiency": "higher",
}

CURVED_LUMEN_TASK = "curved_lumen_navigation"
CURVED_LUMEN_SCHEMA_VERSION = "lumen_evaluation_v1"
COMPARISON_SCHEMA_VERSION = "curved_comparison_v1"
CURVED_NUMERIC_COMPARISON_METRICS = (
    ("curved_final_target_error", ("goal", "final_goal_error"), "lower"),
    ("curved_rms_target_error", ("goal", "rmse"), "lower"),
    (
        "curved_minimum_physical_clearance",
        ("lumen_evaluation", "physical_safety", "minimum_physical_clearance_m"),
        "higher",
    ),
    (
        "curved_collision_duration",
        ("lumen_evaluation", "physical_safety", "collision_duration_s"),
        "lower",
    ),
    (
        "curved_minimum_safety_clearance",
        ("lumen_evaluation", "safety_margin", "minimum_safety_clearance_m"),
        "higher",
    ),
    (
        "curved_safety_margin_violation_duration",
        ("lumen_evaluation", "safety_margin", "violation_duration_s"),
        "lower",
    ),
    (
        "curved_final_normalized_progress",
        ("lumen_evaluation", "progress", "final_normalized_progress"),
        "higher",
    ),
    (
        "curved_maximum_normalized_progress",
        ("lumen_evaluation", "progress", "maximum_normalized_progress"),
        "higher",
    ),
)
CURVED_BOOLEAN_COMPARISON_FIELDS = (
    ("goal_success", ("navigation", "goal_success")),
    ("physical_safety_pass", ("lumen_evaluation", "physical_safety", "physical_safety_pass")),
    ("safety_margin_pass", ("lumen_evaluation", "safety_margin", "safety_margin_pass")),
    ("navigation_success", ("navigation", "navigation_success")),
)
CURVED_IDENTITY_FIELDS = (
    "task",
    "reference_mode",
    "curved_lumen_type",
    "scenario_id",
    "scenario_policy_version",
    "scenario_fingerprint",
    "geometry_frame",
    "geometry_fingerprint",
    "expected_geometry_fingerprint",
    "reconstructed_geometry_fingerprint",
    "geometry_fingerprint_match",
    "shared_environment_hash",
    "derived_target",
    "requested_target",
    "executed_target",
    "override_used",
)


@dataclass(frozen=True)
class EvaluationThresholds:
    configured_duration: float
    configured_control_frequency: float
    tracking_tolerance: float
    transient_stable_cycles: int
    steady_state_window: float
    steady_state_fraction: float
    minimum_valid_sample_count: int
    maximum_invalid_sample_percentage: float
    maximum_saturation_percentage: float
    maximum_deadline_overrun_percentage: float
    required_minimum_baseline_improvement: float
    near_zero_baseline_epsilon: float

    @property
    def control_period(self) -> float:
        return 1.0 / self.configured_control_frequency


@dataclass(frozen=True)
class TrackingMetrics:
    rmse: float
    mean_error: float
    median_error: float
    p95_error: float
    max_error: float
    final_error: float
    steady_state_error: float
    time_to_first_tolerance_entry: float
    transient_duration: float
    time_inside_tolerance_percentage: float
    path_completion_percentage: float


@dataclass(frozen=True)
class ControlMetrics:
    total_control_effort: float
    insertion_control_effort: float
    rotation_control_effort: float
    total_command_variation: float
    command_rms_per_joint: list[float]
    maximum_command_per_joint: list[float]
    saturation_count: int
    saturation_percentage: float
    missing_command_sample_count: int


@dataclass(frozen=True)
class ControlEffortSeries:
    interval_durations: list[float]
    cumulative_total_effort: list[float]
    cumulative_insertion_effort: list[float]
    cumulative_rotation_effort: list[float]
    total_control_effort: float
    insertion_control_effort: float
    rotation_control_effort: float


@dataclass(frozen=True)
class TimingMetrics:
    mean_solve_time: float
    median_solve_time: float
    p95_solve_time: float
    max_solve_time: float
    effective_solve_frequency: float
    configured_control_frequency: float
    deadline_overrun_count: int
    deadline_overrun_percentage: float
    state_publication_rate: float
    reference_publication_rate: float
    command_publication_rate: float
    experiment_wall_duration: float
    valid_aligned_evaluation_duration: float


@dataclass(frozen=True)
class NumericalSafetyMetrics:
    nonfinite_state_samples: int
    nonfinite_reference_samples: int
    nonfinite_command_samples: int
    malformed_dimension_count: int
    command_limit_violation_count: int
    state_limit_violation_count: int
    saturation_count: int
    missing_required_topic_count: int


@dataclass(frozen=True)
class DataQualityMetrics:
    raw_state_sample_count: int
    raw_reference_sample_count: int
    raw_command_sample_count: int
    valid_aligned_sample_count: int
    rejected_aligned_sample_count: int
    invalid_nonfinite_sample_count: int
    mean_alignment_gap: float
    maximum_alignment_gap: float
    reference_interpolation_count: int
    nearest_reference_fallback_count: int
    missing_command_count: int
    missing_topic_count: int
    missing_backbone_sample_count: int = 0


@dataclass(frozen=True)
class GoalMetrics:
    initial_tip_error: float
    final_goal_error: float
    minimum_tip_error: float
    mean_tip_error: float
    rmse: float
    goal_reached: bool
    time_to_goal: float
    goal_hold_duration: float
    time_inside_goal_tolerance_percentage: float


@dataclass(frozen=True)
class LumenSafetyMetrics:
    minimum_backbone_wall_clearance: float
    mean_minimum_backbone_clearance: float
    p05_clearance: float
    minimum_axial_end_cap_clearance: float
    safety_margin_violation_count: int
    safety_margin_violation_duration: float
    radial_collision_count: int
    radial_collision_duration: float
    inlet_violation_count: int
    outlet_violation_count: int
    maximum_penetration_depth: float
    closest_backbone_point_index_over_time: list[int]
    collision_free_pass: bool
    safety_margin_pass: bool
    missing_backbone_sample_count: int


@dataclass(frozen=True)
class MotionMetrics:
    tip_path_length: float
    joint_space_path_length: float
    straight_line_target_distance: float
    path_efficiency: float
    total_control_effort: float
    insertion_control_effort: float
    rotation_control_effort: float
    maximum_command_per_joint: list[float]
    command_saturation_count: int


@dataclass(frozen=True)
class AcceptanceResults:
    functional_pass: bool
    goal_reached_pass: bool
    collision_free_pass: bool
    safety_margin_pass: bool
    numerical_safety_pass: bool
    data_quality_pass: bool
    baseline_improvement_pass: bool
    timing_pass: bool
    real_time_pass: bool
    physical_validation_pass: bool
    hardware_validation_pass: bool
    reasons: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class EvaluationSummary:
    tracking: TrackingMetrics
    control: ControlMetrics
    timing: TimingMetrics
    numerical_safety: NumericalSafetyMetrics
    data_quality: DataQualityMetrics
    acceptance: AcceptanceResults

    def to_dict(self) -> dict[str, Any]:
        return dataclass_to_plain(self)


@dataclass(frozen=True)
class MetricComparison:
    metric: str
    direction: str
    candidate_value: float
    baseline_value: float
    absolute_difference: float
    relative_improvement_percent: float | None
    comparison_valid: bool
    compatibility_valid: bool
    reason: str


@dataclass(frozen=True)
class BooleanMetricComparison:
    metric: str
    candidate_value: bool | None
    baseline_value: bool | None
    comparison_valid: bool
    improved: bool | None
    reason: str


@dataclass(frozen=True)
class ComparisonResult:
    compatibility_valid: bool
    compatibility_reasons: list[str]
    compatibility_details: dict[str, Any]
    metric_comparisons: list[MetricComparison]
    comparison_schema_version: str = COMPARISON_SCHEMA_VERSION
    pair_identity_compatible: bool = True
    baseline_run_valid: bool | None = None
    candidate_run_valid: bool | None = None
    baseline_invalid_reasons: list[str] = field(default_factory=list)
    candidate_invalid_reasons: list[str] = field(default_factory=list)
    comparison_valid: bool = True
    improvement_evaluated: bool = False
    improvement_pass: bool | None = None
    boolean_comparisons: list[BooleanMetricComparison] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return dataclass_to_plain(self)


def compute_tracking_metrics(
    *,
    times: Any,
    tip_positions: Any,
    reference_positions: Any,
    tolerance: float,
    stable_cycles: int,
    steady_state_window: float,
    steady_state_fraction: float,
    path_progress: Any | None = None,
) -> TrackingMetrics:
    time_values = _vector(times, "times", allow_empty=True)
    tip = _matrix3(tip_positions, "tip_positions")
    reference = _matrix3(reference_positions, "reference_positions")
    if tip.shape[0] != reference.shape[0] or tip.shape[0] != time_values.shape[0]:
        raise ValueError("times, tip_positions, and reference_positions must have matching lengths")
    if tip.shape[0] == 0:
        return TrackingMetrics(*([math.nan] * 11))

    relative_times = _relative_times(time_values)
    errors = np.linalg.norm(tip - reference, axis=1)
    tol = _positive_number(tolerance, "tolerance")
    stable = _positive_int(stable_cycles, "stable_cycles")
    window = _nonnegative_number(steady_state_window, "steady_state_window")
    fraction = _bounded_fraction(steady_state_fraction, "steady_state_fraction")

    steady_values = _steady_state_errors(relative_times, errors, window, fraction)
    inside = errors <= tol
    progress = _path_completion_percentage(path_progress)

    return TrackingMetrics(
        rmse=float(math.sqrt(float(np.mean(errors**2)))),
        mean_error=float(np.mean(errors)),
        median_error=float(np.median(errors)),
        p95_error=float(np.percentile(errors, 95.0)),
        max_error=float(np.max(errors)),
        final_error=float(errors[-1]),
        steady_state_error=float(np.mean(steady_values)) if steady_values.size else math.nan,
        time_to_first_tolerance_entry=_time_to_first_tolerance(relative_times, inside),
        transient_duration=_transient_duration(relative_times, inside, stable),
        time_inside_tolerance_percentage=float(100.0 * np.mean(inside)),
        path_completion_percentage=progress,
    )


def compute_control_metrics(
    *,
    times: Any,
    commands: Any,
    saturation_flags: Any | None = None,
    missing_command_flags: Any | None = None,
) -> ControlMetrics:
    time_values = _vector(times, "times", allow_empty=True)
    command_array = _matrix6(commands, "commands")
    if command_array.shape[0] != time_values.shape[0]:
        raise ValueError("times and commands must have matching lengths")
    if command_array.shape[0] == 0:
        return ControlMetrics(
            total_control_effort=0.0,
            insertion_control_effort=0.0,
            rotation_control_effort=0.0,
            total_command_variation=0.0,
            command_rms_per_joint=[math.nan] * 6,
            maximum_command_per_joint=[0.0] * 6,
            saturation_count=0,
            saturation_percentage=0.0,
            missing_command_sample_count=0,
        )

    effort = compute_control_effort_series(times=time_values, commands=command_array)
    saturation = _bool_flags(saturation_flags, command_array.shape[0], "saturation_flags")
    missing = _bool_flags(missing_command_flags, command_array.shape[0], "missing_command_flags")

    variation = 0.0
    if command_array.shape[0] > 1:
        variation = float(np.sum(np.linalg.norm(np.diff(command_array, axis=0), axis=1)))

    return ControlMetrics(
        total_control_effort=effort.total_control_effort,
        insertion_control_effort=effort.insertion_control_effort,
        rotation_control_effort=effort.rotation_control_effort,
        total_command_variation=variation,
        command_rms_per_joint=[float(value) for value in np.sqrt(np.mean(command_array**2, axis=0))],
        maximum_command_per_joint=[float(value) for value in np.max(np.abs(command_array), axis=0)],
        saturation_count=int(np.sum(saturation)),
        saturation_percentage=float(100.0 * np.mean(saturation)),
        missing_command_sample_count=int(np.sum(missing)),
    )


def compute_control_effort_series(*, times: Any, commands: Any) -> ControlEffortSeries:
    time_values = _vector(times, "times", allow_empty=True)
    command_array = _matrix6(commands, "commands")
    if command_array.shape[0] != time_values.shape[0]:
        raise ValueError("times and commands must have matching lengths")

    dt = _sample_durations(time_values)
    if command_array.shape[0] == 0:
        return ControlEffortSeries([], [], [], [], 0.0, 0.0, 0.0)

    insertion = command_array[:, :3]
    rotation = command_array[:, 3:]
    total_increment = np.sum(command_array**2, axis=1) * dt
    insertion_increment = np.sum(insertion**2, axis=1) * dt
    rotation_increment = np.sum(rotation**2, axis=1) * dt
    total = np.cumsum(total_increment)
    insertion_total = np.cumsum(insertion_increment)
    rotation_total = np.cumsum(rotation_increment)
    return ControlEffortSeries(
        interval_durations=[float(value) for value in dt],
        cumulative_total_effort=[float(value) for value in total],
        cumulative_insertion_effort=[float(value) for value in insertion_total],
        cumulative_rotation_effort=[float(value) for value in rotation_total],
        total_control_effort=float(total[-1]),
        insertion_control_effort=float(insertion_total[-1]),
        rotation_control_effort=float(rotation_total[-1]),
    )


def compute_timing_metrics(
    *,
    solve_times: Any,
    solve_timestamps: Any,
    state_timestamps: Any,
    reference_timestamps: Any,
    command_timestamps: Any,
    configured_control_frequency: float,
    experiment_wall_duration: float,
    valid_aligned_evaluation_duration: float,
) -> TimingMetrics:
    control_frequency = _positive_number(configured_control_frequency, "configured_control_frequency")
    period = 1.0 / control_frequency
    solve = _vector(solve_times, "solve_times", allow_empty=True, require_sorted=False)
    solve_stamp = _vector(solve_timestamps, "solve_timestamps", allow_empty=True)
    state_stamp = _vector(state_timestamps, "state_timestamps", allow_empty=True)
    reference_stamp = _vector(reference_timestamps, "reference_timestamps", allow_empty=True)
    command_stamp = _vector(command_timestamps, "command_timestamps", allow_empty=True)
    wall_duration = _nonnegative_number(experiment_wall_duration, "experiment_wall_duration")
    aligned_duration = _nonnegative_number(valid_aligned_evaluation_duration, "valid_aligned_evaluation_duration")

    if solve.size:
        deadline_count = int(np.sum(solve > period))
        deadline_percent = float(100.0 * deadline_count / solve.size)
        mean = float(np.mean(solve))
        median = float(np.median(solve))
        p95 = float(np.percentile(solve, 95.0))
        maximum = float(np.max(solve))
    else:
        deadline_count = 0
        deadline_percent = 0.0
        mean = median = p95 = maximum = math.nan

    return TimingMetrics(
        mean_solve_time=mean,
        median_solve_time=median,
        p95_solve_time=p95,
        max_solve_time=maximum,
        effective_solve_frequency=publication_rate(solve_stamp),
        configured_control_frequency=control_frequency,
        deadline_overrun_count=deadline_count,
        deadline_overrun_percentage=deadline_percent,
        state_publication_rate=publication_rate(state_stamp),
        reference_publication_rate=publication_rate(reference_stamp),
        command_publication_rate=publication_rate(command_stamp),
        experiment_wall_duration=wall_duration,
        valid_aligned_evaluation_duration=aligned_duration,
    )


def compute_goal_metrics(
    *,
    times: Any,
    tip_positions: Any,
    goal_position: Any,
    tolerance: float,
    required_hold_duration: float,
) -> GoalMetrics:
    time_values = _vector(times, "times", allow_empty=True)
    tip = _matrix3(tip_positions, "tip_positions")
    goal = _array_shape(goal_position, "goal_position", (3,))
    if tip.shape[0] != time_values.shape[0]:
        raise ValueError("times and tip_positions must have matching lengths")
    if tip.shape[0] == 0:
        return GoalMetrics(math.nan, math.nan, math.nan, math.nan, math.nan, False, NO_TRANSIENT_REACHED, 0.0, 0.0)
    tol = _positive_number(tolerance, "tolerance")
    required_hold = _nonnegative_number(required_hold_duration, "required_hold_duration")
    relative_times = _relative_times(time_values)
    errors = np.linalg.norm(tip - goal, axis=1)
    inside = errors <= tol
    durations = _sample_durations(relative_times)
    hold_duration = _maximum_contiguous_duration(inside, durations)
    time_to_goal = _time_to_hold(relative_times, inside, durations, required_hold)
    total_duration = float(np.sum(durations))
    inside_duration = _total_true_span_duration(inside, durations)
    inside_percentage = 100.0 * inside_duration / total_duration if total_duration > 0.0 else float(100.0 * np.mean(inside))
    return GoalMetrics(
        initial_tip_error=float(errors[0]),
        final_goal_error=float(errors[-1]),
        minimum_tip_error=float(np.min(errors)),
        mean_tip_error=float(np.mean(errors)),
        rmse=float(math.sqrt(float(np.mean(errors**2)))),
        goal_reached=bool(hold_duration >= required_hold and bool(np.any(inside))),
        time_to_goal=time_to_goal,
        goal_hold_duration=hold_duration,
        time_inside_goal_tolerance_percentage=float(inside_percentage),
    )


def compute_lumen_safety_metrics(
    *,
    times: Any,
    backbone_points: list[np.ndarray | None],
    lumen: CylindricalLumen,
) -> LumenSafetyMetrics:
    time_values = _vector(times, "times", allow_empty=True)
    if len(backbone_points) != time_values.shape[0]:
        raise ValueError("times and backbone_points must have matching lengths")
    if time_values.size == 0:
        return LumenSafetyMetrics(math.nan, math.nan, math.nan, math.nan, 0, 0.0, 0, 0.0, 0, 0, 0.0, [], False, False, 0)

    durations = _sample_durations(_relative_times(time_values))
    minimum_clearances: list[float] = []
    axial_clearances: list[float] = []
    closest_indices: list[int] = []
    safety_flags: list[bool] = []
    radial_collision_flags: list[bool] = []
    inlet_flags: list[bool] = []
    outlet_flags: list[bool] = []
    penetration_depths: list[float] = []
    missing = 0
    for points in backbone_points:
        if points is None:
            missing += 1
            minimum_clearances.append(math.nan)
            axial_clearances.append(math.nan)
            closest_indices.append(-1)
            safety_flags.append(True)
            radial_collision_flags.append(True)
            inlet_flags.append(True)
            outlet_flags.append(True)
            penetration_depths.append(math.nan)
            continue
        clearance = lumen.backbone_clearance(points)
        minimum_clearances.append(clearance.minimum_radial_clearance)
        axial_clearances.append(clearance.minimum_axial_clearance)
        closest_indices.append(clearance.closest_backbone_point_index)
        safety_flags.append(clearance.safety_margin_violation_count > 0)
        radial_collision_flags.append(bool(np.any(clearance.radial_collision_mask)))
        inlet_flags.append(bool(np.any(clearance.inlet_violation_mask)))
        outlet_flags.append(bool(np.any(clearance.outlet_violation_mask)))
        penetration_depths.append(clearance.maximum_penetration_depth)

    min_clearance_array = np.asarray([value for value in minimum_clearances if math.isfinite(value)], dtype=float)
    axial_array = np.asarray([value for value in axial_clearances if math.isfinite(value)], dtype=float)
    penetration_array = np.asarray([value for value in penetration_depths if math.isfinite(value)], dtype=float)
    safety = np.asarray(safety_flags, dtype=bool)
    radial = np.asarray(radial_collision_flags, dtype=bool)
    inlet = np.asarray(inlet_flags, dtype=bool)
    outlet = np.asarray(outlet_flags, dtype=bool)
    collision = radial | inlet | outlet
    return LumenSafetyMetrics(
        minimum_backbone_wall_clearance=float(np.min(min_clearance_array)) if min_clearance_array.size else math.nan,
        mean_minimum_backbone_clearance=float(np.mean(min_clearance_array)) if min_clearance_array.size else math.nan,
        p05_clearance=float(np.percentile(min_clearance_array, 5.0)) if min_clearance_array.size else math.nan,
        minimum_axial_end_cap_clearance=float(np.min(axial_array)) if axial_array.size else math.nan,
        safety_margin_violation_count=int(np.sum(safety)),
        safety_margin_violation_duration=float(np.sum(durations[safety])),
        radial_collision_count=int(np.sum(radial)),
        radial_collision_duration=float(np.sum(durations[radial])),
        inlet_violation_count=int(np.sum(inlet)),
        outlet_violation_count=int(np.sum(outlet)),
        maximum_penetration_depth=float(np.max(penetration_array)) if penetration_array.size else math.nan,
        closest_backbone_point_index_over_time=closest_indices,
        collision_free_pass=bool(not np.any(collision) and missing == 0),
        safety_margin_pass=bool(not np.any(safety) and missing == 0),
        missing_backbone_sample_count=int(missing),
    )


def compute_motion_metrics(
    *,
    times: Any,
    tip_positions: Any,
    q_values: Any,
    goal_position: Any,
    control: ControlMetrics,
) -> MotionMetrics:
    time_values = _vector(times, "times", allow_empty=True)
    tip = _matrix3(tip_positions, "tip_positions")
    q = _array_shape(q_values, "q_values", (-1, 6))
    goal = _array_shape(goal_position, "goal_position", (3,))
    if tip.shape[0] != time_values.shape[0] or q.shape[0] != time_values.shape[0]:
        raise ValueError("times, tip_positions, and q_values must have matching lengths")
    tip_path = float(np.sum(np.linalg.norm(np.diff(tip, axis=0), axis=1))) if tip.shape[0] > 1 else 0.0
    joint_path = float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1))) if q.shape[0] > 1 else 0.0
    straight = float(np.linalg.norm(goal - tip[0])) if tip.shape[0] else math.nan
    efficiency = float(straight / tip_path) if tip_path > 1.0e-12 and math.isfinite(straight) else math.nan
    return MotionMetrics(
        tip_path_length=tip_path,
        joint_space_path_length=joint_path,
        straight_line_target_distance=straight,
        path_efficiency=efficiency,
        total_control_effort=control.total_control_effort,
        insertion_control_effort=control.insertion_control_effort,
        rotation_control_effort=control.rotation_control_effort,
        maximum_command_per_joint=list(control.maximum_command_per_joint),
        command_saturation_count=control.saturation_count,
    )


def compute_acceptance(
    *,
    tracking: TrackingMetrics,
    control: ControlMetrics,
    timing: TimingMetrics,
    numerical_safety: NumericalSafetyMetrics,
    data_quality: DataQualityMetrics,
    thresholds: EvaluationThresholds,
    baseline_improvement_valid: bool,
    goal: GoalMetrics | None = None,
    lumen_safety: LumenSafetyMetrics | None = None,
    physical_validation: bool = False,
    hardware_validation: bool = False,
) -> AcceptanceResults:
    reasons: list[str] = []
    functional_pass = data_quality.valid_aligned_sample_count >= thresholds.minimum_valid_sample_count
    if not functional_pass:
        reasons.append("valid aligned sample count below threshold")

    goal_reached_pass = True if goal is None else bool(goal.goal_reached)
    if not goal_reached_pass:
        reasons.append("goal tolerance hold requirement was not met")

    collision_free_pass = True if lumen_safety is None else bool(lumen_safety.collision_free_pass)
    if not collision_free_pass:
        reasons.append("backbone collision or missing backbone data was recorded")

    safety_margin_pass = True if lumen_safety is None else bool(lumen_safety.safety_margin_pass)
    if not safety_margin_pass:
        reasons.append("backbone safety-margin violation was recorded")

    numerical_safety_pass = (
        numerical_safety.nonfinite_state_samples == 0
        and numerical_safety.nonfinite_reference_samples == 0
        and numerical_safety.nonfinite_command_samples == 0
        and numerical_safety.malformed_dimension_count == 0
        and numerical_safety.command_limit_violation_count == 0
        and numerical_safety.state_limit_violation_count == 0
        and numerical_safety.missing_required_topic_count == 0
    )
    if not numerical_safety_pass:
        reasons.append("numerical safety violations were recorded")

    total_alignment_attempts = data_quality.valid_aligned_sample_count + data_quality.rejected_aligned_sample_count
    invalid_percent = 0.0
    if total_alignment_attempts:
        invalid_percent = 100.0 * data_quality.rejected_aligned_sample_count / total_alignment_attempts
    data_quality_pass = (
        data_quality.valid_aligned_sample_count >= thresholds.minimum_valid_sample_count
        and invalid_percent <= thresholds.maximum_invalid_sample_percentage
    )
    if not data_quality_pass:
        reasons.append("data quality thresholds were not met")

    timing_pass = timing.deadline_overrun_percentage <= thresholds.maximum_deadline_overrun_percentage
    real_time_pass = timing_pass and timing.mean_solve_time <= thresholds.control_period

    saturation_pass = control.saturation_percentage <= thresholds.maximum_saturation_percentage
    if not saturation_pass:
        reasons.append("command saturation exceeded threshold")
    numerical_safety_pass = numerical_safety_pass and saturation_pass

    return AcceptanceResults(
        functional_pass=functional_pass,
        goal_reached_pass=goal_reached_pass,
        collision_free_pass=collision_free_pass,
        safety_margin_pass=safety_margin_pass,
        numerical_safety_pass=numerical_safety_pass,
        data_quality_pass=data_quality_pass,
        baseline_improvement_pass=baseline_improvement_valid,
        timing_pass=timing_pass,
        real_time_pass=real_time_pass,
        physical_validation_pass=bool(physical_validation),
        hardware_validation_pass=bool(hardware_validation),
        reasons=reasons,
    )


def compare_summaries(
    *,
    candidate_summary: dict[str, Any],
    baseline_summary: dict[str, Any],
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    near_zero_epsilon: float,
    duration_tolerance: float,
    initial_state_tolerance: float,
) -> ComparisonResult:
    compatibility = compatibility_report_for(
        candidate_metadata=candidate_metadata,
        baseline_metadata=baseline_metadata,
        duration_tolerance=duration_tolerance,
        initial_state_tolerance=initial_state_tolerance,
    )
    compatibility_reasons = list(compatibility["reasons"])
    curved_comparison = _curved_comparison_requested(candidate_metadata, baseline_metadata)
    baseline_validity = _curved_run_validity(baseline_summary, baseline_metadata, "baseline") if curved_comparison else _validity_result(True)
    candidate_validity = _curved_run_validity(candidate_summary, candidate_metadata, "candidate") if curved_comparison else _validity_result(True)
    if curved_comparison:
        curved_compatibility = _curved_compatibility_report(
            candidate_summary=candidate_summary,
            baseline_summary=baseline_summary,
            candidate_metadata=candidate_metadata,
            baseline_metadata=baseline_metadata,
            candidate_validity=candidate_validity,
            baseline_validity=baseline_validity,
            duration_tolerance=duration_tolerance,
            initial_state_tolerance=initial_state_tolerance,
        )
        compatibility_reasons.extend(curved_compatibility["reasons"])
        if any(
            reason.startswith("required_curved_identity_")
            or reason in {"geometry_fingerprint_not_valid", "geometry_fingerprint_inconsistent"}
            for reason in baseline_validity["reasons"] + candidate_validity["reasons"]
        ):
            compatibility_reasons.append("curved_identity_unavailable")
        compatibility["details"]["curved_mismatch_details"] = curved_compatibility["details"]
    compatibility_reasons = _unique_strings(compatibility_reasons)
    pair_identity_compatible = not compatibility_reasons
    comparison_valid = pair_identity_compatible and baseline_validity["valid"] and candidate_validity["valid"]
    metric_pairs = _flatten_numeric_metrics(candidate_summary)
    baseline_pairs = _flatten_numeric_metrics(baseline_summary)
    comparisons: list[MetricComparison] = []
    boolean_comparisons: list[BooleanMetricComparison] = []
    if not curved_comparison or comparison_valid:
        if curved_comparison:
            comparisons = _curved_metric_comparisons(
                candidate_summary=candidate_summary,
                baseline_summary=baseline_summary,
                near_zero_epsilon=near_zero_epsilon,
            )
            boolean_comparisons = _curved_boolean_comparisons(candidate_summary, baseline_summary)
        else:
            for name, candidate_value in sorted(metric_pairs.items()):
                if name not in baseline_pairs or name not in METRIC_DIRECTIONS:
                    continue
                direction = METRIC_DIRECTIONS[name]
                baseline_value = baseline_pairs[name]
                improvement, valid, reason = relative_improvement_percent(
                    candidate_value=candidate_value,
                    baseline_value=baseline_value,
                    lower_is_better=direction == "lower",
                    near_zero_epsilon=near_zero_epsilon,
                )
                comparisons.append(
                    MetricComparison(
                        metric=name,
                        direction=direction,
                        candidate_value=float(candidate_value),
                        baseline_value=float(baseline_value),
                        absolute_difference=float(candidate_value - baseline_value),
                        relative_improvement_percent=improvement,
                        comparison_valid=bool(valid and pair_identity_compatible),
                        compatibility_valid=pair_identity_compatible,
                        reason=reason if pair_identity_compatible else "; ".join(compatibility_reasons),
                    )
                )
    improvement_evaluated = bool(comparisons or boolean_comparisons) and comparison_valid
    return ComparisonResult(
        compatibility_valid=pair_identity_compatible,
        compatibility_reasons=compatibility_reasons,
        compatibility_details=compatibility["details"],
        metric_comparisons=comparisons,
        pair_identity_compatible=pair_identity_compatible,
        baseline_run_valid=baseline_validity["valid"] if curved_comparison else None,
        candidate_run_valid=candidate_validity["valid"] if curved_comparison else None,
        baseline_invalid_reasons=baseline_validity["reasons"] if curved_comparison else [],
        candidate_invalid_reasons=candidate_validity["reasons"] if curved_comparison else [],
        comparison_valid=comparison_valid,
        improvement_evaluated=improvement_evaluated,
        improvement_pass=None,
        boolean_comparisons=boolean_comparisons,
    )


def _curved_comparison_requested(candidate_metadata: dict[str, Any], baseline_metadata: dict[str, Any]) -> bool:
    return candidate_metadata.get("task") == CURVED_LUMEN_TASK or baseline_metadata.get("task") == CURVED_LUMEN_TASK


def _validity_result(valid: bool, reasons: list[str] | None = None) -> dict[str, Any]:
    return {"valid": valid, "reasons": [] if reasons is None else _unique_strings(reasons), "details": {}}


def _curved_run_validity(summary: dict[str, Any], metadata: dict[str, Any], role: str) -> dict[str, Any]:
    reasons: list[str] = []
    details: dict[str, Any] = {}
    values: dict[str, Any] = {}
    if not isinstance(summary, dict):
        reasons.append("summary_malformed")
        return _validity_result(False, reasons)
    if metadata.get("task") != CURVED_LUMEN_TASK:
        reasons.append("task_mismatch")
    lumen = summary.get("lumen_evaluation")
    if not isinstance(lumen, dict):
        reasons.append("lumen_evaluation_missing")
        return _validity_result(False, reasons)
    if lumen.get("schema_version") != CURVED_LUMEN_SCHEMA_VERSION:
        reasons.append("unsupported_lumen_schema")
    if lumen.get("available") is not True:
        reasons.append("lumen_evaluation_unavailable")
    if lumen.get("run_valid") is not True:
        reasons.append("run_invalid")
    identity = lumen.get("identity")
    if not isinstance(identity, dict):
        reasons.append("required_curved_identity_missing")
        identity = {}
    for field_name in CURVED_IDENTITY_FIELDS:
        kind = "bool" if field_name in {"geometry_fingerprint_match", "override_used"} else (
            "vector" if field_name in {"derived_target", "requested_target", "executed_target"} else "string"
        )
        _validate_required_curved_field(
            field_name,
            identity.get(field_name),
            kind=kind,
            reasons=reasons,
            values=values,
            details=details,
            vector_length=3 if kind == "vector" else None,
        )
    if identity.get("geometry_fingerprint_match") is not True:
        reasons.append("geometry_fingerprint_not_valid")

    geometry = lumen.get("geometry")
    if not isinstance(geometry, dict):
        reasons.append("required_curved_identity_missing:geometry")
        geometry = {}
    for field_name in (
        "fingerprint",
        "ctr_outer_radius_m",
        "safety_margin_m",
        "minimum_lumen_radius_m",
        "maximum_lumen_radius_m",
    ):
        kind = "string" if field_name == "fingerprint" else "number"
        _validate_required_curved_field(
            f"geometry.{field_name}",
            geometry.get(field_name),
            kind=kind,
            reasons=reasons,
            values=values,
            details=details,
        )
    expected_fingerprint = values.get("expected_geometry_fingerprint")
    reconstructed_fingerprint = values.get("reconstructed_geometry_fingerprint")
    geometry_fingerprint = values.get("geometry.fingerprint")
    if (
        expected_fingerprint is not None
        and reconstructed_fingerprint is not None
        and geometry_fingerprint is not None
        and (
            expected_fingerprint != reconstructed_fingerprint
            or expected_fingerprint != geometry_fingerprint
        )
    ):
        reasons.append("geometry_fingerprint_inconsistent")
        details["geometry_fingerprint_inconsistent"] = {
            "expected": sanitize_for_json(expected_fingerprint),
            "reconstructed": sanitize_for_json(reconstructed_fingerprint),
            "geometry": sanitize_for_json(geometry_fingerprint),
        }

    configuration = metadata.get("configuration")
    if not isinstance(configuration, dict):
        configuration = {}
        reasons.append("required_curved_identity_missing:configuration")
    goal_configuration = configuration.get("goal")
    if not isinstance(goal_configuration, dict):
        goal_configuration = {}
        reasons.append("required_curved_identity_missing:target_tolerance")
    _validate_required_curved_field(
        "target_tolerance",
        goal_configuration.get("tolerance"),
        kind="number",
        reasons=reasons,
        values=values,
        details=details,
    )
    _validate_required_curved_field(
        "evaluation_window_duration_s",
        metadata.get("evaluation_window_duration_s"),
        kind="number",
        reasons=reasons,
        values=values,
        details=details,
    )
    for field_name in (
        "model_configuration_hash",
        "frame_id",
        "software_mode",
        "configured_control_period",
        "reference_sample_period",
    ):
        kind = "number" if field_name in {"configured_control_period", "reference_sample_period"} else "string"
        _validate_required_curved_field(
            f"configuration.{field_name}",
            configuration.get(field_name),
            kind=kind,
            reasons=reasons,
            values=values,
            details=details,
        )
    for field_name, value in (
        ("initial_state_q", metadata.get("initial_state_q")),
        ("initial_tip_position", metadata.get("initial_tip_position")),
    ):
        _validate_required_curved_field(
            field_name,
            value,
            kind="vector",
            reasons=reasons,
            values=values,
            details=details,
            vector_length=6 if field_name == "initial_state_q" else 3,
        )
    goal = summary.get("goal")
    if not isinstance(goal, dict):
        reasons.append("goal_metrics_missing")
    physical = lumen.get("physical_safety")
    if not isinstance(physical, dict) or physical.get("physical_safety_pass") is None:
        reasons.append("physical_safety_metrics_missing")
    for metric_name, path, _direction in CURVED_NUMERIC_COMPARISON_METRICS:
        value = _nested_value(summary, path)
        if value is None:
            reasons.append(f"required_metric_missing:{metric_name}")
        elif not _is_finite_number(value):
            reasons.append(f"required_metric_nonfinite:{metric_name}")
    for metric_name, path in CURVED_BOOLEAN_COMPARISON_FIELDS:
        value = _nested_value(summary, path)
        if not isinstance(value, bool):
            reasons.append(f"required_boolean_metric_missing:{metric_name}")
    details["role"] = role
    details["unavailable_reasons"] = lumen.get("unavailable_reasons", [])
    return {
        "valid": not _unique_strings(reasons),
        "reasons": _unique_strings(reasons),
        "details": details,
        "values": values,
    }


def _validate_required_curved_field(
    field_name: str,
    value: Any,
    *,
    kind: str,
    reasons: list[str],
    values: dict[str, Any],
    details: dict[str, Any],
    vector_length: int | None = None,
) -> None:
    if value is None or (isinstance(value, str) and value == ""):
        reasons.append(f"required_curved_identity_missing:{field_name}")
        return
    if kind == "string":
        if not isinstance(value, str):
            reasons.append(f"required_curved_identity_malformed:{field_name}")
            return
        values[field_name] = value
        return
    if kind == "bool":
        if not isinstance(value, bool):
            reasons.append(f"required_curved_identity_malformed:{field_name}")
            return
        values[field_name] = value
        return
    if kind == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
            reasons.append(f"required_curved_identity_malformed:{field_name}")
            return
        if not _is_finite_number(value):
            reasons.append(f"required_curved_identity_nonfinite:{field_name}")
            return
        values[field_name] = float(value)
        return
    if kind == "vector":
        try:
            array = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            reasons.append(f"required_curved_identity_malformed:{field_name}")
            return
        if vector_length is not None and array.shape != (vector_length,):
            reasons.append(f"required_curved_identity_malformed:{field_name}")
            return
        if not np.all(np.isfinite(array)):
            reasons.append(f"required_curved_identity_nonfinite:{field_name}")
            return
        values[field_name] = array.tolist()
        return
    raise ValueError(f"unsupported curved required-field kind: {kind}")


def _curved_compatibility_report(
    *,
    candidate_summary: dict[str, Any],
    baseline_summary: dict[str, Any],
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    candidate_validity: dict[str, Any],
    baseline_validity: dict[str, Any],
    duration_tolerance: float,
    initial_state_tolerance: float,
) -> dict[str, Any]:
    reasons: list[str] = []
    details: list[dict[str, Any]] = []
    candidate_lumen = candidate_summary.get("lumen_evaluation", {}) if isinstance(candidate_summary, dict) else {}
    baseline_lumen = baseline_summary.get("lumen_evaluation", {}) if isinstance(baseline_summary, dict) else {}
    candidate_identity = candidate_lumen.get("identity", {}) if isinstance(candidate_lumen, dict) else {}
    baseline_identity = baseline_lumen.get("identity", {}) if isinstance(baseline_lumen, dict) else {}

    candidate_values = candidate_validity.get("values", {})
    baseline_values = baseline_validity.get("values", {})
    required_fields = (
        "geometry.fingerprint",
        "target_tolerance",
        "evaluation_window_duration_s",
        "configuration.model_configuration_hash",
        "configuration.frame_id",
        "configuration.software_mode",
        "configuration.configured_control_period",
        "configuration.reference_sample_period",
        "geometry.ctr_outer_radius_m",
        "geometry.safety_margin_m",
        "geometry.minimum_lumen_radius_m",
        "geometry.maximum_lumen_radius_m",
        "initial_state_q",
        "initial_tip_position",
    )
    for field_name in required_fields:
        candidate_value = candidate_values.get(field_name)
        baseline_value = baseline_values.get(field_name)
        if candidate_value is None or baseline_value is None:
            reasons.append(f"required_curved_identity_unavailable:{field_name}")
            details.append(
                _mismatch_detail(
                    "required_curved_identity_unavailable",
                    field_name,
                    baseline_value,
                    candidate_value,
                )
            )
            continue
        tolerance = (
            duration_tolerance
            if field_name == "evaluation_window_duration_s"
            else initial_state_tolerance
            if field_name in {"initial_state_q", "initial_tip_position"}
            else TARGET_IDENTITY_ATOL
            if field_name in {
                "target_tolerance",
                "configuration.configured_control_period",
                "configuration.reference_sample_period",
                "geometry.ctr_outer_radius_m",
                "geometry.safety_margin_m",
                "geometry.minimum_lumen_radius_m",
                "geometry.maximum_lumen_radius_m",
            }
            else None
        )
        if not _values_match(candidate_value, baseline_value, tolerance=tolerance):
            code = "duration_mismatch" if field_name == "evaluation_window_duration_s" else (
                "shared_environment_hash_mismatch"
                if field_name == "shared_environment_hash"
                else "geometry_fingerprint_mismatch"
                if field_name == "geometry.fingerprint"
                else "simulator_config_mismatch"
                if field_name.startswith("configuration.")
                else f"{field_name}_mismatch"
            )
            reasons.append(code)
            details.append(_mismatch_detail(code, field_name, baseline_value, candidate_value, tolerance))

    for field_name in CURVED_IDENTITY_FIELDS:
        candidate_value = _curved_identity_value(field_name, candidate_identity, candidate_metadata)
        baseline_value = _curved_identity_value(field_name, baseline_identity, baseline_metadata)
        tolerance = TARGET_IDENTITY_ATOL if field_name in {"derived_target", "requested_target", "executed_target"} else None
        if not _values_match(candidate_value, baseline_value, tolerance=tolerance):
            code = _curved_mismatch_code(field_name)
            reasons.append(code)
            details.append(_mismatch_detail(code, field_name, baseline_value, candidate_value, tolerance))

    candidate_schema = candidate_lumen.get("schema_version") if isinstance(candidate_lumen, dict) else None
    baseline_schema = baseline_lumen.get("schema_version") if isinstance(baseline_lumen, dict) else None
    if candidate_schema != baseline_schema:
        reasons.append("lumen_schema_mismatch")
        details.append(_mismatch_detail("lumen_schema_mismatch", "lumen_evaluation.schema_version", baseline_schema, candidate_schema))

    candidate_summary_schema = candidate_summary.get("schema_version") if isinstance(candidate_summary, dict) else None
    baseline_summary_schema = baseline_summary.get("schema_version") if isinstance(baseline_summary, dict) else None
    if candidate_summary_schema is not None or baseline_summary_schema is not None:
        if candidate_summary_schema != baseline_summary_schema:
            reasons.append("summary_schema_mismatch")
            details.append(_mismatch_detail("summary_schema_mismatch", "summary.schema_version", baseline_summary_schema, candidate_summary_schema))

    candidate_target_tolerance = _target_tolerance(candidate_metadata)
    baseline_target_tolerance = _target_tolerance(baseline_metadata)
    if not _values_match(candidate_target_tolerance, baseline_target_tolerance, tolerance=TARGET_IDENTITY_ATOL):
        reasons.append("target_tolerance_mismatch")
        details.append(_mismatch_detail("target_tolerance_mismatch", "configuration.goal.tolerance", baseline_target_tolerance, candidate_target_tolerance, TARGET_IDENTITY_ATOL))

    for field_name, path in (
        ("ctr_outer_radius_m", ("geometry", "ctr_outer_radius_m")),
        ("safety_margin_m", ("geometry", "safety_margin_m")),
    ):
        candidate_value = _nested_value(candidate_lumen, path)
        baseline_value = _nested_value(baseline_lumen, path)
        if candidate_value is None or baseline_value is None:
            continue
        if not _values_match(candidate_value, baseline_value, tolerance=TARGET_IDENTITY_ATOL):
            reasons.append("geometry_fingerprint_mismatch")
            details.append(_mismatch_detail("geometry_fingerprint_mismatch", f"lumen_evaluation.{field_name}", baseline_value, candidate_value, TARGET_IDENTITY_ATOL))

    for field_name, candidate_value, baseline_value in (
        ("evaluation_window_duration_s", candidate_metadata.get("evaluation_window_duration_s"), baseline_metadata.get("evaluation_window_duration_s")),
        ("actual_evaluation_window_duration_s", candidate_metadata.get("actual_evaluation_window_duration_s"), baseline_metadata.get("actual_evaluation_window_duration_s")),
    ):
        if candidate_value is not None and baseline_value is not None and not _values_match(candidate_value, baseline_value, tolerance=duration_tolerance):
            reasons.append("duration_mismatch")
            details.append(_mismatch_detail("duration_mismatch", field_name, baseline_value, candidate_value, duration_tolerance))

    return {"reasons": _unique_strings(reasons), "details": details}


def _curved_metric_comparisons(*, candidate_summary: dict[str, Any], baseline_summary: dict[str, Any], near_zero_epsilon: float) -> list[MetricComparison]:
    comparisons: list[MetricComparison] = []
    for name, path, direction in CURVED_NUMERIC_COMPARISON_METRICS:
        candidate_value = float(_nested_value(candidate_summary, path))
        baseline_value = float(_nested_value(baseline_summary, path))
        improvement, valid, reason = relative_improvement_percent(
            candidate_value=candidate_value,
            baseline_value=baseline_value,
            lower_is_better=direction == "lower",
            near_zero_epsilon=near_zero_epsilon,
        )
        comparisons.append(
            MetricComparison(
                metric=name,
                direction=direction,
                candidate_value=candidate_value,
                baseline_value=baseline_value,
                absolute_difference=candidate_value - baseline_value,
                relative_improvement_percent=improvement,
                comparison_valid=valid,
                compatibility_valid=True,
                reason=reason,
            )
        )
    return comparisons


def _curved_boolean_comparisons(candidate_summary: dict[str, Any], baseline_summary: dict[str, Any]) -> list[BooleanMetricComparison]:
    comparisons: list[BooleanMetricComparison] = []
    for name, path in CURVED_BOOLEAN_COMPARISON_FIELDS:
        candidate_value = _nested_value(candidate_summary, path)
        baseline_value = _nested_value(baseline_summary, path)
        improved = candidate_value and not baseline_value
        comparisons.append(
            BooleanMetricComparison(
                metric=name,
                candidate_value=candidate_value,
                baseline_value=baseline_value,
                comparison_valid=True,
                improved=bool(improved) if isinstance(improved, bool) else None,
                reason="ok",
            )
        )
    return comparisons


def _curved_identity_value(field_name: str, identity: dict[str, Any], metadata: dict[str, Any]) -> Any:
    if field_name == "task":
        return metadata.get("task")
    if field_name in identity:
        return identity.get(field_name)
    return metadata.get(field_name)


def _curved_mismatch_code(field_name: str) -> str:
    return {
        "task": "task_mismatch",
        "reference_mode": "reference_mode_mismatch",
        "curved_lumen_type": "curved_lumen_type_mismatch",
        "scenario_id": "scenario_id_mismatch",
        "scenario_policy_version": "scenario_policy_version_mismatch",
        "scenario_fingerprint": "scenario_fingerprint_mismatch",
        "geometry_frame": "geometry_frame_mismatch",
        "geometry_fingerprint": "geometry_fingerprint_mismatch",
        "expected_geometry_fingerprint": "geometry_fingerprint_mismatch",
        "reconstructed_geometry_fingerprint": "geometry_fingerprint_mismatch",
        "geometry_fingerprint_match": "geometry_fingerprint_not_valid",
        "shared_environment_hash": "shared_environment_hash_mismatch",
        "derived_target": "derived_target_mismatch",
        "requested_target": "requested_target_mismatch",
        "executed_target": "executed_target_mismatch",
        "override_used": "override_state_mismatch",
    }[field_name]


def _mismatch_detail(code: str, field: str, baseline: Any, candidate: Any, tolerance: float | None = None) -> dict[str, Any]:
    detail = {"code": code, "field": field, "baseline": sanitize_for_json(baseline), "candidate": sanitize_for_json(candidate)}
    if tolerance is not None:
        detail["tolerance"] = float(tolerance)
    return detail


def _nested_value(value: Any, path: tuple[str, ...]) -> Any:
    current = value
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _configuration_value(metadata: dict[str, Any], key: str) -> Any:
    configuration = metadata.get("configuration", {})
    return configuration.get(key) if isinstance(configuration, dict) else None


def _target_tolerance(metadata: dict[str, Any]) -> Any:
    configuration = metadata.get("configuration", {})
    goal = configuration.get("goal", {}) if isinstance(configuration, dict) else {}
    if isinstance(goal, dict) and goal.get("tolerance") is not None:
        return goal.get("tolerance")
    return metadata.get("target_identity_tolerance")


def _values_match(candidate: Any, baseline: Any, *, tolerance: float | None) -> bool:
    if candidate is None or baseline is None:
        return candidate is baseline
    if tolerance is None:
        return candidate == baseline
    try:
        candidate_array = np.asarray(candidate, dtype=float)
        baseline_array = np.asarray(baseline, dtype=float)
        if candidate_array.shape != baseline_array.shape or not np.all(np.isfinite(candidate_array)) or not np.all(np.isfinite(baseline_array)):
            return False
        return bool(np.allclose(candidate_array, baseline_array, atol=tolerance, rtol=0.0))
    except (TypeError, ValueError):
        return False


def _is_finite_number(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float, np.integer, np.floating)) and math.isfinite(float(value))


def _unique_strings(values: Iterable[str]) -> list[str]:
    result: list[str] = []
    for value in values:
        if value not in result:
            result.append(value)
    return result


def relative_improvement_percent(
    *,
    candidate_value: float,
    baseline_value: float,
    lower_is_better: bool,
    near_zero_epsilon: float,
) -> tuple[float | None, bool, str]:
    candidate = _number(candidate_value, "candidate_value")
    baseline = _number(baseline_value, "baseline_value")
    epsilon = _positive_number(near_zero_epsilon, "near_zero_epsilon")
    if abs(baseline) <= epsilon:
        return None, False, "baseline value is near zero"
    if lower_is_better:
        value = 100.0 * (baseline - candidate) / baseline
    else:
        value = 100.0 * (candidate - baseline) / baseline
    return float(value), True, "ok"


def compatibility_reasons_for(
    *,
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    duration_tolerance: float,
    initial_state_tolerance: float,
) -> list[str]:
    return compatibility_report_for(
        candidate_metadata=candidate_metadata,
        baseline_metadata=baseline_metadata,
        duration_tolerance=duration_tolerance,
        initial_state_tolerance=initial_state_tolerance,
    )["reasons"]


def compatibility_report_for(
    *,
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    duration_tolerance: float,
    initial_state_tolerance: float,
) -> dict[str, Any]:
    reasons: list[str] = []
    details: dict[str, Any] = {}
    candidate_config = candidate_metadata.get("configuration", {})
    baseline_config = baseline_metadata.get("configuration", {})
    for key in (
        "trajectory_type",
        "trajectory_parameters_hash",
        "cylindrical_lumen_hash",
        "goal_configuration_hash",
        "frame_id",
        "model_configuration_hash",
        "software_mode",
        "configured_control_period",
        "reference_sample_period",
    ):
        if candidate_config.get(key) != baseline_config.get(key):
            reasons.append(f"incompatible {key}")
    details["candidate_shared_environment_hash"] = candidate_metadata.get("shared_environment_hash")
    details["baseline_shared_environment_hash"] = baseline_metadata.get("shared_environment_hash")
    if (
        candidate_metadata.get("shared_environment_hash") is not None
        or baseline_metadata.get("shared_environment_hash") is not None
    ) and candidate_metadata.get("shared_environment_hash") != baseline_metadata.get("shared_environment_hash"):
        reasons.append("incompatible shared_environment_hash")
    details["candidate_controller_configuration_hash"] = candidate_metadata.get("controller_configuration_hash")
    details["baseline_controller_configuration_hash"] = baseline_metadata.get("controller_configuration_hash")
    duration_tol = _nonnegative_number(duration_tolerance, "duration_tolerance")
    candidate_duration = _optional_float(candidate_config.get("configured_duration"))
    baseline_duration = _optional_float(baseline_config.get("configured_duration"))
    if candidate_duration is not None and baseline_duration is not None:
        if abs(candidate_duration - baseline_duration) > duration_tol:
            reasons.append("configured duration differs beyond tolerance")
    candidate_actual_duration = _optional_float(candidate_metadata.get("actual_duration"))
    baseline_actual_duration = _optional_float(baseline_metadata.get("actual_duration"))
    if candidate_actual_duration is not None and baseline_actual_duration is not None:
        if abs(candidate_actual_duration - baseline_actual_duration) > duration_tol:
            reasons.append("actual duration differs beyond tolerance")
    candidate_window_duration = _optional_float(candidate_metadata.get("evaluation_window_duration_s"))
    baseline_window_duration = _optional_float(baseline_metadata.get("evaluation_window_duration_s"))
    details["candidate_evaluation_window_duration_s"] = candidate_window_duration
    details["baseline_evaluation_window_duration_s"] = baseline_window_duration
    if candidate_window_duration is not None and baseline_window_duration is not None:
        if abs(candidate_window_duration - baseline_window_duration) > duration_tol:
            reasons.append("evaluation-window duration differs beyond tolerance")
    candidate_actual_window_duration = _optional_float(candidate_metadata.get("actual_evaluation_window_duration_s"))
    baseline_actual_window_duration = _optional_float(baseline_metadata.get("actual_evaluation_window_duration_s"))
    details["candidate_actual_evaluation_window_duration_s"] = candidate_actual_window_duration
    details["baseline_actual_evaluation_window_duration_s"] = baseline_actual_window_duration
    if candidate_actual_window_duration is not None and baseline_actual_window_duration is not None:
        if abs(candidate_actual_window_duration - baseline_actual_window_duration) > duration_tol:
            reasons.append("actual evaluation-window duration differs beyond tolerance")
    initial_tol = _nonnegative_number(initial_state_tolerance, "initial_state_tolerance")
    candidate_initial = candidate_metadata.get("initial_state_q")
    baseline_initial = baseline_metadata.get("initial_state_q")
    if candidate_initial is not None and baseline_initial is not None:
        try:
            delta = np.linalg.norm(_array_shape(candidate_initial, "candidate_initial_state_q", (6,)) - _array_shape(
                baseline_initial,
                "baseline_initial_state_q",
                (6,),
            ))
            details["initial_q_difference"] = float(delta)
            if delta > initial_tol:
                reasons.append("initial state differs beyond tolerance")
        except ValueError:
            reasons.append("initial state is malformed")
    orchestrated = _is_orchestrated_comparison(candidate_metadata, baseline_metadata)
    if orchestrated:
        reasons.extend(
            _orchestration_compatibility_reasons(
                candidate_metadata,
                baseline_metadata,
                details,
                duration_tolerance=duration_tol,
                initial_state_tolerance=initial_tol,
            )
        )
    return {"reasons": reasons, "details": details}


def _is_orchestrated_comparison(candidate_metadata: dict[str, Any], baseline_metadata: dict[str, Any]) -> bool:
    return bool(
        candidate_metadata.get("orchestration_id")
        or baseline_metadata.get("orchestration_id")
        or candidate_metadata.get("run_role")
        or baseline_metadata.get("run_role")
    )


def _orchestration_compatibility_reasons(
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    details: dict[str, Any],
    *,
    duration_tolerance: float,
    initial_state_tolerance: float,
) -> list[str]:
    reasons: list[str] = []
    for key in (
        "shared_environment_hash",
        "reference_start_policy",
        "reference_lead_duration_s",
        "reference_phase_offset_s",
        "reference_pre_epoch_behavior",
        "evaluation_window_duration_s",
        "initial_state_q",
        "initial_tip_position",
        "baseline_nonzero_command_count",
        "candidate_command_after_recording",
    ):
        owner = baseline_metadata if key.startswith("baseline_") else candidate_metadata
        if key == "initial_tip_position":
            if candidate_metadata.get(key) is None or baseline_metadata.get(key) is None:
                reasons.append("required orchestration metadata missing: initial_tip_position")
            continue
        if key == "initial_state_q":
            if candidate_metadata.get(key) is None or baseline_metadata.get(key) is None:
                reasons.append("required orchestration metadata missing: initial_state_q")
            continue
        if key == "candidate_command_after_recording":
            if candidate_metadata.get(key) is None:
                reasons.append("required orchestration metadata missing: candidate_command_after_recording")
            continue
        if owner.get(key) is None:
            reasons.append(f"required orchestration metadata missing: {key}")

    for key in ("reference_start_policy", "reference_pre_epoch_behavior"):
        details[f"candidate_{key}"] = candidate_metadata.get(key)
        details[f"baseline_{key}"] = baseline_metadata.get(key)
        if candidate_metadata.get(key) != baseline_metadata.get(key):
            reasons.append(f"incompatible {key}")

    for key in ("reference_lead_duration_s", "reference_phase_offset_s", "requested_evaluation_duration_s"):
        candidate_value = _optional_float(candidate_metadata.get(key))
        baseline_value = _optional_float(baseline_metadata.get(key))
        details[f"candidate_{key}"] = candidate_value
        details[f"baseline_{key}"] = baseline_value
        if candidate_value is not None and baseline_value is not None:
            if abs(candidate_value - baseline_value) > duration_tolerance:
                reasons.append(f"incompatible {key}")

    candidate_tip = candidate_metadata.get("initial_tip_position")
    baseline_tip = baseline_metadata.get("initial_tip_position")
    if candidate_tip is not None and baseline_tip is not None:
        try:
            tip_delta = np.linalg.norm(
                _array_shape(candidate_tip, "candidate_initial_tip_position", (3,))
                - _array_shape(baseline_tip, "baseline_initial_tip_position", (3,))
            )
            details["initial_tip_difference"] = float(tip_delta)
            tip_tol = _orchestration_tip_tolerance(candidate_metadata, baseline_metadata, initial_state_tolerance)
            details["initial_tip_tolerance"] = tip_tol
            if tip_delta > tip_tol:
                reasons.append("initial tip differs beyond tolerance")
        except ValueError:
            reasons.append("initial tip is malformed")

    baseline_nonzero = _optional_float(baseline_metadata.get("baseline_nonzero_command_count"))
    details["baseline_nonzero_command_count"] = baseline_nonzero
    if baseline_nonzero is not None and baseline_nonzero > 0.0:
        reasons.append("baseline received prohibited nonzero command")
    pre_roll_nonzero = _optional_float(baseline_metadata.get("pre_roll_nonzero_command_count"))
    details["baseline_pre_roll_nonzero_command_count"] = pre_roll_nonzero
    if pre_roll_nonzero is not None and pre_roll_nonzero > 0.0:
        reasons.append("baseline pre-roll received prohibited nonzero command")

    candidate_after_recording = candidate_metadata.get("candidate_command_after_recording")
    details["candidate_command_after_recording"] = candidate_after_recording
    if candidate_after_recording is not True:
        reasons.append("candidate command occurred before recording or is missing")
    if _requires_target_identity(candidate_metadata, baseline_metadata):
        reasons.extend(_target_identity_compatibility_reasons(candidate_metadata, baseline_metadata, details))
    return reasons


def _requires_target_identity(candidate_metadata: dict[str, Any], baseline_metadata: dict[str, Any]) -> bool:
    for metadata in (candidate_metadata, baseline_metadata):
        config = metadata.get("configuration", {})
        if isinstance(config, dict):
            goal = config.get("goal", {})
            if config.get("cylindrical_lumen") is not None:
                return True
            if isinstance(goal, dict) and goal.get("position") is not None:
                return True
        if any(_metadata_value(metadata, key) is not None for key in ("requested_target", "executed_target", "target_replaced")):
            return True
    return False


def _target_identity_compatibility_reasons(
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    details: dict[str, Any],
) -> list[str]:
    reasons: list[str] = []
    for role, metadata in (("candidate", candidate_metadata), ("baseline", baseline_metadata)):
        requested = _metadata_value(metadata, "requested_target")
        executed = _metadata_value(metadata, "executed_target")
        replaced = _metadata_value(metadata, "target_replaced")
        identity_valid = _metadata_value(metadata, "target_identity_valid")
        reference_matches = _metadata_value(metadata, "reference_matches_requested_target")
        details[f"{role}_requested_target"] = requested
        details[f"{role}_executed_target"] = executed
        details[f"{role}_target_replaced"] = replaced
        details[f"{role}_target_identity_valid"] = identity_valid
        details[f"{role}_reference_matches_requested_target"] = reference_matches
        if requested is None or executed is None or replaced is None or identity_valid is None:
            reasons.append(f"required target identity metadata missing: {role}")
            continue
        if replaced is not False:
            reasons.append(f"{role} target was replaced")
        if identity_valid is not True:
            reasons.append(f"{role} target identity is invalid")
        try:
            requested_array = _array_shape(requested, f"{role}_requested_target", (3,))
            executed_array = _array_shape(executed, f"{role}_executed_target", (3,))
            requested_executed_delta = float(np.linalg.norm(requested_array - executed_array))
            details[f"{role}_requested_executed_target_difference"] = requested_executed_delta
            if not np.allclose(requested_array, executed_array, atol=TARGET_IDENTITY_ATOL, rtol=0.0):
                reasons.append(f"{role} requested_target differs from executed_target")
        except ValueError:
            reasons.append(f"{role} target identity is malformed")
        if reference_matches is not True:
            reasons.append(f"{role} published reference target does not match requested_target")

    for key in ("requested_target", "executed_target"):
        candidate_value = _metadata_value(candidate_metadata, key)
        baseline_value = _metadata_value(baseline_metadata, key)
        if candidate_value is None or baseline_value is None:
            continue
        try:
            candidate_array = _array_shape(candidate_value, f"candidate_{key}", (3,))
            baseline_array = _array_shape(baseline_value, f"baseline_{key}", (3,))
            delta = float(np.linalg.norm(candidate_array - baseline_array))
            details[f"baseline_candidate_{key}_difference"] = delta
            if not np.allclose(candidate_array, baseline_array, atol=TARGET_IDENTITY_ATOL, rtol=0.0):
                reasons.append(f"baseline and candidate {key} differ")
        except ValueError:
            reasons.append(f"baseline/candidate {key} is malformed")
    return reasons


def _metadata_value(metadata: dict[str, Any], key: str) -> Any:
    if key in metadata:
        return metadata[key]
    override = metadata.get("metadata_override", {})
    if isinstance(override, dict) and key in override:
        return override[key]
    runtime = metadata.get("orchestration_runtime", {})
    if isinstance(runtime, dict) and key in runtime:
        return runtime[key]
    return None


def _orchestration_tip_tolerance(
    candidate_metadata: dict[str, Any],
    baseline_metadata: dict[str, Any],
    fallback: float,
) -> float:
    for metadata in (candidate_metadata, baseline_metadata):
        value = _optional_float(metadata.get("baseline_candidate_tip_tolerance"))
        if value is not None:
            return _nonnegative_number(value, "baseline_candidate_tip_tolerance")
    return fallback


def aggregate_trial_summaries(summaries: Iterable[dict[str, Any]]) -> dict[str, Any]:
    flattened = [_flatten_numeric_metrics(summary) for summary in summaries]
    keys = sorted({key for item in flattened for key in item})
    aggregate: dict[str, Any] = {"count": len(flattened), "metrics": {}}
    for key in keys:
        values = np.asarray([item[key] for item in flattened if key in item and math.isfinite(item[key])], dtype=float)
        if values.size == 0:
            continue
        metric = {
            "mean": float(np.mean(values)),
            "standard_deviation": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
            "median": float(np.median(values)),
            "minimum": float(np.min(values)),
            "maximum": float(np.max(values)),
        }
        if values.size >= 2:
            stderr = metric["standard_deviation"] / math.sqrt(values.size)
            z95 = NormalDist().inv_cdf(0.975)
            metric["confidence_interval_95"] = [float(metric["mean"] - z95 * stderr), float(metric["mean"] + z95 * stderr)]
        aggregate["metrics"][key] = metric
    return aggregate


def publication_rate(timestamps: Any) -> float:
    values = _vector(timestamps, "timestamps", allow_empty=True)
    if values.size < 2:
        return 0.0
    duration = float(values[-1] - values[0])
    if duration <= 0.0:
        return 0.0
    return float((values.size - 1) / duration)


def stable_hash(value: Any) -> str:
    plain = dataclass_to_plain(value)
    encoded = repr(_sorted_plain(plain)).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def dataclass_to_plain(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return dataclass_to_plain(asdict(value))
    if isinstance(value, dict):
        return {str(key): dataclass_to_plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [dataclass_to_plain(item) for item in value]
    if isinstance(value, np.ndarray):
        return dataclass_to_plain(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return value
    return value


def sanitize_for_json(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return sanitize_for_json(asdict(value))
    if isinstance(value, dict):
        return {str(key): sanitize_for_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_for_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return sanitize_for_json(value.tolist())
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return sanitize_for_json(value.item())
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, os.PathLike):
        return str(value)
    return str(value)


def _flatten_numeric_metrics(summary: dict[str, Any]) -> dict[str, float]:
    flat: dict[str, float] = {}
    for section in ("tracking", "control", "timing", "data_quality", "goal", "lumen_safety", "motion"):
        values = summary.get(section, {})
        if not isinstance(values, dict):
            continue
        for key, value in values.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)):
                flat[key] = float(value)
    return flat


def _steady_state_errors(times: np.ndarray, errors: np.ndarray, window: float, fraction: float) -> np.ndarray:
    if errors.size == 0:
        return np.asarray([], dtype=float)
    if window > 0.0 and times[-1] > 0.0:
        start = max(times[0], times[-1] - window)
        selected = errors[times >= start]
        if selected.size:
            return selected
    count = max(1, int(math.ceil(errors.size * fraction)))
    return errors[-count:]


def _time_to_first_tolerance(relative_times: np.ndarray, inside: np.ndarray) -> float:
    indices = np.nonzero(inside)[0]
    if indices.size == 0:
        return NO_TRANSIENT_REACHED
    return float(relative_times[int(indices[0])])


def _transient_duration(relative_times: np.ndarray, inside: np.ndarray, stable_cycles: int) -> float:
    if inside.size < stable_cycles:
        return NO_TRANSIENT_REACHED
    streak = 0
    start_index = 0
    for index, value in enumerate(inside):
        if value:
            if streak == 0:
                start_index = index
            streak += 1
            if streak >= stable_cycles:
                return float(relative_times[start_index])
        else:
            streak = 0
    return NO_TRANSIENT_REACHED


def _sample_durations(times: np.ndarray) -> np.ndarray:
    if times.size == 0:
        return np.asarray([], dtype=float)
    if times.size == 1:
        return np.asarray([0.0], dtype=float)
    deltas = np.diff(times)
    return np.concatenate([[0.0], np.maximum(deltas, 0.0)])


def _maximum_contiguous_duration(flags: np.ndarray, durations: np.ndarray) -> float:
    maximum = 0.0
    current = 0.0
    previous = False
    for flag, duration in zip(flags, durations):
        if flag:
            if previous:
                current += float(duration)
            maximum = max(maximum, current)
        else:
            current = 0.0
        previous = bool(flag)
    return float(maximum)


def _time_to_hold(times: np.ndarray, flags: np.ndarray, durations: np.ndarray, required_hold: float) -> float:
    if required_hold == 0.0:
        indices = np.nonzero(flags)[0]
        return float(times[int(indices[0])]) if indices.size else NO_TRANSIENT_REACHED
    current = 0.0
    start_time = 0.0
    in_streak = False
    for time_value, flag, duration in zip(times, flags, durations):
        if flag:
            if not in_streak:
                start_time = float(time_value)
                in_streak = True
                current = 0.0
            else:
                current += float(duration)
            if current >= required_hold:
                return start_time
        else:
            in_streak = False
            current = 0.0
    return NO_TRANSIENT_REACHED


def _total_true_span_duration(flags: np.ndarray, durations: np.ndarray) -> float:
    total = 0.0
    previous = False
    for flag, duration in zip(flags, durations):
        if flag and previous:
            total += float(duration)
        previous = bool(flag)
    return float(total)


def _relative_times(times: np.ndarray) -> np.ndarray:
    if times.size == 0:
        return times
    return times - times[0]


def _path_completion_percentage(path_progress: Any | None) -> float:
    if path_progress is None:
        return math.nan
    progress = _vector(path_progress, "path_progress", require_sorted=False)
    if progress.size == 0:
        return math.nan
    return float(100.0 * np.clip(np.max(progress), 0.0, 1.0))


def _array_shape(values: Any, label: str, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if any(dim == -1 for dim in shape):
        if len(shape) != array.ndim:
            raise ValueError(f"{label} must have {len(shape)} dimensions")
        for actual, expected in zip(array.shape, shape):
            if expected != -1 and actual != expected:
                raise ValueError(f"{label} must have shape {shape}")
    elif array.shape != shape:
        raise ValueError(f"{label} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    return array.copy()


def _matrix3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        array = np.empty((0, 3), dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape (N, 3) and contain finite values")
    return array.copy()


def _matrix6(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        array = np.empty((0, 6), dtype=float)
    if array.ndim != 2 or array.shape[1] != 6 or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape (N, 6) and contain finite values")
    return array.copy()


def _vector(values: Any, label: str, *, allow_empty: bool = False, require_sorted: bool = True) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1:
        raise ValueError(f"{label} must be one-dimensional")
    if array.size == 0 and not allow_empty:
        raise ValueError(f"{label} must not be empty")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    if require_sorted and array.size > 1 and np.any(np.diff(array) < 0.0):
        raise ValueError(f"{label} must be sorted by time")
    return array.copy()


def _bool_flags(values: Any | None, count: int, label: str) -> np.ndarray:
    if values is None:
        return np.zeros(count, dtype=bool)
    flags = np.asarray(values, dtype=bool)
    if flags.shape != (count,):
        raise ValueError(f"{label} must have shape ({count},)")
    return flags.copy()


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


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


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_number(value, label)
    integer = int(numeric)
    if integer != numeric:
        raise ValueError(f"{label} must be an integer")
    return integer


def _bounded_fraction(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric <= 0.0 or numeric > 1.0:
        raise ValueError(f"{label} must be in (0, 1]")
    return numeric


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return _number(value, "optional_float")
    except (TypeError, ValueError):
        return None


def _sorted_plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _sorted_plain(value[key]) for key in sorted(value)}
    if isinstance(value, list):
        return [_sorted_plain(item) for item in value]
    return value
