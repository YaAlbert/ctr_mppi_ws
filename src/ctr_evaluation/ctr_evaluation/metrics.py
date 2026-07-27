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


NO_TRANSIENT_REACHED = -1.0
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
}


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


@dataclass(frozen=True)
class AcceptanceResults:
    functional_pass: bool
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
class ComparisonResult:
    compatibility_valid: bool
    compatibility_reasons: list[str]
    metric_comparisons: list[MetricComparison]

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


def compute_acceptance(
    *,
    tracking: TrackingMetrics,
    control: ControlMetrics,
    timing: TimingMetrics,
    numerical_safety: NumericalSafetyMetrics,
    data_quality: DataQualityMetrics,
    thresholds: EvaluationThresholds,
    baseline_improvement_valid: bool,
    physical_validation: bool = False,
    hardware_validation: bool = False,
) -> AcceptanceResults:
    reasons: list[str] = []
    functional_pass = data_quality.valid_aligned_sample_count >= thresholds.minimum_valid_sample_count
    if not functional_pass:
        reasons.append("valid aligned sample count below threshold")

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
    if not real_time_pass:
        reasons.append("controller timing is not real-time capable under configured period")

    saturation_pass = control.saturation_percentage <= thresholds.maximum_saturation_percentage
    if not saturation_pass:
        reasons.append("command saturation exceeded threshold")
    numerical_safety_pass = numerical_safety_pass and saturation_pass

    return AcceptanceResults(
        functional_pass=functional_pass,
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
    compatibility_reasons = compatibility_reasons_for(
        candidate_metadata=candidate_metadata,
        baseline_metadata=baseline_metadata,
        duration_tolerance=duration_tolerance,
        initial_state_tolerance=initial_state_tolerance,
    )
    compatibility_valid = not compatibility_reasons
    metric_pairs = _flatten_numeric_metrics(candidate_summary)
    baseline_pairs = _flatten_numeric_metrics(baseline_summary)
    comparisons: list[MetricComparison] = []
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
        if not compatibility_valid:
            valid = False
            reason = "; ".join(compatibility_reasons)
        comparisons.append(
            MetricComparison(
                metric=name,
                direction=direction,
                candidate_value=float(candidate_value),
                baseline_value=float(baseline_value),
                absolute_difference=float(candidate_value - baseline_value),
                relative_improvement_percent=improvement,
                comparison_valid=bool(valid),
                compatibility_valid=compatibility_valid,
                reason=reason,
            )
        )
    return ComparisonResult(
        compatibility_valid=compatibility_valid,
        compatibility_reasons=compatibility_reasons,
        metric_comparisons=comparisons,
    )


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
    reasons: list[str] = []
    candidate_config = candidate_metadata.get("configuration", {})
    baseline_config = baseline_metadata.get("configuration", {})
    for key in (
        "trajectory_type",
        "trajectory_parameters_hash",
        "frame_id",
        "model_configuration_hash",
        "software_mode",
        "configured_control_period",
        "reference_sample_period",
    ):
        if candidate_config.get(key) != baseline_config.get(key):
            reasons.append(f"incompatible {key}")
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
            if delta > initial_tol:
                reasons.append("initial state differs beyond tolerance")
        except ValueError:
            reasons.append("initial state is malformed")
    return reasons


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
    for section in ("tracking", "control", "timing", "data_quality"):
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
