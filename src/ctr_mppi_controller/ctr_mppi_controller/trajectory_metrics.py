"""ROS-independent trajectory tracking metric accumulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


NO_TRANSIENT_REACHED = -1.0


@dataclass(frozen=True)
class TrajectoryMetricsConfig:
    enabled: bool
    publish_frequency: float
    transient_tolerance: float
    stable_cycles: int
    reset_on_new_trajectory: bool

    @classmethod
    def from_project_config(cls, config: dict[str, Any]) -> "TrajectoryMetricsConfig":
        tracking = config["tracking_metrics"]
        return cls(
            enabled=_bool(tracking["enabled"], "tracking_metrics.enabled"),
            publish_frequency=_positive_number(tracking["publish_frequency"], "tracking_metrics.publish_frequency"),
            transient_tolerance=_positive_number(
                tracking["transient_tolerance"],
                "tracking_metrics.transient_tolerance",
            ),
            stable_cycles=_positive_int(tracking["stable_cycles"], "tracking_metrics.stable_cycles"),
            reset_on_new_trajectory=_bool(
                tracking["reset_on_new_trajectory"],
                "tracking_metrics.reset_on_new_trajectory",
            ),
        )


@dataclass(frozen=True)
class TrajectoryMetricsSnapshot:
    trajectory_type: str
    sample_count: int
    invalid_sample_count: int
    rmse: float
    mean_error: float
    max_error: float
    control_effort: float
    transient_duration: float
    mean_solve_time: float
    max_solve_time: float
    min_solve_time: float
    control_period_overrun_count: int
    command_saturation_count: int
    maximum_command_per_joint: np.ndarray
    experiment_elapsed_time: float
    completion_state: str

    @property
    def has_valid_samples(self) -> bool:
        return self.sample_count > 0


class TrajectoryMetricsAccumulator:
    """Accumulate trajectory tracking metrics without ROS dependencies."""

    def __init__(self, *, config: TrajectoryMetricsConfig, command_dimension: int, trajectory_type: str):
        self.config = config
        self.command_dimension = _positive_int(command_dimension, "command_dimension")
        self.trajectory_type = _non_empty_string(trajectory_type, "trajectory_type")
        self.reset(trajectory_type=trajectory_type)

    def reset(self, *, trajectory_type: str | None = None) -> None:
        if trajectory_type is not None:
            self.trajectory_type = _non_empty_string(trajectory_type, "trajectory_type")
        self._errors: list[float] = []
        self._solve_times: list[float] = []
        self._invalid_sample_count = 0
        self._control_effort = 0.0
        self._command_saturation_count = 0
        self._control_period_overrun_count = 0
        self._maximum_command_per_joint = np.zeros(self.command_dimension, dtype=float)
        self._first_time: float | None = None
        self._last_time: float | None = None
        self._stable_streak = 0
        self._stable_streak_start_time: float | None = None
        self._transient_duration: float | None = None
        self._completion_state = "reset"

    def record_invalid_sample(self) -> None:
        self._invalid_sample_count += 1

    def add_sample(
        self,
        *,
        timestamp: float,
        tip_position: Any,
        reference_position: Any,
        command: Any,
        dt: float,
        solve_time: float,
        command_saturated: bool,
        control_period: float | None = None,
        completed: bool = False,
    ) -> None:
        time_s = _nonnegative_number(timestamp, "timestamp")
        tip = _vector3(tip_position, "tip_position")
        reference = _vector3(reference_position, "reference_position")
        command_array = _array_shape(command, "command", (self.command_dimension,))
        dt_s = _positive_number(dt, "dt")
        solve_time_s = _nonnegative_number(solve_time, "solve_time")
        if control_period is not None:
            period = _positive_number(control_period, "control_period")
        else:
            period = None

        error = float(np.linalg.norm(tip - reference))
        if not math.isfinite(error):
            raise ValueError("tracking error must be finite")

        if self._first_time is None:
            self._first_time = time_s
        self._last_time = time_s

        self._errors.append(error)
        self._solve_times.append(solve_time_s)
        self._control_effort += float(np.dot(command_array, command_array) * dt_s)
        self._maximum_command_per_joint = np.maximum(self._maximum_command_per_joint, np.abs(command_array))
        if bool(command_saturated):
            self._command_saturation_count += 1
        if period is not None and solve_time_s > period:
            self._control_period_overrun_count += 1

        self._update_transient(error=error, timestamp=time_s)
        self._completion_state = "completed" if completed else "tracking"

    def snapshot(self) -> TrajectoryMetricsSnapshot:
        if not self._errors:
            return TrajectoryMetricsSnapshot(
                trajectory_type=self.trajectory_type,
                sample_count=0,
                invalid_sample_count=self._invalid_sample_count,
                rmse=math.nan,
                mean_error=math.nan,
                max_error=math.nan,
                control_effort=0.0,
                transient_duration=NO_TRANSIENT_REACHED,
                mean_solve_time=math.nan,
                max_solve_time=math.nan,
                min_solve_time=math.nan,
                control_period_overrun_count=self._control_period_overrun_count,
                command_saturation_count=self._command_saturation_count,
                maximum_command_per_joint=self._maximum_command_per_joint.copy(),
                experiment_elapsed_time=0.0,
                completion_state="no_valid_samples",
            )

        errors = np.asarray(self._errors, dtype=float)
        solve_times = np.asarray(self._solve_times, dtype=float)
        first_time = 0.0 if self._first_time is None else self._first_time
        last_time = first_time if self._last_time is None else self._last_time
        return TrajectoryMetricsSnapshot(
            trajectory_type=self.trajectory_type,
            sample_count=int(errors.shape[0]),
            invalid_sample_count=self._invalid_sample_count,
            rmse=float(math.sqrt(float(np.mean(errors**2)))),
            mean_error=float(np.mean(errors)),
            max_error=float(np.max(errors)),
            control_effort=float(self._control_effort),
            transient_duration=(
                NO_TRANSIENT_REACHED if self._transient_duration is None else float(self._transient_duration)
            ),
            mean_solve_time=float(np.mean(solve_times)),
            max_solve_time=float(np.max(solve_times)),
            min_solve_time=float(np.min(solve_times)),
            control_period_overrun_count=self._control_period_overrun_count,
            command_saturation_count=self._command_saturation_count,
            maximum_command_per_joint=self._maximum_command_per_joint.copy(),
            experiment_elapsed_time=float(max(0.0, last_time - first_time)),
            completion_state=self._completion_state,
        )

    def _update_transient(self, *, error: float, timestamp: float) -> None:
        if error <= self.config.transient_tolerance:
            if self._stable_streak == 0:
                self._stable_streak_start_time = timestamp
            self._stable_streak += 1
            if self._transient_duration is None and self._stable_streak >= self.config.stable_cycles:
                first_time = self._first_time if self._first_time is not None else timestamp
                stable_start = self._stable_streak_start_time
                if stable_start is None:
                    stable_start = timestamp
                self._transient_duration = max(0.0, stable_start - first_time)
            return
        self._stable_streak = 0
        self._stable_streak_start_time = None


def _array_shape(values: Any, label: str, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape {shape} and contain finite values")
    return array.copy()


def _vector3(values: Any, label: str) -> np.ndarray:
    return _array_shape(values, label, (3,))


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _nonnegative_number(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric < 0.0:
        raise ValueError(f"{label} must be non-negative")
    return numeric


def _positive_number(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_number(value, label)
    integer = int(numeric)
    if integer != numeric:
        raise ValueError(f"{label} must be an integer")
    return integer


def _bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be a boolean")
    return value


def _non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value
