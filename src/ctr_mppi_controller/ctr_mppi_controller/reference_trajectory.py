"""ROS-independent reference trajectory generation for MPPI tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


TRAJECTORY_TYPES = ("circle", "ellipse", "helix")
COMPLETION_BEHAVIORS = ("loop", "hold_final")


@dataclass(frozen=True)
class ReferenceHorizon:
    """A fixed-length sequence of 3D target points for one MPPI solve."""

    points: np.ndarray
    indices: np.ndarray
    start_index: int
    completed: bool

    def __post_init__(self) -> None:
        points = _points_array(self.points, "points")
        indices = np.asarray(self.indices)
        if indices.ndim != 1:
            raise ValueError("indices must be a one-dimensional array")
        if indices.shape[0] != points.shape[0]:
            raise ValueError("indices length must match horizon point count")
        if not np.issubdtype(indices.dtype, np.integer):
            raise ValueError("indices must contain integer values")
        if np.any(indices < 0):
            raise ValueError("indices must be non-negative")

        object.__setattr__(self, "points", points)
        object.__setattr__(self, "indices", indices.astype(int, copy=True))
        object.__setattr__(self, "start_index", _nonnegative_int(self.start_index, "start_index"))
        object.__setattr__(self, "completed", bool(self.completed))

    @property
    def current_index(self) -> int:
        return int(self.indices[0])

    @property
    def current_point(self) -> np.ndarray:
        return self.points[0].copy()


@dataclass(frozen=True)
class ReferenceTrajectory:
    """A sampled 3D tip-reference trajectory.

    A zero circle radius is accepted and represents a stationary reference at
    the configured center. Ellipse radii must be positive, and helix height
    must be non-zero so an explicit helix does not degenerate to a circle.
    """

    points: np.ndarray
    sample_period: float
    frame_id: str
    trajectory_type: str
    completion_behavior: str

    def __post_init__(self) -> None:
        points = _points_array(self.points, "points")
        if points.shape[0] < 2:
            raise ValueError("trajectory must contain at least two points")
        sample_period = _positive_number(self.sample_period, "sample_period")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a non-empty string")
        trajectory_type = _choice(self.trajectory_type, "trajectory_type", TRAJECTORY_TYPES)
        completion_behavior = _choice(self.completion_behavior, "completion_behavior", COMPLETION_BEHAVIORS)

        object.__setattr__(self, "points", points)
        object.__setattr__(self, "sample_period", sample_period)
        object.__setattr__(self, "trajectory_type", trajectory_type)
        object.__setattr__(self, "completion_behavior", completion_behavior)

    @property
    def loop(self) -> bool:
        return self.completion_behavior == "loop"

    def index_at_time(self, *, current_time: float, start_time: float) -> int:
        return elapsed_time_index(current_time=current_time, start_time=start_time, sample_period=self.sample_period)

    def horizon_at_time(self, *, current_time: float, start_time: float, horizon_length: int) -> ReferenceHorizon:
        return self.horizon_at_index(
            start_index=self.index_at_time(current_time=current_time, start_time=start_time),
            horizon_length=horizon_length,
        )

    def horizon_at_index(self, *, start_index: int, horizon_length: int) -> ReferenceHorizon:
        start = _nonnegative_int(start_index, "start_index")
        horizon = _positive_int(horizon_length, "horizon_length")
        raw_indices = start + np.arange(horizon, dtype=int)

        if self.loop:
            indices = raw_indices % self.points.shape[0]
            completed = False
        else:
            last_index = self.points.shape[0] - 1
            indices = np.minimum(raw_indices, last_index)
            completed = bool(raw_indices[-1] >= last_index)

        return ReferenceHorizon(
            points=self.points[indices],
            indices=indices,
            start_index=start,
            completed=completed,
        )


def elapsed_time_index(*, current_time: float, start_time: float, sample_period: float) -> int:
    """Return floor((current_time - start_time) / sample_period), clamped at zero."""

    current = _number(current_time, "current_time")
    start = _number(start_time, "start_time")
    period = _positive_number(sample_period, "sample_period")
    elapsed = current - start
    if elapsed <= 0.0:
        return 0
    ratio = elapsed / period
    nearest = round(ratio)
    if abs(ratio - nearest) <= 1.0e-12:
        ratio = float(nearest)
    return int(math.floor(ratio))


def generate_circle(
    *,
    center: Any,
    radius: float,
    angular_velocity: float,
    phase: float,
    duration: float,
    sample_period: float,
    frame_id: str,
    completion_behavior: str,
) -> ReferenceTrajectory:
    """Generate a circle; radius zero is a documented stationary reference."""

    center_array = _array_shape(center, "center", (3,))
    radius_value = _nonnegative_number(radius, "radius")
    omega = _number(angular_velocity, "angular_velocity")
    phase_value = _number(phase, "phase")
    times = _sample_times(duration=duration, sample_period=sample_period)
    angle = omega * times + phase_value
    points = np.column_stack(
        (
            center_array[0] + radius_value * np.cos(angle),
            center_array[1] + radius_value * np.sin(angle),
            np.full(times.shape, center_array[2], dtype=float),
        )
    )
    return ReferenceTrajectory(
        points=points,
        sample_period=sample_period,
        frame_id=frame_id,
        trajectory_type="circle",
        completion_behavior=completion_behavior,
    )


def generate_ellipse(
    *,
    center: Any,
    radii: Any,
    angular_velocity: float,
    phase: float,
    duration: float,
    sample_period: float,
    frame_id: str,
    completion_behavior: str,
) -> ReferenceTrajectory:
    center_array = _array_shape(center, "center", (3,))
    radii_array = _array_shape(radii, "radii", (2,))
    if np.any(radii_array <= 0.0):
        raise ValueError("ellipse radii must be positive")
    omega = _number(angular_velocity, "angular_velocity")
    phase_value = _number(phase, "phase")
    times = _sample_times(duration=duration, sample_period=sample_period)
    angle = omega * times + phase_value
    points = np.column_stack(
        (
            center_array[0] + radii_array[0] * np.cos(angle),
            center_array[1] + radii_array[1] * np.sin(angle),
            np.full(times.shape, center_array[2], dtype=float),
        )
    )
    return ReferenceTrajectory(
        points=points,
        sample_period=sample_period,
        frame_id=frame_id,
        trajectory_type="ellipse",
        completion_behavior=completion_behavior,
    )


def generate_helix(
    *,
    center: Any,
    radius: float,
    height: float,
    angular_velocity: float,
    phase: float,
    duration: float,
    sample_period: float,
    frame_id: str,
    completion_behavior: str,
) -> ReferenceTrajectory:
    center_array = _array_shape(center, "center", (3,))
    radius_value = _nonnegative_number(radius, "radius")
    height_value = _number(height, "height")
    if height_value == 0.0:
        raise ValueError("helix height must be non-zero")
    omega = _number(angular_velocity, "angular_velocity")
    phase_value = _number(phase, "phase")
    times = _sample_times(duration=duration, sample_period=sample_period)
    progress = np.linspace(0.0, 1.0, times.shape[0])
    angle = omega * times + phase_value
    points = np.column_stack(
        (
            center_array[0] + radius_value * np.cos(angle),
            center_array[1] + radius_value * np.sin(angle),
            center_array[2] + height_value * progress,
        )
    )
    return ReferenceTrajectory(
        points=points,
        sample_period=sample_period,
        frame_id=frame_id,
        trajectory_type="helix",
        completion_behavior=completion_behavior,
    )


def generate_trajectory(*, trajectory_type: str, **kwargs: Any) -> ReferenceTrajectory:
    kind = _choice(trajectory_type, "trajectory_type", TRAJECTORY_TYPES)
    if kind == "circle":
        return generate_circle(**kwargs)
    if kind == "ellipse":
        return generate_ellipse(**kwargs)
    if kind == "helix":
        return generate_helix(**kwargs)
    raise ValueError(f"unsupported trajectory_type: {trajectory_type}")


def _sample_times(*, duration: float, sample_period: float) -> np.ndarray:
    duration_value = _positive_number(duration, "duration")
    period = _positive_number(sample_period, "sample_period")
    point_count = elapsed_time_index(current_time=duration_value, start_time=0.0, sample_period=period) + 1
    if point_count < 2:
        raise ValueError("trajectory generation must produce at least two points")
    return np.arange(point_count, dtype=float) * period


def _choice(value: Any, label: str, allowed: tuple[str, ...]) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be one of {allowed}")
    normalized = value.strip()
    if normalized not in allowed:
        raise ValueError(f"{label} must be one of {allowed}")
    return normalized


def _points_array(values: Any, label: str) -> np.ndarray:
    return _array_shape(values, label, (-1, 3))


def _array_shape(values: Any, label: str, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if len(shape) != array.ndim:
        raise ValueError(f"{label} must have shape {shape} and contain finite values")
    for actual, expected in zip(array.shape, shape):
        if expected != -1 and actual != expected:
            raise ValueError(f"{label} must have shape {shape} and contain finite values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape {shape} and contain finite values")
    return array.copy()


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


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer")
    integer = int(value)
    if integer != value or integer < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return integer
