"""Generic offline lumen safety and progress metrics.

The functions here consume recorded full-backbone samples and use
``LumenGeometry.backbone_clearance`` as the only clearance authority.
They are ROS-independent and do not modify caller data or geometry objects.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np

from ctr_mppi_controller.lumen_geometry import BackboneClearance, LumenGeometry


CONSTRAINT_WALL = "wall"
CONSTRAINT_INLET = "inlet"
CONSTRAINT_OUTLET = "outlet"
CONSTRAINT_ORDER = (CONSTRAINT_WALL, CONSTRAINT_INLET, CONSTRAINT_OUTLET)


@dataclass(frozen=True)
class LumenSampleMetrics:
    timestamp: float
    physical_clearance: float
    safety_clearance: float
    physical_collision: bool
    safety_margin_violation: bool
    selected_constraint_type: str
    closest_backbone_index: int
    wall_penetration: float
    inlet_penetration: float
    outlet_penetration: float
    tip_centerline_point: np.ndarray
    tip_centerline_segment_index: int
    tip_centerline_interpolation_fraction: float
    tip_centerline_arc_length: float
    normalized_tip_progress: float
    tip_progress_out_of_extent: bool
    tip_radial_offset: float
    local_lumen_radius: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "tip_centerline_point",
            _readonly_vector3(self.tip_centerline_point, "tip_centerline_point"),
        )


@dataclass(frozen=True)
class LumenConstraintMetrics:
    constraint_type: str
    physical_violation_sample_count: int
    physical_violation_event_count: int
    physical_violation_duration: float
    first_physical_violation_time: float | None
    maximum_penetration: float
    minimum_physical_clearance: float
    worst_sample_index: int
    worst_backbone_index: int


@dataclass(frozen=True)
class LumenSafetyMetrics:
    sample_count: int
    minimum_physical_clearance: float
    minimum_safety_clearance: float
    physical_collision_detected: bool
    physical_collision_sample_count: int
    physical_collision_event_count: int
    physical_collision_duration: float
    first_physical_collision_time: float | None
    safety_margin_violation_detected: bool
    safety_margin_violation_sample_count: int
    safety_margin_violation_event_count: int
    safety_margin_violation_duration: float
    first_safety_margin_violation_time: float | None
    final_physical_clearance: float
    final_safety_clearance: float
    worst_physical_constraint: str
    worst_physical_sample_index: int
    worst_physical_backbone_index: int
    worst_safety_constraint: str
    worst_safety_sample_index: int
    worst_safety_backbone_index: int
    per_constraint_breakdown: tuple[LumenConstraintMetrics, ...]
    physical_safety_pass: bool
    safety_margin_pass: bool


@dataclass(frozen=True)
class LumenProgressMetrics:
    initial_centerline_arc_length: float
    final_centerline_arc_length: float
    minimum_centerline_arc_length: float
    maximum_centerline_arc_length: float
    initial_normalized_progress: float
    final_normalized_progress: float
    maximum_normalized_progress: float
    mean_tip_radial_offset: float
    rms_tip_radial_offset: float
    maximum_tip_radial_offset: float
    final_tip_radial_offset: float
    mean_local_lumen_radius: float
    final_local_lumen_radius: float
    centerline_tracking_rmse: float | None = None


@dataclass(frozen=True)
class LumenEvaluationMetrics:
    safety: LumenSafetyMetrics
    progress: LumenProgressMetrics
    samples: tuple[LumenSampleMetrics, ...]


def compute_lumen_evaluation_metrics(
    *,
    geometry: LumenGeometry,
    times: Any,
    backbone_points: Sequence[Any],
    tip_points: Any | None = None,
    compute_centerline_tracking_rmse: bool = False,
    tip_backbone_tolerance: float = 1.0e-9,
) -> LumenEvaluationMetrics:
    """Compute whole-backbone lumen safety and tip progress metrics.

    ``backbone_clearance`` is called exactly once per recorded backbone sample.
    The final point of each backbone is the default tip authority. If explicit
    tip samples are supplied, they must match the final backbone point.
    """

    time_values = _time_vector(times)
    if len(backbone_points) != time_values.shape[0]:
        raise ValueError("times and backbone_points must have matching lengths")
    if time_values.size == 0:
        raise ValueError("at least one lumen sample is required")
    tolerance = _nonnegative_number(tip_backbone_tolerance, "tip_backbone_tolerance")
    explicit_tips = None if tip_points is None else _tip_matrix(tip_points, time_values.shape[0])
    length = _geometry_length(geometry)
    durations = _sample_durations(_relative_times(time_values))

    clearances: list[BackboneClearance] = []
    samples: list[LumenSampleMetrics] = []
    for sample_index, raw_points in enumerate(backbone_points):
        points = _backbone_array(raw_points, f"backbone_points[{sample_index}]")
        tip = points[-1]
        if explicit_tips is not None and not np.allclose(explicit_tips[sample_index], tip, atol=tolerance, rtol=0.0):
            raise ValueError("tip_points must match the final point of each backbone sample")
        clearance = geometry.backbone_clearance(points)
        _validate_clearance(clearance, points.shape[0], f"clearance[{sample_index}]")
        clearances.append(clearance)
        sample = _sample_metrics(
            timestamp=float(time_values[sample_index]),
            clearance=clearance,
            geometry_length=length,
        )
        samples.append(sample)

    sample_tuple = tuple(samples)
    safety = _aggregate_safety(
        relative_times=_relative_times(time_values),
        durations=durations,
        samples=sample_tuple,
        clearances=clearances,
    )
    progress = _aggregate_progress(
        samples=sample_tuple,
        compute_centerline_tracking_rmse=bool(compute_centerline_tracking_rmse),
    )
    return LumenEvaluationMetrics(safety=safety, progress=progress, samples=sample_tuple)


def event_count(flags: Any) -> int:
    values = _bool_vector(flags, "flags")
    count = 0
    previous = False
    for value in values:
        current = bool(value)
        if current and not previous:
            count += 1
        previous = current
    return int(count)


def event_duration(times: Any, flags: Any) -> float:
    time_values = _time_vector(times)
    values = _bool_vector(flags, "flags")
    if values.shape[0] != time_values.shape[0]:
        raise ValueError("times and flags must have matching lengths")
    durations = _sample_durations(_relative_times(time_values))
    return float(np.sum(durations[values]))


def _aggregate_safety(
    *,
    relative_times: np.ndarray,
    durations: np.ndarray,
    samples: tuple[LumenSampleMetrics, ...],
    clearances: list[BackboneClearance],
) -> LumenSafetyMetrics:
    physical_flags = np.asarray([sample.physical_collision for sample in samples], dtype=bool)
    safety_flags = np.asarray([sample.safety_margin_violation for sample in samples], dtype=bool)
    physical_min = np.asarray([sample.physical_clearance for sample in samples], dtype=float)
    safety_min = np.asarray([sample.safety_clearance for sample in samples], dtype=float)
    worst_physical = _worst_physical(clearances)
    worst_safety = _worst_safety(clearances)
    breakdown = tuple(
        _constraint_breakdown(
            constraint=constraint,
            clearances=clearances,
            relative_times=relative_times,
            durations=durations,
        )
        for constraint in CONSTRAINT_ORDER
    )
    return LumenSafetyMetrics(
        sample_count=len(samples),
        minimum_physical_clearance=float(np.min(physical_min)),
        minimum_safety_clearance=float(np.min(safety_min)),
        physical_collision_detected=bool(np.any(physical_flags)),
        physical_collision_sample_count=int(np.sum(physical_flags)),
        physical_collision_event_count=event_count(physical_flags),
        physical_collision_duration=float(np.sum(durations[physical_flags])),
        first_physical_collision_time=_first_true_time(relative_times, physical_flags),
        safety_margin_violation_detected=bool(np.any(safety_flags)),
        safety_margin_violation_sample_count=int(np.sum(safety_flags)),
        safety_margin_violation_event_count=event_count(safety_flags),
        safety_margin_violation_duration=float(np.sum(durations[safety_flags])),
        first_safety_margin_violation_time=_first_true_time(relative_times, safety_flags),
        final_physical_clearance=samples[-1].physical_clearance,
        final_safety_clearance=samples[-1].safety_clearance,
        worst_physical_constraint=worst_physical[0],
        worst_physical_sample_index=worst_physical[1],
        worst_physical_backbone_index=worst_physical[2],
        worst_safety_constraint=worst_safety[0],
        worst_safety_sample_index=worst_safety[1],
        worst_safety_backbone_index=worst_safety[2],
        per_constraint_breakdown=breakdown,
        physical_safety_pass=not bool(np.any(physical_flags)),
        safety_margin_pass=not bool(np.any(safety_flags)),
    )


def _aggregate_progress(
    *,
    samples: tuple[LumenSampleMetrics, ...],
    compute_centerline_tracking_rmse: bool,
) -> LumenProgressMetrics:
    arc = np.asarray([sample.tip_centerline_arc_length for sample in samples], dtype=float)
    progress = np.asarray([sample.normalized_tip_progress for sample in samples], dtype=float)
    radial = np.asarray([sample.tip_radial_offset for sample in samples], dtype=float)
    radii = np.asarray([sample.local_lumen_radius for sample in samples], dtype=float)
    return LumenProgressMetrics(
        initial_centerline_arc_length=float(arc[0]),
        final_centerline_arc_length=float(arc[-1]),
        minimum_centerline_arc_length=float(np.min(arc)),
        maximum_centerline_arc_length=float(np.max(arc)),
        initial_normalized_progress=float(progress[0]),
        final_normalized_progress=float(progress[-1]),
        maximum_normalized_progress=float(np.max(progress)),
        mean_tip_radial_offset=float(np.mean(radial)),
        rms_tip_radial_offset=float(math.sqrt(float(np.mean(radial**2)))),
        maximum_tip_radial_offset=float(np.max(radial)),
        final_tip_radial_offset=float(radial[-1]),
        mean_local_lumen_radius=float(np.mean(radii)),
        final_local_lumen_radius=float(radii[-1]),
        centerline_tracking_rmse=(
            float(math.sqrt(float(np.mean(radial**2)))) if compute_centerline_tracking_rmse else None
        ),
    )


def _sample_metrics(
    *,
    timestamp: float,
    clearance: BackboneClearance,
    geometry_length: float,
) -> LumenSampleMetrics:
    physical = np.asarray(clearance.physical_clearances, dtype=float)
    safety = np.asarray(clearance.safety_margin_clearances, dtype=float)
    selected_constraint, selected_index = _status_constraint(clearance)
    tip_index = physical.shape[0] - 1
    raw_progress = float(clearance.centerline_progress[tip_index])
    normalized = float(np.clip(raw_progress / geometry_length, 0.0, 1.0))
    out_of_extent = bool(clearance.inlet_violation_mask[tip_index] or clearance.outlet_violation_mask[tip_index])
    return LumenSampleMetrics(
        timestamp=timestamp,
        physical_clearance=float(np.min(physical)),
        safety_clearance=float(np.min(safety)),
        physical_collision=bool(np.any(clearance.collision_mask)),
        safety_margin_violation=bool(np.any(clearance.safety_margin_violation_mask)),
        selected_constraint_type=selected_constraint,
        closest_backbone_index=selected_index,
        wall_penetration=float(np.max(clearance.wall_penetrations)),
        inlet_penetration=float(np.max(clearance.inlet_penetrations)),
        outlet_penetration=float(np.max(clearance.outlet_penetrations)),
        tip_centerline_point=np.asarray(clearance.closest_geometry_points[tip_index], dtype=float),
        tip_centerline_segment_index=int(clearance.closest_geometry_indices[tip_index]),
        tip_centerline_interpolation_fraction=float(clearance.closest_geometry_parameters[tip_index]),
        tip_centerline_arc_length=raw_progress,
        normalized_tip_progress=normalized,
        tip_progress_out_of_extent=out_of_extent,
        tip_radial_offset=float(clearance.radial_distance[tip_index]),
        local_lumen_radius=float(clearance.local_radius[tip_index]),
    )


def _status_constraint(clearance: BackboneClearance) -> tuple[str, int]:
    if np.any(clearance.collision_mask):
        constraint, index = _max_penetration_constraint(clearance)
        return constraint, index
    if np.any(clearance.safety_margin_violation_mask):
        index = _first_min_index(clearance.safety_margin_clearances)
        return _constraint_at_index(clearance, index), index
    index = _first_min_index(clearance.physical_clearances)
    return _constraint_at_index(clearance, index), index


def _constraint_at_index(clearance: BackboneClearance, index: int) -> str:
    if bool(clearance.inlet_violation_mask[index]):
        return CONSTRAINT_INLET
    if bool(clearance.outlet_violation_mask[index]):
        return CONSTRAINT_OUTLET
    return CONSTRAINT_WALL


def _max_penetration_constraint(clearance: BackboneClearance) -> tuple[str, int]:
    candidates: list[tuple[float, int, int, str]] = []
    arrays = {
        CONSTRAINT_WALL: clearance.wall_penetrations,
        CONSTRAINT_INLET: clearance.inlet_penetrations,
        CONSTRAINT_OUTLET: clearance.outlet_penetrations,
    }
    for order, constraint in enumerate(CONSTRAINT_ORDER):
        values = np.asarray(arrays[constraint], dtype=float)
        for index, value in enumerate(values):
            candidates.append((-float(value), int(index), order, constraint))
    candidates.sort()
    _, index, _, constraint = candidates[0]
    return constraint, index


def _worst_physical(clearances: list[BackboneClearance]) -> tuple[str, int, int]:
    if any(np.any(clearance.collision_mask) for clearance in clearances):
        candidates: list[tuple[float, int, int, int, str]] = []
        for sample_index, clearance in enumerate(clearances):
            for order, constraint in enumerate(CONSTRAINT_ORDER):
                values = _penetrations_for(clearance, constraint)
                for backbone_index, penetration in enumerate(values):
                    if float(penetration) > 0.0:
                        candidates.append((-float(penetration), sample_index, backbone_index, order, constraint))
        candidates.sort()
        _, sample_index, backbone_index, _, constraint = candidates[0]
        return constraint, int(sample_index), int(backbone_index)
    candidates = []
    for sample_index, clearance in enumerate(clearances):
        for backbone_index, value in enumerate(clearance.physical_clearances):
            candidates.append((float(value), sample_index, backbone_index))
    candidates.sort()
    _, sample_index, backbone_index = candidates[0]
    return CONSTRAINT_WALL, int(sample_index), int(backbone_index)


def _worst_safety(clearances: list[BackboneClearance]) -> tuple[str, int, int]:
    candidates = []
    for sample_index, clearance in enumerate(clearances):
        for backbone_index, value in enumerate(clearance.safety_margin_clearances):
            candidates.append((float(value), sample_index, backbone_index, _constraint_at_index(clearance, backbone_index)))
    candidates.sort(key=lambda item: (item[0], item[1], item[2], CONSTRAINT_ORDER.index(item[3])))
    _, sample_index, backbone_index, constraint = candidates[0]
    return constraint, int(sample_index), int(backbone_index)


def _constraint_breakdown(
    *,
    constraint: str,
    clearances: list[BackboneClearance],
    relative_times: np.ndarray,
    durations: np.ndarray,
) -> LumenConstraintMetrics:
    flags = np.asarray([np.any(_mask_for(clearance, constraint)) for clearance in clearances], dtype=bool)
    penetrations = [float(np.max(_penetrations_for(clearance, constraint))) for clearance in clearances]
    worst_sample = int(np.argmax(penetrations)) if penetrations else -1
    worst_backbone = -1
    if worst_sample >= 0:
        worst_backbone = int(np.argmax(_penetrations_for(clearances[worst_sample], constraint)))
    minimum_clearance = _constraint_minimum_clearance(constraint, clearances)
    return LumenConstraintMetrics(
        constraint_type=constraint,
        physical_violation_sample_count=int(np.sum(flags)),
        physical_violation_event_count=event_count(flags),
        physical_violation_duration=float(np.sum(durations[flags])),
        first_physical_violation_time=_first_true_time(relative_times, flags),
        maximum_penetration=float(max(penetrations)) if penetrations else 0.0,
        minimum_physical_clearance=minimum_clearance,
        worst_sample_index=worst_sample,
        worst_backbone_index=worst_backbone,
    )


def _constraint_minimum_clearance(constraint: str, clearances: list[BackboneClearance]) -> float:
    if constraint == CONSTRAINT_WALL:
        return float(min(float(np.min(clearance.physical_clearances)) for clearance in clearances))
    if constraint == CONSTRAINT_INLET:
        return float(min(-float(np.max(clearance.inlet_penetrations)) for clearance in clearances))
    if constraint == CONSTRAINT_OUTLET:
        return float(min(-float(np.max(clearance.outlet_penetrations)) for clearance in clearances))
    raise ValueError(f"unsupported constraint type {constraint!r}")


def _mask_for(clearance: BackboneClearance, constraint: str) -> np.ndarray:
    if constraint == CONSTRAINT_WALL:
        return np.asarray(clearance.radial_collision_mask, dtype=bool)
    if constraint == CONSTRAINT_INLET:
        return np.asarray(clearance.inlet_violation_mask, dtype=bool)
    if constraint == CONSTRAINT_OUTLET:
        return np.asarray(clearance.outlet_violation_mask, dtype=bool)
    raise ValueError(f"unsupported constraint type {constraint!r}")


def _penetrations_for(clearance: BackboneClearance, constraint: str) -> np.ndarray:
    if constraint == CONSTRAINT_WALL:
        return np.asarray(clearance.wall_penetrations, dtype=float)
    if constraint == CONSTRAINT_INLET:
        return np.asarray(clearance.inlet_penetrations, dtype=float)
    if constraint == CONSTRAINT_OUTLET:
        return np.asarray(clearance.outlet_penetrations, dtype=float)
    raise ValueError(f"unsupported constraint type {constraint!r}")


def _first_true_time(times: np.ndarray, flags: np.ndarray) -> float | None:
    indices = np.nonzero(flags)[0]
    if indices.size == 0:
        return None
    return float(times[int(indices[0])])


def _first_min_index(values: Any) -> int:
    array = np.asarray(values, dtype=float)
    minimum = float(np.min(array))
    indices = np.nonzero(array == minimum)[0]
    return int(indices[0])


def _validate_clearance(clearance: Any, point_count: int, label: str) -> None:
    arrays = {
        "points": np.asarray(clearance.points, dtype=float),
        "physical_clearances": np.asarray(clearance.physical_clearances, dtype=float),
        "safety_margin_clearances": np.asarray(clearance.safety_margin_clearances, dtype=float),
        "wall_penetrations": np.asarray(clearance.wall_penetrations, dtype=float),
        "inlet_penetrations": np.asarray(clearance.inlet_penetrations, dtype=float),
        "outlet_penetrations": np.asarray(clearance.outlet_penetrations, dtype=float),
        "centerline_progress": np.asarray(clearance.centerline_progress, dtype=float),
        "closest_geometry_parameters": np.asarray(clearance.closest_geometry_parameters, dtype=float),
        "closest_geometry_points": np.asarray(clearance.closest_geometry_points, dtype=float),
        "radial_distance": np.asarray(clearance.radial_distance, dtype=float),
        "local_radius": np.asarray(clearance.local_radius, dtype=float),
    }
    for name, array in arrays.items():
        expected = (point_count, 3) if name in ("points", "closest_geometry_points") else (point_count,)
        if array.shape != expected or not np.all(np.isfinite(array)):
            raise ValueError(f"{label}.{name} must have shape {expected} and finite values")
    for name in (
        "collision_mask",
        "safety_margin_violation_mask",
        "radial_collision_mask",
        "inlet_violation_mask",
        "outlet_violation_mask",
    ):
        array = np.asarray(getattr(clearance, name), dtype=bool)
        if array.shape != (point_count,):
            raise ValueError(f"{label}.{name} must have shape ({point_count},)")
    indices = np.asarray(clearance.closest_geometry_indices, dtype=int)
    if indices.shape != (point_count,):
        raise ValueError(f"{label}.closest_geometry_indices must have shape ({point_count},)")
    if int(clearance.closest_backbone_index) < 0 or int(clearance.closest_backbone_index) >= point_count:
        raise ValueError(f"{label}.closest_backbone_index must be in range")


def _geometry_length(geometry: LumenGeometry) -> float:
    length = getattr(geometry, "length", None)
    if length is not None:
        return _positive_number(length, "geometry.length")
    centerline = getattr(geometry, "centerline_points", None)
    if centerline is None:
        raise ValueError("geometry must expose length or centerline_points for progress metrics")
    points = _centerline_array(centerline)
    segment_lengths = np.linalg.norm(points[1:] - points[:-1], axis=1)
    total = float(np.sum(segment_lengths))
    if total <= 0.0 or not math.isfinite(total):
        raise ValueError("geometry centerline total length must be positive and finite")
    return total


def _centerline_array(values: Any) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] < 2 or not np.all(np.isfinite(array)):
        raise ValueError("geometry.centerline_points must have shape (M, 3), M >= 2, and finite values")
    segment_lengths = np.linalg.norm(array[1:] - array[:-1], axis=1)
    if np.any(segment_lengths <= 0.0):
        raise ValueError("geometry.centerline_points must not contain duplicate consecutive points")
    return array.copy()


def _time_vector(values: Any) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or not np.all(np.isfinite(array)):
        raise ValueError("times must be a one-dimensional finite numeric sequence")
    if np.any(np.diff(array) < 0.0):
        raise ValueError("times must be monotonically nondecreasing")
    return array.astype(float).copy()


def _relative_times(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return values.copy()
    return values - values[0]


def _sample_durations(times: np.ndarray) -> np.ndarray:
    if times.size == 0:
        return np.asarray([], dtype=float)
    if times.size == 1:
        return np.asarray([0.0], dtype=float)
    deltas = np.diff(times)
    if np.any(deltas < 0.0):
        raise ValueError("times must be monotonically nondecreasing")
    return np.concatenate(([0.0], deltas))


def _backbone_array(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] == 0 or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape (N, 3) with N >= 1 and finite values")
    return array.astype(float).copy()


def _tip_matrix(values: Any, count: int) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (count, 3) or not np.all(np.isfinite(array)):
        raise ValueError(f"tip_points must have shape ({count}, 3) and finite values")
    return array.astype(float).copy()


def _bool_vector(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=bool)
    if array.ndim != 1:
        raise ValueError(f"{label} must be a one-dimensional boolean sequence")
    return array.copy()


def _readonly_vector3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape (3,) and finite values")
    result = array.astype(float).copy()
    result.setflags(write=False)
    return result


def _positive_number(value: Any, label: str) -> float:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
    return numeric


def _nonnegative_number(value: Any, label: str) -> float:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{label} must be nonnegative and finite")
    return numeric
