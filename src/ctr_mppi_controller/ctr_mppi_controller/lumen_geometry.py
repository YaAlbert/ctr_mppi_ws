"""Shared ROS-independent lumen geometry interfaces and result types."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Protocol, runtime_checkable

import numpy as np


BOUNDARY_TOLERANCE = 0.0
PROJECTION_TIE_TOLERANCE = 1.0e-12


@runtime_checkable
class LumenGeometry(Protocol):
    frame_id: str
    ctr_outer_radius: float
    safety_margin: float

    def validate_target(
        self,
        target: Any,
        *,
        frame_id: str | None = None,
        require_safety_margin: bool = True,
    ) -> "TargetValidation":
        ...

    def point_clearance(self, point: Any) -> "PointClearance":
        ...

    def backbone_clearance(self, points: Any) -> "BackboneClearance":
        ...


@dataclass(frozen=True)
class PointClearance:
    point: np.ndarray
    physical_clearance: float
    safety_margin_clearance: float
    collision: bool
    safety_margin_violation: bool
    inlet_violation: bool
    outlet_violation: bool
    maximum_penetration: float
    centerline_progress: float
    closest_geometry_index: int
    closest_geometry_parameter: float
    closest_geometry_point: np.ndarray
    radial_distance: float
    local_radius: float
    wall_penetration: float
    inlet_penetration: float
    outlet_penetration: float
    end_cap_penetration: float
    axial_position: float
    radial_clearance: float
    axial_clearance: float
    radial_collision: bool


@dataclass(frozen=True)
class BackboneClearance:
    points: np.ndarray
    physical_clearances: np.ndarray
    safety_margin_clearances: np.ndarray
    collision_mask: np.ndarray
    safety_margin_violation_mask: np.ndarray
    radial_collision_mask: np.ndarray
    inlet_violation_mask: np.ndarray
    outlet_violation_mask: np.ndarray
    wall_penetrations: np.ndarray
    inlet_penetrations: np.ndarray
    outlet_penetrations: np.ndarray
    end_cap_penetrations: np.ndarray
    maximum_penetration_depth: float
    minimum_clearance: float
    mean_clearance: float
    p05_clearance: float
    closest_backbone_index: int
    closest_geometry_indices: np.ndarray
    closest_geometry_parameters: np.ndarray
    closest_geometry_points: np.ndarray
    centerline_progress: np.ndarray
    radial_distance: np.ndarray
    local_radius: np.ndarray
    axial_position: np.ndarray
    radial_clearance: np.ndarray
    axial_clearance: np.ndarray
    closest_backbone_point_index: int
    minimum_radial_clearance: float
    minimum_axial_clearance: float
    collision_count: int
    safety_margin_violation_count: int

    @property
    def collision_free(self) -> bool:
        return self.collision_count == 0

    @property
    def safety_margin_clear(self) -> bool:
        return self.safety_margin_violation_count == 0

    @property
    def maximum_penetration(self) -> float:
        return self.maximum_penetration_depth

    @property
    def closest_geometry_index(self) -> int:
        if self.closest_backbone_index < 0:
            return -1
        return int(self.closest_geometry_indices[self.closest_backbone_index])

    @property
    def closest_geometry_parameter(self) -> float:
        if self.closest_backbone_index < 0:
            return float("nan")
        return float(self.closest_geometry_parameters[self.closest_backbone_index])

    @property
    def minimum_progress(self) -> float:
        return float(np.min(self.centerline_progress))

    @property
    def maximum_progress(self) -> float:
        return float(np.max(self.centerline_progress))


@dataclass(frozen=True)
class TargetValidation:
    valid: bool
    reasons: list[str]
    target: np.ndarray
    clearance: PointClearance | None


@dataclass(frozen=True)
class LumenCostWeights:
    safety_margin_weight: float
    radial_collision_weight: float
    end_cap_weight: float
    terminal_collision_weight: float

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "LumenCostWeights":
        values = config.get("lumen_cost", config.get("cylindrical_lumen_cost", config.get("curved_lumen_cost", config)))
        if not isinstance(values, dict):
            raise ValueError("lumen cost configuration must be a map")
        return cls(
            safety_margin_weight=nonnegative_number(values["safety_margin_weight"], "safety_margin_weight"),
            radial_collision_weight=nonnegative_number(values["radial_collision_weight"], "radial_collision_weight"),
            end_cap_weight=nonnegative_number(values["end_cap_weight"], "end_cap_weight"),
            terminal_collision_weight=nonnegative_number(
                values["terminal_collision_weight"],
                "terminal_collision_weight",
            ),
        )


@dataclass(frozen=True)
class LumenCostBreakdown:
    """Unweighted lumen terms plus their exact weighted total."""

    safety_margin_raw: float
    wall_collision_raw: float
    end_cap_collision_raw: float
    terminal_collision_raw: float
    weighted_total: float


def make_backbone_clearance(
    *,
    points: np.ndarray,
    physical_clearances: np.ndarray,
    safety_margin: float,
    radial_distance: np.ndarray,
    local_radius: np.ndarray,
    centerline_progress: np.ndarray,
    closest_geometry_indices: np.ndarray,
    closest_geometry_parameters: np.ndarray,
    closest_geometry_points: np.ndarray,
    inlet_violation_mask: np.ndarray,
    outlet_violation_mask: np.ndarray,
    radial_collision_mask: np.ndarray,
    axial_clearance: np.ndarray,
    radial_penetration: np.ndarray,
    inlet_penetration: np.ndarray,
    outlet_penetration: np.ndarray,
) -> BackboneClearance:
    radial_penetration = np.asarray(radial_penetration, dtype=float)
    inlet_penetration = np.asarray(inlet_penetration, dtype=float)
    outlet_penetration = np.asarray(outlet_penetration, dtype=float)
    end_cap_penetration = np.maximum(inlet_penetration, outlet_penetration)
    if (
        radial_penetration.shape != physical_clearances.shape
        or inlet_penetration.shape != physical_clearances.shape
        or outlet_penetration.shape != physical_clearances.shape
    ):
        raise ValueError("penetration arrays must match physical_clearances shape")
    collision_mask = radial_collision_mask | inlet_violation_mask | outlet_violation_mask
    safety_margin_violation_mask = (
        (physical_clearances < safety_margin) | inlet_violation_mask | outlet_violation_mask
    )
    closest_backbone_index = int(np.argmin(physical_clearances))
    maximum_penetration_depth = float(
        np.max(np.maximum.reduce([radial_penetration, inlet_penetration, outlet_penetration]))
    )
    return BackboneClearance(
        points=points,
        physical_clearances=physical_clearances,
        safety_margin_clearances=physical_clearances - safety_margin,
        collision_mask=collision_mask,
        safety_margin_violation_mask=safety_margin_violation_mask,
        radial_collision_mask=radial_collision_mask,
        inlet_violation_mask=inlet_violation_mask,
        outlet_violation_mask=outlet_violation_mask,
        wall_penetrations=radial_penetration.copy(),
        inlet_penetrations=inlet_penetration.copy(),
        outlet_penetrations=outlet_penetration.copy(),
        end_cap_penetrations=end_cap_penetration,
        maximum_penetration_depth=maximum_penetration_depth,
        minimum_clearance=float(np.min(physical_clearances)),
        mean_clearance=float(np.mean(physical_clearances)),
        p05_clearance=float(np.percentile(physical_clearances, 5.0)),
        closest_backbone_index=closest_backbone_index,
        closest_geometry_indices=closest_geometry_indices,
        closest_geometry_parameters=closest_geometry_parameters,
        closest_geometry_points=closest_geometry_points,
        centerline_progress=centerline_progress,
        radial_distance=radial_distance,
        local_radius=local_radius,
        axial_position=centerline_progress,
        radial_clearance=physical_clearances,
        axial_clearance=axial_clearance,
        closest_backbone_point_index=closest_backbone_index,
        minimum_radial_clearance=float(np.min(physical_clearances)),
        minimum_axial_clearance=float(np.min(axial_clearance)),
        collision_count=int(np.sum(collision_mask)),
        safety_margin_violation_count=int(np.sum(safety_margin_violation_mask)),
    )


def compute_lumen_cost(
    *,
    lumen: LumenGeometry,
    weights: LumenCostWeights,
    backbone_points: Any,
    terminal: bool = False,
) -> float:
    return compute_lumen_cost_breakdown(
        lumen=lumen,
        weights=weights,
        backbone_points=backbone_points,
        terminal=terminal,
    ).weighted_total


def compute_lumen_cost_breakdown(
    *,
    lumen: LumenGeometry,
    weights: LumenCostWeights,
    backbone_points: Any,
    terminal: bool = False,
) -> LumenCostBreakdown:
    """Return the exact component values used by :func:`compute_lumen_cost`."""

    clearance = lumen.backbone_clearance(backbone_points)
    _validate_clearance_arrays(clearance)
    denominator = max(float(lumen.safety_margin), 1.0e-12)
    soft = np.maximum(0.0, float(lumen.safety_margin) - clearance.physical_clearances) / denominator
    wall = clearance.wall_penetrations / denominator
    inlet = clearance.inlet_penetrations / denominator
    outlet = clearance.outlet_penetrations / denominator
    end_cap = clearance.end_cap_penetrations / denominator
    safety_margin_raw = float(np.mean(soft**2))
    wall_collision_raw = float(np.mean(wall**2))
    end_cap_collision_raw = float(np.mean(end_cap**2))
    terminal_collision_raw = 0.0
    cost = (
        weights.safety_margin_weight * safety_margin_raw
        + weights.radial_collision_weight * wall_collision_raw
        + weights.end_cap_weight * end_cap_collision_raw
    )
    if terminal and clearance.points.shape[0] > 0:
        terminal_violation = max(float(np.max(wall)), float(np.max(inlet)), float(np.max(outlet)))
        if terminal_violation > 0.0:
            terminal_collision_raw = terminal_violation**2
            cost += weights.terminal_collision_weight * terminal_collision_raw
    if not math.isfinite(cost):
        raise ValueError("lumen cost is not finite")
    return LumenCostBreakdown(
        safety_margin_raw=safety_margin_raw,
        wall_collision_raw=wall_collision_raw,
        end_cap_collision_raw=end_cap_collision_raw,
        terminal_collision_raw=terminal_collision_raw,
        weighted_total=float(cost),
    )


def _validate_clearance_arrays(clearance: BackboneClearance) -> None:
    arrays = (
        clearance.physical_clearances,
        clearance.wall_penetrations,
        clearance.inlet_penetrations,
        clearance.outlet_penetrations,
        clearance.end_cap_penetrations,
    )
    expected_shape = clearance.physical_clearances.shape
    for array in arrays:
        if array.shape != expected_shape or not np.all(np.isfinite(array)):
            raise ValueError("lumen clearance output contains non-finite or inconsistent arrays")


def points_array(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] == 0 or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape (N, 3) with N > 0 and finite values")
    return array.copy()


def vector3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array.copy()


def non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def positive_number(value: Any, label: str) -> float:
    numeric = finite_number(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def nonnegative_number(value: Any, label: str) -> float:
    numeric = finite_number(value, label)
    if numeric < 0.0:
        raise ValueError(f"{label} must be nonnegative")
    return numeric


def unit_vector(values: Any, label: str) -> np.ndarray:
    vector = vector3(values, label)
    norm = float(np.linalg.norm(vector))
    if norm <= 0.0:
        raise ValueError(f"{label} must be non-zero")
    return vector / norm
