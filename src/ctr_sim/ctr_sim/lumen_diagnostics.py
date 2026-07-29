"""ROS-independent dynamic lumen diagnostic helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen
from ctr_mppi_controller.lumen_geometry import BackboneClearance, LumenGeometry, points_array


CONSTRAINT_WALL = "wall"
CONSTRAINT_INLET = "inlet"
CONSTRAINT_OUTLET = "outlet"
CONSTRAINT_TYPES = (CONSTRAINT_WALL, CONSTRAINT_INLET, CONSTRAINT_OUTLET)

STATUS_SAFE = "SAFE"
STATUS_MARGIN = "SAFETY_MARGIN_VIOLATION"
STATUS_COLLISION = "PHYSICAL_COLLISION"
STATUS_UNAVAILABLE = "UNAVAILABLE"
DIAGNOSTIC_STATUSES = (STATUS_SAFE, STATUS_MARGIN, STATUS_COLLISION, STATUS_UNAVAILABLE)

WITNESS_TOLERANCE = 1.0e-12


@dataclass(frozen=True)
class LumenRuntimeDiagnostic:
    frame_id: str
    geometry_mode: str
    constraint_type: str
    backbone_index: int
    backbone_center_point: np.ndarray
    ctr_surface_point: np.ndarray
    lumen_reference_point: np.ndarray
    lumen_boundary_point: np.ndarray
    physical_clearance: float
    safety_clearance: float
    physical_collision: bool
    safety_margin_violation: bool
    status: str
    valid: bool
    reason: str
    witness_available: bool = True
    minimum_physical_clearance: float | None = None
    minimum_safety_clearance: float | None = None

    def __post_init__(self) -> None:
        status = _status(self.status)
        reason = str(self.reason)
        valid = bool(self.valid)
        if valid:
            frame = _non_empty_string(self.frame_id, "frame_id")
            constraint = _constraint(self.constraint_type)
            index = _nonnegative_int(self.backbone_index, "backbone_index")
            physical = _finite_number(self.physical_clearance, "physical_clearance")
            safety = _finite_number(self.safety_clearance, "safety_clearance")
            if bool(self.physical_collision) and not bool(self.safety_margin_violation):
                raise ValueError("physical_collision requires safety_margin_violation")
            expected_status = _status_from_booleans(self.physical_collision, self.safety_margin_violation)
            if status != expected_status:
                raise ValueError(
                    f"status {status} is inconsistent with collision={self.physical_collision} "
                    f"and safety_margin_violation={self.safety_margin_violation}"
                )
        else:
            frame = str(self.frame_id or "")
            constraint = CONSTRAINT_WALL if str(self.constraint_type) not in CONSTRAINT_TYPES else str(self.constraint_type)
            index = int(self.backbone_index)
            physical = float(self.physical_clearance)
            safety = float(self.safety_clearance)
            if status != STATUS_UNAVAILABLE:
                raise ValueError("invalid diagnostics must use status UNAVAILABLE")
            if not reason:
                raise ValueError("invalid diagnostics require a reason")

        object.__setattr__(self, "frame_id", frame)
        object.__setattr__(self, "constraint_type", constraint)
        object.__setattr__(self, "backbone_index", index)
        object.__setattr__(self, "backbone_center_point", _readonly_vector(self.backbone_center_point, "backbone_center_point"))
        object.__setattr__(self, "ctr_surface_point", _readonly_vector(self.ctr_surface_point, "ctr_surface_point"))
        object.__setattr__(self, "lumen_reference_point", _readonly_vector(self.lumen_reference_point, "lumen_reference_point"))
        object.__setattr__(self, "lumen_boundary_point", _readonly_vector(self.lumen_boundary_point, "lumen_boundary_point"))
        object.__setattr__(self, "physical_clearance", physical)
        object.__setattr__(self, "safety_clearance", safety)
        object.__setattr__(self, "physical_collision", bool(self.physical_collision))
        object.__setattr__(self, "safety_margin_violation", bool(self.safety_margin_violation))
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "valid", valid)
        object.__setattr__(self, "reason", reason)
        object.__setattr__(self, "witness_available", bool(self.witness_available))
        object.__setattr__(
            self,
            "minimum_physical_clearance",
            _finite_number(
                self.physical_clearance if self.minimum_physical_clearance is None else self.minimum_physical_clearance,
                "minimum_physical_clearance",
            ),
        )
        object.__setattr__(
            self,
            "minimum_safety_clearance",
            _finite_number(
                self.safety_clearance if self.minimum_safety_clearance is None else self.minimum_safety_clearance,
                "minimum_safety_clearance",
            ),
        )


def build_lumen_runtime_diagnostic(
    geometry: LumenGeometry,
    backbone_points: Any,
    geometry_mode: str,
) -> LumenRuntimeDiagnostic:
    """Build one status-driving diagnostic from a full CTR backbone."""

    mode = _geometry_mode(geometry_mode)
    if mode == "curved" and not isinstance(geometry, CurvedLumen):
        raise ValueError("geometry_mode curved requires CurvedLumen")
    if mode == "cylindrical" and not isinstance(geometry, CylindricalLumen):
        raise ValueError("geometry_mode cylindrical requires CylindricalLumen")

    points = points_array(backbone_points, "backbone_points")
    clearance = geometry.backbone_clearance(points)
    _validate_clearance(clearance, points.shape[0])

    physical_collision = clearance.collision_count > 0
    safety_margin_violation = clearance.safety_margin_violation_count > 0
    status = _status_from_booleans(physical_collision, safety_margin_violation)

    if physical_collision:
        index, constraint = _select_collision_constraint(clearance)
    elif safety_margin_violation:
        index, constraint = _select_safety_margin_constraint(clearance)
    else:
        index = int(clearance.closest_backbone_point_index)
        constraint = CONSTRAINT_WALL
    _validate_index(index, points.shape[0])

    physical_clearance = _constraint_physical_clearance(clearance, index, constraint)
    safety_clearance = float(clearance.safety_margin_clearances[index])
    if constraint == CONSTRAINT_WALL:
        ctr_surface, reference, boundary, witness_available = _wall_witness(geometry, clearance, index)
    else:
        ctr_surface, reference, boundary = _end_plane_witness(geometry, clearance, index, constraint)
        witness_available = True

    return LumenRuntimeDiagnostic(
        frame_id=str(geometry.frame_id),
        geometry_mode=mode,
        constraint_type=constraint,
        backbone_index=index,
        backbone_center_point=clearance.points[index],
        ctr_surface_point=ctr_surface,
        lumen_reference_point=reference,
        lumen_boundary_point=boundary,
        physical_clearance=physical_clearance,
        safety_clearance=safety_clearance,
        physical_collision=physical_collision,
        safety_margin_violation=safety_margin_violation,
        status=status,
        valid=True,
        reason="updated",
        witness_available=witness_available,
        minimum_physical_clearance=float(clearance.minimum_clearance),
        minimum_safety_clearance=float(np.min(clearance.safety_margin_clearances)),
    )


def unavailable_lumen_runtime_diagnostic(
    *,
    geometry_mode: str,
    reason: str,
    frame_id: str = "",
) -> LumenRuntimeDiagnostic:
    zeros = np.zeros(3, dtype=np.float64)
    return LumenRuntimeDiagnostic(
        frame_id=frame_id,
        geometry_mode=str(geometry_mode),
        constraint_type=CONSTRAINT_WALL,
        backbone_index=-1,
        backbone_center_point=zeros,
        ctr_surface_point=zeros,
        lumen_reference_point=zeros,
        lumen_boundary_point=zeros,
        physical_clearance=0.0,
        safety_clearance=0.0,
        physical_collision=False,
        safety_margin_violation=False,
        status=STATUS_UNAVAILABLE,
        valid=False,
        reason=reason,
        witness_available=False,
    )


def _validate_clearance(clearance: BackboneClearance, point_count: int) -> None:
    vector_arrays = (clearance.points, clearance.closest_geometry_points)
    for array in vector_arrays:
        if array.shape != (point_count, 3) or not np.all(np.isfinite(array)):
            raise ValueError("lumen clearance output contains invalid point arrays")
    scalar_arrays = (
        clearance.physical_clearances,
        clearance.safety_margin_clearances,
        clearance.wall_penetrations,
        clearance.inlet_penetrations,
        clearance.outlet_penetrations,
        clearance.radial_distance,
        clearance.local_radius,
        clearance.axial_clearance,
    )
    for array in scalar_arrays:
        if array.shape != (point_count,) or not np.all(np.isfinite(array)):
            raise ValueError("lumen clearance output contains invalid scalar arrays")
    mask_arrays = (
        clearance.collision_mask,
        clearance.safety_margin_violation_mask,
        clearance.radial_collision_mask,
        clearance.inlet_violation_mask,
        clearance.outlet_violation_mask,
    )
    for array in mask_arrays:
        if array.shape != (point_count,):
            raise ValueError("lumen clearance output contains invalid mask arrays")
    _validate_index(int(clearance.closest_backbone_point_index), point_count)
    for value_name, value in (
        ("minimum_clearance", clearance.minimum_clearance),
        ("minimum_radial_clearance", clearance.minimum_radial_clearance),
        ("minimum_axial_clearance", clearance.minimum_axial_clearance),
    ):
        _finite_number(value, value_name)
    if int(np.sum(clearance.collision_mask)) != int(clearance.collision_count):
        raise ValueError("lumen clearance collision count does not match collision mask")
    if int(np.sum(clearance.safety_margin_violation_mask)) != int(clearance.safety_margin_violation_count):
        raise ValueError("lumen clearance safety count does not match safety mask")


def _select_collision_constraint(clearance: BackboneClearance) -> tuple[int, str]:
    candidates: list[tuple[float, int, int, str]] = []
    for order, (constraint, penetrations) in enumerate(
        (
            (CONSTRAINT_WALL, clearance.wall_penetrations),
            (CONSTRAINT_INLET, clearance.inlet_penetrations),
            (CONSTRAINT_OUTLET, clearance.outlet_penetrations),
        )
    ):
        for index, value in enumerate(penetrations):
            penetration = float(value)
            if penetration > 0.0:
                candidates.append((-penetration, int(index), order, constraint))
    if not candidates:
        raise ValueError("collision result contains no positive wall, inlet, or outlet penetration")
    _negative_penetration, index, _order, constraint = sorted(candidates)[0]
    return index, constraint


def _select_safety_margin_constraint(clearance: BackboneClearance) -> tuple[int, str]:
    indices = np.flatnonzero(clearance.safety_margin_violation_mask)
    if indices.size == 0:
        raise ValueError("safety-margin result contains no violating point")
    values = clearance.safety_margin_clearances[indices]
    minimum = float(np.min(values))
    tied = [int(index) for index, value in zip(indices, values) if float(value) == minimum]
    return min(tied), CONSTRAINT_WALL


def _constraint_physical_clearance(clearance: BackboneClearance, index: int, constraint: str) -> float:
    if constraint == CONSTRAINT_WALL:
        return float(clearance.physical_clearances[index])
    if constraint == CONSTRAINT_INLET:
        return -float(clearance.inlet_penetrations[index])
    if constraint == CONSTRAINT_OUTLET:
        return -float(clearance.outlet_penetrations[index])
    raise ValueError(f"unsupported constraint_type {constraint}")


def _wall_witness(
    geometry: LumenGeometry,
    clearance: BackboneClearance,
    index: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
    backbone_center = clearance.points[index]
    reference = clearance.closest_geometry_points[index]
    radial_distance = float(clearance.radial_distance[index])
    if radial_distance <= WITNESS_TOLERANCE:
        return backbone_center, reference, reference, False
    direction = (backbone_center - reference) / radial_distance
    ctr_surface = backbone_center + direction * float(geometry.ctr_outer_radius)
    boundary = reference + direction * float(clearance.local_radius[index])
    _validate_finite_points(ctr_surface, reference, boundary)
    return ctr_surface, reference, boundary, True


def _end_plane_witness(
    geometry: LumenGeometry,
    clearance: BackboneClearance,
    index: int,
    constraint: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    point = clearance.points[index]
    if isinstance(geometry, CylindricalLumen):
        if constraint == CONSTRAINT_INLET:
            origin = geometry.axis_origin
        elif constraint == CONSTRAINT_OUTLET:
            origin = geometry.axis_origin + geometry.length * geometry.axis_direction
        else:
            raise ValueError(f"unsupported end-plane constraint {constraint}")
        normal = geometry.axis_direction
    elif isinstance(geometry, CurvedLumen):
        if constraint == CONSTRAINT_INLET:
            origin = geometry.centerline_points[0]
            normal = geometry.inlet_tangent
        elif constraint == CONSTRAINT_OUTLET:
            origin = geometry.centerline_points[-1]
            normal = geometry.outlet_tangent
        else:
            raise ValueError(f"unsupported end-plane constraint {constraint}")
    else:
        raise ValueError("end-plane witness requires cylindrical or curved lumen geometry")

    signed_distance = float(np.dot(point - origin, normal))
    plane_point = point - signed_distance * normal
    _validate_finite_points(point, origin, plane_point)
    return point, origin, plane_point


def _status_from_booleans(physical_collision: bool, safety_margin_violation: bool) -> str:
    if bool(physical_collision):
        return STATUS_COLLISION
    if bool(safety_margin_violation):
        return STATUS_MARGIN
    return STATUS_SAFE


def _geometry_mode(value: Any) -> str:
    mode = str(value)
    if mode not in {"curved", "cylindrical"}:
        raise ValueError("geometry_mode must be `curved` or `cylindrical`")
    return mode


def _constraint(value: Any) -> str:
    constraint = str(value)
    if constraint not in CONSTRAINT_TYPES:
        raise ValueError(f"constraint_type must be one of {CONSTRAINT_TYPES}")
    return constraint


def _status(value: Any) -> str:
    status = str(value)
    if status not in DIAGNOSTIC_STATUSES:
        raise ValueError(f"status must be one of {DIAGNOSTIC_STATUSES}")
    return status


def _non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a nonnegative integer")
    return int(value)


def _validate_index(index: int, point_count: int) -> None:
    if index < 0 or index >= point_count:
        raise ValueError(f"backbone_index {index} is outside backbone point count {point_count}")


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _readonly_vector(values: Any, label: str) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain three finite values") from exc
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain three finite values")
    result = array.astype(np.float64, copy=True)
    result.setflags(write=False)
    return result


def _validate_finite_points(*points: np.ndarray) -> None:
    for point in points:
        if np.asarray(point).shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("witness points must be finite 3D vectors")
