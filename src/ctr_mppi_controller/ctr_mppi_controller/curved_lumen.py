"""Sampled centerline tubular-lumen geometry for software simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .lumen_geometry import (
    BOUNDARY_TOLERANCE,
    PROJECTION_TIE_TOLERANCE,
    BackboneClearance,
    PointClearance,
    TargetValidation,
    finite_number,
    make_backbone_clearance,
    non_empty_string,
    nonnegative_number,
    points_array,
    positive_number,
    unit_vector,
    vector3,
)


@dataclass(frozen=True)
class CenterlineProjection:
    closest_point: np.ndarray
    segment_index: int
    segment_parameter: float
    progress: float
    radial_distance: float
    local_radius: float


@dataclass(frozen=True)
class CurvedLumen:
    frame_id: str
    centerline_points: np.ndarray
    lumen_radius: float | np.ndarray
    ctr_outer_radius: float
    safety_margin: float

    def __post_init__(self) -> None:
        frame = non_empty_string(self.frame_id, "curved_lumen.frame_id")
        centerline = points_array(self.centerline_points, "curved_lumen.centerline_points")
        if centerline.shape[0] < 2:
            raise ValueError("curved_lumen.centerline_points must contain at least two points")
        segment_vectors = centerline[1:] - centerline[:-1]
        segment_lengths = np.linalg.norm(segment_vectors, axis=1)
        if np.any(segment_lengths <= 0.0):
            raise ValueError("curved_lumen.centerline_points must not contain duplicate consecutive points")
        radius_profile, stored_radius = _radius_profile(self.lumen_radius, centerline.shape[0])
        outer_radius = nonnegative_number(self.ctr_outer_radius, "curved_lumen.ctr_outer_radius")
        safety_margin = nonnegative_number(self.safety_margin, "curved_lumen.safety_margin")
        usable_radius = float(np.min(radius_profile) - outer_radius)
        if usable_radius <= 0.0:
            raise ValueError("curved_lumen.lumen_radius must exceed ctr_outer_radius")
        if safety_margin > usable_radius:
            raise ValueError("curved_lumen usable radius must be at least safety_margin")
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        if np.any(np.diff(cumulative) <= 0.0):
            raise ValueError("curved_lumen cumulative arc length must be strictly increasing")
        object.__setattr__(self, "frame_id", frame)
        object.__setattr__(self, "centerline_points", centerline)
        object.__setattr__(self, "lumen_radius", stored_radius)
        object.__setattr__(self, "ctr_outer_radius", outer_radius)
        object.__setattr__(self, "safety_margin", safety_margin)
        object.__setattr__(self, "radius_profile", radius_profile)
        object.__setattr__(self, "segment_vectors", segment_vectors)
        object.__setattr__(self, "segment_lengths", segment_lengths)
        object.__setattr__(self, "segment_unit_vectors", segment_vectors / segment_lengths[:, None])
        object.__setattr__(self, "cumulative_arc_lengths", cumulative)
        object.__setattr__(self, "length", float(cumulative[-1]))
        object.__setattr__(self, "inlet_tangent", segment_vectors[0] / segment_lengths[0])
        object.__setattr__(self, "outlet_tangent", segment_vectors[-1] / segment_lengths[-1])

    @property
    def minimum_lumen_radius(self) -> float:
        return float(np.min(self.radius_profile))

    @property
    def minimum_usable_radius(self) -> float:
        return self.minimum_lumen_radius - self.ctr_outer_radius

    def project_point(self, point: Any) -> CenterlineProjection:
        point_array = vector3(point, "point")
        starts = self.centerline_points[:-1]
        vectors = self.segment_vectors
        squared_lengths = self.segment_lengths * self.segment_lengths
        offsets = point_array - starts
        raw_parameters = np.einsum(
            "ij,ij->i", offsets, vectors
        ) / squared_lengths
        parameters = np.clip(raw_parameters, 0.0, 1.0)
        closest_points = starts + parameters[:, None] * vectors
        deltas = point_array - closest_points
        distance_squared = np.einsum("ij,ij->i", deltas, deltas)

        best_distance_squared = float("inf")
        best_index = 0
        best_parameter = 0.0
        for index, (distance, parameter) in enumerate(
            zip(distance_squared, parameters)
        ):
            distance = float(distance)
            parameter = float(parameter)
            if _is_better_projection(
                distance,
                index,
                parameter,
                best_distance_squared,
                best_index,
                best_parameter,
            ):
                best_distance_squared = distance
                best_index = index
                best_parameter = parameter
        best_closest = closest_points[best_index]
        progress = float(
            self.cumulative_arc_lengths[best_index]
            + best_parameter * self.segment_lengths[best_index]
        )
        radius = float(
            (1.0 - best_parameter) * self.radius_profile[best_index]
            + best_parameter * self.radius_profile[best_index + 1]
        )
        return CenterlineProjection(
            closest_point=best_closest.copy(),
            segment_index=best_index,
            segment_parameter=best_parameter,
            progress=progress,
            radial_distance=float(np.sqrt(best_distance_squared)),
            local_radius=radius,
        )

    def point_clearance(self, point: Any) -> PointClearance:
        point_array = vector3(point, "point")
        projection = self.project_point(point_array)
        inlet_signed = float(np.dot(point_array - self.centerline_points[0], self.inlet_tangent))
        outlet_signed = float(np.dot(point_array - self.centerline_points[-1], self.outlet_tangent))
        inlet_violation = inlet_signed < -BOUNDARY_TOLERANCE
        outlet_violation = outlet_signed > BOUNDARY_TOLERANCE
        physical_clearance = float(projection.local_radius - self.ctr_outer_radius - projection.radial_distance)
        # Match the straight-cylinder convention: exact wall contact is not negative penetration.
        radial_collision = physical_clearance < -BOUNDARY_TOLERANCE
        collision = bool(radial_collision or inlet_violation or outlet_violation)
        safety_margin_violation = bool(
            physical_clearance < self.safety_margin or inlet_violation or outlet_violation
        )
        inlet_penetration = max(0.0, -inlet_signed)
        outlet_penetration = max(0.0, outlet_signed)
        radial_penetration = max(0.0, -physical_clearance)
        return PointClearance(
            point=point_array,
            physical_clearance=physical_clearance,
            safety_margin_clearance=float(physical_clearance - self.safety_margin),
            collision=collision,
            safety_margin_violation=safety_margin_violation,
            inlet_violation=bool(inlet_violation),
            outlet_violation=bool(outlet_violation),
            maximum_penetration=float(max(radial_penetration, inlet_penetration, outlet_penetration)),
            centerline_progress=projection.progress,
            closest_geometry_index=projection.segment_index,
            closest_geometry_parameter=projection.segment_parameter,
            closest_geometry_point=projection.closest_point,
            radial_distance=projection.radial_distance,
            local_radius=projection.local_radius,
            wall_penetration=float(radial_penetration),
            inlet_penetration=float(inlet_penetration),
            outlet_penetration=float(outlet_penetration),
            end_cap_penetration=float(max(inlet_penetration, outlet_penetration)),
            axial_position=projection.progress,
            radial_clearance=physical_clearance,
            axial_clearance=float(min(inlet_signed, -outlet_signed)),
            radial_collision=bool(radial_collision),
        )

    def backbone_clearance(self, backbone_points: Any) -> BackboneClearance:
        points = points_array(backbone_points, "backbone_points")
        (
            closest_geometry_indices,
            closest_geometry_parameters,
            closest_geometry_points,
            centerline_progress,
            radial_distance,
            local_radius,
        ) = self._project_points(points)
        inlet_signed = (points - self.centerline_points[0]) @ self.inlet_tangent
        outlet_signed = (points - self.centerline_points[-1]) @ self.outlet_tangent
        inlet_violation = inlet_signed < -BOUNDARY_TOLERANCE
        outlet_violation = outlet_signed > BOUNDARY_TOLERANCE
        physical_clearances = local_radius - self.ctr_outer_radius - radial_distance
        radial_collision = physical_clearances < -BOUNDARY_TOLERANCE
        axial_clearance = np.minimum(inlet_signed, -outlet_signed)
        radial_penetration = np.maximum(-physical_clearances, 0.0)
        inlet_penetration = np.maximum(-inlet_signed, 0.0)
        outlet_penetration = np.maximum(outlet_signed, 0.0)
        return make_backbone_clearance(
            points=points,
            physical_clearances=physical_clearances,
            safety_margin=self.safety_margin,
            radial_distance=radial_distance,
            local_radius=local_radius,
            centerline_progress=centerline_progress,
            closest_geometry_indices=closest_geometry_indices,
            closest_geometry_parameters=closest_geometry_parameters,
            closest_geometry_points=closest_geometry_points,
            inlet_violation_mask=inlet_violation,
            outlet_violation_mask=outlet_violation,
            radial_collision_mask=radial_collision,
            axial_clearance=axial_clearance,
            radial_penetration=radial_penetration,
            inlet_penetration=inlet_penetration,
            outlet_penetration=outlet_penetration,
        )

    def cost_clearance_components(
        self, backbone_points: Any
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return only the arrays used by the MPPI lumen objective.

        This evaluation-selectable path deliberately omits summary percentiles,
        masks, and closest-point evidence that do not contribute to cost.  The
        safety/evidence path continues to use :meth:`backbone_clearance`.
        """

        points = points_array(backbone_points, "backbone_points")
        radial_distance, local_radius = self._project_points_for_cost(points)
        inlet_signed = (points - self.centerline_points[0]) @ self.inlet_tangent
        outlet_signed = (points - self.centerline_points[-1]) @ self.outlet_tangent
        physical_clearances = local_radius - self.ctr_outer_radius - radial_distance
        return (
            physical_clearances,
            np.maximum(-physical_clearances, 0.0),
            np.maximum(-inlet_signed, 0.0),
            np.maximum(outlet_signed, 0.0),
        )

    def _project_points_for_cost(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized projection preserving the sequential tolerance tie rule."""

        starts = self.centerline_points[:-1]
        vectors = self.segment_vectors
        squared_lengths = self.segment_lengths * self.segment_lengths
        offsets = points[:, None, :] - starts[None, :, :]
        raw_parameters = np.einsum("nsi,si->ns", offsets, vectors) / squared_lengths[None, :]
        parameters = np.clip(raw_parameters, 0.0, 1.0)
        closest_points = starts[None, :, :] + parameters[:, :, None] * vectors[None, :, :]
        deltas = points[:, None, :] - closest_points
        distance_squared = np.einsum("nsi,nsi->ns", deltas, deltas)

        global_minimum = np.min(distance_squared, axis=1)
        # With monotonically increasing segment indices, the reference loop
        # retains the earliest distance that the global minimum cannot improve
        # by more than PROJECTION_TIE_TOLERANCE.  Express the same comparison
        # in one NumPy batch using the reference subtraction order.
        retained = ~(
            global_minimum[:, None]
            < distance_squared - PROJECTION_TIE_TOLERANCE
        )
        best_indices = np.argmax(retained, axis=1)
        rows = np.arange(points.shape[0])
        best_parameters = parameters[rows, best_indices]
        best_distance_squared = distance_squared[rows, best_indices]
        radii = (
            (1.0 - best_parameters) * self.radius_profile[best_indices]
            + best_parameters * self.radius_profile[best_indices + 1]
        )
        return np.sqrt(best_distance_squared), radii

    def _project_points(
        self, points: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Project a backbone batch while preserving the scalar tie rule exactly."""

        starts = self.centerline_points[:-1]
        vectors = self.segment_vectors
        squared_lengths = self.segment_lengths * self.segment_lengths
        offsets = points[:, None, :] - starts[None, :, :]
        raw_parameters = np.einsum("nsi,si->ns", offsets, vectors) / squared_lengths[None, :]
        parameters = np.clip(raw_parameters, 0.0, 1.0)
        closest_points = starts[None, :, :] + parameters[:, :, None] * vectors[None, :, :]
        deltas = points[:, None, :] - closest_points
        distance_squared = np.einsum("nsi,nsi->ns", deltas, deltas)

        point_count = points.shape[0]
        best_distance_squared = np.full(point_count, np.inf, dtype=float)
        best_indices = np.zeros(point_count, dtype=int)
        best_parameters = np.zeros(point_count, dtype=float)
        # The tolerance relation is deliberately sequential and non-transitive.
        # Loop only over centerline segments while evaluating every backbone
        # point as one NumPy batch; this retains `_is_better_projection`.
        for index in range(vectors.shape[0]):
            distances = distance_squared[:, index]
            segment_parameters = parameters[:, index]
            better = distances < best_distance_squared - PROJECTION_TIE_TOLERANCE
            tied = np.abs(distances - best_distance_squared) <= PROJECTION_TIE_TOLERANCE
            better |= tied & (
                (index < best_indices)
                | ((index == best_indices) & (segment_parameters < best_parameters))
            )
            best_distance_squared[better] = distances[better]
            best_indices[better] = index
            best_parameters[better] = segment_parameters[better]

        rows = np.arange(point_count)
        selected_points = closest_points[rows, best_indices].copy()
        progress = (
            self.cumulative_arc_lengths[best_indices]
            + best_parameters * self.segment_lengths[best_indices]
        )
        radii = (
            (1.0 - best_parameters) * self.radius_profile[best_indices]
            + best_parameters * self.radius_profile[best_indices + 1]
        )
        return (
            best_indices,
            best_parameters,
            selected_points,
            progress,
            np.sqrt(best_distance_squared),
            radii,
        )

    def validate_target(
        self,
        target: Any,
        *,
        frame_id: str | None = None,
        require_safety_margin: bool = True,
    ) -> TargetValidation:
        reasons: list[str] = []
        try:
            point = vector3(target, "target")
        except ValueError as exc:
            return TargetValidation(False, [str(exc)], np.asarray([], dtype=float), None)
        if frame_id is not None and frame_id != self.frame_id:
            reasons.append(f"target frame_id `{frame_id}` does not match lumen frame_id `{self.frame_id}`")
        clearance = self.point_clearance(point)
        if clearance.inlet_violation:
            reasons.append("target is before the curved lumen inlet")
        if clearance.outlet_violation:
            reasons.append("target is after the curved lumen outlet")
        if clearance.radial_collision:
            reasons.append("target intersects the curved lumen wall")
        if require_safety_margin and clearance.physical_clearance < self.safety_margin:
            reasons.append("target violates the curved lumen safety margin")
        return TargetValidation(not reasons, reasons, point, clearance)


def circular_arc_centerline(
    *,
    inlet_position: Any,
    initial_tangent: Any,
    bend_normal: Any,
    curvature_radius: Any,
    arc_angle: Any,
    sample_spacing: Any,
) -> np.ndarray:
    inlet = vector3(inlet_position, "circular_arc.inlet_position")
    tangent = unit_vector(initial_tangent, "circular_arc.initial_tangent")
    normal = _orthogonal_unit_vector(bend_normal, tangent, "circular_arc.bend_normal")
    radius = positive_number(curvature_radius, "circular_arc.curvature_radius")
    angle = finite_number(arc_angle, "circular_arc.arc_angle")
    spacing = positive_number(sample_spacing, "circular_arc.sample_spacing")
    if angle == 0.0:
        raise ValueError("circular_arc.arc_angle must be non-zero")
    signed_normal = normal if angle > 0.0 else -normal
    total_length = radius * abs(angle)
    intervals = max(1, int(np.ceil(total_length / spacing)))
    theta = np.linspace(0.0, abs(angle), intervals + 1)
    points = (
        inlet[None, :]
        + radius * np.sin(theta)[:, None] * tangent[None, :]
        + radius * (1.0 - np.cos(theta))[:, None] * signed_normal[None, :]
    )
    return _deduplicated_generator_points(points, "circular_arc")


def s_curve_centerline(
    *,
    inlet_position: Any,
    initial_tangent: Any,
    bend_plane_normal: Any,
    total_length: Any,
    lateral_amplitude: Any,
    sample_spacing: Any,
) -> np.ndarray:
    inlet = vector3(inlet_position, "s_curve.inlet_position")
    tangent = unit_vector(initial_tangent, "s_curve.initial_tangent")
    normal = _orthogonal_unit_vector(bend_plane_normal, tangent, "s_curve.bend_plane_normal")
    length = positive_number(total_length, "s_curve.total_length")
    amplitude = finite_number(lateral_amplitude, "s_curve.lateral_amplitude")
    spacing = positive_number(sample_spacing, "s_curve.sample_spacing")
    intervals = max(4, int(np.ceil(length / spacing)))
    u = np.linspace(0.0, 1.0, intervals + 1)
    axial = length * u
    lateral = amplitude * np.sin(2.0 * np.pi * u)
    points = inlet[None, :] + axial[:, None] * tangent[None, :] + lateral[:, None] * normal[None, :]
    return _deduplicated_generator_points(points, "s_curve")


def _radius_profile(radius: Any, point_count: int) -> tuple[np.ndarray, float | np.ndarray]:
    array = np.asarray(radius, dtype=float)
    if array.shape == ():
        scalar = positive_number(float(array), "curved_lumen.lumen_radius")
        return np.full(point_count, scalar, dtype=float), scalar
    if array.shape != (point_count,):
        raise ValueError("curved_lumen.lumen_radius profile must have one value per centerline point")
    if not np.all(np.isfinite(array)):
        raise ValueError("curved_lumen.lumen_radius profile must contain finite values")
    if np.any(array <= 0.0):
        raise ValueError("curved_lumen.lumen_radius profile values must be positive")
    return array.astype(float).copy(), array.astype(float).copy()


def _orthogonal_unit_vector(values: Any, tangent: np.ndarray, label: str) -> np.ndarray:
    raw = vector3(values, label)
    normal = raw - float(np.dot(raw, tangent)) * tangent
    norm = float(np.linalg.norm(normal))
    if norm <= 1.0e-12:
        raise ValueError(f"{label} must not be parallel to the tangent")
    return normal / norm


def _is_better_projection(
    distance_squared: float,
    index: int,
    parameter: float,
    best_distance_squared: float,
    best_index: int,
    best_parameter: float,
) -> bool:
    if distance_squared < best_distance_squared - PROJECTION_TIE_TOLERANCE:
        return True
    if abs(distance_squared - best_distance_squared) > PROJECTION_TIE_TOLERANCE:
        return False
    if index != best_index:
        return index < best_index
    return parameter < best_parameter


def _deduplicated_generator_points(points: np.ndarray, label: str) -> np.ndarray:
    result = points_array(points, f"{label}.centerline_points")
    segment_lengths = np.linalg.norm(result[1:] - result[:-1], axis=1)
    if np.any(segment_lengths <= 0.0):
        raise ValueError(f"{label} generated duplicate consecutive centerline points")
    return result
