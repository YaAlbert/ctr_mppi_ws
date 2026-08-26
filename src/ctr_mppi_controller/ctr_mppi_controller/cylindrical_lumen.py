"""Analytical straight cylindrical-lumen geometry for simulation milestones."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .lumen_geometry import (
    BackboneClearance,
    LumenCostWeights,
    LumenGeometry,
    PointClearance,
    TargetValidation,
    compute_lumen_cost,
    make_backbone_clearance,
)


@dataclass(frozen=True)
class CylindricalLumen:
    frame_id: str
    axis_origin: np.ndarray
    axis_direction: np.ndarray
    radius: float
    length: float
    ctr_outer_radius: float
    safety_margin: float

    def __post_init__(self) -> None:
        frame = _non_empty_string(self.frame_id, "cylindrical_lumen.frame_id")
        origin = _vector3(self.axis_origin, "cylindrical_lumen.axis_origin")
        direction = _vector3(self.axis_direction, "cylindrical_lumen.axis_direction")
        norm = float(np.linalg.norm(direction))
        if norm <= 0.0:
            raise ValueError("cylindrical_lumen.axis_direction must be non-zero")
        radius = _positive_number(self.radius, "cylindrical_lumen.radius")
        length = _positive_number(self.length, "cylindrical_lumen.length")
        outer_radius = _positive_number(self.ctr_outer_radius, "cylindrical_lumen.ctr_outer_radius")
        safety_margin = _positive_number(self.safety_margin, "cylindrical_lumen.safety_margin")
        if radius <= outer_radius:
            raise ValueError("cylindrical_lumen.radius must exceed ctr_outer_radius")
        if radius - outer_radius <= safety_margin:
            raise ValueError("cylindrical_lumen usable radius must exceed safety_margin")
        object.__setattr__(self, "frame_id", frame)
        object.__setattr__(self, "axis_origin", origin)
        object.__setattr__(self, "axis_direction", direction / norm)
        object.__setattr__(self, "radius", radius)
        object.__setattr__(self, "length", length)
        object.__setattr__(self, "ctr_outer_radius", outer_radius)
        object.__setattr__(self, "safety_margin", safety_margin)

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "CylindricalLumen":
        lumen = config.get("cylindrical_lumen", config)
        if not isinstance(lumen, dict):
            raise ValueError("cylindrical_lumen must be a map")
        return cls(
            frame_id=lumen["frame_id"],
            axis_origin=lumen["axis_origin"],
            axis_direction=lumen["axis_direction"],
            radius=lumen["radius"],
            length=lumen["length"],
            ctr_outer_radius=lumen["ctr_outer_radius"],
            safety_margin=lumen["safety_margin"],
        )

    @property
    def usable_radius(self) -> float:
        return self.radius - self.ctr_outer_radius

    @property
    def preferred_radius(self) -> float:
        return self.usable_radius - self.safety_margin

    def point_clearance(self, point: Any) -> "PointClearance":
        point_array = _vector3(point, "point")
        offset = point_array - self.axis_origin
        axial = float(np.dot(offset, self.axis_direction))
        radial_vector = offset - axial * self.axis_direction
        radial_distance = float(np.linalg.norm(radial_vector))
        radial_clearance = float(self.usable_radius - radial_distance)
        axial_clearance = float(min(axial, self.length - axial))
        radial_collision = radial_clearance < 0.0
        inlet_violation = axial < 0.0
        outlet_violation = axial > self.length
        clamped_axial = float(np.clip(axial, 0.0, self.length))
        radial_penetration = max(0.0, -radial_clearance)
        inlet_penetration = max(0.0, -axial)
        outlet_penetration = max(0.0, axial - self.length)
        end_cap_penetration = max(inlet_penetration, outlet_penetration)
        maximum_penetration = max(radial_penetration, end_cap_penetration)
        return PointClearance(
            point=point_array,
            physical_clearance=radial_clearance,
            safety_margin_clearance=float(radial_clearance - self.safety_margin),
            collision=bool(radial_collision or inlet_violation or outlet_violation),
            safety_margin_violation=radial_clearance < self.safety_margin,
            inlet_violation=inlet_violation,
            outlet_violation=outlet_violation,
            maximum_penetration=float(maximum_penetration),
            centerline_progress=axial,
            closest_geometry_index=0,
            closest_geometry_parameter=clamped_axial / self.length,
            closest_geometry_point=self.axis_origin + clamped_axial * self.axis_direction,
            radial_distance=radial_distance,
            local_radius=self.radius,
            wall_penetration=float(radial_penetration),
            inlet_penetration=float(inlet_penetration),
            outlet_penetration=float(outlet_penetration),
            end_cap_penetration=float(end_cap_penetration),
            axial_position=axial,
            radial_clearance=radial_clearance,
            axial_clearance=axial_clearance,
            radial_collision=radial_collision,
        )

    def backbone_clearance(self, backbone_points: Any) -> "BackboneClearance":
        points = _points(backbone_points, "backbone_points")
        offsets = points - self.axis_origin
        axial = offsets @ self.axis_direction
        radial_vectors = offsets - axial[:, None] * self.axis_direction[None, :]
        radial_distance = np.linalg.norm(radial_vectors, axis=1)
        radial_clearance = self.usable_radius - radial_distance
        axial_clearance = np.minimum(axial, self.length - axial)
        radial_collision = radial_clearance < 0.0
        inlet_violation = axial < 0.0
        outlet_violation = axial > self.length
        collision = radial_collision | inlet_violation | outlet_violation
        safety_margin_violation = (radial_clearance < self.safety_margin) | inlet_violation | outlet_violation
        radial_penetration = np.maximum(-radial_clearance, 0.0)
        inlet_penetration = np.maximum(-axial, 0.0)
        outlet_penetration = np.maximum(axial - self.length, 0.0)
        clamped_axial = np.clip(axial, 0.0, self.length)
        result = make_backbone_clearance(
            points=points,
            physical_clearances=radial_clearance,
            safety_margin=self.safety_margin,
            radial_distance=radial_distance,
            local_radius=np.full(points.shape[0], self.radius, dtype=float),
            centerline_progress=axial,
            closest_geometry_indices=np.zeros(points.shape[0], dtype=int),
            closest_geometry_parameters=clamped_axial / self.length,
            closest_geometry_points=self.axis_origin[None, :] + clamped_axial[:, None] * self.axis_direction[None, :],
            inlet_violation_mask=inlet_violation,
            outlet_violation_mask=outlet_violation,
            radial_collision_mask=radial_collision,
            axial_clearance=axial_clearance,
            radial_penetration=radial_penetration,
            inlet_penetration=inlet_penetration,
            outlet_penetration=outlet_penetration,
        )
        if (
            not np.array_equal(result.collision_mask, collision)
            or not np.array_equal(result.safety_margin_violation_mask, safety_margin_violation)
        ):
            raise RuntimeError("cylindrical lumen shared clearance conversion changed collision semantics")
        return result

    def validate_target(
        self,
        target: Any,
        *,
        frame_id: str | None = None,
        require_safety_margin: bool = True,
    ) -> "TargetValidation":
        reasons: list[str] = []
        try:
            point = _vector3(target, "target")
        except ValueError as exc:
            return TargetValidation(False, [str(exc)], np.asarray([], dtype=float), None)
        if frame_id is not None and frame_id != self.frame_id:
            reasons.append(f"target frame_id `{frame_id}` does not match lumen frame_id `{self.frame_id}`")
        clearance = self.point_clearance(point)
        if clearance.axial_position < 0.0:
            reasons.append("target is before the cylinder inlet")
        if clearance.axial_position > self.length:
            reasons.append("target is after the cylinder outlet")
        if clearance.radial_clearance < 0.0:
            reasons.append("target intersects the cylindrical wall")
        if require_safety_margin and clearance.radial_clearance < self.safety_margin:
            reasons.append("target violates the cylindrical safety margin")
        return TargetValidation(not reasons, reasons, point, clearance)

    def nearest_valid_target(self, target: Any, *, use_safety_margin: bool = True) -> np.ndarray:
        point = _vector3(target, "target")
        offset = point - self.axis_origin
        raw_axial = float(np.dot(offset, self.axis_direction))
        axial = float(np.clip(raw_axial, 0.0, self.length))
        radial_vector = offset - raw_axial * self.axis_direction
        radial_distance = float(np.linalg.norm(radial_vector))
        allowed_radius = self.preferred_radius if use_safety_margin else self.usable_radius
        if use_safety_margin:
            allowed_radius = max(0.0, allowed_radius - 1.0e-12)
        if radial_distance > allowed_radius and radial_distance > 0.0:
            radial_vector = radial_vector * (allowed_radius / radial_distance)
        return self.axis_origin + axial * self.axis_direction + radial_vector


def cylindrical_lumen_enabled(config: dict[str, Any]) -> bool:
    from .lumen_factory import cylindrical_lumen_enabled as _enabled

    return _enabled(config)


def curved_lumen_enabled(config: dict[str, Any]) -> bool:
    from .lumen_factory import curved_lumen_enabled as _enabled

    return _enabled(config)


def lumen_mode_from_config(config: dict[str, Any]) -> str:
    from .lumen_factory import lumen_mode_from_config as _mode

    return _mode(config)


def lumen_geometry_from_config(config: dict[str, Any]) -> LumenGeometry | None:
    from .lumen_factory import lumen_geometry_from_config as _geometry

    return _geometry(config)


def lumen_cost_weights_from_config(config: dict[str, Any]) -> LumenCostWeights | None:
    from .lumen_factory import lumen_cost_weights_from_config as _weights

    return _weights(config)


def goal_position_from_config(config: dict[str, Any]) -> np.ndarray:
    return _vector3(config["goal"]["position"], "goal.position")


def goal_tolerance_from_config(config: dict[str, Any]) -> float:
    return _positive_number(config["goal"]["tolerance"], "goal.tolerance")


def goal_hold_duration_from_config(config: dict[str, Any]) -> float:
    return _nonnegative_number(config["goal"]["required_hold_duration"], "goal.required_hold_duration")


def config_with_mppi_profile(config: dict[str, Any], profile_name: str | None) -> dict[str, Any]:
    from .lumen_factory import config_with_mppi_profile as _with_profile

    return _with_profile(config, profile_name)


def config_with_cylinder_overrides(
    config: dict[str, Any],
    *,
    enabled: bool | None = None,
    target_position: Any | None = None,
    mppi_profile: str | None = None,
    random_seed: Any | None = None,
) -> dict[str, Any]:
    from .lumen_factory import config_with_lumen_overrides

    return config_with_lumen_overrides(
        config,
        enable_cylindrical_lumen=enabled,
        target=target_position,
        cylinder_profile=mppi_profile,
        random_seed=random_seed,
    )


def _points(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] == 0 or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape (N, 3) with N > 0 and finite values")
    return array.copy()


def _vector3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array.copy()


def _non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


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
    if integer != float(value) or integer < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return integer
