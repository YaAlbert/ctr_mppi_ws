"""Analytical straight cylindrical-lumen geometry for simulation milestones."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Any

import numpy as np


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
        return PointClearance(
            point=point_array,
            axial_position=axial,
            radial_distance=radial_distance,
            radial_clearance=radial_clearance,
            axial_clearance=axial_clearance,
            radial_collision=radial_clearance < 0.0,
            inlet_violation=axial < 0.0,
            outlet_violation=axial > self.length,
            safety_margin_violation=radial_clearance < self.safety_margin,
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
        closest_index = int(np.argmin(radial_clearance)) if radial_clearance.size else -1
        radial_penetration = np.maximum(-radial_clearance, 0.0)
        inlet_penetration = np.maximum(-axial, 0.0)
        outlet_penetration = np.maximum(axial - self.length, 0.0)
        max_penetration = float(np.max(np.maximum.reduce([radial_penetration, inlet_penetration, outlet_penetration])))
        return BackboneClearance(
            points=points,
            axial_position=axial,
            radial_distance=radial_distance,
            radial_clearance=radial_clearance,
            axial_clearance=axial_clearance,
            collision_mask=collision,
            safety_margin_violation_mask=safety_margin_violation,
            radial_collision_mask=radial_collision,
            inlet_violation_mask=inlet_violation,
            outlet_violation_mask=outlet_violation,
            closest_backbone_point_index=closest_index,
            minimum_radial_clearance=float(np.min(radial_clearance)),
            minimum_axial_clearance=float(np.min(axial_clearance)),
            collision_count=int(np.sum(collision)),
            safety_margin_violation_count=int(np.sum(safety_margin_violation)),
            maximum_penetration_depth=max_penetration,
        )

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


@dataclass(frozen=True)
class PointClearance:
    point: np.ndarray
    axial_position: float
    radial_distance: float
    radial_clearance: float
    axial_clearance: float
    radial_collision: bool
    inlet_violation: bool
    outlet_violation: bool
    safety_margin_violation: bool

    @property
    def collision(self) -> bool:
        return bool(self.radial_collision or self.inlet_violation or self.outlet_violation)


@dataclass(frozen=True)
class BackboneClearance:
    points: np.ndarray
    axial_position: np.ndarray
    radial_distance: np.ndarray
    radial_clearance: np.ndarray
    axial_clearance: np.ndarray
    collision_mask: np.ndarray
    safety_margin_violation_mask: np.ndarray
    radial_collision_mask: np.ndarray
    inlet_violation_mask: np.ndarray
    outlet_violation_mask: np.ndarray
    closest_backbone_point_index: int
    minimum_radial_clearance: float
    minimum_axial_clearance: float
    collision_count: int
    safety_margin_violation_count: int
    maximum_penetration_depth: float

    @property
    def collision_free(self) -> bool:
        return self.collision_count == 0

    @property
    def safety_margin_clear(self) -> bool:
        return self.safety_margin_violation_count == 0


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
        values = config.get("cylindrical_lumen_cost", config)
        if not isinstance(values, dict):
            raise ValueError("cylindrical_lumen_cost must be a map")
        return cls(
            safety_margin_weight=_nonnegative_number(values["safety_margin_weight"], "safety_margin_weight"),
            radial_collision_weight=_nonnegative_number(values["radial_collision_weight"], "radial_collision_weight"),
            end_cap_weight=_nonnegative_number(values["end_cap_weight"], "end_cap_weight"),
            terminal_collision_weight=_nonnegative_number(
                values["terminal_collision_weight"],
                "terminal_collision_weight",
            ),
        )


def cylindrical_lumen_enabled(config: dict[str, Any]) -> bool:
    return bool(config.get("cylindrical_lumen", {}).get("enabled", False))


def goal_position_from_config(config: dict[str, Any]) -> np.ndarray:
    return _vector3(config["goal"]["position"], "goal.position")


def goal_tolerance_from_config(config: dict[str, Any]) -> float:
    return _positive_number(config["goal"]["tolerance"], "goal.tolerance")


def goal_hold_duration_from_config(config: dict[str, Any]) -> float:
    return _nonnegative_number(config["goal"]["required_hold_duration"], "goal.required_hold_duration")


def config_with_mppi_profile(config: dict[str, Any], profile_name: str | None) -> dict[str, Any]:
    if profile_name is None or str(profile_name) == "":
        return deepcopy(config)
    name = str(profile_name)
    profiles = config.get("mppi_profiles", {})
    if name not in profiles:
        raise ValueError(f"unknown MPPI profile `{name}`")
    profile = profiles[name]
    result = deepcopy(config)
    result["mppi"]["num_samples"] = _positive_int(profile["samples"], f"mppi_profiles.{name}.samples")
    result["mppi"]["horizon"] = _positive_int(profile["horizon"], f"mppi_profiles.{name}.horizon")
    result["mppi"]["dt"] = _positive_number(profile["dt"], f"mppi_profiles.{name}.dt")
    if "lambda" in profile:
        result["mppi"]["lambda"] = _positive_number(profile["lambda"], f"mppi_profiles.{name}.lambda")
    if "noise_std" in profile:
        result["mppi"]["noise_std"] = deepcopy(profile["noise_std"])
    if "weights" in profile:
        result["mppi"].setdefault("weights", {}).update(deepcopy(profile["weights"]))
    if "control_frequency" in profile:
        result["mppi"]["control_frequency"] = _positive_number(
            profile["control_frequency"],
            f"mppi_profiles.{name}.control_frequency",
        )
    else:
        period = _positive_number(profile["control_period"], f"mppi_profiles.{name}.control_period")
        result["mppi"]["control_frequency"] = 1.0 / period
    result.setdefault("mppi", {})["active_profile"] = name
    return result


def config_with_cylinder_overrides(
    config: dict[str, Any],
    *,
    enabled: bool | None = None,
    target_position: Any | None = None,
    mppi_profile: str | None = None,
    random_seed: Any | None = None,
) -> dict[str, Any]:
    result = config_with_mppi_profile(config, mppi_profile)
    if enabled is not None:
        result.setdefault("cylindrical_lumen", {})["enabled"] = bool(enabled)
    if target_position is not None:
        result.setdefault("goal", {})["position"] = [float(value) for value in _vector3(target_position, "target_position")]
    if random_seed is not None and str(random_seed) != "":
        seed = _nonnegative_int(random_seed, "mppi.random_seed")
        result.setdefault("mppi", {})["random_seed"] = seed
    return result


def compute_lumen_cost(
    *,
    lumen: CylindricalLumen,
    weights: LumenCostWeights,
    backbone_points: Any,
    terminal: bool = False,
) -> float:
    clearance = lumen.backbone_clearance(backbone_points)
    denominator = max(lumen.safety_margin, 1.0e-12)
    soft = np.maximum(0.0, lumen.safety_margin - clearance.radial_clearance) / denominator
    radial = np.maximum(0.0, -clearance.radial_clearance) / denominator
    inlet = np.maximum(0.0, -clearance.axial_position) / denominator
    outlet = np.maximum(0.0, clearance.axial_position - lumen.length) / denominator
    end_cap = np.maximum(inlet, outlet)
    cost = (
        weights.safety_margin_weight * float(np.mean(soft**2))
        + weights.radial_collision_weight * float(np.mean(radial**2))
        + weights.end_cap_weight * float(np.mean(end_cap**2))
    )
    if terminal and clearance.points.shape[0] > 0:
        terminal_violation = max(float(np.max(radial)), float(np.max(end_cap)))
        if terminal_violation > 0.0:
            cost += weights.terminal_collision_weight * terminal_violation**2
    if not math.isfinite(cost):
        raise ValueError("cylindrical lumen cost is not finite")
    return float(cost)


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
