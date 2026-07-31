"""Deterministic curved-lumen fixed-target scenario resolution.

This module is ROS-independent.  It consumes an already resolved project
configuration and, optionally, an already constructed curved-lumen geometry.
Scenario resolution uses geometry-relative arc length and validates every
requested target with the committed lumen geometry API.
"""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from typing import Any

import numpy as np

from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_mppi_controller.lumen_factory import (
    CURVED_LUMEN_TYPES,
    config_with_lumen_overrides,
    lumen_geometry_fingerprint,
    lumen_geometry_fingerprint_payload,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)


CURVED_SCENARIO_POLICY_VERSION = "curved_scenario_v1"

CENTERLINE_TARGET = "centerline_target"
LATERAL_OFFSET_TARGET = "lateral_offset_target"
NEAR_SAFETY_BOUNDARY_TARGET = "near_safety_boundary_target"
CURVED_LUMEN_SCENARIO_IDS = (
    CENTERLINE_TARGET,
    LATERAL_OFFSET_TARGET,
    NEAR_SAFETY_BOUNDARY_TARGET,
)

SCENARIO_CENTERLINE_FRACTIONS = {
    CENTERLINE_TARGET: 0.70,
    LATERAL_OFFSET_TARGET: 0.72,
    NEAR_SAFETY_BOUNDARY_TARGET: 0.75,
}

GEOMETRY_MODE_CURVED = "curved"
_FALLBACK_TOLERANCE = 1.0e-12


@dataclass(frozen=True)
class CurvedLumenScenario:
    """Resolved deterministic fixed-target scenario for a curved lumen."""

    scenario_id: str
    policy_version: str
    curved_lumen_type: str
    geometry_mode: str
    geometry_frame: str
    geometry_fingerprint: str
    geometry_fingerprint_payload: tuple[tuple[str, Any], ...]
    scenario_fingerprint: str
    scenario_identity_payload: tuple[tuple[str, Any], ...]
    centerline_fraction: float
    centerline_arc_length: float
    normalized_centerline_fraction: float
    centerline_segment_index: int
    centerline_segment_parameter: float
    centerline_point: np.ndarray
    local_tangent: np.ndarray
    radial_direction: np.ndarray
    radial_offset: float
    local_radius: float
    preferred_radius: float
    boundary_guard: float
    derived_target: np.ndarray
    requested_target: np.ndarray
    validated_target: np.ndarray
    override_used: bool
    require_safety_margin: bool
    near_boundary: bool
    validation_reasons: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "centerline_point", _readonly_vector3(self.centerline_point, "centerline_point"))
        object.__setattr__(self, "local_tangent", _readonly_unit_vector(self.local_tangent, "local_tangent"))
        object.__setattr__(self, "radial_direction", _readonly_unit_vector(self.radial_direction, "radial_direction"))
        object.__setattr__(self, "derived_target", _readonly_vector3(self.derived_target, "derived_target"))
        object.__setattr__(self, "requested_target", _readonly_vector3(self.requested_target, "requested_target"))
        object.__setattr__(self, "validated_target", _readonly_vector3(self.validated_target, "validated_target"))
        _finite_float(self.centerline_fraction, "centerline_fraction")
        _finite_float(self.centerline_arc_length, "centerline_arc_length")
        _finite_float(self.normalized_centerline_fraction, "normalized_centerline_fraction")
        _finite_float(self.centerline_segment_parameter, "centerline_segment_parameter")
        _finite_float(self.radial_offset, "radial_offset")
        _finite_float(self.local_radius, "local_radius")
        _finite_float(self.preferred_radius, "preferred_radius")
        _finite_float(self.boundary_guard, "boundary_guard")
        if int(self.centerline_segment_index) != self.centerline_segment_index or self.centerline_segment_index < 0:
            raise ValueError("centerline_segment_index must be a non-negative integer")
        if not isinstance(self.override_used, bool):
            raise ValueError("override_used must be a bool")
        if not isinstance(self.require_safety_margin, bool):
            raise ValueError("require_safety_margin must be a bool")
        if not isinstance(self.near_boundary, bool):
            raise ValueError("near_boundary must be a bool")
        if not isinstance(self.validation_reasons, tuple):
            object.__setattr__(self, "validation_reasons", tuple(self.validation_reasons))


def resolve_curved_lumen_scenario(
    config: Mapping[str, Any],
    scenario_id: str,
    target_override: Any | None = None,
    *,
    curved_lumen_type: str | None = None,
    geometry: CurvedLumen | None = None,
) -> CurvedLumenScenario:
    """Resolve one deterministic curved-lumen fixed-target scenario.

    When ``geometry`` is omitted, the function constructs exactly one
    configured curved geometry through ``lumen_geometry_from_config`` after
    applying the effective curved-lumen mode overrides.  When ``geometry`` is
    supplied, the same object is used for sampling, radius lookup, validation,
    and fingerprint identity.
    """

    scenario_key = _scenario_id(scenario_id)
    lumen_type = _curved_lumen_type(config, curved_lumen_type)
    effective_config = config_with_lumen_overrides(
        deepcopy(dict(config)),
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type=lumen_type,
    )
    if lumen_mode_from_config(effective_config) != GEOMETRY_MODE_CURVED:
        raise ValueError("curved scenario resolution requires curved lumen mode")

    if geometry is None:
        resolved_geometry = lumen_geometry_from_config(effective_config)
    else:
        resolved_geometry = geometry
    if not isinstance(resolved_geometry, CurvedLumen):
        raise ValueError("curved scenario resolution requires a CurvedLumen geometry")

    centerline = _centerline_data(resolved_geometry)
    fraction = SCENARIO_CENTERLINE_FRACTIONS[scenario_key]
    sample = _sample_centerline(centerline, fraction)
    reference_normal = _reference_normal(effective_config, lumen_type)
    radial_direction = _radial_direction(sample.tangent, reference_normal)
    local_radius = _local_radius(resolved_geometry, sample.segment_index, sample.segment_parameter)
    preferred_radius = _preferred_radius(resolved_geometry, local_radius)
    boundary_guard = 0.0
    if scenario_key == CENTERLINE_TARGET:
        radial_offset = 0.0
    elif scenario_key == LATERAL_OFFSET_TARGET:
        radial_offset = 0.5 * preferred_radius
    else:
        boundary_guard = min(0.001, 0.10 * preferred_radius)
        if boundary_guard <= 0.0 or boundary_guard >= preferred_radius:
            raise ValueError("near-boundary scenario requires a positive interior boundary guard")
        radial_offset = preferred_radius - boundary_guard

    derived_target = sample.point + radial_offset * radial_direction
    requested_target = _target_override(target_override) if target_override is not None else derived_target
    validation = resolved_geometry.validate_target(
        requested_target,
        frame_id=resolved_geometry.frame_id,
        require_safety_margin=True,
    )
    if not validation.valid:
        reasons = "; ".join(str(reason) for reason in validation.reasons)
        raise ValueError(
            f"curved scenario `{scenario_key}` for `{lumen_type}` produced an invalid target: {reasons}"
        )
    validated_target = _readonly_vector3(validation.target, "validated_target")
    if not np.array_equal(validated_target, np.asarray(requested_target, dtype=np.float64)):
        raise ValueError("target validation changed the requested target coordinate")

    geometry_payload = _freeze_jsonable(lumen_geometry_fingerprint_payload(resolved_geometry))
    geometry_fingerprint = lumen_geometry_fingerprint(resolved_geometry)
    identity_payload = _scenario_identity_payload(
        scenario_id=scenario_key,
        curved_lumen_type=lumen_type,
        geometry_frame=resolved_geometry.frame_id,
        geometry_fingerprint=geometry_fingerprint,
        centerline_fraction=fraction,
        centerline_arc_length=sample.arc_length,
        centerline_segment_index=sample.segment_index,
        centerline_segment_parameter=sample.segment_parameter,
        radial_direction=radial_direction,
        radial_offset=radial_offset,
        local_radius=local_radius,
        preferred_radius=preferred_radius,
        boundary_guard=boundary_guard,
        derived_target=derived_target,
        override_used=target_override is not None,
        requested_target=requested_target,
        validated_target=validated_target,
    )
    scenario_fingerprint = _canonical_fingerprint(identity_payload)

    return CurvedLumenScenario(
        scenario_id=scenario_key,
        policy_version=CURVED_SCENARIO_POLICY_VERSION,
        curved_lumen_type=lumen_type,
        geometry_mode=GEOMETRY_MODE_CURVED,
        geometry_frame=resolved_geometry.frame_id,
        geometry_fingerprint=geometry_fingerprint,
        geometry_fingerprint_payload=geometry_payload,
        scenario_fingerprint=scenario_fingerprint,
        scenario_identity_payload=identity_payload,
        centerline_fraction=fraction,
        centerline_arc_length=sample.arc_length,
        normalized_centerline_fraction=fraction,
        centerline_segment_index=sample.segment_index,
        centerline_segment_parameter=sample.segment_parameter,
        centerline_point=sample.point,
        local_tangent=sample.tangent,
        radial_direction=radial_direction,
        radial_offset=radial_offset,
        local_radius=local_radius,
        preferred_radius=preferred_radius,
        boundary_guard=boundary_guard,
        derived_target=derived_target,
        requested_target=requested_target,
        validated_target=validated_target,
        override_used=target_override is not None,
        require_safety_margin=True,
        near_boundary=scenario_key == NEAR_SAFETY_BOUNDARY_TARGET,
        validation_reasons=tuple(validation.reasons),
    )


@dataclass(frozen=True)
class _CenterlineData:
    points: np.ndarray
    segment_vectors: np.ndarray
    segment_lengths: np.ndarray
    segment_unit_vectors: np.ndarray
    cumulative_arc_lengths: np.ndarray
    total_length: float


@dataclass(frozen=True)
class _CenterlineSample:
    fraction: float
    arc_length: float
    segment_index: int
    segment_parameter: float
    point: np.ndarray
    tangent: np.ndarray


def _scenario_id(value: str) -> str:
    scenario = str(value)
    if scenario not in CURVED_LUMEN_SCENARIO_IDS:
        raise ValueError(f"unsupported curved scenario `{scenario}`")
    return scenario


def _curved_lumen_type(config: Mapping[str, Any], override: str | None) -> str:
    if override is None or str(override) == "":
        section = config.get("curved_lumen", {})
        if not isinstance(section, Mapping):
            raise ValueError("curved_lumen must be a map")
        value = section.get("type")
    else:
        value = override
    lumen_type = str(value)
    if lumen_type not in CURVED_LUMEN_TYPES:
        raise ValueError(f"unsupported curved lumen type `{lumen_type}`")
    return lumen_type


def _centerline_data(geometry: CurvedLumen) -> _CenterlineData:
    points = _readonly_points(geometry.centerline_points, "centerline_points")
    segment_vectors = np.asarray(geometry.segment_vectors, dtype=np.float64)
    segment_lengths = np.asarray(geometry.segment_lengths, dtype=np.float64)
    unit_vectors = np.asarray(geometry.segment_unit_vectors, dtype=np.float64)
    cumulative = np.asarray(geometry.cumulative_arc_lengths, dtype=np.float64)
    if segment_vectors.shape != (points.shape[0] - 1, 3):
        raise ValueError("segment_vectors must have shape (M - 1, 3)")
    if segment_lengths.shape != (points.shape[0] - 1,):
        raise ValueError("segment_lengths must have one value per centerline segment")
    if unit_vectors.shape != segment_vectors.shape:
        raise ValueError("segment_unit_vectors must match segment_vectors shape")
    if cumulative.shape != (points.shape[0],):
        raise ValueError("cumulative_arc_lengths must have one value per centerline point")
    if not (
        np.all(np.isfinite(segment_vectors))
        and np.all(np.isfinite(segment_lengths))
        and np.all(np.isfinite(unit_vectors))
        and np.all(np.isfinite(cumulative))
    ):
        raise ValueError("centerline segment data must contain finite values")
    if not np.all(segment_lengths > 0.0):
        raise ValueError("centerline segment lengths must be positive")
    if not np.allclose(np.linalg.norm(unit_vectors, axis=1), 1.0, atol=1.0e-12, rtol=0.0):
        raise ValueError("centerline segment unit vectors must be unit length")
    if cumulative[0] != 0.0 or not np.all(np.diff(cumulative) > 0.0):
        raise ValueError("cumulative_arc_lengths must be strictly increasing from zero")
    total_length = float(cumulative[-1])
    if not math.isfinite(total_length) or total_length <= 0.0:
        raise ValueError("centerline total length must be positive and finite")
    if not math.isclose(float(getattr(geometry, "length")), total_length, rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError("geometry length must match cumulative arc length")
    return _CenterlineData(
        points=points,
        segment_vectors=segment_vectors.copy(),
        segment_lengths=segment_lengths.copy(),
        segment_unit_vectors=unit_vectors.copy(),
        cumulative_arc_lengths=cumulative.copy(),
        total_length=total_length,
    )


def _sample_centerline(centerline: _CenterlineData, fraction: float) -> _CenterlineSample:
    normalized = _fraction(fraction, "centerline_fraction")
    arc_length = normalized * centerline.total_length
    segment_count = centerline.segment_lengths.shape[0]
    exact_boundary = np.where(np.isclose(centerline.cumulative_arc_lengths, arc_length, atol=0.0, rtol=0.0))[0]
    if exact_boundary.size and int(exact_boundary[0]) > 0:
        boundary = int(exact_boundary[0])
        segment = min(boundary - 1, segment_count - 1)
        parameter = 1.0
    else:
        segment = int(np.searchsorted(centerline.cumulative_arc_lengths, arc_length, side="right") - 1)
        segment = int(np.clip(segment, 0, segment_count - 1))
        parameter = (arc_length - centerline.cumulative_arc_lengths[segment]) / centerline.segment_lengths[segment]
    parameter = float(np.clip(parameter, 0.0, 1.0))
    point = centerline.points[segment] + parameter * centerline.segment_vectors[segment]
    tangent = centerline.segment_unit_vectors[segment]
    return _CenterlineSample(
        fraction=normalized,
        arc_length=float(arc_length),
        segment_index=segment,
        segment_parameter=parameter,
        point=_readonly_vector3(point, "centerline_point"),
        tangent=_readonly_unit_vector(tangent, "local_tangent"),
    )


def _reference_normal(config: Mapping[str, Any], lumen_type: str) -> np.ndarray:
    section = config.get("curved_lumen", {})
    if not isinstance(section, Mapping):
        raise ValueError("curved_lumen must be a map")
    if lumen_type == "circular_arc":
        values = section.get("circular_arc", {})
        label = "curved_lumen.circular_arc.bend_normal"
        key = "bend_normal"
    else:
        values = section.get("s_curve", {})
        label = "curved_lumen.s_curve.bend_plane_normal"
        key = "bend_plane_normal"
    if not isinstance(values, Mapping):
        raise ValueError(f"{label.rsplit('.', 1)[0]} must be a map")
    return _readonly_unit_vector(values.get(key), label)


def _radial_direction(tangent: np.ndarray, reference_normal: np.ndarray) -> np.ndarray:
    tangent = _readonly_unit_vector(tangent, "local_tangent")
    reference = _readonly_unit_vector(reference_normal, "reference_normal")
    radial = reference - float(np.dot(reference, tangent)) * tangent
    norm = float(np.linalg.norm(radial))
    if norm <= _FALLBACK_TOLERANCE:
        bases = (
            np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
            np.asarray([0.0, 1.0, 0.0], dtype=np.float64),
            np.asarray([0.0, 0.0, 1.0], dtype=np.float64),
        )
        basis = min(bases, key=lambda item: (abs(float(np.dot(item, tangent))), int(np.argmax(item))))
        radial = basis - float(np.dot(basis, tangent)) * tangent
        norm = float(np.linalg.norm(radial))
    if norm <= _FALLBACK_TOLERANCE:
        raise ValueError("could not construct a deterministic radial direction")
    return _readonly_unit_vector(radial / norm, "radial_direction")


def _local_radius(geometry: CurvedLumen, segment_index: int, parameter: float) -> float:
    radii = np.asarray(geometry.radius_profile, dtype=np.float64)
    if radii.shape != (geometry.centerline_points.shape[0],):
        raise ValueError("radius_profile must have one value per centerline point")
    if not np.all(np.isfinite(radii)) or np.any(radii <= 0.0):
        raise ValueError("radius_profile values must be positive and finite")
    radius = (1.0 - parameter) * radii[segment_index] + parameter * radii[segment_index + 1]
    return _positive_float(radius, "local_radius")


def _preferred_radius(geometry: CurvedLumen, local_radius: float) -> float:
    radius = _positive_float(local_radius, "local_radius")
    outer = _nonnegative_float(geometry.ctr_outer_radius, "ctr_outer_radius")
    margin = _nonnegative_float(geometry.safety_margin, "safety_margin")
    preferred = radius - outer - margin
    if not math.isfinite(preferred) or preferred <= 0.0:
        raise ValueError("local lumen radius must exceed ctr_outer_radius plus safety_margin")
    return float(preferred)


def _target_override(value: Any) -> np.ndarray:
    return _readonly_vector3(value, "target_override")


def _scenario_identity_payload(**values: Any) -> tuple[tuple[str, Any], ...]:
    payload = {
        "policy_version": CURVED_SCENARIO_POLICY_VERSION,
        **values,
    }
    return _freeze_jsonable(payload)


def _canonical_fingerprint(payload: tuple[tuple[str, Any], ...]) -> str:
    encoded = json.dumps(_jsonable(payload), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _freeze_jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return tuple((str(key), _freeze_jsonable(item)) for key, item in sorted(value.items(), key=lambda pair: str(pair[0])))
    if isinstance(value, np.ndarray):
        return _freeze_jsonable(value.tolist())
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_jsonable(item) for item in value)
    if isinstance(value, np.generic):
        return _freeze_jsonable(value.item())
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("identity payload contains a non-finite float")
        return float(value)
    if isinstance(value, int):
        return int(value)
    if value is None or isinstance(value, str):
        return value
    raise ValueError(f"identity payload contains unsupported value type {type(value).__name__}")


def _jsonable(value: Any) -> Any:
    if isinstance(value, tuple):
        if all(isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], str) for item in value):
            return {key: _jsonable(item) for key, item in value}
        return [_jsonable(item) for item in value]
    return value


def _readonly_points(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{label} must have shape (N, 3)")
    if array.shape[0] < 2:
        raise ValueError(f"{label} must contain at least two points")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    result = array.copy()
    result.setflags(write=False)
    return result


def _readonly_vector3(values: Any, label: str) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain 3 finite values") from exc
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    result = array.copy()
    result.setflags(write=False)
    return result


def _readonly_unit_vector(values: Any, label: str) -> np.ndarray:
    vector = _readonly_vector3(values, label)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{label} must be non-zero")
    unit = np.asarray(vector / norm, dtype=np.float64)
    unit.setflags(write=False)
    return unit


def _fraction(value: Any, label: str) -> float:
    numeric = _finite_float(value, label)
    if numeric < 0.0 or numeric > 1.0:
        raise ValueError(f"{label} must be in [0, 1]")
    return numeric


def _positive_float(value: Any, label: str) -> float:
    numeric = _finite_float(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _nonnegative_float(value: Any, label: str) -> float:
    numeric = _finite_float(value, label)
    if numeric < 0.0:
        raise ValueError(f"{label} must be non-negative")
    return numeric


def _finite_float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric
