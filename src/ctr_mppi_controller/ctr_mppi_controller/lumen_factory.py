"""ROS-independent lumen geometry construction from project configuration."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from collections.abc import Mapping
from typing import Any

import numpy as np

from .curved_lumen import CurvedLumen, circular_arc_centerline, s_curve_centerline
from .cylindrical_lumen import CylindricalLumen
from .lumen_geometry import LumenCostWeights, LumenGeometry


LUMEN_MODES = ("none", "cylindrical", "curved")
CURVED_LUMEN_TYPES = ("circular_arc", "s_curve")


def cylindrical_lumen_enabled(config: Mapping[str, Any]) -> bool:
    section = config.get("cylindrical_lumen", {})
    if not isinstance(section, Mapping):
        raise ValueError("cylindrical_lumen must be a map")
    if "enabled" not in section:
        return False
    return _require_bool(section["enabled"], "cylindrical_lumen.enabled")


def curved_lumen_enabled(config: Mapping[str, Any]) -> bool:
    section = config.get("curved_lumen", {})
    if not isinstance(section, Mapping):
        raise ValueError("curved_lumen must be a map")
    if "enabled" not in section:
        return False
    return _require_bool(section["enabled"], "curved_lumen.enabled")


def lumen_mode_from_config(config: Mapping[str, Any]) -> str:
    cylindrical_enabled = cylindrical_lumen_enabled(config)
    curved_enabled = curved_lumen_enabled(config)
    if cylindrical_enabled and curved_enabled:
        raise ValueError("exactly one lumen geometry mode may be enabled")
    if cylindrical_enabled:
        return "cylindrical"
    if curved_enabled:
        return "curved"
    return "none"


def lumen_geometry_from_config(config: Mapping[str, Any]) -> LumenGeometry | None:
    mode = lumen_mode_from_config(config)
    if mode == "none":
        return None
    if mode == "cylindrical":
        return CylindricalLumen.from_config(dict(config))
    return _curved_lumen_from_config(config)


def lumen_cost_weights_from_config(config: Mapping[str, Any]) -> LumenCostWeights | None:
    if lumen_mode_from_config(config) == "none":
        return None
    return LumenCostWeights.from_config(dict(config))


def config_with_mppi_profile(config: Mapping[str, Any], profile_name: str | None) -> dict[str, Any]:
    if profile_name is None or str(profile_name) == "":
        return deepcopy(dict(config))
    name = str(profile_name)
    profiles = config.get("mppi_profiles", {})
    if not isinstance(profiles, Mapping) or name not in profiles:
        raise ValueError(f"unknown MPPI profile `{name}`")
    profile = profiles[name]
    if not isinstance(profile, Mapping):
        raise ValueError(f"mppi_profiles.{name} must be a map")
    result = deepcopy(dict(config))
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


def config_with_lumen_overrides(
    config: Mapping[str, Any],
    *,
    enable_cylindrical_lumen: bool | None = None,
    enable_curved_lumen: bool | None = None,
    curved_lumen_type: str | None = None,
    cylinder_profile: str | None = None,
    target: Any | None = None,
    random_seed: Any | None = None,
) -> dict[str, Any]:
    result = config_with_mppi_profile(config, cylinder_profile)
    if enable_cylindrical_lumen is not None:
        result.setdefault("cylindrical_lumen", {})["enabled"] = _require_bool(
            enable_cylindrical_lumen,
            "enable_cylindrical_lumen",
        )
    if enable_curved_lumen is not None:
        result.setdefault("curved_lumen", {})["enabled"] = _require_bool(
            enable_curved_lumen,
            "enable_curved_lumen",
        )
    if curved_lumen_type is not None and str(curved_lumen_type) != "":
        result.setdefault("curved_lumen", {})["type"] = str(curved_lumen_type)
    if target is not None:
        result.setdefault("goal", {})["position"] = [float(value) for value in _vector3(target, "target")]
    if random_seed is not None and str(random_seed) != "":
        seed = _nonnegative_int(random_seed, "mppi.random_seed")
        result.setdefault("mppi", {})["random_seed"] = seed
    # Validate only mode selection here; full schema validation remains in ctr_bringup.
    lumen_mode_from_config(result)
    return result


def lumen_geometry_fingerprint(config_or_geometry: Mapping[str, Any] | LumenGeometry | None) -> str:
    payload = lumen_geometry_fingerprint_payload(config_or_geometry)
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def lumen_geometry_log_line(config: Mapping[str, Any], *, role: str) -> str:
    role_value = str(role)
    if role_value == "":
        raise ValueError("lumen geometry log role must be non-empty")
    mode = lumen_mode_from_config(config)
    geometry = lumen_geometry_from_config(config)
    lumen_type = "none"
    frame = _default_frame_id(config)
    point_count = 0
    if geometry is not None:
        frame = geometry.frame_id
        if mode == "curved":
            lumen_type = str(config.get("curved_lumen", {}).get("type", "unknown"))
            point_count = int(getattr(geometry, "centerline_points", np.empty((0, 3))).shape[0])
    return (
        "LUMEN_GEOMETRY "
        f"role={role_value} "
        f"mode={mode} "
        f"type={lumen_type} "
        f"frame={frame} "
        f"points={point_count} "
        f"fingerprint={lumen_geometry_fingerprint(config)}"
    )


def lumen_geometry_fingerprint_payload(config_or_geometry: Mapping[str, Any] | LumenGeometry | None) -> dict[str, Any]:
    if config_or_geometry is None:
        return {"mode": "none"}
    if isinstance(config_or_geometry, Mapping):
        mode = lumen_mode_from_config(config_or_geometry)
        geometry = lumen_geometry_from_config(config_or_geometry)
        curved_type = None
        if mode == "curved":
            curved_type = str(config_or_geometry.get("curved_lumen", {}).get("type"))
        return _payload_from_geometry(geometry, mode=mode, curved_type=curved_type)
    geometry = config_or_geometry
    if isinstance(geometry, CylindricalLumen):
        return _payload_from_geometry(geometry, mode="cylindrical")
    if isinstance(geometry, CurvedLumen):
        return _payload_from_geometry(geometry, mode="curved", curved_type="sampled_centerline")
    raise ValueError("lumen geometry fingerprint input must be a config mapping, LumenGeometry object, or None")


def _curved_lumen_from_config(config: Mapping[str, Any]) -> CurvedLumen:
    values = config.get("curved_lumen", {})
    if not isinstance(values, Mapping):
        raise ValueError("curved_lumen must be a map")
    lumen_type = values.get("type")
    sample_spacing = values["centerline_sample_spacing"]
    if lumen_type == "circular_arc":
        arc = values.get("circular_arc", {})
        if not isinstance(arc, Mapping):
            raise ValueError("curved_lumen.circular_arc must be a map")
        centerline = circular_arc_centerline(
            inlet_position=arc["inlet_position"],
            initial_tangent=arc["initial_tangent"],
            bend_normal=arc["bend_normal"],
            curvature_radius=arc["curvature_radius"],
            arc_angle=arc["arc_angle"],
            sample_spacing=sample_spacing,
        )
    elif lumen_type == "s_curve":
        s_curve = values.get("s_curve", {})
        if not isinstance(s_curve, Mapping):
            raise ValueError("curved_lumen.s_curve must be a map")
        centerline = s_curve_centerline(
            inlet_position=s_curve["inlet_position"],
            initial_tangent=s_curve["initial_tangent"],
            bend_plane_normal=s_curve["bend_plane_normal"],
            total_length=s_curve["total_length"],
            lateral_amplitude=s_curve["lateral_amplitude"],
            sample_spacing=sample_spacing,
        )
    else:
        raise ValueError("curved_lumen.type must be `circular_arc` or `s_curve`")
    return CurvedLumen(
        frame_id=values["frame_id"],
        centerline_points=centerline,
        lumen_radius=values["lumen_radius"],
        ctr_outer_radius=values["ctr_outer_radius"],
        safety_margin=values["safety_margin"],
    )


def _payload_from_geometry(
    geometry: LumenGeometry | None,
    *,
    mode: str,
    curved_type: str | None = None,
) -> dict[str, Any]:
    if mode not in LUMEN_MODES:
        raise ValueError(f"unsupported lumen mode `{mode}`")
    if mode == "none":
        return {"mode": "none"}
    if geometry is None:
        raise ValueError("lumen geometry is required for enabled modes")
    if mode == "cylindrical":
        if not isinstance(geometry, CylindricalLumen):
            raise ValueError("cylindrical fingerprint requires CylindricalLumen")
        return {
            "mode": "cylindrical",
            "frame_id": geometry.frame_id,
            "axis_origin": _array_payload(geometry.axis_origin, "cylindrical_lumen.axis_origin"),
            "axis_direction": _array_payload(geometry.axis_direction, "cylindrical_lumen.axis_direction"),
            "length": _float_payload(geometry.length, "cylindrical_lumen.length"),
            "lumen_radius": _float_payload(geometry.radius, "cylindrical_lumen.radius"),
            "ctr_outer_radius": _float_payload(geometry.ctr_outer_radius, "cylindrical_lumen.ctr_outer_radius"),
            "safety_margin": _float_payload(geometry.safety_margin, "cylindrical_lumen.safety_margin"),
        }
    if not isinstance(geometry, CurvedLumen):
        raise ValueError("curved fingerprint requires CurvedLumen")
    return {
        "mode": "curved",
        "curved_type": str(curved_type or "unknown"),
        "frame_id": geometry.frame_id,
        "centerline_points": _array_payload(geometry.centerline_points, "curved_lumen.centerline_points"),
        "lumen_radius": _array_payload(geometry.radius_profile, "curved_lumen.lumen_radius"),
        "ctr_outer_radius": _float_payload(geometry.ctr_outer_radius, "curved_lumen.ctr_outer_radius"),
        "safety_margin": _float_payload(geometry.safety_margin, "curved_lumen.safety_margin"),
    }


def _array_payload(values: Any, label: str) -> list[Any]:
    array = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} contains non-finite values")
    return array.tolist()


def _default_frame_id(config: Mapping[str, Any]) -> str:
    frame = config.get("robot", {}).get("frames", {}).get("base", "unknown")
    return str(frame) if frame is not None else "unknown"


def _float_payload(value: Any, label: str) -> float:
    numeric = float(np.float64(value))
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _vector3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array.copy()


def _require_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be a bool, got {type(value).__name__}")
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
