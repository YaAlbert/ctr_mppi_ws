"""Project YAML loading and validation for Milestone 1.

This module validates configuration shape and conservative placeholder
defaults. It does not tune parameters or implement model/controller logic.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Iterable

import math
import yaml

from ctr_bringup.slice_7g_profile import (
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
    validate_slice_7g_profile,
)


UNRESOLVED_TODO_IDS = {
    "CTR-001": "Tube lengths require CAD or measurement.",
    "CTR-002": "Tube outer diameters require datasheet or measurement.",
    "CTR-003": "Tube inner diameters require datasheet or measurement.",
    "CTR-004": "Tube precurvature requires CAD or calibration.",
    "CTR-005": "Tube precurved lengths require CAD or measurement.",
    "MODEL-004": "Python CTR model implementation is unresolved.",
    "MODEL-006": "YAML and MATLAB tube parameter source of truth is unresolved.",
    "ROS-001": "ROS2 skeleton must be built and discovered by colcon.",
    "ROS-002": "Custom interfaces must be generated and tested.",
    "SNS-001": "Tactile sensor model is unresolved.",
    "SNS-002": "Tactile output type is unresolved.",
    "SIM-001": "Simulation timing must be benchmarked in ROS2 runtime.",
    "HW-001": "Motor model is unresolved.",
    "HW-002": "Driver model is unresolved.",
    "HW-003": "Hardware communication protocol is unresolved.",
    "SAFE-001": "Maximum insertion speed requires hardware testing.",
    "SAFE-002": "Maximum rotation speed requires hardware testing.",
}


class ParameterValidationError(ValueError):
    """Raised when one or more project parameter files fail validation."""


REFERENCE_MODES = ("fixed_target", "trajectory", "external_target")
REFERENCE_TRAJECTORY_TYPES = ("circle", "ellipse", "helix")


def validate_config_paths(
    paths: Iterable[str | Path] | None,
    *,
    required: bool = True,
    label: str = "config_paths",
) -> list[str]:
    """Validate ROS-provided project YAML paths and preserve their order."""

    if paths is None:
        path_values: list[str | Path] = []
    elif isinstance(paths, (str, bytes)):
        raise ParameterValidationError(f"`{label}` must be a string array, not a scalar string.")
    else:
        path_values = list(paths)

    if required and not path_values:
        raise ParameterValidationError(f"`{label}` must contain at least one YAML path.")

    validated: list[str] = []
    seen: set[str] = set()
    for index, value in enumerate(path_values):
        if not isinstance(value, (str, Path)):
            raise ParameterValidationError(f"`{label}[{index}]` must be a string path.")
        path = Path(value)
        if not path.exists():
            raise FileNotFoundError(f"Parameter file not found: {path}")
        if not path.is_file():
            raise ParameterValidationError(f"`{label}[{index}]` is not a file: {path}")
        resolved = str(path.resolve())
        if resolved in seen:
            raise ParameterValidationError(f"Duplicate parameter path in `{label}`: {resolved}")
        seen.add(resolved)
        validated.append(resolved)

    return validated


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    """Load one project YAML file and return a dictionary."""

    yaml_path = Path(path)
    if not yaml_path.exists():
        raise FileNotFoundError(f"Parameter file not found: {yaml_path}")

    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise ParameterValidationError(f"Parameter file must contain a map: {yaml_path}")

    return data


def load_parameter_files(paths: Iterable[str | Path]) -> dict[str, Any]:
    """Load and shallow-merge the configured project YAML files."""

    merged: dict[str, Any] = {}
    for path in paths:
        data = load_yaml_file(path)
        for key, value in data.items():
            if key in merged:
                raise ParameterValidationError(f"Duplicate parameter section: {key}")
            merged[key] = value
    return merged


def validate_or_raise(config: dict[str, Any]) -> None:
    """Raise when project configuration fails Milestone 1 validation."""

    errors = validate_project_config(config)
    if errors:
        joined = "\n".join(f"- {error}" for error in errors)
        raise ParameterValidationError(f"Invalid project parameters:\n{joined}")


def validate_project_config(config: dict[str, Any]) -> list[str]:
    """Return validation errors for the full project configuration."""

    errors: list[str] = []
    reference_mode = _reference_mode_for_validation(config)
    for section in (
        "robot",
        "model",
        "mppi",
        "reference",
        "tracking_metrics",
        "hardware",
        "safety",
        "simulation",
        "tactile",
    ):
        if section not in config:
            errors.append(f"Missing required section `{section}`.")

    if "robot" in config:
        errors.extend(_validate_robot(config["robot"]))
    if "model" in config:
        errors.extend(_validate_model(config["model"]))
    if "mppi" in config:
        errors.extend(_validate_mppi(config["mppi"]))
        errors.extend(_validate_mppi_tactile(config["mppi"], config.get("tactile", {})))
    if "mppi_profiles" in config:
        errors.extend(_validate_mppi_profiles(config["mppi_profiles"]))
    if "cylindrical_lumen" in config:
        errors.extend(_validate_cylindrical_lumen(config["cylindrical_lumen"]))
    if "curved_lumen" in config:
        errors.extend(_validate_curved_lumen(config["curved_lumen"]))
    errors.extend(_validate_lumen_mode_and_frames(config))
    if "cylindrical_lumen_cost" in config:
        errors.extend(_validate_cylindrical_lumen_cost(config["cylindrical_lumen_cost"]))
    if "goal" in config:
        errors.extend(_validate_goal(config["goal"], reference_mode=reference_mode))
    if "reference" in config:
        errors.extend(_validate_reference(config["reference"]))
    if "tracking_metrics" in config:
        errors.extend(_validate_tracking_metrics(config["tracking_metrics"]))
    if "evaluation" in config:
        errors.extend(_validate_evaluation(config["evaluation"]))
    if "hardware" in config:
        errors.extend(_validate_hardware(config["hardware"]))
    if "safety" in config:
        errors.extend(_validate_safety(config["safety"]))
    if "simulation" in config:
        errors.extend(_validate_simulation(config["simulation"]))
    if "tactile" in config:
        errors.extend(_validate_tactile(config["tactile"]))
    if "slice_7g_runtime" in config:
        try:
            validate_slice_7g_profile(config)
        except ValueError as exc:
            errors.append(str(exc))

    return errors


def project_config_with_overrides(
    config: dict[str, Any],
    *,
    runtime_mode: str | None = None,
    hardware_implementation: str | None = None,
    reference_mode: Any | None = None,
    reference_type: Any | None = None,
    enable_cylindrical_lumen: Any | None = None,
    enable_curved_lumen: Any | None = None,
    curved_lumen_type: Any | None = None,
    cylinder_target_position: Any | None = None,
    mppi_profile: Any | None = None,
    mppi_random_seed: Any | None = None,
    slice_7g_profile: Any | None = None,
    development_simulation: Any | None = None,
) -> dict[str, Any]:
    """Return a copy of config with launch-mode overrides applied."""

    result = deepcopy(config)
    if runtime_mode is not None:
        result.setdefault("runtime", {})["mode"] = runtime_mode
    if hardware_implementation is not None:
        result.setdefault("hardware", {})["implementation"] = hardware_implementation
    if reference_mode is not None and reference_mode != "":
        result.setdefault("reference", {})["mode"] = _choice(reference_mode, "reference_mode", REFERENCE_MODES)
    if reference_type is not None and reference_type != "":
        result.setdefault("reference", {})["trajectory_type"] = _choice(
            reference_type,
            "reference_type",
            REFERENCE_TRAJECTORY_TYPES,
        )
    if enable_cylindrical_lumen is not None and enable_cylindrical_lumen != "":
        result.setdefault("cylindrical_lumen", {})["enabled"] = parse_launch_bool(
            enable_cylindrical_lumen,
            "enable_cylindrical_lumen",
        )
    if enable_curved_lumen is not None and enable_curved_lumen != "":
        result.setdefault("curved_lumen", {})["enabled"] = parse_launch_bool(
            enable_curved_lumen,
            "enable_curved_lumen",
        )
    if curved_lumen_type is not None and curved_lumen_type != "":
        result.setdefault("curved_lumen", {})["type"] = _choice(
            curved_lumen_type,
            "curved_lumen_type",
            ("circular_arc", "s_curve"),
        )
    if mppi_profile is not None and mppi_profile != "":
        _apply_mppi_profile(result, mppi_profile)
    if (
        cylinder_target_position is not None
        and cylinder_target_position != ""
        and _reference_mode_for_validation(result) == "fixed_target"
    ):
        if not (isinstance(cylinder_target_position, (list, tuple)) and len(cylinder_target_position) == 0):
            result.setdefault("goal", {})["position"] = deepcopy(cylinder_target_position)
    if mppi_random_seed is not None and mppi_random_seed != "":
        seed = _optional_seed(mppi_random_seed, "mppi_random_seed")
        if seed is not None:
            result.setdefault("mppi", {})["random_seed"] = seed
    if slice_7g_profile is not None and slice_7g_profile != "":
        result = apply_slice_7g_simulation_profile(
            result,
            enabled=parse_launch_bool(slice_7g_profile, "slice_7g_profile"),
        )
    if development_simulation is not None and development_simulation != "":
        development_enabled = parse_launch_bool(
            development_simulation, "development_simulation"
        )
        if development_enabled:
            result = apply_slice_7g_development_simulation_profile(result, enabled=True)
    return result


def _apply_mppi_profile(config: dict[str, Any], profile_name: Any) -> None:
    name = _non_empty_string(profile_name, "cylinder_profile")
    profiles = config.get("mppi_profiles", {})
    if not isinstance(profiles, dict) or name not in profiles:
        raise ParameterValidationError(f"`cylinder_profile` must name an existing MPPI profile.")
    profile = profiles[name]
    if not isinstance(profile, dict):
        raise ParameterValidationError(f"`mppi_profiles.{name}` must be a map.")
    mppi = config.setdefault("mppi", {})
    mppi["num_samples"] = _positive_int(profile.get("samples"), f"mppi_profiles.{name}.samples")
    mppi["horizon"] = _positive_int(profile.get("horizon"), f"mppi_profiles.{name}.horizon")
    mppi["dt"] = _positive_number_value(profile.get("dt"), f"mppi_profiles.{name}.dt")
    if "lambda" in profile:
        mppi["lambda"] = _positive_number_value(profile["lambda"], f"mppi_profiles.{name}.lambda")
    if "noise_std" in profile:
        mppi["noise_std"] = deepcopy(profile["noise_std"])
    if "weights" in profile:
        if not isinstance(profile["weights"], dict):
            raise ParameterValidationError(f"`mppi_profiles.{name}.weights` must be a map.")
        mppi.setdefault("weights", {}).update(deepcopy(profile["weights"]))
    if "control_frequency" in profile:
        mppi["control_frequency"] = _positive_number_value(
            profile["control_frequency"],
            f"mppi_profiles.{name}.control_frequency",
        )
    else:
        period = _positive_number_value(profile.get("control_period"), f"mppi_profiles.{name}.control_period")
        mppi["control_frequency"] = 1.0 / period
    mppi["active_profile"] = name


def _choice(value: Any, label: str, allowed: tuple[str, ...]) -> str:
    if not isinstance(value, str) or value not in allowed:
        raise ParameterValidationError(f"`{label}` must be one of {allowed}.")
    return value


def _non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ParameterValidationError(f"`{label}` must be a non-empty string.")
    return value


def _vector3(values: Any, label: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ParameterValidationError(f"`{label}` must contain exactly three finite numeric values.")
    result = []
    for value in values:
        numeric = _finite_number(value, label)
        result.append(float(numeric))
    return result


def _optional_seed(value: Any, label: str) -> int | None:
    numeric = _finite_number(value, label)
    if int(numeric) != numeric:
        raise ParameterValidationError(f"`{label}` must be an integer.")
    seed = int(numeric)
    return None if seed < 0 else seed


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_number_value(value, label)
    if int(numeric) != numeric:
        raise ParameterValidationError(f"`{label}` must be an integer.")
    return int(numeric)


def _positive_number_value(value: Any, label: str) -> float:
    numeric = _finite_number(value, label)
    if numeric <= 0:
        raise ParameterValidationError(f"`{label}` must be positive.")
    return float(numeric)


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ParameterValidationError(f"`{label}` must be numeric, not boolean.")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ParameterValidationError(f"`{label}` must be finite.") from exc
    if not math.isfinite(numeric):
        raise ParameterValidationError(f"`{label}` must be finite.")
    return numeric


def parse_launch_bool(value: Any, label: str) -> bool:
    """Parse launch-provided booleans without Python truthiness coercion."""

    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
    raise ParameterValidationError(f"`{label}` must be a launch boolean string `true` or `false`.")


def _reference_mode_for_validation(config: dict[str, Any]) -> str:
    reference = config.get("reference")
    if isinstance(reference, dict) and reference.get("mode") in REFERENCE_MODES:
        return str(reference["mode"])
    return "fixed_target"


def _validate_robot(robot: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(robot, dict):
        return ["`robot` must be a map."]

    if robot.get("number_of_tubes") != 3:
        errors.append("`robot.number_of_tubes` must be 3 for the documented CTR.")

    frames = robot.get("frames", {})
    for key in ("world", "base", "tip", "tactile"):
        if not isinstance(frames.get(key), str) or not frames[key]:
            errors.append(f"`robot.frames.{key}` must be a non-empty string.")

    initial = robot.get("initial_configuration", {})
    errors.extend(_require_numeric_list(initial, "insertion", 3, "robot.initial_configuration"))
    errors.extend(_require_numeric_list(initial, "rotation", 3, "robot.initial_configuration"))

    tube = robot.get("tube", {})
    for key in ("length", "outer_diameter", "inner_diameter", "precurvature", "precurved_length"):
        errors.extend(_require_numeric_list(tube, key, 3, "robot.tube", positive=True))

    material = robot.get("material", {})
    errors.extend(_require_numeric_list(material, "young_modulus", 3, "robot.material", positive=True))
    errors.extend(_require_numeric_list(material, "shear_modulus", 3, "robot.material", positive=True))

    limits = robot.get("limits", {})
    for key in (
        "insertion_min",
        "insertion_max",
        "rotation_min",
        "rotation_max",
        "insertion_velocity_max",
        "rotation_velocity_max",
        "insertion_acceleration_max",
        "rotation_acceleration_max",
    ):
        errors.extend(_require_numeric_list(limits, key, 3, "robot.limits"))

    errors.extend(_validate_bounds(limits, "insertion_min", "insertion_max", "robot.limits"))
    errors.extend(_validate_bounds(limits, "rotation_min", "rotation_max", "robot.limits"))
    errors.extend(_require_positive_list(limits, "insertion_velocity_max", "robot.limits"))
    errors.extend(_require_positive_list(limits, "rotation_velocity_max", "robot.limits"))
    errors.extend(_require_positive_list(limits, "insertion_acceleration_max", "robot.limits"))
    errors.extend(_require_positive_list(limits, "rotation_acceleration_max", "robot.limits"))
    return errors


def _validate_model(model: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(model, dict):
        return ["`model` must be a map."]
    if model.get("implementation") not in {"approximate", "cosserat", "lookup_table", "learned_residual"}:
        errors.append("`model.implementation` must name a documented model type.")
    errors.extend(_require_positive_number(model, "backbone_points", "model", integer=True))
    errors.extend(_require_positive_number(model, "integration_step", "model"))
    return errors


def _validate_mppi(mppi: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(mppi, dict):
        return ["`mppi` must be a map."]
    errors.extend(_require_positive_number(mppi, "control_frequency", "mppi"))
    errors.extend(_require_positive_number(mppi, "dt", "mppi"))
    errors.extend(_require_positive_number(mppi, "horizon", "mppi", integer=True))
    errors.extend(_require_positive_number(mppi, "num_samples", "mppi", integer=True))
    errors.extend(_require_positive_number(mppi, "lambda", "mppi"))
    if mppi.get("command_type") != "joint_velocity":
        errors.append("`mppi.command_type` must be `joint_velocity` for Milestone 1.")
    noise = mppi.get("noise_std", {})
    errors.extend(_require_numeric_list(noise, "insertion", 3, "mppi.noise_std", nonnegative=True))
    errors.extend(_require_numeric_list(noise, "rotation", 3, "mppi.noise_std", nonnegative=True))
    weights = mppi.get("weights", {})
    for key in ("tip", "shape", "control", "smoothness", "obstacle", "terminal", "force", "joint_limit", "stability"):
        errors.extend(_require_number(weights, key, "mppi.weights", nonnegative=True))
    return errors


def _validate_mppi_tactile(mppi: Any, tactile: Any) -> list[str]:
    """Validate the disabled-by-default MPPI tactile integration contract."""

    errors: list[str] = []
    values = mppi.get("tactile", {}) if isinstance(mppi, dict) else {}
    if not isinstance(values, dict):
        return ["`mppi.tactile` must be a map."]
    if not isinstance(values.get("enabled"), bool):
        errors.append("`mppi.tactile.enabled` must be a boolean.")
    for key in ("max_age_s", "force_saturation_n", "proximity_margin_m"):
        errors.extend(_require_positive_number(values, key, "mppi.tactile"))
    for key in ("no_contact_multiplier", "contact_multiplier", "warning_multiplier", "stop_multiplier"):
        errors.extend(_require_number(values, key, "mppi.tactile", nonnegative=True))
    if all(key in values for key in ("no_contact_multiplier", "contact_multiplier", "warning_multiplier", "stop_multiplier")):
        multipliers = [values[key] for key in ("no_contact_multiplier", "contact_multiplier", "warning_multiplier", "stop_multiplier")]
        if multipliers[0] != 0.0:
            errors.append("`mppi.tactile.no_contact_multiplier` must equal zero.")
        if any(left > right for left, right in zip(multipliers, multipliers[1:])):
            errors.append("`mppi.tactile` multipliers must be nondecreasing.")
    weights = mppi.get("weights", {}) if isinstance(mppi, dict) else {}
    force_weight = _as_finite_number(weights.get("force")) if isinstance(weights, dict) else None
    if values.get("enabled") is True and (force_weight is None or force_weight <= 0.0):
        errors.append("enabled `mppi.tactile` requires a positive `mppi.weights.force`.")
    return errors


def _validate_mppi_profiles(profiles: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(profiles, dict):
        return ["`mppi_profiles` must be a map."]
    for name, profile in profiles.items():
        label = f"mppi_profiles.{name}"
        if not isinstance(name, str) or not name:
            errors.append("`mppi_profiles` keys must be non-empty strings.")
            continue
        if not isinstance(profile, dict):
            errors.append(f"`{label}` must be a map.")
            continue
        errors.extend(_require_positive_number(profile, "samples", label, integer=True))
        errors.extend(_require_positive_number(profile, "horizon", label, integer=True))
        errors.extend(_require_positive_number(profile, "dt", label))
        if "control_frequency" in profile:
            errors.extend(_require_positive_number(profile, "control_frequency", label))
        else:
            errors.extend(_require_positive_number(profile, "control_period", label))
        if "simulation_default" in profile and not isinstance(profile["simulation_default"], bool):
            errors.append(f"`{label}.simulation_default` must be a boolean when present.")
        if "lambda" in profile:
            errors.extend(_require_positive_number(profile, "lambda", label))
        if "weights" in profile:
            weights = profile["weights"]
            if not isinstance(weights, dict):
                errors.append(f"`{label}.weights` must be a map.")
            else:
                for weight_name in ("tip", "shape", "control", "smoothness", "obstacle", "terminal", "force", "joint_limit", "stability"):
                    if weight_name in weights:
                        errors.extend(_require_number(weights, weight_name, f"{label}.weights", nonnegative=True))
        if "noise_std" in profile:
            noise = profile["noise_std"]
            if not isinstance(noise, dict):
                errors.append(f"`{label}.noise_std` must be a map.")
            else:
                for key in ("insertion", "rotation"):
                    if key in noise:
                        errors.extend(_require_numeric_list(noise, key, 3, f"{label}.noise_std", positive=True))
    return errors


def _validate_cylindrical_lumen(lumen: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(lumen, dict):
        return ["`cylindrical_lumen` must be a map."]
    if not isinstance(lumen.get("enabled"), bool):
        errors.append("`cylindrical_lumen.enabled` must be a boolean.")
    if "simulation_default" in lumen and not isinstance(lumen["simulation_default"], bool):
        errors.append("`cylindrical_lumen.simulation_default` must be a boolean when present.")
    if not isinstance(lumen.get("frame_id"), str) or not lumen["frame_id"]:
        errors.append("`cylindrical_lumen.frame_id` must be a non-empty string.")
    errors.extend(_require_numeric_list(lumen, "axis_origin", 3, "cylindrical_lumen"))
    errors.extend(_require_numeric_list(lumen, "axis_direction", 3, "cylindrical_lumen"))
    errors.extend(_require_positive_number(lumen, "radius", "cylindrical_lumen"))
    errors.extend(_require_positive_number(lumen, "length", "cylindrical_lumen"))
    errors.extend(_require_positive_number(lumen, "ctr_outer_radius", "cylindrical_lumen"))
    errors.extend(_require_positive_number(lumen, "safety_margin", "cylindrical_lumen"))
    direction = _as_finite_vector(lumen.get("axis_direction"), 3)
    if direction is not None and math.sqrt(sum(value * value for value in direction)) <= 0.0:
        errors.append("`cylindrical_lumen.axis_direction` must be non-zero.")
    radius = _as_finite_number(lumen.get("radius"))
    outer = _as_finite_number(lumen.get("ctr_outer_radius"))
    margin = _as_finite_number(lumen.get("safety_margin"))
    if radius is not None and outer is not None and radius <= outer:
        errors.append("`cylindrical_lumen.radius` must exceed `cylindrical_lumen.ctr_outer_radius`.")
    if radius is not None and outer is not None and margin is not None and radius - outer <= margin:
        errors.append("`cylindrical_lumen` usable radius must exceed `safety_margin`.")
    return errors


def _validate_curved_lumen(lumen: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(lumen, dict):
        return ["`curved_lumen` must be a map."]
    enabled = lumen.get("enabled")
    enabled_valid = isinstance(enabled, bool)
    enabled_required = enabled is True
    if not enabled_valid:
        errors.append("`curved_lumen.enabled` must be a boolean.")
    if "simulation_default" in lumen and not isinstance(lumen["simulation_default"], bool):
        errors.append("`curved_lumen.simulation_default` must be a boolean when present.")
    lumen_type = lumen.get("type")
    if enabled_required or "type" in lumen:
        if not isinstance(lumen_type, str):
            errors.append("`curved_lumen.type` must be a string.")
        elif lumen_type not in {"circular_arc", "s_curve"}:
            errors.append("`curved_lumen.type` must be `circular_arc` or `s_curve`.")
    if enabled_required or "frame_id" in lumen:
        if not isinstance(lumen.get("frame_id"), str) or not lumen["frame_id"]:
            errors.append("`curved_lumen.frame_id` must be a non-empty string.")

    errors.extend(_require_positive_number_if_present(lumen, "lumen_radius", "curved_lumen", required=enabled_required))
    errors.extend(_require_number_if_present(lumen, "ctr_outer_radius", "curved_lumen", required=enabled_required, nonnegative=True))
    errors.extend(_require_number_if_present(lumen, "safety_margin", "curved_lumen", required=enabled_required, nonnegative=True))
    errors.extend(
        _require_positive_number_if_present(
            lumen,
            "centerline_sample_spacing",
            "curved_lumen",
            required=enabled_required,
        )
    )

    radius = _as_finite_number(lumen.get("lumen_radius")) if "lumen_radius" in lumen else None
    outer = _as_finite_number(lumen.get("ctr_outer_radius")) if "ctr_outer_radius" in lumen else None
    margin = _as_finite_number(lumen.get("safety_margin")) if "safety_margin" in lumen else None
    if radius is not None and outer is not None and radius - outer <= 0.0:
        errors.append("`curved_lumen.lumen_radius` must exceed `curved_lumen.ctr_outer_radius`.")
    if radius is not None and outer is not None and margin is not None and radius - outer - margin <= 0.0:
        errors.append("`curved_lumen` usable radius after safety margin must be positive.")

    if (enabled_required and lumen_type == "circular_arc") or "circular_arc" in lumen:
        errors.extend(_validate_curved_lumen_circular_arc(lumen.get("circular_arc", {})))
    if (enabled_required and lumen_type == "s_curve") or "s_curve" in lumen:
        errors.extend(_validate_curved_lumen_s_curve(lumen.get("s_curve", {})))
    return errors


def _require_number_if_present(
    container: Any,
    key: str,
    prefix: str,
    *,
    required: bool,
    nonnegative: bool = False,
) -> list[str]:
    if required or (isinstance(container, dict) and key in container):
        return _require_number(container, key, prefix, nonnegative=nonnegative)
    return []


def _require_positive_number_if_present(
    container: Any,
    key: str,
    prefix: str,
    *,
    required: bool,
    integer: bool = False,
) -> list[str]:
    if required or (isinstance(container, dict) and key in container):
        return _require_positive_number(container, key, prefix, integer=integer)
    return []


def _validate_curved_lumen_circular_arc(arc: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(arc, dict):
        return ["`curved_lumen.circular_arc` must be a map."]
    errors.extend(_require_numeric_list(arc, "inlet_position", 3, "curved_lumen.circular_arc"))
    errors.extend(_require_numeric_list(arc, "initial_tangent", 3, "curved_lumen.circular_arc"))
    errors.extend(_require_numeric_list(arc, "bend_normal", 3, "curved_lumen.circular_arc"))
    errors.extend(_require_positive_number(arc, "curvature_radius", "curved_lumen.circular_arc"))
    errors.extend(_require_number(arc, "arc_angle", "curved_lumen.circular_arc"))
    tangent = _as_finite_vector(arc.get("initial_tangent"), 3)
    normal = _as_finite_vector(arc.get("bend_normal"), 3)
    errors.extend(_validate_nonzero_vector(tangent, "curved_lumen.circular_arc.initial_tangent"))
    errors.extend(_validate_nonzero_vector(normal, "curved_lumen.circular_arc.bend_normal"))
    errors.extend(
        _validate_nonparallel_vectors(
            tangent,
            normal,
            "curved_lumen.circular_arc.initial_tangent",
            "curved_lumen.circular_arc.bend_normal",
        )
    )
    angle = _as_finite_number(arc.get("arc_angle"))
    if angle == 0.0:
        errors.append("`curved_lumen.circular_arc.arc_angle` must be non-zero.")
    return errors


def _validate_curved_lumen_s_curve(s_curve: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(s_curve, dict):
        return ["`curved_lumen.s_curve` must be a map."]
    errors.extend(_require_numeric_list(s_curve, "inlet_position", 3, "curved_lumen.s_curve"))
    errors.extend(_require_numeric_list(s_curve, "initial_tangent", 3, "curved_lumen.s_curve"))
    errors.extend(_require_numeric_list(s_curve, "bend_plane_normal", 3, "curved_lumen.s_curve"))
    errors.extend(_require_positive_number(s_curve, "total_length", "curved_lumen.s_curve"))
    errors.extend(_require_number(s_curve, "lateral_amplitude", "curved_lumen.s_curve"))
    tangent = _as_finite_vector(s_curve.get("initial_tangent"), 3)
    normal = _as_finite_vector(s_curve.get("bend_plane_normal"), 3)
    errors.extend(_validate_nonzero_vector(tangent, "curved_lumen.s_curve.initial_tangent"))
    errors.extend(_validate_nonzero_vector(normal, "curved_lumen.s_curve.bend_plane_normal"))
    errors.extend(
        _validate_nonparallel_vectors(
            tangent,
            normal,
            "curved_lumen.s_curve.initial_tangent",
            "curved_lumen.s_curve.bend_plane_normal",
        )
    )
    return errors


def _validate_lumen_mode_and_frames(config: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    cylindrical = config.get("cylindrical_lumen", {})
    curved = config.get("curved_lumen", {})
    cylindrical_enabled = isinstance(cylindrical, dict) and cylindrical.get("enabled") is True
    curved_enabled = isinstance(curved, dict) and curved.get("enabled") is True
    if cylindrical_enabled and curved_enabled:
        errors.append("`cylindrical_lumen.enabled` and `curved_lumen.enabled` cannot both be true.")
        return errors
    selected = cylindrical if cylindrical_enabled else curved if curved_enabled else None
    if not isinstance(selected, dict):
        return errors
    lumen_frame = selected.get("frame_id")
    if not isinstance(lumen_frame, str) or not lumen_frame:
        return errors
    frame_sources = [
        ("robot.frames.base", config.get("robot", {}).get("frames", {}).get("base")),
        ("reference.frame_id", config.get("reference", {}).get("frame_id")),
    ]
    if _reference_mode_for_validation(config) == "fixed_target":
        frame_sources.append(("goal.frame_id", config.get("goal", {}).get("frame_id")))
    for label, frame in frame_sources:
        if isinstance(frame, str) and frame and frame != lumen_frame:
            errors.append(f"selected lumen frame `{lumen_frame}` must match `{label}` `{frame}`.")
    return errors


def _validate_cylindrical_lumen_cost(cost: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(cost, dict):
        return ["`cylindrical_lumen_cost` must be a map."]
    if "simulation_default" in cost and not isinstance(cost["simulation_default"], bool):
        errors.append("`cylindrical_lumen_cost.simulation_default` must be a boolean when present.")
    for key in ("safety_margin_weight", "radial_collision_weight", "end_cap_weight", "terminal_collision_weight"):
        errors.extend(_require_number(cost, key, "cylindrical_lumen_cost", nonnegative=True))
    return errors


def _validate_goal(goal: Any, *, reference_mode: str = "fixed_target") -> list[str]:
    errors: list[str] = []
    if not isinstance(goal, dict):
        return ["`goal` must be a map."]
    if "simulation_default" in goal and not isinstance(goal["simulation_default"], bool):
        errors.append("`goal.simulation_default` must be a boolean when present.")
    if not isinstance(goal.get("frame_id"), str) or not goal["frame_id"]:
        errors.append("`goal.frame_id` must be a non-empty string.")
    if reference_mode == "fixed_target":
        errors.extend(_require_numeric_list(goal, "position", 3, "goal"))
    errors.extend(_require_positive_number(goal, "tolerance", "goal"))
    errors.extend(_require_number(goal, "required_hold_duration", "goal", nonnegative=True))
    if "reachability_samples" in goal:
        errors.extend(_require_positive_number(goal, "reachability_samples", "goal", integer=True))
    if "reachability_seed" in goal:
        errors.extend(_require_number(goal, "reachability_seed", "goal", nonnegative=True))
        seed = _as_finite_number(goal.get("reachability_seed"))
        if seed is not None and int(seed) != seed:
            errors.append("`goal.reachability_seed` must be an integer.")
    return errors


def _validate_reference(reference: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(reference, dict):
        return ["`reference` must be a map."]

    mode = reference.get("mode")
    if not isinstance(mode, str) or not mode:
        errors.append("`reference.mode` must be `fixed_target`, `trajectory`, or `external_target`.")
    elif mode not in REFERENCE_MODES:
        errors.append("`reference.mode` must be `fixed_target`, `trajectory`, or `external_target`.")
    if reference.get("trajectory_type") not in {"circle", "ellipse", "helix"}:
        errors.append("`reference.trajectory_type` must be `circle`, `ellipse`, or `helix`.")
    if not isinstance(reference.get("frame_id"), str) or not reference["frame_id"]:
        errors.append("`reference.frame_id` must be a non-empty string.")
    if not isinstance(reference.get("loop"), bool):
        errors.append("`reference.loop` must be a boolean.")
    completion = reference.get("completion_behavior")
    if completion not in {"loop", "hold_final"}:
        errors.append("`reference.completion_behavior` must be `loop` or `hold_final`.")
    elif isinstance(reference.get("loop"), bool) and reference["loop"] != (completion == "loop"):
        errors.append("`reference.loop` must match `reference.completion_behavior`.")

    errors.extend(_require_positive_number(reference, "sample_period", "reference"))
    errors.extend(_require_positive_number(reference, "duration", "reference"))
    errors.extend(_require_positive_number(reference, "publish_frequency", "reference"))
    errors.extend(_require_positive_number(reference, "stale_timeout", "reference"))
    errors.extend(_require_numeric_list(reference, "fixed_target", 3, "reference"))

    sample_period = _as_finite_number(reference.get("sample_period"))
    duration = _as_finite_number(reference.get("duration"))
    if sample_period is not None and duration is not None and sample_period > 0.0 and duration > 0.0:
        if int(duration / sample_period) + 1 < 2:
            errors.append("`reference.duration` and `reference.sample_period` must produce at least two points.")

    circle = reference.get("circle", {})
    if not isinstance(circle, dict):
        errors.append("`reference.circle` must be a map.")
    else:
        errors.extend(_require_numeric_list(circle, "center", 3, "reference.circle"))
        errors.extend(_require_number(circle, "radius", "reference.circle", nonnegative=True))
        errors.extend(_require_number(circle, "angular_velocity", "reference.circle"))
        errors.extend(_require_number(circle, "phase", "reference.circle"))

    ellipse = reference.get("ellipse", {})
    if not isinstance(ellipse, dict):
        errors.append("`reference.ellipse` must be a map.")
    else:
        errors.extend(_require_numeric_list(ellipse, "center", 3, "reference.ellipse"))
        errors.extend(_require_numeric_list(ellipse, "radii", 2, "reference.ellipse", positive=True))
        errors.extend(_require_number(ellipse, "angular_velocity", "reference.ellipse"))
        errors.extend(_require_number(ellipse, "phase", "reference.ellipse"))

    helix = reference.get("helix", {})
    if not isinstance(helix, dict):
        errors.append("`reference.helix` must be a map.")
    else:
        errors.extend(_require_numeric_list(helix, "center", 3, "reference.helix"))
        errors.extend(_require_number(helix, "radius", "reference.helix", nonnegative=True))
        errors.extend(_require_number(helix, "height", "reference.helix"))
        height = _as_finite_number(helix.get("height"))
        if height == 0.0:
            errors.append("`reference.helix.height` must be non-zero.")
        errors.extend(_require_number(helix, "angular_velocity", "reference.helix"))
        errors.extend(_require_number(helix, "phase", "reference.helix"))

    return errors


def _validate_tracking_metrics(tracking_metrics: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(tracking_metrics, dict):
        return ["`tracking_metrics` must be a map."]
    if not isinstance(tracking_metrics.get("enabled"), bool):
        errors.append("`tracking_metrics.enabled` must be a boolean.")
    errors.extend(_require_positive_number(tracking_metrics, "publish_frequency", "tracking_metrics"))
    errors.extend(_require_positive_number(tracking_metrics, "transient_tolerance", "tracking_metrics"))
    errors.extend(_require_positive_number(tracking_metrics, "stable_cycles", "tracking_metrics", integer=True))
    if not isinstance(tracking_metrics.get("reset_on_new_trajectory"), bool):
        errors.append("`tracking_metrics.reset_on_new_trajectory` must be a boolean.")
    return errors


def _validate_evaluation(evaluation: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(evaluation, dict):
        return ["`evaluation` must be a map."]
    for key in (
        "enabled",
        "auto_finalize_on_shutdown",
        "require_command_for_alignment",
        "plot_generation",
        "report_generation",
        "physical_validation",
        "hardware_validation",
    ):
        if not isinstance(evaluation.get(key), bool):
            errors.append(f"`evaluation.{key}` must be a boolean.")
    for key in ("output_root", "experiment_group", "controller_label", "baseline_label", "baseline_result_dir"):
        if key not in evaluation or not isinstance(evaluation[key], str):
            errors.append(f"`evaluation.{key}` must be a string.")
    for key in (
        "configured_duration",
        "maximum_reference_alignment_gap",
        "maximum_command_alignment_gap",
        "maximum_solve_alignment_gap",
        "steady_state_window",
        "tracking_tolerance",
        "duration_compatibility_tolerance",
        "initial_state_compatibility_tolerance",
        "near_zero_baseline_epsilon",
    ):
        if key == "steady_state_window":
            errors.extend(_require_number(evaluation, key, "evaluation", nonnegative=True))
        elif key in ("duration_compatibility_tolerance", "initial_state_compatibility_tolerance"):
            errors.extend(_require_number(evaluation, key, "evaluation", nonnegative=True))
        else:
            errors.extend(_require_positive_number(evaluation, key, "evaluation"))
    for key in ("max_samples_per_topic", "transient_stable_cycles", "minimum_valid_sample_count"):
        errors.extend(_require_positive_number(evaluation, key, "evaluation", integer=True))
    for key in (
        "steady_state_fraction",
        "maximum_invalid_sample_percentage",
        "maximum_saturation_percentage",
        "maximum_deadline_overrun_percentage",
    ):
        errors.extend(_require_number(evaluation, key, "evaluation", nonnegative=True))
        numeric = _as_finite_number(evaluation.get(key))
        if numeric is not None and numeric > (1.0 if key == "steady_state_fraction" else 100.0):
            limit = 1.0 if key == "steady_state_fraction" else 100.0
            errors.append(f"`evaluation.{key}` must be <= {limit}.")
    errors.extend(_require_number(evaluation, "required_minimum_baseline_improvement", "evaluation"))
    errors.extend(_validate_evaluation_orchestration(evaluation.get("orchestration", {})))
    return errors


def _validate_evaluation_orchestration(orchestration: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(orchestration, dict):
        return ["`evaluation.orchestration` must be a map."]
    for key in (
        "enabled",
        "allow_sigkill_cleanup",
        "require_no_baseline_command",
        "require_recording_before_candidate_command",
    ):
        if not isinstance(orchestration.get(key), bool):
            errors.append(f"`evaluation.orchestration.{key}` must be a boolean.")
    for key in (
        "startup_timeout",
        "service_timeout",
        "topic_ready_timeout",
        "reference_ready_timeout",
        "finalization_timeout",
        "initial_stability_duration",
        "reference_lead_time",
        "shutdown_sigint_timeout",
        "shutdown_sigterm_timeout",
    ):
        errors.extend(_require_positive_number(orchestration, key, "evaluation.orchestration"))
    for key in (
        "initial_q_stability_tolerance",
        "initial_tip_stability_tolerance",
        "baseline_candidate_q_tolerance",
        "baseline_candidate_tip_tolerance",
        "command_zero_tolerance",
    ):
        errors.extend(_require_number(orchestration, key, "evaluation.orchestration", nonnegative=True))
    errors.extend(
        _require_positive_number(
            orchestration,
            "initial_stability_samples",
            "evaluation.orchestration",
            integer=True,
        )
    )
    return errors


def _validate_hardware(hardware: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(hardware, dict):
        return ["`hardware` must be a map."]
    if hardware.get("implementation") not in {"mock", "physical", "simulated"}:
        errors.append("`hardware.implementation` must be `mock`, `physical`, or `simulated`.")
    communication = hardware.get("communication", {})
    errors.extend(_require_positive_number(communication, "timeout", "hardware.communication"))
    motors = hardware.get("motors", {})
    if motors.get("count") != 6:
        errors.append("`hardware.motors.count` must be 6.")
    ids = motors.get("ids")
    if not isinstance(ids, list) or len(ids) != 6:
        errors.append("`hardware.motors.ids` must contain 6 motor IDs.")
    for group in ("direction", "encoder_resolution", "conversion_scale"):
        values = motors.get(group, {})
        errors.extend(_require_numeric_list(values, "insertion", 3, f"hardware.motors.{group}"))
        errors.extend(_require_numeric_list(values, "rotation", 3, f"hardware.motors.{group}"))
    watchdog = hardware.get("watchdog", {})
    errors.extend(_require_positive_number(watchdog, "timeout", "hardware.watchdog"))
    return errors


def _validate_safety(safety: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(safety, dict):
        return ["`safety` must be a map."]
    for key in ("enabled", "tactile_enabled", "stop_on_state_timeout", "stop_on_tactile_timeout", "stop_on_invalid_value"):
        if not isinstance(safety.get(key), bool):
            errors.append(f"`safety.{key}` must be a boolean.")
    for key in ("state_timeout", "command_timeout", "tactile_timeout"):
        errors.extend(_require_positive_number(safety, key, "safety"))
    errors.extend(_require_number(safety, "tactile_startup_grace_s", "safety", nonnegative=True))
    errors.extend(_require_number(safety, "tactile_future_skew_s", "safety", nonnegative=True))
    errors.extend(_require_positive_number(safety, "watchdog_period_s", "safety"))
    watchdog = _as_finite_number(safety.get("watchdog_period_s"))
    command_timeout = _as_finite_number(safety.get("command_timeout"))
    if watchdog is not None and command_timeout is not None and watchdog > command_timeout:
        errors.append("`safety.watchdog_period_s` must be <= `safety.command_timeout`.")
    if safety.get("tactile_enabled") is True and safety.get("stop_on_tactile_timeout") is not True:
        errors.append("`safety.stop_on_tactile_timeout` must be true when tactile safety is enabled.")
    soft = safety.get("soft_contact", {})
    errors.extend(_require_positive_number(soft, "velocity_scale", "safety.soft_contact"))
    errors.extend(_require_positive_number(soft, "force_weight_scale", "safety.soft_contact"))
    errors.extend(_require_positive_number(soft, "obstacle_weight_scale", "safety.soft_contact"))
    retreat = safety.get("retreat", {})
    errors.extend(_require_positive_number(retreat, "distance", "safety.retreat"))
    errors.extend(_require_positive_number(retreat, "speed", "safety.retreat"))
    errors.extend(_require_positive_number(retreat, "maximum_duration", "safety.retreat"))
    return errors


def _validate_simulation(simulation: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(simulation, dict):
        return ["`simulation` must be a map."]
    errors.extend(_require_positive_number(simulation, "update_frequency", "simulation"))
    actuator = simulation.get("actuator", {})
    errors.extend(_require_number(actuator, "command_delay", "simulation.actuator", nonnegative=True))
    errors.extend(_require_number(actuator, "dead_zone", "simulation.actuator", nonnegative=True))
    errors.extend(_require_number(actuator, "backlash", "simulation.actuator", nonnegative=True))
    noise = simulation.get("noise", {})
    for key in ("joint_position_std", "joint_velocity_std", "tactile_std"):
        errors.extend(_require_number(noise, key, "simulation.noise", nonnegative=True))
    comms = simulation.get("communication", {})
    for key in ("command_dropout_probability", "state_dropout_probability"):
        errors.extend(_require_probability(comms, key, "simulation.communication"))
    errors.extend(_validate_simulation_visualization(simulation.get("visualization", {})))
    errors.extend(
        _validate_development_target_selection(
            simulation.get("development_target_selection", {})
        )
    )
    return errors


def _validate_simulation_visualization(visualization: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(visualization, dict):
        return ["`simulation.visualization` must be a map."]
    if not isinstance(visualization.get("publish_lumen_markers"), bool):
        errors.append("`simulation.visualization.publish_lumen_markers` must be a boolean.")
    if not isinstance(visualization.get("publish_lumen_diagnostics"), bool):
        errors.append("`simulation.visualization.publish_lumen_diagnostics` must be a boolean.")
    if not isinstance(visualization.get("publish_lumen_surface"), bool):
        errors.append("`simulation.visualization.publish_lumen_surface` must be a boolean.")
    errors.extend(
        _require_exact_int(
            visualization,
            "centerline_stride",
            "simulation.visualization",
            minimum=1,
        )
    )
    errors.extend(
        _require_exact_int(
            visualization,
            "ring_stride",
            "simulation.visualization",
            minimum=1,
        )
    )
    errors.extend(
        _require_exact_int(
            visualization,
            "ring_segments",
            "simulation.visualization",
            minimum=8,
            maximum=128,
        )
    )
    errors.extend(_require_positive_number(visualization, "marker_publish_rate", "simulation.visualization"))
    errors.extend(_require_probability(visualization, "surface_alpha", "simulation.visualization"))
    errors.extend(
        _require_exact_int(
            visualization,
            "actual_tip_history_max_points",
            "simulation.visualization",
            minimum=2,
            maximum=5000,
        )
    )
    errors.extend(
        _require_positive_number(
            visualization,
            "actual_tip_history_min_interval",
            "simulation.visualization",
        )
    )
    return errors


def _validate_development_target_selection(selection: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(selection, dict):
        return ["`simulation.development_target_selection` must be a map."]
    prefix = "simulation.development_target_selection"
    errors.extend(_require_positive_number(selection, "projection_limit", prefix))
    errors.extend(_require_positive_number(selection, "candidate_max_age", prefix))
    errors.extend(
        _require_number(
            selection,
            "candidate_future_tolerance",
            prefix,
            nonnegative=True,
        )
    )
    return errors


def _validate_tactile(tactile: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(tactile, dict):
        return ["`tactile` must be a map."]
    if not isinstance(tactile.get("enabled"), bool):
        errors.append("`tactile.enabled` must be a boolean.")
    if tactile.get("mode") != "simulated":
        errors.append("`tactile.mode` must be `simulated` for the current runtime.")
    sensor = tactile.get("sensor", {})
    errors.extend(_require_positive_number(sensor, "input_dimension", "tactile.sensor", integer=True))
    errors.extend(_require_positive_number(sensor, "output_dimension", "tactile.sensor", integer=True))
    errors.extend(_require_positive_number(sensor, "sample_frequency", "tactile.sensor"))
    calibration = tactile.get("calibration", {})
    errors.extend(_require_number(calibration, "zero_offset", "tactile.calibration"))
    errors.extend(_require_positive_number(calibration, "scale", "tactile.calibration"))
    errors.extend(_require_number(calibration, "bias", "tactile.calibration"))
    filtering = tactile.get("filter", {})
    errors.extend(_require_number(filtering, "alpha", "tactile.filter"))
    if isinstance(filtering, dict) and "alpha" in filtering:
        alpha = _as_finite_number(filtering["alpha"])
        if alpha is not None and not 0.0 < alpha <= 1.0:
            errors.append("`tactile.filter.alpha` must satisfy 0 < alpha <= 1.")
    thresholds = tactile.get("thresholds", {})
    for key in ("contact", "warning", "stop", "release"):
        errors.extend(_require_number(thresholds, key, "tactile.thresholds", nonnegative=True))
    for key in ("contact_off", "warning_off", "stop_off"):
        errors.extend(_require_number(thresholds, key, "tactile.thresholds", nonnegative=True))
    if all(key in thresholds for key in ("release", "contact", "warning", "stop")):
        if not thresholds["release"] <= thresholds["contact"] <= thresholds["warning"] <= thresholds["stop"]:
            errors.append("`tactile.thresholds` must satisfy release <= contact <= warning <= stop.")
    if all(key in thresholds for key in ("contact", "contact_off", "warning", "warning_off", "stop", "stop_off")):
        if not thresholds["contact_off"] < thresholds["contact"]:
            errors.append("`contact_off` must be below `contact`.")
        if not thresholds["contact"] <= thresholds["warning_off"] < thresholds["warning"]:
            errors.append("contact <= warning_off < warning is required.")
        if not thresholds["warning"] <= thresholds["stop_off"] < thresholds["stop"]:
            errors.append("warning <= stop_off < stop is required.")
    simulation = tactile.get("simulation", {})
    errors.extend(_require_number(simulation, "contact_stiffness", "tactile.simulation", nonnegative=True))
    errors.extend(_require_number(simulation, "contact_damping", "tactile.simulation", nonnegative=True))
    errors.extend(_require_positive_number(simulation, "force_saturation_n", "tactile.simulation"))
    if all(key in thresholds for key in ("stop",)) and "force_saturation_n" in simulation:
        saturation = _as_finite_number(simulation["force_saturation_n"])
        stop = _as_finite_number(thresholds["stop"])
        if saturation is not None and stop is not None and stop > saturation:
            errors.append("`tactile.thresholds.stop` must not exceed `force_saturation_n`.")
    errors.extend(_require_number(simulation, "latency", "tactile.simulation", nonnegative=True))
    errors.extend(_require_number(simulation, "noise_std", "tactile.simulation", nonnegative=True))
    return errors


def _require_number(container: Any, key: str, prefix: str, *, nonnegative: bool = False) -> list[str]:
    if not isinstance(container, dict) or key not in container:
        return [f"Missing `{prefix}.{key}`."]
    value = container[key]
    numeric = _as_finite_number(value)
    if numeric is None:
        return [f"`{prefix}.{key}` must be a finite number."]
    if nonnegative and numeric < 0:
        return [f"`{prefix}.{key}` must be nonnegative."]
    return []


def _require_positive_number(
    container: Any,
    key: str,
    prefix: str,
    *,
    integer: bool = False,
) -> list[str]:
    errors = _require_number(container, key, prefix)
    if errors:
        return errors
    value = _as_finite_number(container[key])
    if value is None or value <= 0:
        return [f"`{prefix}.{key}` must be positive."]
    if integer and int(value) != value:
        return [f"`{prefix}.{key}` must be an integer."]
    return []


def _require_probability(container: Any, key: str, prefix: str) -> list[str]:
    errors = _require_number(container, key, prefix)
    if errors:
        return errors
    value = _as_finite_number(container[key])
    if value is None or value < 0 or value > 1:
        return [f"`{prefix}.{key}` must be in [0, 1]."]
    return []


def _require_exact_int(
    container: Any,
    key: str,
    prefix: str,
    *,
    minimum: int,
    maximum: int | None = None,
) -> list[str]:
    if not isinstance(container, dict) or key not in container:
        return [f"Missing `{prefix}.{key}`."]
    value = container[key]
    label = f"`{prefix}.{key}`"
    if isinstance(value, bool) or not isinstance(value, int):
        return [f"{label} must be an integer."]
    if value < minimum:
        return [f"{label} must be >= {minimum}."]
    if maximum is not None and value > maximum:
        return [f"{label} must be <= {maximum}."]
    return []


def _require_numeric_list(
    container: Any,
    key: str,
    length: int,
    prefix: str,
    *,
    positive: bool = False,
    nonnegative: bool = False,
) -> list[str]:
    if not isinstance(container, dict) or key not in container:
        return [f"Missing `{prefix}.{key}`."]
    values = container[key]
    if not isinstance(values, list) or len(values) != length:
        return [f"`{prefix}.{key}` must contain {length} numeric values."]
    errors: list[str] = []
    for index, value in enumerate(values):
        label = f"`{prefix}.{key}[{index}]`"
        numeric = _as_finite_number(value)
        if numeric is None:
            errors.append(f"{label} must be finite.")
        elif positive and numeric <= 0:
            errors.append(f"{label} must be positive.")
        elif nonnegative and numeric < 0:
            errors.append(f"{label} must be nonnegative.")
    return errors


def _require_positive_list(container: Any, key: str, prefix: str) -> list[str]:
    if not isinstance(container, dict) or key not in container:
        return [f"Missing `{prefix}.{key}`."]
    values = container[key]
    if not isinstance(values, list):
        return [f"`{prefix}.{key}` must be a list."]
    return [
        f"`{prefix}.{key}[{i}]` must be positive."
        for i, value in enumerate(values)
        if _as_finite_number(value) is None or _as_finite_number(value) <= 0
    ]


def _validate_bounds(container: Any, low_key: str, high_key: str, prefix: str) -> list[str]:
    if not isinstance(container, dict):
        return [f"`{prefix}` must be a map."]
    lows = container.get(low_key)
    highs = container.get(high_key)
    if not isinstance(lows, list) or not isinstance(highs, list):
        return []
    if len(lows) != len(highs):
        return [f"`{prefix}.{low_key}` and `{prefix}.{high_key}` must have matching lengths."]
    errors: list[str] = []
    for index, (low, high) in enumerate(zip(lows, highs)):
        low_numeric = _as_finite_number(low)
        high_numeric = _as_finite_number(high)
        if low_numeric is None or high_numeric is None:
            errors.append(f"`{prefix}` bound {index} must contain finite values.")
        elif low_numeric >= high_numeric:
            errors.append(f"`{prefix}` bound {index} must satisfy {low_key} < {high_key}.")
    return errors


def _as_finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        numeric = float(value)
    elif isinstance(value, str):
        try:
            numeric = float(value)
        except ValueError:
            return None
    else:
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def _as_finite_vector(value: Any, length: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    result: list[float] = []
    for item in value:
        numeric = _as_finite_number(item)
        if numeric is None:
            return None
        result.append(numeric)
    return result


def _validate_nonzero_vector(vector: list[float] | None, label: str) -> list[str]:
    if vector is None:
        return []
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 0.0:
        return [f"`{label}` must be non-zero."]
    return []


def _validate_nonparallel_vectors(
    first: list[float] | None,
    second: list[float] | None,
    first_label: str,
    second_label: str,
) -> list[str]:
    if first is None or second is None:
        return []
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm <= 0.0 or second_norm <= 0.0:
        return []
    cross = (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )
    cross_norm = math.sqrt(sum(value * value for value in cross))
    if cross_norm / (first_norm * second_norm) <= 1.0e-12:
        return [f"`{first_label}` and `{second_label}` must not be parallel."]
    return []
