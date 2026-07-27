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

    return errors


def project_config_with_overrides(
    config: dict[str, Any],
    *,
    runtime_mode: str | None = None,
    hardware_implementation: str | None = None,
) -> dict[str, Any]:
    """Return a copy of config with launch-mode overrides applied."""

    result = deepcopy(config)
    if runtime_mode is not None:
        result.setdefault("runtime", {})["mode"] = runtime_mode
    if hardware_implementation is not None:
        result.setdefault("hardware", {})["implementation"] = hardware_implementation
    return result


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


def _validate_reference(reference: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(reference, dict):
        return ["`reference` must be a map."]

    if reference.get("mode") not in {"fixed_target", "trajectory"}:
        errors.append("`reference.mode` must be `fixed_target` or `trajectory`.")
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
    for key in ("state_timeout", "command_timeout", "tactile_timeout"):
        errors.extend(_require_positive_number(safety, key, "safety"))
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
    return errors


def _validate_tactile(tactile: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(tactile, dict):
        return ["`tactile` must be a map."]
    sensor = tactile.get("sensor", {})
    errors.extend(_require_positive_number(sensor, "input_dimension", "tactile.sensor", integer=True))
    errors.extend(_require_positive_number(sensor, "output_dimension", "tactile.sensor", integer=True))
    errors.extend(_require_positive_number(sensor, "sample_frequency", "tactile.sensor"))
    calibration = tactile.get("calibration", {})
    errors.extend(_require_number(calibration, "zero_offset", "tactile.calibration"))
    errors.extend(_require_number(calibration, "scale", "tactile.calibration"))
    errors.extend(_require_number(calibration, "bias", "tactile.calibration"))
    thresholds = tactile.get("thresholds", {})
    for key in ("contact", "warning", "stop", "release"):
        errors.extend(_require_number(thresholds, key, "tactile.thresholds", nonnegative=True))
    if all(key in thresholds for key in ("release", "contact", "warning", "stop")):
        if not thresholds["release"] <= thresholds["contact"] <= thresholds["warning"] <= thresholds["stop"]:
            errors.append("`tactile.thresholds` must satisfy release <= contact <= warning <= stop.")
    simulation = tactile.get("simulation", {})
    errors.extend(_require_number(simulation, "contact_stiffness", "tactile.simulation", nonnegative=True))
    errors.extend(_require_number(simulation, "contact_damping", "tactile.simulation", nonnegative=True))
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
