"""Strict source-owned effective configuration for the Slice 7G simulation profile.

The base project configuration remains conservative and disabled by default.
This module applies the simulation-only profile only when an authenticated
production coordinator or the explicit owner-selected development workflow
requests it. The profile remains disabled by default.
"""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Any


class Slice7GProfileError(ValueError):
    """Stable profile validation failure."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(f"{code}:{message}")


def apply_slice_7g_simulation_profile(config: dict[str, Any], *, enabled: bool) -> dict[str, Any]:
    """Return a detached effective config; never mutate the supplied mapping."""

    if type(config) is not dict:
        raise Slice7GProfileError("profile_config_type", "config must be an exact dictionary")
    if type(enabled) is not bool:
        raise Slice7GProfileError("profile_enabled_type", "enabled must be an exact bool")
    result = deepcopy(config)
    if not enabled:
        return result
    requested_runtime_mode = result.get("runtime", {}).get("mode")
    if requested_runtime_mode not in (None, "simulation"):
        raise Slice7GProfileError(
            "profile_runtime_mode",
            "the Slice 7G simulation profile cannot override a non-simulation runtime request",
        )
    profile = validate_slice_7g_profile(result)
    result.setdefault("runtime", {})["mode"] = "simulation"
    result["runtime"]["slice_7g_profile"] = True
    result.setdefault("curved_lumen", {})["enabled"] = True
    result["curved_lumen"]["type"] = "circular_arc"
    result.setdefault("cylindrical_lumen", {})["enabled"] = False
    result.setdefault("tactile", {})["enabled"] = True
    result.setdefault("safety", {})["enabled"] = True
    result["safety"]["tactile_enabled"] = True
    result.setdefault("mppi", {}).setdefault("tactile", {})["enabled"] = True
    result["mppi"].setdefault("weights", {})["force"] = float(profile["tactile_force_weight"])
    for name in ("shape", "obstacle", "stability"):
        result["mppi"]["weights"][name] = 0.0
    validate_slice_7g_effective_config(result)
    return result


def apply_slice_7g_development_simulation_profile(
    config: dict[str, Any], *, enabled: bool
) -> dict[str, Any]:
    """Apply the explicit non-production profile without weakening production."""

    if type(enabled) is not bool:
        raise Slice7GProfileError("development_enabled_type", "enabled must be an exact bool")
    if not enabled:
        return deepcopy(config)
    result = apply_slice_7g_simulation_profile(config, enabled=True)
    values = result.get("slice_7g_development_simulation")
    required = {
        "safety_command_timeout_seconds",
        "controller_tactile_max_age_seconds",
    }
    if type(values) is not dict or set(values) != required:
        raise Slice7GProfileError(
            "development_profile_fields",
            "slice_7g_development_simulation has missing or unknown fields",
        )
    command_timeout = _bounded_development_timeout(
        values["safety_command_timeout_seconds"], "safety_command_timeout_seconds"
    )
    tactile_age = _bounded_development_timeout(
        values["controller_tactile_max_age_seconds"],
        "controller_tactile_max_age_seconds",
    )
    result.setdefault("runtime", {})["development_simulation"] = True
    result["safety"]["command_timeout"] = command_timeout
    result["mppi"]["tactile"]["max_age_s"] = tactile_age
    validate_or_raise_development_profile(result)
    return result


def validate_or_raise_development_profile(config: dict[str, Any]) -> None:
    if config.get("runtime", {}).get("mode") != "simulation":
        raise Slice7GProfileError("development_runtime_mode", "runtime must be simulation")
    if config.get("runtime", {}).get("development_simulation") is not True:
        raise Slice7GProfileError("development_runtime_marker", "development marker is missing")
    safety_timeout = config.get("safety", {}).get("command_timeout")
    tactile_age = config.get("mppi", {}).get("tactile", {}).get("max_age_s")
    _bounded_development_timeout(safety_timeout, "safety.command_timeout")
    _bounded_development_timeout(tactile_age, "mppi.tactile.max_age_s")


def _bounded_development_timeout(value: Any, label: str) -> float:
    if (
        type(value) not in (int, float)
        or type(value) is bool
        or not math.isfinite(float(value))
        or not 0.1 <= float(value) <= 10.0
    ):
        raise Slice7GProfileError(
            "development_timeout", f"{label} must be finite and in [0.1, 10.0] seconds"
        )
    return float(value)


def validate_slice_7g_profile(config: dict[str, Any]) -> dict[str, Any]:
    profile = config.get("slice_7g_runtime")
    if type(profile) is not dict:
        raise Slice7GProfileError("profile_missing", "slice_7g_runtime must be an object")
    required = {
        "profile_id", "runtime_mode", "task", "geometry_profile", "tactile_force_weight",
        "tactile_enabled", "tactile_cost_enabled", "safety_supervisor_enabled",
        "safety_tactile_enabled", "controller_safe_command_bypass_enabled",
        "unfinished_costs", "readiness", "acceptance",
    }
    if set(profile) != required:
        raise Slice7GProfileError("profile_fields", "slice_7g_runtime has missing or unknown fields")
    expected_strings = {
        "profile_id": "simulation_promotion_v1",
        "runtime_mode": "simulation",
        "task": "curved_lumen_navigation",
        "geometry_profile": "circular_arc",
    }
    for field, expected in expected_strings.items():
        if type(profile[field]) is not str or profile[field] != expected:
            raise Slice7GProfileError("profile_identity", f"{field} must equal {expected}")
    for field in (
        "tactile_enabled", "tactile_cost_enabled", "safety_supervisor_enabled", "safety_tactile_enabled",
    ):
        if profile[field] is not True:
            raise Slice7GProfileError("profile_feature_disabled", f"{field} must be true")
    if profile["controller_safe_command_bypass_enabled"] is not False:
        raise Slice7GProfileError("profile_command_bypass", "controller safe-command bypass must be false")
    _positive(profile["tactile_force_weight"], "tactile_force_weight")
    unfinished = profile["unfinished_costs"]
    if type(unfinished) is not dict or set(unfinished) != {"obstacle", "shape", "stability"}:
        raise Slice7GProfileError("unfinished_cost_fields", "unfinished costs must be obstacle, shape and stability")
    todo_ids = {"obstacle": "TODO-COST-001", "shape": "TODO-COST-005", "stability": "TODO-COST-006"}
    for name, todo_id in todo_ids.items():
        item = unfinished[name]
        if type(item) is not dict or set(item) != {"todo_id", "enabled", "weight"}:
            raise Slice7GProfileError("unfinished_cost_contract", f"{name} contract is not closed")
        if item["todo_id"] != todo_id or item["enabled"] is not False or not _exact_zero(item["weight"]):
            raise Slice7GProfileError("unfinished_cost_reachable", f"{todo_id} must be disabled with zero weight")
    _validate_readiness(profile["readiness"])
    _validate_acceptance(profile["acceptance"])
    return profile


def validate_slice_7g_effective_config(config: dict[str, Any]) -> None:
    profile = validate_slice_7g_profile(config)
    if config.get("runtime", {}).get("mode") != "simulation":
        raise Slice7GProfileError("effective_runtime_mode", "runtime must be simulation")
    if config.get("curved_lumen", {}).get("enabled") is not True or config["curved_lumen"].get("type") != "circular_arc":
        raise Slice7GProfileError("effective_geometry", "circular-arc curved lumen must be enabled")
    if config.get("cylindrical_lumen", {}).get("enabled") is not False:
        raise Slice7GProfileError("effective_geometry", "cylindrical lumen must be disabled")
    if config.get("tactile", {}).get("enabled") is not True:
        raise Slice7GProfileError("effective_tactile", "simulated tactile publication must be enabled")
    if config.get("safety", {}).get("enabled") is not True or config["safety"].get("tactile_enabled") is not True:
        raise Slice7GProfileError("effective_safety", "safety and safety tactile handling must be enabled")
    mppi = config.get("mppi", {})
    if mppi.get("tactile", {}).get("enabled") is not True:
        raise Slice7GProfileError("effective_tactile_cost", "MPPI tactile cost must be enabled")
    if float(mppi.get("weights", {}).get("force", 0.0)) != float(profile["tactile_force_weight"]):
        raise Slice7GProfileError("effective_tactile_weight", "MPPI tactile force weight differs from the profile")
    for name in ("shape", "obstacle", "stability"):
        if not _exact_zero(mppi.get("weights", {}).get(name)):
            raise Slice7GProfileError("unfinished_cost_reachable", f"{name} weight must equal zero")
    _require_effective_numbers(
        config.get("evaluation", {}),
        {
            "tracking_tolerance": 0.001,
            "minimum_valid_sample_count": 20,
            "maximum_invalid_sample_percentage": 10.0,
            "maximum_saturation_percentage": 1.0,
            "maximum_deadline_overrun_percentage": 5.0,
        },
        "evaluation",
    )
    _require_effective_numbers(
        config.get("evaluation", {}).get("orchestration", {}),
        {
            "topic_ready_timeout": 10.0,
            "initial_stability_duration": 0.5,
            "initial_stability_samples": 10,
            "initial_q_stability_tolerance": 5.0e-5,
            "initial_tip_stability_tolerance": 5.0e-5,
        },
        "evaluation.orchestration",
    )
    _require_effective_numbers(
        config.get("goal", {}),
        {"tolerance": 0.003, "required_hold_duration": 0.5},
        "goal",
    )
    _require_effective_numbers(
        config.get("curved_lumen", {}),
        {"safety_margin": 0.002},
        "curved_lumen",
    )


def _validate_readiness(value: Any) -> None:
    expected = {
        "timeout_seconds": 10.0, "minimum_stable_samples": 10,
        "minimum_stable_interval_seconds": 0.5, "q_variation_tolerance": 5.0e-5,
        "tip_variation_tolerance_m": 5.0e-5, "tactile_max_age_seconds": 0.10,
        "safety_max_age_seconds": 0.10,
    }
    _exact_numeric_object(value, expected, "readiness")


def _validate_acceptance(value: Any) -> None:
    expected = {
        "minimum_valid_aligned_samples": 20, "maximum_invalid_sample_percentage": 10.0,
        "maximum_saturation_percentage": 1.0, "maximum_deadline_overrun_percentage": 5.0,
        "steady_state_error_m": 0.003, "final_goal_error_m": 0.003,
        "goal_hold_duration_seconds": 0.5, "minimum_physical_wall_clearance_m": 0.0,
        "minimum_safety_margin_wall_clearance_m": 0.002,
    }
    _exact_numeric_object(value, expected, "acceptance")


def _exact_numeric_object(value: Any, expected: dict[str, int | float], label: str) -> None:
    if type(value) is not dict or set(value) != set(expected):
        raise Slice7GProfileError(f"{label}_fields", f"{label} contract is not closed")
    for field, required in expected.items():
        observed = value[field]
        if type(required) is int:
            if type(observed) is not int or observed != required:
                raise Slice7GProfileError(f"{label}_value", f"{field} must equal {required}")
        elif type(observed) not in (int, float) or type(observed) is bool or not math.isfinite(float(observed)) or float(observed) != required:
            raise Slice7GProfileError(f"{label}_value", f"{field} must equal {required}")


def _positive(value: Any, label: str) -> float:
    if type(value) not in (int, float) or type(value) is bool or not math.isfinite(float(value)) or float(value) <= 0.0:
        raise Slice7GProfileError("profile_number", f"{label} must be positive and finite")
    return float(value)


def _exact_zero(value: Any) -> bool:
    return type(value) in (int, float) and type(value) is not bool and math.isfinite(float(value)) and float(value) == 0.0


def _require_effective_numbers(value: Any, expected: dict[str, int | float], label: str) -> None:
    if type(value) is not dict:
        raise Slice7GProfileError("effective_threshold_section", f"{label} must be an object")
    for field, required in expected.items():
        observed = value.get(field)
        if type(required) is int:
            valid = type(observed) is int and observed == required
        else:
            valid = (
                type(observed) in (int, float)
                and type(observed) is not bool
                and math.isfinite(float(observed))
                and float(observed) == required
            )
        if not valid:
            raise Slice7GProfileError(
                "effective_threshold",
                f"{label}.{field} must equal {required}",
            )
