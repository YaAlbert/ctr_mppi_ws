"""Deterministic, geometry-independent simulated tactile signal model."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


REGION_NO_CONTACT = 0
REGION_CONTACT = 1
REGION_WARNING = 2
REGION_STOP = 3


@dataclass(frozen=True)
class SimulatedTactileParameters:
    """Finite simulation-only parameters, not physical calibration values."""

    zero_offset: float
    scale: float
    contact_stiffness_n_per_m: float
    force_saturation_n: float
    contact_threshold_n: float
    warning_threshold_n: float
    stop_threshold_n: float
    noise_std: float = 0.0

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> "SimulatedTactileParameters":
        tactile = config.get("tactile") if isinstance(config, Mapping) else None
        if not isinstance(tactile, Mapping):
            raise ValueError("tactile configuration must be a mapping")
        calibration = tactile.get("calibration", {})
        thresholds = tactile.get("thresholds", {})
        simulation = tactile.get("simulation", {})
        return cls(
            zero_offset=_finite(calibration.get("zero_offset"), "zero_offset"),
            scale=_positive(calibration.get("scale"), "scale"),
            contact_stiffness_n_per_m=_nonnegative(
                simulation.get("contact_stiffness"), "contact_stiffness"
            ),
            force_saturation_n=_positive(
                simulation.get("force_saturation_n"), "force_saturation_n"
            ),
            contact_threshold_n=_nonnegative(thresholds.get("contact"), "contact"),
            warning_threshold_n=_nonnegative(thresholds.get("warning"), "warning"),
            stop_threshold_n=_nonnegative(thresholds.get("stop"), "stop"),
            noise_std=_nonnegative(simulation.get("noise_std"), "noise_std"),
        )

    def __post_init__(self) -> None:
        _finite(self.zero_offset, "zero_offset")
        _positive(self.scale, "scale")
        _nonnegative(self.contact_stiffness_n_per_m, "contact_stiffness_n_per_m")
        _positive(self.force_saturation_n, "force_saturation_n")
        _nonnegative(self.contact_threshold_n, "contact_threshold_n")
        _nonnegative(self.warning_threshold_n, "warning_threshold_n")
        _nonnegative(self.stop_threshold_n, "stop_threshold_n")
        _nonnegative(self.noise_std, "noise_std")
        if self.noise_std != 0.0:
            raise ValueError("Slice 7B requires noise_std=0.0")
        if not self.contact_threshold_n <= self.warning_threshold_n <= self.stop_threshold_n:
            raise ValueError("tactile thresholds must satisfy contact <= warning <= stop")


@dataclass(frozen=True)
class SimulatedTactileSample:
    clearance_m: float
    penetration_m: float
    force_n: float
    raw_signal: float
    filtered_signal: float
    contact: bool
    warning: bool
    stop: bool
    region: int
    valid: bool
    saturated: bool
    diagnostic_status: str


def simulate_tactile(
    clearance_m: Any,
    parameters: SimulatedTactileParameters,
) -> SimulatedTactileSample:
    """Convert one signed clearance into deterministic Slice 7B state."""

    try:
        clearance = float(clearance_m)
    except (TypeError, ValueError):
        return _invalid("invalid clearance")
    if not math.isfinite(clearance):
        return _invalid("nonfinite clearance")

    contact = clearance <= 0.0
    penetration = max(0.0, -clearance)
    unsaturated_force = parameters.contact_stiffness_n_per_m * penetration
    force = min(unsaturated_force, parameters.force_saturation_n)
    saturated = unsaturated_force > parameters.force_saturation_n
    raw = parameters.zero_offset + force / parameters.scale
    if force >= parameters.stop_threshold_n:
        region = REGION_STOP
    elif force >= parameters.warning_threshold_n:
        region = REGION_WARNING
    elif force >= parameters.contact_threshold_n:
        region = REGION_CONTACT
    else:
        region = REGION_NO_CONTACT
    status = "simulation_only;slice=7B;directional_force_unavailable"
    if contact:
        status += ";geometric_contact"
    if saturated:
        status += ";force_saturated"
    return SimulatedTactileSample(
        clearance_m=clearance,
        penetration_m=penetration,
        force_n=force,
        raw_signal=raw,
        filtered_signal=raw,
        contact=contact,
        warning=force >= parameters.warning_threshold_n,
        stop=force >= parameters.stop_threshold_n,
        region=region,
        valid=True,
        saturated=saturated,
        diagnostic_status=status,
    )


def _invalid(reason: str) -> SimulatedTactileSample:
    return SimulatedTactileSample(
        clearance_m=float("nan"),
        penetration_m=float("nan"),
        force_n=float("nan"),
        raw_signal=float("nan"),
        filtered_signal=float("nan"),
        contact=False,
        warning=False,
        stop=False,
        region=REGION_NO_CONTACT,
        valid=False,
        saturated=False,
        diagnostic_status=f"simulation_only;slice=7B;invalid:{reason}",
    )


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative(value: Any, name: str) -> float:
    result = _finite(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be nonnegative")
    return result
