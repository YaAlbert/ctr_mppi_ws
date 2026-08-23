"""Deterministic calibration, filtering, estimation, and force hysteresis."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

from .simulated_tactile import REGION_CONTACT, REGION_NO_CONTACT, REGION_STOP, REGION_WARNING


@dataclass(frozen=True)
class TactileProcessingParameters:
    """Simulation-only processing parameters; these are not hardware calibration."""

    zero_offset: float
    scale: float
    force_saturation_n: float
    alpha: float
    contact_on_n: float
    contact_off_n: float
    warning_on_n: float
    warning_off_n: float
    stop_on_n: float
    stop_off_n: float

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> "TactileProcessingParameters":
        tactile = config.get("tactile") if isinstance(config, Mapping) else None
        if not isinstance(tactile, Mapping):
            raise ValueError("tactile configuration must be a mapping")
        calibration = tactile.get("calibration", {})
        filtering = tactile.get("filter", {})
        thresholds = tactile.get("thresholds", {})
        simulation = tactile.get("simulation", {})
        return cls(
            zero_offset=_finite(calibration.get("zero_offset"), "zero_offset"),
            scale=_positive(calibration.get("scale"), "scale"),
            force_saturation_n=_positive(simulation.get("force_saturation_n"), "force_saturation_n"),
            alpha=_alpha(filtering.get("alpha")),
            contact_on_n=_nonnegative(thresholds.get("contact"), "contact_on_n"),
            contact_off_n=_nonnegative(thresholds.get("contact_off"), "contact_off_n"),
            warning_on_n=_nonnegative(thresholds.get("warning"), "warning_on_n"),
            warning_off_n=_nonnegative(thresholds.get("warning_off"), "warning_off_n"),
            stop_on_n=_nonnegative(thresholds.get("stop"), "stop_on_n"),
            stop_off_n=_nonnegative(thresholds.get("stop_off"), "stop_off_n"),
        )

    def __post_init__(self) -> None:
        _finite(self.zero_offset, "zero_offset")
        _positive(self.scale, "scale")
        _positive(self.force_saturation_n, "force_saturation_n")
        _alpha(self.alpha)
        for name in (
            "contact_on_n", "contact_off_n", "warning_on_n", "warning_off_n",
            "stop_on_n", "stop_off_n",
        ):
            _nonnegative(getattr(self, name), name)
        if not 0.0 <= self.contact_off_n < self.contact_on_n:
            raise ValueError("contact_off_n must be below contact_on_n")
        if not self.contact_on_n <= self.warning_off_n < self.warning_on_n:
            raise ValueError("contact_on_n <= warning_off_n < warning_on_n is required")
        if not self.warning_on_n <= self.stop_off_n < self.stop_on_n:
            raise ValueError("warning_on_n <= stop_off_n < stop_on_n is required")
        if self.stop_on_n > self.force_saturation_n:
            raise ValueError("stop_on_n must not exceed force_saturation_n")


@dataclass(frozen=True)
class TactileProcessingSample:
    raw_signal: float
    filtered_signal: float
    force_n: float
    clearance_m: float
    contact: bool
    warning: bool
    stop: bool
    region: int
    valid: bool
    diagnostic_status: str


class TactileProcessor:
    """Stateful single-channel processor with deterministic hysteresis."""

    def __init__(self, parameters: TactileProcessingParameters):
        self.parameters = parameters
        self.reset()

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> "TactileProcessor":
        return cls(TactileProcessingParameters.from_mapping(config))

    def reset(self) -> None:
        self._filtered_calibrated: float | None = None
        self._region = REGION_NO_CONTACT

    @property
    def region(self) -> int:
        return self._region

    def process(
        self,
        raw_values: Sequence[Any] | None,
        *,
        clearance_m: Any,
        geometric_contact: bool,
        timestamp_s: Any | None = None,
    ) -> TactileProcessingSample:
        raw = _single_finite(raw_values)
        clearance = _finite_or_none(clearance_m)
        if timestamp_s is not None and _finite_or_none(timestamp_s) is None:
            return self._invalid("timestamp")
        if raw is None or clearance is None:
            return self._invalid("input")

        calibrated = raw - self.parameters.zero_offset
        if not math.isfinite(calibrated):
            return self._invalid("calibration")
        if self._filtered_calibrated is None:
            filtered_calibrated = calibrated
        else:
            filtered_calibrated = (
                self.parameters.alpha * calibrated
                + (1.0 - self.parameters.alpha) * self._filtered_calibrated
            )
        if not math.isfinite(filtered_calibrated):
            return self._invalid("filter")
        estimated_force = min(
            max(0.0, self.parameters.scale * filtered_calibrated),
            self.parameters.force_saturation_n,
        )
        if not math.isfinite(estimated_force):
            return self._invalid("force")

        self._filtered_calibrated = filtered_calibrated
        self._region = self._next_region(estimated_force)
        return TactileProcessingSample(
            raw_signal=raw,
            filtered_signal=filtered_calibrated + self.parameters.zero_offset,
            force_n=estimated_force,
            clearance_m=clearance,
            contact=bool(geometric_contact),
            warning=self._region in (REGION_WARNING, REGION_STOP),
            stop=self._region == REGION_STOP,
            region=self._region,
            valid=True,
            diagnostic_status="simulation_only;slice=7C;directional_force_unavailable",
        )

    def _next_region(self, force: float) -> int:
        p = self.parameters
        if self._region == REGION_STOP and force >= p.stop_off_n:
            return REGION_STOP
        if self._region == REGION_WARNING and force >= p.warning_off_n:
            return REGION_WARNING if force < p.stop_on_n else REGION_STOP
        if self._region == REGION_CONTACT and force >= p.contact_off_n:
            if force >= p.stop_on_n:
                return REGION_STOP
            if force >= p.warning_on_n:
                return REGION_WARNING
            return REGION_CONTACT
        if force >= p.stop_on_n:
            return REGION_STOP
        if force >= p.warning_on_n:
            return REGION_WARNING
        if force >= p.contact_on_n:
            return REGION_CONTACT
        if self._region == REGION_STOP and force >= p.stop_off_n:
            return REGION_STOP
        if self._region == REGION_WARNING and force >= p.warning_off_n:
            return REGION_WARNING
        if self._region == REGION_CONTACT and force >= p.contact_off_n:
            return REGION_CONTACT
        return REGION_NO_CONTACT

    def _invalid(self, reason: str) -> TactileProcessingSample:
        return TactileProcessingSample(
            raw_signal=float("nan"),
            filtered_signal=float("nan"),
            force_n=float("nan"),
            clearance_m=float("nan"),
            contact=False,
            warning=self._region in (REGION_WARNING, REGION_STOP),
            stop=self._region == REGION_STOP,
            region=self._region,
            valid=False,
            diagnostic_status=f"simulation_only;slice=7C;invalid:{reason}",
        )


def _single_finite(values: Sequence[Any] | None) -> float | None:
    if values is None or isinstance(values, (str, bytes)) or len(values) != 1:
        return None
    return _finite_or_none(values[0])


def _finite_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _finite(value: Any, name: str) -> float:
    result = _finite_or_none(value)
    if result is None:
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


def _alpha(value: Any) -> float:
    result = _finite(value, "alpha")
    if not 0.0 < result <= 1.0:
        raise ValueError("alpha must satisfy 0 < alpha <= 1")
    return result
