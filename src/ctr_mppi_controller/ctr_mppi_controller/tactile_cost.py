"""ROS-independent tactile snapshot validation and MPPI cost helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


REGION_NO_CONTACT = 0
REGION_CONTACT = 1
REGION_WARNING = 2
REGION_STOP = 3
VALID_REGIONS = (REGION_NO_CONTACT, REGION_CONTACT, REGION_WARNING, REGION_STOP)


@dataclass(frozen=True)
class TactileSnapshot:
    """Immutable controller-domain copy of one processed tactile message."""

    timestamp_s: float
    frame_id: str
    source: str
    valid: bool
    contact: bool
    warning: bool
    stop: bool
    region: int
    force_magnitude_n: float


@dataclass(frozen=True)
class TactileCostConfig:
    enabled: bool
    max_age_s: float
    tactile_weight: float
    force_saturation_n: float
    proximity_margin_m: float
    no_contact_multiplier: float
    contact_multiplier: float
    warning_multiplier: float
    stop_multiplier: float

    @classmethod
    def from_project_config(cls, config: dict[str, Any]) -> "TactileCostConfig":
        mppi = config.get("mppi", {})
        values = mppi.get("tactile", {})
        weights = mppi.get("weights", {})
        result = cls(
            enabled=_bool(values.get("enabled", False), "mppi.tactile.enabled"),
            max_age_s=_positive(values.get("max_age_s"), "mppi.tactile.max_age_s"),
            tactile_weight=_nonnegative(weights.get("force"), "mppi.weights.force"),
            force_saturation_n=_positive(
                values.get("force_saturation_n"),
                "mppi.tactile.force_saturation_n",
            ),
            proximity_margin_m=_positive(
                values.get("proximity_margin_m"),
                "mppi.tactile.proximity_margin_m",
            ),
            no_contact_multiplier=_nonnegative(
                values.get("no_contact_multiplier"),
                "mppi.tactile.no_contact_multiplier",
            ),
            contact_multiplier=_nonnegative(
                values.get("contact_multiplier"),
                "mppi.tactile.contact_multiplier",
            ),
            warning_multiplier=_nonnegative(
                values.get("warning_multiplier"),
                "mppi.tactile.warning_multiplier",
            ),
            stop_multiplier=_nonnegative(
                values.get("stop_multiplier"),
                "mppi.tactile.stop_multiplier",
            ),
        )
        if result.enabled and result.tactile_weight <= 0.0:
            raise ValueError("enabled mppi.tactile requires a positive mppi.weights.force")
        if result.no_contact_multiplier != 0.0:
            raise ValueError("mppi.tactile.no_contact_multiplier must equal zero")
        if not (
            result.no_contact_multiplier
            <= result.contact_multiplier
            <= result.warning_multiplier
            <= result.stop_multiplier
        ):
            raise ValueError("mppi.tactile multipliers must be nondecreasing")
        return result


def snapshot_from_values(**values: Any) -> TactileSnapshot:
    """Build a snapshot without importing ROS message types."""

    snapshot = TactileSnapshot(
        timestamp_s=float(values["timestamp_s"]),
        frame_id=str(values["frame_id"]),
        source=str(values["source"]),
        valid=bool(values["valid"]),
        contact=bool(values["contact"]),
        warning=bool(values["warning"]),
        stop=bool(values["stop"]),
        region=int(values["region"]),
        force_magnitude_n=float(values["force_magnitude_n"]),
    )
    return snapshot


def snapshot_eligibility(
    snapshot: TactileSnapshot | None,
    *,
    now_s: float,
    max_age_s: float,
    expected_frame: str,
) -> tuple[bool, str]:
    """Return deterministic eligibility and a machine-readable reason."""

    if snapshot is None:
        return False, "missing_snapshot"
    if not math.isfinite(now_s):
        return False, "invalid_controller_time"
    if not snapshot.valid:
        return False, "snapshot_invalid"
    if snapshot.source != "simulated":
        return False, "unsupported_source"
    if not snapshot.frame_id or snapshot.frame_id != expected_frame:
        return False, "frame_mismatch"
    if snapshot.region not in VALID_REGIONS:
        return False, "unknown_region"
    if not math.isfinite(snapshot.timestamp_s) or snapshot.timestamp_s <= 0.0:
        return False, "invalid_timestamp"
    if not math.isfinite(snapshot.force_magnitude_n) or snapshot.force_magnitude_n < 0.0:
        return False, "invalid_force"
    age_s = now_s - snapshot.timestamp_s
    if not math.isfinite(age_s) or age_s < 0.0:
        return False, "future_timestamp"
    if age_s > max_age_s:
        return False, "stale_snapshot"
    return True, "eligible"


def region_multiplier(snapshot: TactileSnapshot, config: TactileCostConfig) -> float:
    values = (
        config.no_contact_multiplier,
        config.contact_multiplier,
        config.warning_multiplier,
        config.stop_multiplier,
    )
    return values[snapshot.region]


def tactile_cost_value(
    *,
    enabled: bool,
    snapshot: TactileSnapshot | None,
    predicted_clearance_m: float | None,
    config: TactileCostConfig,
) -> float:
    """Compute a finite candidate-dependent tactile cost contribution."""

    if not enabled or snapshot is None or predicted_clearance_m is None:
        return 0.0
    if snapshot.region not in VALID_REGIONS:
        return 0.0
    if not math.isfinite(predicted_clearance_m):
        raise ValueError("predicted tactile clearance must be finite")

    multiplier = region_multiplier(snapshot, config)
    if snapshot.region == REGION_NO_CONTACT or multiplier == 0.0:
        return 0.0

    force_fraction = min(max(snapshot.force_magnitude_n / config.force_saturation_n, 0.0), 1.0)
    # A classified non-contact region is neutral. For a valid classified
    # contact state with a zero force field, retain a finite severity rather
    # than silently turning WARNING or STOP into NO_CONTACT.
    severity = max(force_fraction, 1.0e-12)
    proximity_fraction = min(
        max((config.proximity_margin_m - predicted_clearance_m) / config.proximity_margin_m, 0.0),
        1.0,
    )
    value = config.tactile_weight * multiplier * severity * proximity_fraction**2
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("tactile cost must be finite and non-negative")
    return float(value)


def tactile_cost_raw_value(
    *,
    enabled: bool,
    snapshot: TactileSnapshot | None,
    predicted_clearance_m: float | None,
    config: TactileCostConfig,
) -> float:
    """Return the dimensionless tactile severity before its configured weight."""

    if not enabled or snapshot is None or predicted_clearance_m is None:
        return 0.0
    if config.tactile_weight <= 0.0:
        return 0.0
    return tactile_cost_value(
        enabled=enabled,
        snapshot=snapshot,
        predicted_clearance_m=predicted_clearance_m,
        config=config,
    ) / config.tactile_weight


def _bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be boolean")
    return value


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise ValueError(f"{label} must be positive")
    return result


def _nonnegative(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result < 0.0:
        raise ValueError(f"{label} must be non-negative")
    return result
