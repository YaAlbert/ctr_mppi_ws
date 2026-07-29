"""ROS-independent reference validation and activation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ctr_mppi_controller.lumen_geometry import LumenGeometry


FIXED_TARGET = "fixed_target"
TRAJECTORY = "trajectory"
EXTERNAL_TARGET = "external_target"
REFERENCE_MODES = (FIXED_TARGET, TRAJECTORY, EXTERNAL_TARGET)

NO_ACTIVE_REFERENCE = "NO_ACTIVE_REFERENCE"
PENDING_VALIDATION = "PENDING_VALIDATION"
VALID_REFERENCE = "VALID_REFERENCE"
INVALID_REFERENCE = "INVALID_REFERENCE"
REFERENCE_STATES = (
    NO_ACTIVE_REFERENCE,
    PENDING_VALIDATION,
    VALID_REFERENCE,
    INVALID_REFERENCE,
)

SOURCE_NONE = "none"


@dataclass(frozen=True)
class ActiveReference:
    """Immutable snapshot of the controller's active point reference."""

    mode: str
    state: str
    source: str
    revision: int
    target: np.ndarray | None = None
    target_frame: str = ""
    last_validation_error: str = ""

    def __post_init__(self) -> None:
        validate_reference_mode(self.mode)
        if self.state not in REFERENCE_STATES:
            raise ValueError(f"reference state must be one of {REFERENCE_STATES}")
        if not isinstance(self.source, str) or not self.source:
            raise ValueError("reference source must be a non-empty string")
        if not isinstance(self.revision, int) or self.revision < 0:
            raise ValueError("reference revision must be a nonnegative integer")
        if self.target is None:
            if self.state == VALID_REFERENCE:
                raise ValueError("VALID_REFERENCE requires an active target")
            object.__setattr__(self, "target", None)
            object.__setattr__(self, "target_frame", str(self.target_frame or ""))
            return
        target = vector3(self.target, "reference target")
        if not isinstance(self.target_frame, str) or not self.target_frame:
            raise ValueError("target_frame must be a non-empty string when a target is active")
        object.__setattr__(self, "target", target)

    @property
    def has_valid_target(self) -> bool:
        return self.state == VALID_REFERENCE and self.target is not None


def validate_reference_mode(value: Any, label: str = "reference.mode") -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be one of {REFERENCE_MODES}")
    if value not in REFERENCE_MODES:
        raise ValueError(f"{label} must be one of {REFERENCE_MODES}")
    return value


def initial_reference(mode: str) -> ActiveReference:
    return ActiveReference(
        mode=validate_reference_mode(mode),
        state=NO_ACTIVE_REFERENCE,
        source=SOURCE_NONE,
        revision=0,
    )


def validate_reference_point(
    point: Any,
    *,
    received_frame: str,
    expected_frame: str,
    lumen_geometry: LumenGeometry | None = None,
    require_safety_margin: bool = True,
    label: str = "reference",
) -> np.ndarray:
    """Validate one reference point without projection or coordinate repair."""

    frame = non_empty_frame(received_frame, f"{label}.frame_id")
    expected = non_empty_frame(expected_frame, "expected_reference_frame")
    if frame != expected:
        raise ValueError(f"{label}.frame_id `{frame}` does not match expected frame `{expected}`")
    target = vector3(point, label)
    if lumen_geometry is None:
        return target
    validation = lumen_geometry.validate_target(
        target,
        frame_id=frame,
        require_safety_margin=require_safety_margin,
    )
    if not validation.valid:
        reasons = "; ".join(validation.reasons)
        raise ValueError(f"{label} is invalid for selected lumen geometry: {reasons}")
    return target


def accept_point_reference(
    previous: ActiveReference,
    *,
    source: str,
    point: Any,
    frame: str,
) -> ActiveReference:
    target = vector3(point, "reference target")
    target_frame = non_empty_frame(frame, "reference frame")
    changed = (
        not previous.has_valid_target
        or previous.target_frame != target_frame
        or not np.array_equal(previous.target, target)
    )
    revision = previous.revision + 1 if changed else previous.revision
    return ActiveReference(
        mode=previous.mode,
        state=VALID_REFERENCE,
        source=source,
        revision=revision,
        target=target,
        target_frame=target_frame,
    )


def reject_reference_update(previous: ActiveReference, error: str) -> ActiveReference:
    reason = str(error)
    if previous.has_valid_target:
        return ActiveReference(
            mode=previous.mode,
            state=previous.state,
            source=previous.source,
            revision=previous.revision,
            target=previous.target,
            target_frame=previous.target_frame,
            last_validation_error=reason,
        )
    return ActiveReference(
        mode=previous.mode,
        state=INVALID_REFERENCE,
        source=SOURCE_NONE,
        revision=previous.revision,
        last_validation_error=reason,
    )


def reference_kwargs_from_active(reference: ActiveReference) -> dict[str, np.ndarray]:
    if not reference.has_valid_target:
        raise ValueError(f"{reference.mode} mode requires a valid active reference")
    return {"target_tip": vector3(reference.target, "active reference target")}


def reference_state_log_line(reference: ActiveReference, *, reason: str) -> str:
    frame = reference.target_frame if reference.target_frame else "none"
    return (
        "REFERENCE_STATE "
        f"mode={_token(reference.mode)} "
        f"state={_token(reference.state)} "
        f"source={_token(reference.source)} "
        f"revision={reference.revision} "
        f"frame={_token(frame)} "
        f"reason={_token(reason)}"
    )


def vector3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array.copy()


def non_empty_frame(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _token(value: Any) -> str:
    text = str(value)
    if not text:
        return "none"
    return "".join(char if char.isalnum() or char in "._:-/" else "_" for char in text)
