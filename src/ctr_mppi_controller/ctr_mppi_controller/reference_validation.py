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
SOURCE_TRAJECTORY_HORIZON = "trajectory_horizon"


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


@dataclass(frozen=True)
class ActiveTrajectoryReference:
    """Immutable snapshot of the controller's active trajectory horizon."""

    mode: str
    state: str
    source: str
    revision: int
    points: np.ndarray | None = None
    frame_id: str = ""
    stamp_s: float | None = None
    last_validation_error: str = ""

    def __post_init__(self) -> None:
        mode = validate_reference_mode(self.mode)
        if mode != TRAJECTORY:
            raise ValueError("trajectory reference snapshot requires trajectory mode")
        if self.state not in REFERENCE_STATES:
            raise ValueError(f"reference state must be one of {REFERENCE_STATES}")
        if not isinstance(self.source, str) or not self.source:
            raise ValueError("reference source must be a non-empty string")
        if not isinstance(self.revision, int) or self.revision < 0:
            raise ValueError("reference revision must be a nonnegative integer")
        if self.points is None:
            if self.state == VALID_REFERENCE:
                raise ValueError("VALID_REFERENCE requires active trajectory points")
            object.__setattr__(self, "points", None)
            object.__setattr__(self, "frame_id", str(self.frame_id or ""))
            object.__setattr__(self, "stamp_s", None)
            return

        points = _reference_sequence_array(self.points, "trajectory reference points")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a non-empty string when a trajectory is active")
        stamp_s = _finite_float(self.stamp_s, "trajectory horizon stamp_s")
        points.setflags(write=False)
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "stamp_s", stamp_s)

    @property
    def has_valid_horizon(self) -> bool:
        return self.state == VALID_REFERENCE and self.points is not None

    @property
    def point_count(self) -> int:
        return 0 if self.points is None else int(self.points.shape[0])


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


def initial_trajectory_reference() -> ActiveTrajectoryReference:
    return ActiveTrajectoryReference(
        mode=TRAJECTORY,
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


def validate_reference_sequence(
    points: Any,
    *,
    received_frame: str,
    expected_frame: str,
    lumen_geometry: LumenGeometry | None = None,
    expected_count: int | None = None,
    require_safety_margin: bool = True,
    label: str = "reference horizon",
) -> np.ndarray:
    """Validate an ordered reference-point sequence without coordinate repair."""

    frame = non_empty_frame(received_frame, f"{label}.frame_id")
    expected = non_empty_frame(expected_frame, "expected_reference_frame")
    if frame != expected:
        raise ValueError(f"{label} frame mismatch: expected `{expected}`, got `{frame}`")
    if lumen_geometry is not None and frame != lumen_geometry.frame_id:
        raise ValueError(f"{label} frame mismatch: expected lumen frame `{lumen_geometry.frame_id}`, got `{frame}`")

    sequence = _reference_sequence_array(points, label)
    if sequence.shape[0] == 0:
        raise ValueError(f"{label} must contain at least one point")
    if expected_count is not None:
        count = _positive_int(expected_count, "expected_count")
        if sequence.shape[0] != count:
            raise ValueError(f"{label} must have shape ({count}, 3), got {sequence.shape}")

    if lumen_geometry is not None:
        for index, point in enumerate(sequence):
            validation = lumen_geometry.validate_target(
                point,
                frame_id=frame,
                require_safety_margin=require_safety_margin,
            )
            if not validation.valid:
                reasons = "; ".join(validation.reasons)
                raise ValueError(f"{label} point[{index}] is invalid: {reasons}")

    accepted = sequence.copy()
    accepted.setflags(write=False)
    return accepted


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


def accept_trajectory_reference(
    previous: ActiveTrajectoryReference,
    *,
    points: Any,
    frame: str,
    stamp_s: float,
) -> ActiveTrajectoryReference:
    sequence = _reference_sequence_array(points, "trajectory reference points")
    sequence.setflags(write=False)
    frame_id = non_empty_frame(frame, "trajectory reference frame")
    stamp = _finite_float(stamp_s, "trajectory horizon stamp_s")
    changed = (
        not previous.has_valid_horizon
        or previous.frame_id != frame_id
        or previous.points.shape != sequence.shape
        or not np.array_equal(previous.points, sequence)
    )
    revision = previous.revision + 1 if changed else previous.revision
    return ActiveTrajectoryReference(
        mode=TRAJECTORY,
        state=VALID_REFERENCE,
        source=SOURCE_TRAJECTORY_HORIZON,
        revision=revision,
        points=sequence,
        frame_id=frame_id,
        stamp_s=stamp,
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


def reject_trajectory_update(previous: ActiveTrajectoryReference, error: str) -> ActiveTrajectoryReference:
    reason = str(error)
    if previous.has_valid_horizon:
        return ActiveTrajectoryReference(
            mode=previous.mode,
            state=previous.state,
            source=previous.source,
            revision=previous.revision,
            points=previous.points,
            frame_id=previous.frame_id,
            stamp_s=previous.stamp_s,
            last_validation_error=reason,
        )
    return ActiveTrajectoryReference(
        mode=TRAJECTORY,
        state=INVALID_REFERENCE,
        source=SOURCE_NONE,
        revision=previous.revision,
        last_validation_error=reason,
    )


def reference_kwargs_from_active(reference: ActiveReference) -> dict[str, np.ndarray]:
    if not reference.has_valid_target:
        raise ValueError(f"{reference.mode} mode requires a valid active reference")
    return {"target_tip": vector3(reference.target, "active reference target")}


def trajectory_kwargs_from_active(
    reference: ActiveTrajectoryReference,
    *,
    current_time_s: float,
    stale_timeout: float,
) -> dict[str, np.ndarray]:
    if not reference.has_valid_horizon or reference.stamp_s is None:
        raise ValueError("trajectory mode requires a valid active trajectory horizon")
    now_s = _finite_float(current_time_s, "current_time_s")
    stamp_s = _finite_float(reference.stamp_s, "trajectory horizon stamp_s")
    timeout = _positive_float(stale_timeout, "stale_timeout")
    age_s = now_s - stamp_s
    if age_s < -1.0e-9:
        raise ValueError("trajectory horizon timestamp is in the future")
    if age_s > timeout:
        raise ValueError(f"trajectory horizon is stale: age_s={age_s:.6f}")
    return {"target_tip_sequence": reference.points.copy()}


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


def trajectory_state_log_line(reference: ActiveTrajectoryReference, *, reason: str) -> str:
    frame = reference.frame_id if reference.frame_id else "none"
    return (
        "REFERENCE_STATE "
        f"mode={_token(reference.mode)} "
        f"state={_token(reference.state)} "
        f"source={_token(reference.source)} "
        f"revision={reference.revision} "
        f"frame={_token(frame)} "
        f"points={reference.point_count} "
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


def _reference_sequence_array(values: Any, label: str) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must have shape (N, 3) and contain finite values") from exc
    if array.size == 0:
        raise ValueError(f"{label} must contain at least one point")
    if array.ndim != 2:
        raise ValueError(f"{label} must have shape (N, 3), got rank {array.ndim}")
    if array.shape[1] != 3:
        raise ValueError(f"{label} must have shape (N, 3), got {array.shape}")
    nonfinite = np.argwhere(~np.isfinite(array))
    if nonfinite.size:
        point_index = int(nonfinite[0, 0])
        raise ValueError(f"{label} point[{point_index}] contains non-finite coordinates")
    return array.copy()


def _finite_float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be finite")
    numeric = float(value)
    if not np.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _positive_float(value: Any, label: str) -> float:
    numeric = _finite_float(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_float(value, label)
    integer = int(numeric)
    if integer != numeric:
        raise ValueError(f"{label} must be an integer")
    return integer


def _token(value: Any) -> str:
    text = str(value)
    if not text:
        return "none"
    return "".join(char if char.isalnum() or char in "._:-/" else "_" for char in text)
