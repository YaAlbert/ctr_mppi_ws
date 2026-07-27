"""Timestamp alignment for CTR evaluation samples."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class AlignmentConfig:
    maximum_reference_gap: float
    maximum_command_gap: float
    maximum_solve_gap: float
    require_command: bool = False


@dataclass(frozen=True)
class TimedState:
    timestamp: float
    q: np.ndarray
    q_dot: np.ndarray
    tip_position: np.ndarray


@dataclass(frozen=True)
class TimedReference:
    timestamp: float
    position: np.ndarray
    progress: float | None = None


@dataclass(frozen=True)
class TimedCommand:
    timestamp: float
    command: np.ndarray
    saturated: bool = False
    source: str = ""


@dataclass(frozen=True)
class TimedSolve:
    timestamp: float
    solve_time: float
    saturated: bool = False


@dataclass(frozen=True)
class AlignedSample:
    timestamp: float
    q: np.ndarray
    q_dot: np.ndarray
    tip_position: np.ndarray
    reference_position: np.ndarray
    command: np.ndarray
    solve_time: float
    command_saturated: bool
    missing_command: bool
    reference_gap: float
    command_gap: float
    solve_gap: float
    used_reference_interpolation: bool
    used_nearest_reference: bool
    reference_progress: float | None = None


@dataclass(frozen=True)
class AlignmentDiagnostics:
    raw_state_sample_count: int
    raw_reference_sample_count: int
    raw_command_sample_count: int
    valid_aligned_sample_count: int
    rejected_aligned_sample_count: int
    invalid_nonfinite_sample_count: int
    mean_alignment_gap: float
    maximum_alignment_gap: float
    reference_interpolation_count: int
    nearest_reference_fallback_count: int
    missing_command_count: int
    rejection_reasons: dict[str, int] = field(default_factory=dict)


@dataclass(frozen=True)
class AlignmentResult:
    samples: list[AlignedSample]
    diagnostics: AlignmentDiagnostics


def align_samples(
    *,
    states: list[TimedState],
    references: list[TimedReference],
    commands: list[TimedCommand],
    solves: list[TimedSolve],
    config: AlignmentConfig,
) -> AlignmentResult:
    """Align reference and command data to state timestamps."""

    cfg = _validate_config(config)
    valid_states, invalid_state_count = _valid_states(states)
    valid_refs, invalid_ref_count = _valid_references(references)
    valid_commands, invalid_command_count = _valid_commands(commands)
    valid_solves, invalid_solve_count = _valid_solves(solves)
    nonfinite_count = invalid_state_count + invalid_ref_count + invalid_command_count + invalid_solve_count

    valid_states.sort(key=lambda item: item.timestamp)
    valid_refs.sort(key=lambda item: item.timestamp)
    valid_commands.sort(key=lambda item: item.timestamp)
    valid_solves.sort(key=lambda item: item.timestamp)

    reference_times = [item.timestamp for item in valid_refs]
    command_times = [item.timestamp for item in valid_commands]
    solve_times = [item.timestamp for item in valid_solves]

    samples: list[AlignedSample] = []
    rejection_reasons: dict[str, int] = {}
    interpolation_count = 0
    nearest_count = 0
    missing_command_count = 0
    alignment_gaps: list[float] = []

    for state in valid_states:
        reference_match = _reference_at_time(state.timestamp, valid_refs, reference_times, cfg.maximum_reference_gap)
        if reference_match is None:
            _increment(rejection_reasons, "reference_gap")
            continue
        reference, reference_gap, interpolated, nearest, progress = reference_match

        command_match = _latest_not_later(state.timestamp, valid_commands, command_times)
        missing_command = command_match is None
        if command_match is None:
            command = np.zeros(6, dtype=float)
            command_gap = math.nan
            saturated = False
            missing_command_count += 1
            if cfg.require_command:
                _increment(rejection_reasons, "missing_command")
                continue
        else:
            command_gap = state.timestamp - command_match.timestamp
            if command_gap > cfg.maximum_command_gap:
                missing_command_count += 1
                if cfg.require_command:
                    _increment(rejection_reasons, "command_gap")
                    continue
                command = np.zeros(6, dtype=float)
                command_gap = math.nan
                saturated = False
                missing_command = True
            else:
                command = command_match.command
                saturated = command_match.saturated

        solve_match = _latest_not_later(state.timestamp, valid_solves, solve_times)
        if solve_match is None:
            solve_time = math.nan
            solve_gap = math.nan
        else:
            solve_gap = state.timestamp - solve_match.timestamp
            solve_time = solve_match.solve_time if solve_gap <= cfg.maximum_solve_gap else math.nan

        if interpolated:
            interpolation_count += 1
        if nearest:
            nearest_count += 1

        finite_gaps = [reference_gap]
        if math.isfinite(command_gap):
            finite_gaps.append(command_gap)
        if math.isfinite(solve_gap):
            finite_gaps.append(solve_gap)
        sample_gap = max(finite_gaps) if finite_gaps else math.nan
        if math.isfinite(sample_gap):
            alignment_gaps.append(sample_gap)

        samples.append(
            AlignedSample(
                timestamp=state.timestamp,
                q=state.q,
                q_dot=state.q_dot,
                tip_position=state.tip_position,
                reference_position=reference,
                command=command,
                solve_time=solve_time,
                command_saturated=saturated,
                missing_command=missing_command,
                reference_gap=reference_gap,
                command_gap=command_gap,
                solve_gap=solve_gap,
                used_reference_interpolation=interpolated,
                used_nearest_reference=nearest,
                reference_progress=progress,
            )
        )

    rejected = len(valid_states) - len(samples)
    diagnostics = AlignmentDiagnostics(
        raw_state_sample_count=len(states),
        raw_reference_sample_count=len(references),
        raw_command_sample_count=len(commands),
        valid_aligned_sample_count=len(samples),
        rejected_aligned_sample_count=rejected,
        invalid_nonfinite_sample_count=nonfinite_count,
        mean_alignment_gap=float(np.mean(alignment_gaps)) if alignment_gaps else math.nan,
        maximum_alignment_gap=float(np.max(alignment_gaps)) if alignment_gaps else math.nan,
        reference_interpolation_count=interpolation_count,
        nearest_reference_fallback_count=nearest_count,
        missing_command_count=missing_command_count,
        rejection_reasons=rejection_reasons,
    )
    return AlignmentResult(samples=samples, diagnostics=diagnostics)


def state_sample(timestamp: Any, q: Any, q_dot: Any, tip_position: Any) -> TimedState:
    return TimedState(
        timestamp=_time(timestamp),
        q=_array(q, "q", (6,)),
        q_dot=_array(q_dot, "q_dot", (6,)),
        tip_position=_array(tip_position, "tip_position", (3,)),
    )


def reference_sample(timestamp: Any, position: Any, progress: Any | None = None) -> TimedReference:
    progress_value = None if progress is None else _progress(progress)
    return TimedReference(
        timestamp=_time(timestamp),
        position=_array(position, "reference_position", (3,)),
        progress=progress_value,
    )


def command_sample(timestamp: Any, command: Any, *, saturated: bool = False, source: str = "") -> TimedCommand:
    return TimedCommand(
        timestamp=_time(timestamp),
        command=_array(command, "command", (6,)),
        saturated=bool(saturated),
        source=str(source),
    )


def solve_sample(timestamp: Any, solve_time: Any, *, saturated: bool = False) -> TimedSolve:
    return TimedSolve(
        timestamp=_time(timestamp),
        solve_time=_nonnegative(solve_time, "solve_time"),
        saturated=bool(saturated),
    )


def aligned_arrays(samples: list[AlignedSample]) -> dict[str, np.ndarray]:
    return {
        "timestamps": np.asarray([sample.timestamp for sample in samples], dtype=float),
        "q": np.asarray([sample.q for sample in samples], dtype=float),
        "q_dot": np.asarray([sample.q_dot for sample in samples], dtype=float),
        "tip_positions": np.asarray([sample.tip_position for sample in samples], dtype=float),
        "reference_positions": np.asarray([sample.reference_position for sample in samples], dtype=float),
        "commands": np.asarray([sample.command for sample in samples], dtype=float),
        "solve_times": np.asarray([sample.solve_time for sample in samples if math.isfinite(sample.solve_time)], dtype=float),
        "solve_timestamps": np.asarray(
            [sample.timestamp for sample in samples if math.isfinite(sample.solve_time)],
            dtype=float,
        ),
        "saturation_flags": np.asarray([sample.command_saturated for sample in samples], dtype=bool),
        "missing_command_flags": np.asarray([sample.missing_command for sample in samples], dtype=bool),
        "reference_progress": np.asarray(
            [
                math.nan if sample.reference_progress is None else sample.reference_progress
                for sample in samples
            ],
            dtype=float,
        ),
    }


def _reference_at_time(
    timestamp: float,
    references: list[TimedReference],
    reference_times: list[float],
    maximum_gap: float,
) -> tuple[np.ndarray, float, bool, bool, float | None] | None:
    if not references:
        return None
    index = bisect_left(reference_times, timestamp)
    if index < len(references) and references[index].timestamp == timestamp:
        reference = references[index]
        return reference.position, 0.0, False, False, reference.progress
    if 0 < index < len(references):
        left = references[index - 1]
        right = references[index]
        left_gap = timestamp - left.timestamp
        right_gap = right.timestamp - timestamp
        gap = max(left_gap, right_gap)
        if gap <= maximum_gap and right.timestamp > left.timestamp:
            alpha = (timestamp - left.timestamp) / (right.timestamp - left.timestamp)
            position = (1.0 - alpha) * left.position + alpha * right.position
            progress = _interpolated_progress(left.progress, right.progress, alpha)
            return position, gap, True, False, progress
    nearest_candidates = []
    if index > 0:
        nearest_candidates.append(references[index - 1])
    if index < len(references):
        nearest_candidates.append(references[index])
    if not nearest_candidates:
        return None
    nearest = min(nearest_candidates, key=lambda item: abs(item.timestamp - timestamp))
    gap = abs(nearest.timestamp - timestamp)
    if gap <= maximum_gap:
        return nearest.position, gap, False, True, nearest.progress
    return None


def _latest_not_later(timestamp: float, samples: list[Any], times: list[float]):
    if not samples:
        return None
    index = bisect_right(times, timestamp) - 1
    if index < 0:
        return None
    return samples[index]


def _valid_states(samples: list[TimedState]) -> tuple[list[TimedState], int]:
    valid: list[TimedState] = []
    invalid = 0
    for sample in samples:
        if _sample_is_finite(sample.timestamp, sample.q, sample.q_dot, sample.tip_position):
            valid.append(sample)
        else:
            invalid += 1
    return valid, invalid


def _valid_references(samples: list[TimedReference]) -> tuple[list[TimedReference], int]:
    valid: list[TimedReference] = []
    invalid = 0
    for sample in samples:
        progress_valid = sample.progress is None or math.isfinite(sample.progress)
        if _sample_is_finite(sample.timestamp, sample.position) and progress_valid:
            valid.append(sample)
        else:
            invalid += 1
    return valid, invalid


def _valid_commands(samples: list[TimedCommand]) -> tuple[list[TimedCommand], int]:
    valid: list[TimedCommand] = []
    invalid = 0
    for sample in samples:
        if _sample_is_finite(sample.timestamp, sample.command):
            valid.append(sample)
        else:
            invalid += 1
    return valid, invalid


def _valid_solves(samples: list[TimedSolve]) -> tuple[list[TimedSolve], int]:
    valid: list[TimedSolve] = []
    invalid = 0
    for sample in samples:
        if _sample_is_finite(sample.timestamp, sample.solve_time):
            valid.append(sample)
        else:
            invalid += 1
    return valid, invalid


def _sample_is_finite(*values: Any) -> bool:
    for value in values:
        array = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(array)):
            return False
    return True


def _validate_config(config: AlignmentConfig) -> AlignmentConfig:
    return AlignmentConfig(
        maximum_reference_gap=_positive(config.maximum_reference_gap, "maximum_reference_gap"),
        maximum_command_gap=_positive(config.maximum_command_gap, "maximum_command_gap"),
        maximum_solve_gap=_positive(config.maximum_solve_gap, "maximum_solve_gap"),
        require_command=bool(config.require_command),
    )


def _array(values: Any, label: str, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape {shape} and contain finite values")
    return array.copy()


def _time(value: Any) -> float:
    return _nonnegative(value, "timestamp")


def _nonnegative(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{label} must be finite and non-negative")
    return numeric


def _positive(value: Any, label: str) -> float:
    numeric = _nonnegative(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _progress(value: Any) -> float:
    numeric = _nonnegative(value, "progress")
    return float(np.clip(numeric, 0.0, 1.0))


def _interpolated_progress(left: float | None, right: float | None, alpha: float) -> float | None:
    if left is None or right is None:
        return None
    return float((1.0 - alpha) * left + alpha * right)


def _increment(values: dict[str, int], key: str) -> None:
    values[key] = values.get(key, 0) + 1
