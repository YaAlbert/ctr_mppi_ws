"""Cost terms for the ROS-independent MPPI core.

Milestone 4 enables tip, control, smoothness, hard limits, and terminal tip
terms only. Shape, obstacle, tactile, and stability hooks are present but
disabled by default.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def tip_tracking_cost(tip_position: np.ndarray, target_tip: np.ndarray) -> float:
    error = np.asarray(tip_position, dtype=float) - np.asarray(target_tip, dtype=float)
    return float(np.dot(error, error))


def terminal_tip_cost(tip_position: np.ndarray, target_tip: np.ndarray) -> float:
    return tip_tracking_cost(tip_position, target_tip)


def control_magnitude_cost(command: np.ndarray) -> float:
    command = np.asarray(command, dtype=float)
    return float(np.dot(command, command))


def control_smoothness_cost(command: np.ndarray, previous_command: np.ndarray) -> float:
    delta = np.asarray(command, dtype=float) - np.asarray(previous_command, dtype=float)
    return float(np.dot(delta, delta))


def shape_tracking_cost(*, enabled: bool, **_: Any) -> float:
    if enabled:
        raise NotImplementedError("TODO-COST-005: shape cost interface exists but is disabled in Milestone 4.")
    return 0.0


def obstacle_cost(*, enabled: bool, **_: Any) -> float:
    if enabled:
        raise NotImplementedError("TODO-COST-001: obstacle cost interface exists but is disabled in Milestone 4.")
    return 0.0


def tactile_cost(*, enabled: bool, **_: Any) -> float:
    if enabled:
        raise NotImplementedError("TODO-SNS-001: tactile cost interface exists but is disabled in Milestone 4.")
    return 0.0


def stability_cost(*, enabled: bool, **_: Any) -> float:
    if enabled:
        raise NotImplementedError("TODO-COST-006: stability cost interface exists but is disabled in Milestone 4.")
    return 0.0
