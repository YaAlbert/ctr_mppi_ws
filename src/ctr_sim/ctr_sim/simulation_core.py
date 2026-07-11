"""Core state update for the CTR simulation loop."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import math
import numpy as np


@dataclass(frozen=True)
class SimulationStepResult:
    q: np.ndarray
    q_dot: np.ndarray
    command_saturated: bool


class CTRSimulationCore:
    """Numerical state update for the Milestone 3 simulator."""

    def __init__(self, config: dict[str, Any]):
        robot = config["robot"]
        limits = robot["limits"]
        initial = robot["initial_configuration"]

        self.q = np.concatenate(
            (
                _array3(initial["insertion"], "robot.initial_configuration.insertion"),
                _array3(initial["rotation"], "robot.initial_configuration.rotation"),
            )
        )
        self.q_dot = np.zeros(6, dtype=float)
        self.q_min = np.concatenate(
            (
                _array3(limits["insertion_min"], "robot.limits.insertion_min"),
                _array3(limits["rotation_min"], "robot.limits.rotation_min"),
            )
        )
        self.q_max = np.concatenate(
            (
                _array3(limits["insertion_max"], "robot.limits.insertion_max"),
                _array3(limits["rotation_max"], "robot.limits.rotation_max"),
            )
        )
        self.velocity_max = np.concatenate(
            (
                _array3(limits["insertion_velocity_max"], "robot.limits.insertion_velocity_max"),
                _array3(limits["rotation_velocity_max"], "robot.limits.rotation_velocity_max"),
            )
        )

    def step(self, command_q_dot: np.ndarray | list[float] | tuple[float, ...], dt: float) -> SimulationStepResult:
        if dt <= 0 or not math.isfinite(dt):
            raise ValueError("dt must be positive and finite")

        command = np.asarray(command_q_dot, dtype=float)
        if command.shape != (6,):
            raise ValueError("command_q_dot must have shape (6,)")
        if not np.all(np.isfinite(command)):
            raise ValueError("command_q_dot must contain only finite values")

        clipped_command = np.clip(command, -self.velocity_max, self.velocity_max)
        next_q = np.clip(self.q + dt * clipped_command, self.q_min, self.q_max)
        saturated = not np.allclose(command, clipped_command) or not np.allclose(self.q + dt * clipped_command, next_q)

        self.q = next_q
        self.q_dot = clipped_command
        return SimulationStepResult(q=self.q.copy(), q_dot=self.q_dot.copy(), command_saturated=bool(saturated))


def _array3(values: Any, label: str) -> np.ndarray:
    array = np.asarray([float(value) for value in values], dtype=float)
    if array.shape != (3,):
        raise ValueError(f"{label} must contain 3 numeric values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    return array
