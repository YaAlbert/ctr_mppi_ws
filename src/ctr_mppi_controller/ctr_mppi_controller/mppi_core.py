"""ROS-independent MPPI controller core for Milestone 4."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Any

import numpy as np

from .cost_functions import (
    control_magnitude_cost,
    control_smoothness_cost,
    obstacle_cost,
    shape_tracking_cost,
    stability_cost,
    tactile_cost,
    terminal_tip_cost,
    tip_tracking_cost,
)


@dataclass(frozen=True)
class MPPIResult:
    command: np.ndarray
    nominal_sequence: np.ndarray
    solve_time: float
    minimum_cost: float
    mean_cost: float
    effective_sample_weight: float
    command_magnitude: float
    command_saturated: bool
    diagnostic_status: str


class MPPICore:
    """A small MPPI optimizer independent of ROS2."""

    def __init__(self, config: dict[str, Any], model: Any):
        self._config = config
        self._model = model

        mppi = config["mppi"]
        robot_limits = config["robot"]["limits"]
        self.dt = float(mppi["dt"])
        self.horizon = int(mppi["horizon"])
        self.num_samples = int(mppi["num_samples"])
        self.temperature = float(mppi["lambda"])
        self.warm_start = bool(mppi.get("warm_start", True))
        self.shift_previous_solution = bool(mppi.get("shift_previous_solution", True))

        self.weights = mppi.get("weights", {})
        self.noise_std = np.concatenate(
            (
                _array3(mppi["noise_std"]["insertion"], "mppi.noise_std.insertion"),
                _array3(mppi["noise_std"]["rotation"], "mppi.noise_std.rotation"),
            )
        )
        self.q_min = np.concatenate(
            (
                _array3(robot_limits["insertion_min"], "robot.limits.insertion_min"),
                _array3(robot_limits["rotation_min"], "robot.limits.rotation_min"),
            )
        )
        self.q_max = np.concatenate(
            (
                _array3(robot_limits["insertion_max"], "robot.limits.insertion_max"),
                _array3(robot_limits["rotation_max"], "robot.limits.rotation_max"),
            )
        )
        self.velocity_max = np.concatenate(
            (
                _array3(robot_limits["insertion_velocity_max"], "robot.limits.insertion_velocity_max"),
                _array3(robot_limits["rotation_velocity_max"], "robot.limits.rotation_velocity_max"),
            )
        )

        if self.horizon <= 0:
            raise ValueError("mppi.horizon must be positive")
        if self.num_samples <= 0:
            raise ValueError("mppi.num_samples must be positive")
        if self.temperature <= 0:
            raise ValueError("mppi.lambda must be positive")

        self.nominal_sequence = np.zeros((self.horizon, 6), dtype=float)
        self.rng = np.random.default_rng(int(mppi.get("random_seed", 0)))
        self._validate_disabled_costs()

    def reset(self) -> None:
        self.nominal_sequence.fill(0.0)

    def solve(
        self,
        *,
        q: np.ndarray | list[float] | tuple[float, ...],
        q_dot: np.ndarray | list[float] | tuple[float, ...],
        target_tip: np.ndarray | list[float] | tuple[float, ...],
    ) -> MPPIResult:
        start = perf_counter()
        q0 = _array6(q, "q")
        previous_command = _array6(q_dot, "q_dot")
        target = _array3(target_tip, "target_tip")

        if self.warm_start and self.shift_previous_solution:
            self.nominal_sequence[:-1] = self.nominal_sequence[1:]
            self.nominal_sequence[-1] = 0.0
        elif not self.warm_start:
            self.nominal_sequence.fill(0.0)

        perturbations = self.rng.normal(
            loc=0.0,
            scale=self.noise_std,
            size=(self.num_samples, self.horizon, 6),
        )
        perturbations[0, :, :] = 0.0
        candidate_sequences = np.clip(
            self.nominal_sequence[None, :, :] + perturbations,
            -self.velocity_max,
            self.velocity_max,
        )

        costs = np.zeros(self.num_samples, dtype=float)
        for sample_index in range(self.num_samples):
            costs[sample_index] = self._rollout_cost(
                q0=q0,
                sequence=candidate_sequences[sample_index],
                previous_command=previous_command,
                target_tip=target,
            )

        beta = float(np.min(costs))
        weights = np.exp(-(costs - beta) / self.temperature)
        weight_sum = float(np.sum(weights))
        if not np.isfinite(weight_sum) or weight_sum <= 0:
            normalized = np.zeros_like(weights)
            normalized[int(np.argmin(costs))] = 1.0
        else:
            normalized = weights / weight_sum

        self.nominal_sequence = np.tensordot(normalized, candidate_sequences, axes=(0, 0))
        command_unclipped = self.nominal_sequence[0].copy()
        command = np.clip(command_unclipped, -self.velocity_max, self.velocity_max)
        command_saturated = not np.allclose(command_unclipped, command)
        self.nominal_sequence[0] = command

        effective_sample_weight = 1.0 / max(float(np.sum(normalized**2)), 1e-12)
        solve_time = perf_counter() - start
        return MPPIResult(
            command=command,
            nominal_sequence=self.nominal_sequence.copy(),
            solve_time=solve_time,
            minimum_cost=beta,
            mean_cost=float(np.mean(costs)),
            effective_sample_weight=effective_sample_weight,
            command_magnitude=float(np.linalg.norm(command)),
            command_saturated=bool(command_saturated),
            diagnostic_status="MPPI Milestone 4: tip/control/smoothness/terminal costs enabled; advanced costs disabled.",
        )

    def _rollout_cost(
        self,
        *,
        q0: np.ndarray,
        sequence: np.ndarray,
        previous_command: np.ndarray,
        target_tip: np.ndarray,
    ) -> float:
        q = q0.copy()
        total = 0.0
        previous = previous_command.copy()
        command_saturated = False

        for command in sequence:
            clipped_command = np.clip(command, -self.velocity_max, self.velocity_max)
            next_q_unclipped = q + self.dt * clipped_command
            q = np.clip(next_q_unclipped, self.q_min, self.q_max)
            command_saturated = command_saturated or not np.allclose(command, clipped_command)
            command_saturated = command_saturated or not np.allclose(next_q_unclipped, q)

            model_result = self._model.forward_kinematics(q)
            total += float(self.weights.get("tip", 0.0)) * tip_tracking_cost(model_result.tip_position, target_tip)
            total += float(self.weights.get("control", 0.0)) * control_magnitude_cost(clipped_command)
            total += float(self.weights.get("smoothness", 0.0)) * control_smoothness_cost(clipped_command, previous)
            total += shape_tracking_cost(enabled=float(self.weights.get("shape", 0.0)) > 0.0)
            total += obstacle_cost(enabled=float(self.weights.get("obstacle", 0.0)) > 0.0)
            total += tactile_cost(enabled=float(self.weights.get("force", 0.0)) > 0.0)
            total += stability_cost(enabled=float(self.weights.get("stability", 0.0)) > 0.0)
            previous = clipped_command

        terminal = self._model.forward_kinematics(q).tip_position
        total += float(self.weights.get("terminal", 0.0)) * terminal_tip_cost(terminal, target_tip)
        if command_saturated:
            total += 1e-12
        return float(total)

    def _validate_disabled_costs(self) -> None:
        disabled = {
            "shape": "TODO-COST-005",
            "obstacle": "TODO-COST-001",
            "force": "TODO-SNS-001",
            "stability": "TODO-COST-006",
        }
        for key, todo_id in disabled.items():
            if float(self.weights.get(key, 0.0)) != 0.0:
                raise NotImplementedError(f"{todo_id}: `{key}` cost must remain disabled in Milestone 4.")


def _array3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array


def _array6(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (6,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 6 finite values")
    return array
