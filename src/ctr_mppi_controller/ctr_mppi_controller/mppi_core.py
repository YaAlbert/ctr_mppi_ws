"""ROS-independent MPPI controller core."""

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
from .lumen_geometry import LumenCostWeights, LumenGeometry, compute_lumen_cost


REQUIRED_WEIGHT_KEYS = (
    "tip",
    "shape",
    "control",
    "smoothness",
    "obstacle",
    "terminal",
    "force",
    "joint_limit",
    "stability",
)


@dataclass(frozen=True)
class MPPIConfig:
    dt: float
    horizon: int
    num_samples: int
    temperature: float
    warm_start: bool
    shift_previous_solution: bool
    weights: dict[str, float]
    noise_std: np.ndarray
    q_min: np.ndarray
    q_max: np.ndarray
    velocity_max: np.ndarray
    control_dimension: int
    random_seed: int

    @classmethod
    def from_project_config(cls, config: dict[str, Any]) -> "MPPIConfig":
        robot = config["robot"]
        mppi = config["mppi"]
        limits = robot["limits"]
        weight_config = mppi["weights"]

        tube_count = _positive_int(robot["number_of_tubes"], "robot.number_of_tubes")
        control_dimension = 2 * tube_count
        missing_weights = [key for key in REQUIRED_WEIGHT_KEYS if key not in weight_config]
        if missing_weights:
            raise ValueError(f"mppi.weights missing required keys: {missing_weights}")
        weights = {key: _number(value, f"mppi.weights.{key}") for key, value in weight_config.items()}

        q_min = np.concatenate(
            (
                _array(limits["insertion_min"], "robot.limits.insertion_min", tube_count),
                _array(limits["rotation_min"], "robot.limits.rotation_min", tube_count),
            )
        )
        q_max = np.concatenate(
            (
                _array(limits["insertion_max"], "robot.limits.insertion_max", tube_count),
                _array(limits["rotation_max"], "robot.limits.rotation_max", tube_count),
            )
        )
        velocity_max = np.concatenate(
            (
                _array(limits["insertion_velocity_max"], "robot.limits.insertion_velocity_max", tube_count),
                _array(limits["rotation_velocity_max"], "robot.limits.rotation_velocity_max", tube_count),
            )
        )
        noise_std = np.concatenate(
            (
                _array(mppi["noise_std"]["insertion"], "mppi.noise_std.insertion", tube_count),
                _array(mppi["noise_std"]["rotation"], "mppi.noise_std.rotation", tube_count),
            )
        )

        if np.any(q_max < q_min):
            raise ValueError("joint position maximum limits must be greater than or equal to minimum limits")
        if np.any(velocity_max <= 0.0):
            raise ValueError("joint velocity limits must be positive")
        if np.any(noise_std < 0.0):
            raise ValueError("mppi.noise_std values must be non-negative")

        dt = _positive_number(mppi["dt"], "mppi.dt")
        horizon = _positive_int(mppi["horizon"], "mppi.horizon")
        num_samples = _positive_int(mppi["num_samples"], "mppi.num_samples")
        temperature = _positive_number(mppi["lambda"], "mppi.lambda")
        random_seed = _nonnegative_int(mppi["random_seed"], "mppi.random_seed")

        return cls(
            dt=dt,
            horizon=horizon,
            num_samples=num_samples,
            temperature=temperature,
            warm_start=bool(mppi.get("warm_start", True)),
            shift_previous_solution=bool(mppi.get("shift_previous_solution", True)),
            weights=weights,
            noise_std=noise_std,
            q_min=q_min,
            q_max=q_max,
            velocity_max=velocity_max,
            control_dimension=control_dimension,
            random_seed=random_seed,
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


@dataclass(frozen=True)
class MPPIRollout:
    cost: float
    final_q: np.ndarray
    final_tip: np.ndarray
    command_saturated: bool


class MPPICore:
    """A small MPPI optimizer independent of ROS2."""

    def __init__(
        self,
        config: dict[str, Any],
        model: Any,
        *,
        lumen_geometry: LumenGeometry | None = None,
        lumen_cost_weights: LumenCostWeights | None = None,
    ):
        self._config = config
        self._model = model

        self.mppi_config = MPPIConfig.from_project_config(config)
        self.dt = self.mppi_config.dt
        self.horizon = self.mppi_config.horizon
        self.num_samples = self.mppi_config.num_samples
        self.temperature = self.mppi_config.temperature
        self.warm_start = self.mppi_config.warm_start
        self.shift_previous_solution = self.mppi_config.shift_previous_solution
        self.weights = self.mppi_config.weights
        self.noise_std = self.mppi_config.noise_std
        self.q_min = self.mppi_config.q_min
        self.q_max = self.mppi_config.q_max
        self.velocity_max = self.mppi_config.velocity_max
        self.control_dimension = self.mppi_config.control_dimension

        self.nominal_sequence = np.zeros((self.horizon, self.control_dimension), dtype=float)
        self.rng = np.random.default_rng(self.mppi_config.random_seed)
        self.last_candidate_sequences = np.zeros((0, self.horizon, self.control_dimension), dtype=float)
        self.last_costs = np.zeros(0, dtype=float)
        self.last_normalized_weights = np.zeros(0, dtype=float)
        self.last_rollout_final_q = np.zeros((0, self.control_dimension), dtype=float)
        self.lumen = lumen_geometry
        self.lumen_cost_weights = None
        if self.lumen is not None:
            self.lumen_cost_weights = lumen_cost_weights or LumenCostWeights.from_config(config)
        self._validate_disabled_costs()

    def reset(self) -> None:
        self.nominal_sequence.fill(0.0)

    def sample_control_noise(self) -> np.ndarray:
        noise = self.rng.normal(
            loc=0.0,
            scale=self.noise_std,
            size=(self.num_samples, self.horizon, self.control_dimension),
        )
        if not np.all(np.isfinite(noise)):
            raise ValueError("sampled MPPI control noise contains non-finite values")
        noise[0, :, :] = 0.0
        return noise

    def candidate_sequences(self, perturbations: np.ndarray) -> np.ndarray:
        perturbation_array = _array_shape(
            perturbations,
            "perturbations",
            (self.num_samples, self.horizon, self.control_dimension),
        )
        candidates = np.clip(
            self.nominal_sequence[None, :, :] + perturbation_array,
            -self.velocity_max,
            self.velocity_max,
        )
        if not np.all(np.isfinite(candidates)):
            raise ValueError("candidate control sequences contain non-finite values")
        return candidates

    def rollout_candidate(
        self,
        *,
        q0: np.ndarray | list[float] | tuple[float, ...],
        sequence: np.ndarray,
        previous_command: np.ndarray | list[float] | tuple[float, ...],
        target_tip: np.ndarray | list[float] | tuple[float, ...] | None = None,
        target_tip_sequence: np.ndarray | list[list[float]] | tuple[tuple[float, ...], ...] | None = None,
    ) -> MPPIRollout:
        q = _array_shape(q0, "q0", (self.control_dimension,)).copy()
        sequence_array = _array_shape(sequence, "sequence", (self.horizon, self.control_dimension))
        previous = _array_shape(previous_command, "previous_command", (self.control_dimension,)).copy()
        reference_sequence = self._reference_sequence(target_tip=target_tip, target_tip_sequence=target_tip_sequence)

        total = 0.0
        command_saturated = False
        final_model_result = None

        for step_index, command in enumerate(sequence_array):
            clipped_command = np.clip(command, -self.velocity_max, self.velocity_max)
            next_q_unclipped = q + self.dt * clipped_command
            q = np.clip(next_q_unclipped, self.q_min, self.q_max)
            command_saturated = command_saturated or not np.allclose(command, clipped_command)
            command_saturated = command_saturated or not np.allclose(next_q_unclipped, q)

            model_result = self._validated_model_result(q)
            final_model_result = model_result
            total += self._weight("tip") * tip_tracking_cost(model_result.tip_position, reference_sequence[step_index])
            total += self._weight("control") * control_magnitude_cost(clipped_command)
            total += self._weight("smoothness") * control_smoothness_cost(clipped_command, previous)
            total += self._lumen_cost(model_result.backbone_points, terminal=False)
            total += shape_tracking_cost(enabled=self._weight("shape") > 0.0)
            total += obstacle_cost(enabled=self._weight("obstacle") > 0.0)
            total += tactile_cost(enabled=self._weight("force") > 0.0)
            total += stability_cost(enabled=self._weight("stability") > 0.0)
            previous = clipped_command

        if final_model_result is None:
            raise ValueError("rollout sequence is empty")
        terminal_result = self._validated_model_result(q)
        terminal = terminal_result.tip_position
        total += self._weight("terminal") * terminal_tip_cost(terminal, reference_sequence[-1])
        total += self._lumen_cost(terminal_result.backbone_points, terminal=True)
        if not np.isfinite(total):
            raise ValueError("rollout cost is not finite")
        return MPPIRollout(
            cost=float(total),
            final_q=q.copy(),
            final_tip=terminal.copy(),
            command_saturated=bool(command_saturated),
        )

    def normalized_importance_weights(self, costs: np.ndarray) -> np.ndarray:
        cost_array = _array_shape(costs, "costs", (self.num_samples,))
        beta = float(np.min(cost_array))
        scaled = -(cost_array - beta) / self.temperature
        weights = np.exp(scaled)
        weight_sum = float(np.sum(weights))
        if not np.isfinite(weight_sum) or weight_sum <= 0.0:
            normalized = np.zeros_like(weights)
            normalized[int(np.argmin(cost_array))] = 1.0
        else:
            normalized = weights / weight_sum
        if not np.all(np.isfinite(normalized)):
            raise ValueError("normalized MPPI weights contain non-finite values")
        return normalized

    def solve(
        self,
        *,
        q: np.ndarray | list[float] | tuple[float, ...],
        q_dot: np.ndarray | list[float] | tuple[float, ...],
        target_tip: np.ndarray | list[float] | tuple[float, ...] | None = None,
        target_tip_sequence: np.ndarray | list[list[float]] | tuple[tuple[float, ...], ...] | None = None,
    ) -> MPPIResult:
        start = perf_counter()
        q0 = _array_shape(q, "q", (self.control_dimension,))
        previous_command = _array_shape(q_dot, "q_dot", (self.control_dimension,))
        reference_sequence = self._reference_sequence(target_tip=target_tip, target_tip_sequence=target_tip_sequence)

        if self.warm_start and self.shift_previous_solution:
            self.nominal_sequence[:-1] = self.nominal_sequence[1:]
            self.nominal_sequence[-1] = 0.0
        elif not self.warm_start:
            self.nominal_sequence.fill(0.0)

        perturbations = self.sample_control_noise()
        candidate_sequences = self.candidate_sequences(perturbations)

        costs = np.zeros(self.num_samples, dtype=float)
        final_q = np.zeros((self.num_samples, self.control_dimension), dtype=float)
        for sample_index in range(self.num_samples):
            rollout = self.rollout_candidate(
                q0=q0,
                sequence=candidate_sequences[sample_index],
                previous_command=previous_command,
                target_tip_sequence=reference_sequence,
            )
            costs[sample_index] = rollout.cost
            final_q[sample_index] = rollout.final_q

        normalized = self.normalized_importance_weights(costs)

        self.nominal_sequence = np.tensordot(normalized, candidate_sequences, axes=(0, 0))
        if not np.all(np.isfinite(self.nominal_sequence)):
            raise ValueError("updated nominal control sequence contains non-finite values")
        command_unclipped = self.nominal_sequence[0].copy()
        command = np.clip(command_unclipped, -self.velocity_max, self.velocity_max)
        command_saturated = not np.allclose(command_unclipped, command)
        self.nominal_sequence[0] = command

        self.last_candidate_sequences = candidate_sequences.copy()
        self.last_costs = costs.copy()
        self.last_normalized_weights = normalized.copy()
        self.last_rollout_final_q = final_q.copy()

        effective_sample_weight = 1.0 / max(float(np.sum(normalized**2)), 1e-12)
        solve_time = perf_counter() - start
        return MPPIResult(
            command=command,
            nominal_sequence=self.nominal_sequence.copy(),
            solve_time=solve_time,
            minimum_cost=float(np.min(costs)),
            mean_cost=float(np.mean(costs)),
            effective_sample_weight=effective_sample_weight,
            command_magnitude=float(np.linalg.norm(command)),
            command_saturated=bool(command_saturated),
            diagnostic_status=self._diagnostic_status(),
        )

    def _rollout_cost(
        self,
        *,
        q0: np.ndarray,
        sequence: np.ndarray,
        previous_command: np.ndarray,
        target_tip: np.ndarray | None = None,
        target_tip_sequence: np.ndarray | None = None,
    ) -> float:
        return self.rollout_candidate(
            q0=q0,
            sequence=sequence,
            previous_command=previous_command,
            target_tip=target_tip,
            target_tip_sequence=target_tip_sequence,
        ).cost

    def _reference_sequence(
        self,
        *,
        target_tip: np.ndarray | list[float] | tuple[float, ...] | None,
        target_tip_sequence: np.ndarray | list[list[float]] | tuple[tuple[float, ...], ...] | None,
    ) -> np.ndarray:
        if target_tip is None and target_tip_sequence is None:
            raise ValueError("exactly one of target_tip or target_tip_sequence must be provided")
        if target_tip is not None and target_tip_sequence is not None:
            raise ValueError("exactly one of target_tip or target_tip_sequence must be provided")
        if target_tip is not None:
            target = _array_shape(target_tip, "target_tip", (3,))
            sequence = np.tile(target, (self.horizon, 1))
        else:
            sequence = _array_shape(target_tip_sequence, "target_tip_sequence", (self.horizon, 3)).copy()
        self._validate_reference_inside_lumen(sequence)
        return sequence

    def _validate_disabled_costs(self) -> None:
        disabled = {
            "shape": "TODO-COST-005",
            "obstacle": "TODO-COST-001",
            "force": "TODO-SNS-001",
            "stability": "TODO-COST-006",
        }
        for key, todo_id in disabled.items():
            if self._weight(key) != 0.0:
                raise NotImplementedError(f"{todo_id}: `{key}` cost must remain disabled in Milestone 4.")

    def _validated_model_result(self, q: np.ndarray):
        result = self._model.forward_kinematics(q)
        backbone = np.asarray(result.backbone_points, dtype=float)
        tip = np.asarray(result.tip_position, dtype=float)
        if backbone.ndim != 2 or backbone.shape[1] != 3:
            raise ValueError("model backbone output must have shape (N, 3)")
        if tip.shape != (3,):
            raise ValueError("model tip output must have shape (3,)")
        if not np.all(np.isfinite(backbone)) or not np.all(np.isfinite(tip)):
            raise ValueError("model output contains non-finite values")
        return result

    def _validate_reference_inside_lumen(self, reference_sequence: np.ndarray) -> None:
        if self.lumen is None:
            return
        for index, point in enumerate(reference_sequence):
            validation = self.lumen.validate_target(point, frame_id=self._config.get("goal", {}).get("frame_id"), require_safety_margin=True)
            if not validation.valid:
                raise ValueError(f"target_tip_sequence[{index}] is outside selected lumen geometry: {validation.reasons}")

    def _lumen_cost(self, backbone_points: np.ndarray, *, terminal: bool) -> float:
        if self.lumen is None or self.lumen_cost_weights is None:
            return 0.0
        return compute_lumen_cost(
            lumen=self.lumen,
            weights=self.lumen_cost_weights,
            backbone_points=backbone_points,
            terminal=terminal,
        )

    def _diagnostic_status(self) -> str:
        costs = "tip/control/smoothness/terminal"
        if self.lumen is not None:
            costs += "/lumen"
        return f"MPPI controller: {costs} costs enabled; unsupported advanced costs disabled."

    def _weight(self, name: str) -> float:
        return float(self.weights.get(name, 0.0))


def _array(values: Any, label: str, size: int) -> np.ndarray:
    return _array_shape(values, label, (size,))


def _array_shape(values: Any, label: str, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must have shape {shape} and contain finite values")
    return array


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not np.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _positive_number(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_number(value, label)
    integer = int(numeric)
    if integer != numeric:
        raise ValueError(f"{label} must be an integer")
    return integer


def _nonnegative_int(value: Any, label: str) -> int:
    numeric = _number(value, label)
    integer = int(numeric)
    if integer != numeric or integer < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return integer
