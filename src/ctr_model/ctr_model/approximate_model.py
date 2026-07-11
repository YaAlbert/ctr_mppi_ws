"""Approximate CTR model used by the Milestone 3 simulation loop.

TODO-MODEL-004: This is a lightweight software scaffold model. It is not a
validated physical CTR model and should be replaced or validated during the
CTR model milestone.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import math
import numpy as np


@dataclass(frozen=True)
class CTRModelResult:
    backbone_points: np.ndarray
    tip_position: np.ndarray
    diagnostic_status: str


class ApproximateCTRModel:
    """Small deterministic model that exposes `forward_kinematics(q)`."""

    def __init__(self, config: dict[str, Any]):
        self._robot = config["robot"]
        self._model = config["model"]
        self._num_points = int(_number(self._model.get("backbone_points", 50)))
        if self._num_points < 2:
            raise ValueError("model.backbone_points must be at least 2")

        tube = self._robot["tube"]
        approximate = self._model.get("approximate", {})
        self._tube_lengths = _array3(tube["length"], "robot.tube.length")
        self._precurvature = _array3(tube["precurvature"], "robot.tube.precurvature")
        self._precurved_length = _array3(tube["precurved_length"], "robot.tube.precurved_length")
        self._curvature_scale = _array3(approximate.get("curvature_scale", [1.0, 1.0, 1.0]), "model.approximate.curvature_scale")
        self._limits = self._robot["limits"]

    def forward_kinematics(self, q: np.ndarray | list[float] | tuple[float, ...]) -> CTRModelResult:
        q_array = np.asarray(q, dtype=float)
        if q_array.shape != (6,):
            raise ValueError("q must have shape (6,)")
        if not np.all(np.isfinite(q_array)):
            raise ValueError("q must contain only finite values")

        insertion_min = _array3(self._limits["insertion_min"], "robot.limits.insertion_min")
        insertion_max = _array3(self._limits["insertion_max"], "robot.limits.insertion_max")
        rotation_min = _array3(self._limits["rotation_min"], "robot.limits.rotation_min")
        rotation_max = _array3(self._limits["rotation_max"], "robot.limits.rotation_max")

        rho = np.clip(q_array[:3], insertion_min, insertion_max)
        theta = np.clip(q_array[3:], rotation_min, rotation_max)

        exposed_length = float(np.clip(np.max(self._precurved_length + rho), 0.01, np.max(self._tube_lengths)))
        s = np.linspace(0.0, exposed_length, self._num_points)

        curvature_components = self._precurvature * self._curvature_scale
        insertion_weight = 1.0 + rho / np.maximum(insertion_max, 1e-9)
        weighted = curvature_components * insertion_weight
        normalizer = max(float(np.sum(insertion_weight)), 1e-9)

        kx = float(np.sum(weighted * np.cos(theta)) / normalizer)
        ky = float(np.sum(weighted * np.sin(theta)) / normalizer)

        # Smoothly increase bending away from the base. This gives RViz and
        # simulation users a stable shape while the real model remains TODO-MODEL-004.
        x = 0.5 * kx * s**2
        y = 0.5 * ky * s**2
        z = s
        backbone = np.column_stack((x, y, z))
        tip = backbone[-1].copy()

        return CTRModelResult(
            backbone_points=backbone,
            tip_position=tip,
            diagnostic_status="TODO-MODEL-004 approximate PCC scaffold model",
        )


def _array3(values: Any, label: str) -> np.ndarray:
    array = np.asarray([_number(value) for value in values], dtype=float)
    if array.shape != (3,):
        raise ValueError(f"{label} must contain 3 numeric values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    return array


def _number(value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError("booleans are not valid numeric parameters")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError("numeric parameter must be finite")
    return numeric
