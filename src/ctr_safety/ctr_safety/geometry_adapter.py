"""Thin safety-facing adapter for the authoritative lumen geometry."""

from __future__ import annotations

from typing import Any

import numpy as np

from ctr_mppi_controller.lumen_factory import lumen_geometry_from_config


class GeometryAdapter:
    """Expose whole-backbone clearance without owning lumen equations."""

    def __init__(self, config: dict[str, Any]):
        self.geometry = lumen_geometry_from_config(config)

    def check_backbone(self, backbone: Any) -> tuple[bool, str, float | None]:
        points = np.asarray(backbone, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
            return False, "state_backbone_dimension", None
        if not np.all(np.isfinite(points)):
            return False, "state_backbone_nonfinite", None
        if self.geometry is None:
            return True, "geometry_disabled", float("inf")
        try:
            result = self.geometry.backbone_clearance(points)
        except (TypeError, ValueError, RuntimeError) as exc:
            return False, f"geometry_error:{type(exc).__name__}", None
        minimum = float(result.minimum_clearance)
        safe_margin = float(result.safety_margin_clear)
        if not np.isfinite(minimum) or not np.isfinite(safe_margin):
            return False, "geometry_nonfinite", None
        if bool(result.safety_margin_violation_mask.any()):
            return False, "whole_backbone_safety_margin", safe_margin
        return True, "geometry_safe", safe_margin
