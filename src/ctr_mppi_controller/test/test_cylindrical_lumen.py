import math
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_mppi_controller.cylindrical_lumen import (  # noqa: E402
    CylindricalLumen,
    LumenCostWeights,
    compute_lumen_cost,
    config_with_mppi_profile,
    config_with_cylinder_overrides,
    cylindrical_lumen_enabled,
)


def lumen(**overrides):
    values = {
        "frame_id": "base_link",
        "axis_origin": [0.0, 0.0, 0.0],
        "axis_direction": [0.0, 0.0, 2.0],
        "radius": 0.030,
        "length": 0.120,
        "ctr_outer_radius": 0.0015,
        "safety_margin": 0.0020,
    }
    values.update(overrides)
    return CylindricalLumen(**values)


def terminal_only_weights(weight=200000.0):
    return LumenCostWeights(0.0, 0.0, 0.0, weight)


class CylindricalLumenGeometryTest(unittest.TestCase):
    def test_axis_normalization(self):
        cyl = lumen(axis_direction=[0.0, 0.0, 3.0])
        self.assertTrue(np.allclose([0.0, 0.0, 1.0], cyl.axis_direction))

    def test_point_on_axis(self):
        c = lumen().point_clearance([0.0, 0.0, 0.05])
        self.assertAlmostEqual(0.05, c.axial_position)
        self.assertAlmostEqual(0.0, c.radial_distance)
        self.assertAlmostEqual(0.0285, c.radial_clearance)
        self.assertFalse(c.collision)

    def test_point_inside_wall(self):
        c = lumen().point_clearance([0.010, 0.0, 0.05])
        self.assertAlmostEqual(0.0185, c.radial_clearance)
        self.assertFalse(c.collision)
        self.assertFalse(c.safety_margin_violation)

    def test_point_at_safety_margin(self):
        cyl = lumen()
        c = cyl.point_clearance([cyl.usable_radius - cyl.safety_margin, 0.0, 0.05])
        self.assertAlmostEqual(cyl.safety_margin, c.radial_clearance)
        self.assertFalse(c.safety_margin_violation)

    def test_point_on_collision_boundary(self):
        cyl = lumen()
        c = cyl.point_clearance([cyl.usable_radius, 0.0, 0.05])
        self.assertAlmostEqual(0.0, c.radial_clearance)
        self.assertFalse(c.collision)
        self.assertTrue(c.safety_margin_violation)

    def test_point_outside_radial_wall(self):
        cyl = lumen()
        c = cyl.point_clearance([cyl.usable_radius + 0.001, 0.0, 0.05])
        self.assertTrue(c.radial_collision)
        self.assertTrue(c.collision)

    def test_point_before_inlet(self):
        c = lumen().point_clearance([0.0, 0.0, -0.001])
        self.assertTrue(c.inlet_violation)
        self.assertTrue(c.collision)

    def test_point_after_outlet(self):
        c = lumen().point_clearance([0.0, 0.0, 0.121])
        self.assertTrue(c.outlet_violation)
        self.assertTrue(c.collision)

    def test_arbitrary_cylinder_axis(self):
        cyl = lumen(axis_direction=[1.0, 0.0, 0.0])
        c = cyl.point_clearance([0.05, 0.010, 0.0])
        self.assertAlmostEqual(0.05, c.axial_position)
        self.assertAlmostEqual(0.010, c.radial_distance)

    def test_complete_backbone_clearance(self):
        cyl = lumen()
        result = cyl.backbone_clearance([[0.0, 0.0, 0.0], [0.010, 0.0, 0.05], [0.020, 0.0, 0.10]])
        self.assertEqual(0, result.collision_count)
        self.assertEqual(0, result.safety_margin_violation_count)
        self.assertAlmostEqual(0.0085, result.minimum_radial_clearance)

    def test_closest_point_index(self):
        cyl = lumen()
        result = cyl.backbone_clearance([[0.0, 0.0, 0.0], [0.020, 0.0, 0.05], [0.010, 0.0, 0.10]])
        self.assertEqual(1, result.closest_backbone_point_index)

    def test_nan_and_inf_rejection(self):
        with self.assertRaises(ValueError):
            lumen(axis_origin=[math.nan, 0.0, 0.0])
        with self.assertRaises(ValueError):
            lumen().point_clearance([math.inf, 0.0, 0.0])
        with self.assertRaises(ValueError):
            lumen().backbone_clearance([[0.0, 0.0, 0.0], [math.nan, 0.0, 0.0]])

    def test_invalid_radius_and_length(self):
        with self.assertRaises(ValueError):
            lumen(radius=0.0)
        with self.assertRaises(ValueError):
            lumen(length=-1.0)
        with self.assertRaises(ValueError):
            lumen(radius=0.001, ctr_outer_radius=0.0015)

    def test_invalid_target(self):
        validation = lumen().validate_target([0.04, 0.0, 0.05], frame_id="base_link")
        self.assertFalse(validation.valid)
        self.assertTrue(any("wall" in reason for reason in validation.reasons))

    def test_valid_target(self):
        validation = lumen().validate_target([0.015, 0.005, 0.100], frame_id="base_link")
        self.assertTrue(validation.valid)
        self.assertEqual([], validation.reasons)

    def test_deterministic_output(self):
        cyl = lumen()
        points = [[0.0, 0.0, 0.0], [0.010, 0.0, 0.05]]
        first = cyl.backbone_clearance(points)
        second = cyl.backbone_clearance(points)
        self.assertTrue(np.array_equal(first.radial_clearance, second.radial_clearance))

    def test_nearest_valid_target_suggests_without_mutating(self):
        cyl = lumen()
        requested = np.array([1.0, 0.0, 1.0])
        suggested = cyl.nearest_valid_target(requested)
        validation = cyl.validate_target(suggested)
        self.assertTrue(validation.valid)
        self.assertFalse(np.allclose(requested, suggested))

    def test_lumen_cost_terms_are_ordered_by_violation_severity(self):
        cyl = lumen()
        weights = LumenCostWeights(100.0, 100000.0, 100000.0, 200000.0)
        safe = compute_lumen_cost(lumen=cyl, weights=weights, backbone_points=[[0.0, 0.0, 0.0], [0.010, 0.0, 0.05]])
        near_wall = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[[0.0, 0.0, 0.0], [cyl.usable_radius - 0.001, 0.0, 0.05]],
        )
        penetration = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[[0.0, 0.0, 0.0], [cyl.usable_radius + 0.001, 0.0, 0.05]],
        )
        self.assertEqual(0.0, safe)
        self.assertGreater(near_wall, safe)
        self.assertGreater(penetration, near_wall)

    def test_terminal_surcharge_is_zero_for_valid_final_backbone(self):
        cyl = lumen()
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=terminal_only_weights(),
            backbone_points=[[0.0, 0.0, 0.0], [0.010, 0.0, 0.05], [0.012, 0.0, 0.08]],
            terminal=True,
        )
        self.assertEqual(0.0, cost)

    def test_terminal_surcharge_uses_middle_radial_penetration_when_tip_is_valid(self):
        cyl = lumen()
        weights = terminal_only_weights()
        penetration = 0.003
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[
                [0.0, 0.0, 0.0],
                [cyl.usable_radius + penetration, 0.0, 0.05],
                [0.010, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (penetration / cyl.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_terminal_surcharge_detects_middle_inlet_violation_when_tip_is_valid(self):
        cyl = lumen()
        weights = terminal_only_weights()
        penetration = 0.001
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[
                [0.0, 0.0, 0.02],
                [0.0, 0.0, -penetration],
                [0.010, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (penetration / cyl.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_terminal_surcharge_detects_middle_outlet_violation_when_tip_is_valid(self):
        cyl = lumen()
        weights = terminal_only_weights()
        penetration = 0.001
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[
                [0.0, 0.0, 0.02],
                [0.0, 0.0, cyl.length + penetration],
                [0.010, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (penetration / cyl.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_terminal_surcharge_uses_worst_violation_once(self):
        cyl = lumen()
        weights = terminal_only_weights()
        smaller_penetration = 0.001
        larger_penetration = 0.003
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[
                [cyl.usable_radius + smaller_penetration, 0.0, 0.02],
                [0.0, 0.0, cyl.length + larger_penetration],
                [0.010, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (larger_penetration / cyl.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_terminal_surcharge_still_detects_tip_violation(self):
        cyl = lumen()
        weights = terminal_only_weights()
        penetration = 0.002
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=weights,
            backbone_points=[
                [0.0, 0.0, 0.0],
                [0.010, 0.0, 0.05],
                [cyl.usable_radius + penetration, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (penetration / cyl.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_nonterminal_cost_does_not_apply_terminal_surcharge(self):
        cyl = lumen()
        cost = compute_lumen_cost(
            lumen=cyl,
            weights=terminal_only_weights(),
            backbone_points=[
                [0.0, 0.0, 0.0],
                [cyl.usable_radius + 0.002, 0.0, 0.05],
                [0.010, 0.0, 0.08],
            ],
            terminal=False,
        )
        self.assertEqual(0.0, cost)

    def test_terminal_cost_values_remain_finite(self):
        cyl = lumen()
        weights = terminal_only_weights()
        costs = [
            compute_lumen_cost(
                lumen=cyl,
                weights=weights,
                backbone_points=[[0.0, 0.0, 0.0], [0.010, 0.0, 0.05]],
                terminal=True,
            ),
            compute_lumen_cost(
                lumen=cyl,
                weights=weights,
                backbone_points=[[0.0, 0.0, -0.001], [cyl.usable_radius + 0.002, 0.0, 0.05]],
                terminal=True,
            ),
            compute_lumen_cost(
                lumen=cyl,
                weights=weights,
                backbone_points=[[0.0, 0.0, cyl.length + 0.001], [0.010, 0.0, 0.05]],
                terminal=True,
            ),
        ]
        self.assertTrue(all(math.isfinite(cost) for cost in costs))

    def test_lumen_disabled_configuration_remains_disabled(self):
        config = {"cylindrical_lumen": {"enabled": True}}
        updated = config_with_cylinder_overrides(config, enabled=False)
        self.assertTrue(cylindrical_lumen_enabled(config))
        self.assertFalse(cylindrical_lumen_enabled(updated))

    def test_mppi_profile_applies_local_weight_overrides(self):
        config = {
            "mppi": {
                "num_samples": 1,
                "horizon": 1,
                "dt": 0.1,
                "control_frequency": 10.0,
                "lambda": 0.001,
                "weights": {"tip": 100.0, "terminal": 100.0, "control": 0.5},
                "noise_std": {"insertion": [0.1, 0.1, 0.1], "rotation": [0.1, 0.1, 0.1]},
            },
            "mppi_profiles": {
                "cylinder_fast": {
                    "samples": 36,
                    "horizon": 7,
                    "dt": 0.55,
                    "control_period": 0.1,
                    "noise_std": {"insertion": [0.003] * 3, "rotation": [0.1] * 3},
                    "weights": {"tip": 15000.0, "terminal": 15000.0, "control": 0.005, "smoothness": 0.01},
                }
            },
        }
        updated = config_with_mppi_profile(config, "cylinder_fast")
        self.assertEqual(100.0, config["mppi"]["weights"]["tip"])
        self.assertEqual(36, updated["mppi"]["num_samples"])
        self.assertEqual(7, updated["mppi"]["horizon"])
        self.assertEqual(0.55, updated["mppi"]["dt"])
        self.assertEqual([0.003] * 3, updated["mppi"]["noise_std"]["insertion"])
        self.assertEqual([0.1] * 3, updated["mppi"]["noise_std"]["rotation"])
        self.assertEqual(15000.0, updated["mppi"]["weights"]["tip"])
        self.assertEqual(15000.0, updated["mppi"]["weights"]["terminal"])
        self.assertEqual(0.005, updated["mppi"]["weights"]["control"])
        self.assertEqual(0.01, updated["mppi"]["weights"]["smoothness"])


if __name__ == "__main__":
    unittest.main()
