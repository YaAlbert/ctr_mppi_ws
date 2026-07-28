import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_model.approximate_model import ApproximateCTRModel  # noqa: E402
from ctr_mppi_controller.curved_lumen import CurvedLumen  # noqa: E402
from ctr_mppi_controller.cylindrical_lumen import (  # noqa: E402
    CylindricalLumen,
    lumen_cost_weights_from_config,
    lumen_geometry_from_config,
)
from ctr_mppi_controller.lumen_geometry import LumenCostWeights, compute_lumen_cost  # noqa: E402
from ctr_mppi_controller.mppi_core import MPPICore  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def make_config():
    config = load_parameter_files(CONFIG_FILES)
    validate_or_raise(config)
    config = copy.deepcopy(config)
    tube_count = config["robot"]["number_of_tubes"]
    config["cylindrical_lumen"]["enabled"] = False
    config["curved_lumen"]["enabled"] = False
    config["mppi"]["horizon"] = 3
    config["mppi"]["num_samples"] = 32
    config["mppi"]["lambda"] = 0.01
    config["mppi"]["random_seed"] = 17
    config["mppi"]["noise_std"]["insertion"] = [0.003] * tube_count
    config["mppi"]["noise_std"]["rotation"] = [0.08] * tube_count
    config["mppi"]["weights"]["tip"] = 1.0
    config["mppi"]["weights"]["terminal"] = 1.0
    config["mppi"]["weights"]["control"] = 0.1
    config["mppi"]["weights"]["smoothness"] = 0.1
    return config


def make_curved_config(*, lumen_type="circular_arc"):
    config = make_config()
    config["curved_lumen"]["enabled"] = True
    config["curved_lumen"]["type"] = lumen_type
    return config


def make_core(config, model, *, lumen=None, weights=None):
    if lumen is None:
        lumen = lumen_geometry_from_config(config)
    if weights is None:
        weights = lumen_cost_weights_from_config(config)
    return MPPICore(config, model, lumen_geometry=lumen, lumen_cost_weights=weights)


def straight_curved_lumen(*, safety_margin=0.002):
    return CurvedLumen(
        frame_id="base_link",
        centerline_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.120]],
        lumen_radius=0.030,
        ctr_outer_radius=0.0015,
        safety_margin=safety_margin,
    )


def arc_lumen():
    config = make_curved_config()
    return lumen_geometry_from_config(config)


def centerline_midpoint(lumen):
    return np.asarray(lumen.centerline_points[len(lumen.centerline_points) // 2], dtype=float)


class StaticBackboneModel:
    def __init__(self, backbone):
        self.backbone = np.asarray(backbone, dtype=float)

    def forward_kinematics(self, _q):
        return SimpleNamespace(backbone_points=self.backbone.copy(), tip_position=self.backbone[-1].copy())


class LinearTipModel:
    def forward_kinematics(self, q):
        q_array = np.asarray(q, dtype=float)
        tip = q_array[:3].copy()
        return SimpleNamespace(backbone_points=np.vstack((np.zeros(3), tip)), tip_position=tip)


class CurvedLumenConfigurationTest(unittest.TestCase):
    def test_mppi_accepts_curved_lumen_geometry(self):
        config = make_curved_config()
        lumen = lumen_geometry_from_config(config)
        target = centerline_midpoint(lumen)
        config["goal"]["position"] = target.tolist()
        controller = make_core(config, StaticBackboneModel([target, target]), lumen=lumen)
        result = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=np.zeros((controller.horizon, controller.control_dimension)),
            previous_command=np.zeros(controller.control_dimension),
            target_tip=target,
        )
        self.assertTrue(np.isfinite(result.cost))

    def test_lumen_disabled_preserves_prior_deterministic_output(self):
        config = make_config()
        model = LinearTipModel()
        target = np.array([0.02, 0.0, 0.0])
        first = MPPICore(config, model).solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        second = MPPICore(config, model).solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        self.assertTrue(np.allclose(first.command, second.command))
        self.assertAlmostEqual(first.minimum_cost, second.minimum_cost)

    def test_cylindrical_lumen_preserves_known_cost(self):
        config = make_config()
        config["cylindrical_lumen"]["enabled"] = True
        lumen = CylindricalLumen.from_config(config)
        weights = LumenCostWeights.from_config(config)
        penetration = 0.001
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=weights,
            backbone_points=[[0.0, 0.0, 0.0], [lumen.usable_radius + penetration, 0.0, 0.06]],
            terminal=False,
        )
        denominator = lumen.safety_margin
        expected = (
            weights.safety_margin_weight * (((lumen.safety_margin + penetration) / denominator) ** 2) / 2.0
            + weights.radial_collision_weight * ((penetration / denominator) ** 2) / 2.0
        )
        self.assertAlmostEqual(expected, cost)

    def test_curved_target_inside_lumen_is_accepted(self):
        config = make_curved_config()
        lumen = lumen_geometry_from_config(config)
        target = centerline_midpoint(lumen)
        controller = make_core(config, StaticBackboneModel([target, target]), lumen=lumen)
        sequence = controller._reference_sequence(target_tip=target, target_tip_sequence=None)
        self.assertEqual((controller.horizon, 3), sequence.shape)

    def test_curved_target_outside_wall_is_rejected(self):
        config = make_curved_config()
        lumen = straight_curved_lumen()
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]]), lumen=lumen)
        with self.assertRaisesRegex(ValueError, "outside selected lumen geometry"):
            controller.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=[0.040, 0.0, 0.06])

    def test_curved_target_before_inlet_is_rejected(self):
        config = make_curved_config()
        lumen = straight_curved_lumen()
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]]), lumen=lumen)
        with self.assertRaisesRegex(ValueError, "inlet"):
            controller.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=[0.0, 0.0, -0.001])

    def test_curved_target_after_outlet_is_rejected(self):
        config = make_curved_config()
        lumen = straight_curved_lumen()
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]]), lumen=lumen)
        with self.assertRaisesRegex(ValueError, "outlet"):
            controller.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=[0.0, 0.0, 0.121])

    def test_both_lumen_modes_enabled_is_rejected(self):
        config = make_curved_config()
        config["cylindrical_lumen"]["enabled"] = True
        with self.assertRaisesRegex(ValueError, "exactly one"):
            lumen_geometry_from_config(config)

    def test_unsupported_curved_type_is_rejected(self):
        config = make_curved_config(lumen_type="spiral")
        with self.assertRaisesRegex(ValueError, "curved_lumen.type"):
            lumen_geometry_from_config(config)

    def test_circular_arc_configuration_constructs_deterministic_centerline(self):
        config = make_curved_config(lumen_type="circular_arc")
        first = lumen_geometry_from_config(config)
        second = lumen_geometry_from_config(config)
        self.assertTrue(np.array_equal(first.centerline_points, second.centerline_points))

    def test_s_curve_configuration_constructs_deterministic_centerline(self):
        config = make_curved_config(lumen_type="s_curve")
        first = lumen_geometry_from_config(config)
        second = lumen_geometry_from_config(config)
        self.assertTrue(np.array_equal(first.centerline_points, second.centerline_points))


class CurvedLumenCostTest(unittest.TestCase):
    def setUp(self):
        self.weights = LumenCostWeights(100.0, 100000.0, 100000.0, 200000.0)

    def test_running_cost_evaluates_every_curved_backbone_point(self):
        config = make_curved_config()
        config["mppi"]["weights"].update({"tip": 0.0, "terminal": 0.0, "control": 0.0, "smoothness": 0.0})
        lumen = straight_curved_lumen()
        safe_tip = [0.0, 0.0, 0.08]
        colliding_middle = [lumen.minimum_usable_radius + 0.002, 0.0, 0.06]
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], colliding_middle, safe_tip]), lumen=lumen)
        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=np.zeros((controller.horizon, controller.control_dimension)),
            previous_command=np.zeros(controller.control_dimension),
            target_tip=safe_tip,
        )
        self.assertGreater(rollout.cost, 0.0)

    def test_valid_tip_with_middle_wall_penetration_receives_hard_cost(self):
        lumen = straight_curved_lumen()
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[0.0, 0.0, 0.0], [lumen.minimum_usable_radius + 0.001, 0.0, 0.06], [0.0, 0.0, 0.08]],
        )
        self.assertGreater(cost, self.weights.safety_margin_weight)

    def test_valid_tip_with_middle_inlet_violation_receives_hard_cost(self):
        lumen = straight_curved_lumen()
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[0.0, 0.0, 0.02], [0.0, 0.0, -0.001], [0.0, 0.0, 0.08]],
        )
        self.assertGreater(cost, 0.0)

    def test_valid_tip_with_middle_outlet_violation_receives_hard_cost(self):
        lumen = straight_curved_lumen()
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[0.0, 0.0, 0.02], [0.0, 0.0, 0.121], [0.0, 0.0, 0.08]],
        )
        self.assertGreater(cost, 0.0)

    def test_near_wall_backbone_receives_soft_safety_margin_cost(self):
        lumen = straight_curved_lumen()
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[0.0, 0.0, 0.0], [lumen.minimum_usable_radius - 0.001, 0.0, 0.06]],
        )
        self.assertGreater(cost, 0.0)
        self.assertLess(cost, self.weights.radial_collision_weight)

    def test_collision_free_centerline_backbone_has_no_hard_collision_cost(self):
        lumen = straight_curved_lumen()
        weights = LumenCostWeights(0.0, 100000.0, 100000.0, 0.0)
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=weights,
            backbone_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.06], [0.0, 0.0, 0.10]],
        )
        self.assertEqual(0.0, cost)

    def test_terminal_valid_tip_with_middle_collision_receives_surcharge(self):
        lumen = arc_lumen()
        tip = centerline_midpoint(lumen)
        middle = tip + np.array([0.0, lumen.minimum_usable_radius + 0.002, 0.0])
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=LumenCostWeights(0.0, 0.0, 0.0, 200000.0),
            backbone_points=[lumen.centerline_points[0], middle, tip],
            terminal=True,
        )
        self.assertGreater(cost, 0.0)

    def test_terminal_surcharge_uses_worst_complete_backbone_violation(self):
        lumen = straight_curved_lumen()
        weights = LumenCostWeights(0.0, 0.0, 0.0, 200000.0)
        smaller = 0.001
        larger = 0.003
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=weights,
            backbone_points=[
                [lumen.minimum_usable_radius + smaller, 0.0, 0.02],
                [0.0, 0.0, 0.120 + larger],
                [0.0, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (larger / lumen.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_terminal_surcharge_is_applied_once(self):
        lumen = straight_curved_lumen()
        weights = LumenCostWeights(0.0, 0.0, 0.0, 200000.0)
        penetration = 0.002
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=weights,
            backbone_points=[
                [lumen.minimum_usable_radius + penetration, 0.0, 0.02],
                [lumen.minimum_usable_radius + penetration, 0.0, 0.06],
                [0.0, 0.0, 0.08],
            ],
            terminal=True,
        )
        expected = weights.terminal_collision_weight * (penetration / lumen.safety_margin) ** 2
        self.assertAlmostEqual(expected, cost)

    def test_physical_wall_contact_has_existing_nonnegative_convention(self):
        lumen = straight_curved_lumen()
        clearance = lumen.point_clearance([lumen.minimum_usable_radius, 0.0, 0.06])
        self.assertAlmostEqual(0.0, clearance.physical_clearance)
        self.assertFalse(clearance.collision)
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=LumenCostWeights(0.0, 100000.0, 0.0, 0.0),
            backbone_points=[[lumen.minimum_usable_radius, 0.0, 0.06]],
        )
        self.assertEqual(0.0, cost)

    def test_zero_safety_margin_does_not_divide_by_zero(self):
        lumen = straight_curved_lumen(safety_margin=0.0)
        cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[lumen.minimum_usable_radius + 1.0e-6, 0.0, 0.06]],
            terminal=True,
        )
        self.assertTrue(np.isfinite(cost))

    def test_all_costs_remain_finite(self):
        lumen = straight_curved_lumen()
        points = [
            [0.0, 0.0, -0.001],
            [lumen.minimum_usable_radius + 0.001, 0.0, 0.06],
            [0.0, 0.0, 0.121],
        ]
        for terminal in (False, True):
            self.assertTrue(np.isfinite(compute_lumen_cost(lumen=lumen, weights=self.weights, backbone_points=points, terminal=terminal)))

    def test_goal_cost_remains_active_with_curved_lumen(self):
        config = make_curved_config()
        config["mppi"]["weights"].update({"tip": 10.0, "terminal": 0.0, "control": 0.0, "smoothness": 0.0})
        lumen = straight_curved_lumen()
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], [0.0, 0.0, 0.03]]), lumen=lumen)
        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=np.zeros((controller.horizon, controller.control_dimension)),
            previous_command=np.zeros(controller.control_dimension),
            target_tip=[0.0, 0.0, 0.06],
        )
        self.assertGreater(rollout.cost, 0.0)

    def test_control_cost_remains_active_with_curved_lumen(self):
        config = make_curved_config()
        config["mppi"]["weights"].update({"tip": 0.0, "terminal": 0.0, "control": 1.0, "smoothness": 0.0})
        lumen = straight_curved_lumen()
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], [0.0, 0.0, 0.03]]), lumen=lumen)
        sequence = np.ones((controller.horizon, controller.control_dimension)) * 0.01
        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=sequence,
            previous_command=np.zeros(controller.control_dimension),
            target_tip=[0.0, 0.0, 0.03],
        )
        self.assertGreater(rollout.cost, 0.0)

    def test_smoothness_cost_remains_active_with_curved_lumen(self):
        config = make_curved_config()
        config["mppi"]["weights"].update({"tip": 0.0, "terminal": 0.0, "control": 0.0, "smoothness": 1.0})
        lumen = straight_curved_lumen()
        controller = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], [0.0, 0.0, 0.03]]), lumen=lumen)
        sequence = np.zeros((controller.horizon, controller.control_dimension))
        sequence[0, 0] = 0.01
        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=sequence,
            previous_command=np.zeros(controller.control_dimension),
            target_tip=[0.0, 0.0, 0.03],
        )
        self.assertGreater(rollout.cost, 0.0)

    def test_collision_cost_changes_candidate_rollout_preference(self):
        config = make_curved_config()
        config["mppi"]["weights"].update({"tip": 0.0, "terminal": 0.0, "control": 0.0, "smoothness": 0.0})
        lumen = straight_curved_lumen()
        target = [0.0, 0.0, 0.08]
        safe = make_core(config, StaticBackboneModel([[0.0, 0.0, 0.0], target]), lumen=lumen)
        colliding = make_core(
            config,
            StaticBackboneModel([[0.0, 0.0, 0.0], [lumen.minimum_usable_radius + 0.002, 0.0, 0.06], target]),
            lumen=lumen,
        )
        sequence = np.zeros((safe.horizon, safe.control_dimension))
        safe_cost = safe.rollout_candidate(
            q0=np.zeros(6),
            sequence=sequence,
            previous_command=np.zeros(6),
            target_tip=target,
        ).cost
        colliding_cost = colliding.rollout_candidate(
            q0=np.zeros(6),
            sequence=sequence,
            previous_command=np.zeros(6),
            target_tip=target,
        ).cost
        self.assertLess(safe_cost, colliding_cost)

    def test_safe_rollout_ranks_better_than_similar_colliding_rollout(self):
        lumen = straight_curved_lumen()
        safe_cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.08]],
        )
        colliding_cost = compute_lumen_cost(
            lumen=lumen,
            weights=self.weights,
            backbone_points=[[0.0, 0.0, 0.0], [lumen.minimum_usable_radius + 0.002, 0.0, 0.06], [0.0, 0.0, 0.08]],
        )
        self.assertLess(safe_cost, colliding_cost)

    def test_fixed_seed_remains_deterministic_with_curved_lumen(self):
        config = make_curved_config()
        lumen = lumen_geometry_from_config(config)
        target = centerline_midpoint(lumen)
        model = ApproximateCTRModel(config)
        first = make_core(config, model, lumen=lumen).solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        second = make_core(config, ApproximateCTRModel(config), lumen=lumen).solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        self.assertTrue(np.allclose(first.command, second.command))
        self.assertAlmostEqual(first.minimum_cost, second.minimum_cost)


if __name__ == "__main__":
    unittest.main()
