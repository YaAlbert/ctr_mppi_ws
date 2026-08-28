import copy
import gc
import json
import sys
import tracemalloc
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_model.approximate_model import ApproximateCTRModel  # noqa: E402
from ctr_mppi_controller.lumen_factory import (  # noqa: E402
    config_with_mppi_profile,
    config_with_lumen_overrides,
    lumen_cost_weights_from_config,
    lumen_geometry_from_config,
)
from ctr_mppi_controller.lumen_geometry import compute_lumen_cost_breakdown  # noqa: E402
from ctr_mppi_controller.mppi_core import MPPICore  # noqa: E402
from ctr_mppi_controller.tactile_cost import REGION_NO_CONTACT, snapshot_from_values  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / name
    for name in (
        "robot_params.yaml",
        "model_params.yaml",
        "mppi_params.yaml",
        "simulation_params.yaml",
        "safety_params.yaml",
        "tactile_params.yaml",
        "hardware_params.yaml",
    )
]


def make_config(*, optimized: bool, diagnostics_size: bool = False):
    config = load_parameter_files(CONFIG_FILES)
    validate_or_raise(config)
    config = config_with_lumen_overrides(
        config,
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type="circular_arc",
        cylinder_profile="cylinder_fast",
        random_seed=19,
    )
    config["mppi"]["behavior_preserving_optimization_enabled"] = optimized
    if not diagnostics_size:
        config["mppi"]["num_samples"] = 12
        config["mppi"]["horizon"] = 4
    return config


def make_core(*, optimized: bool, diagnostics: bool = True, diagnostics_size: bool = False):
    config = make_config(optimized=optimized, diagnostics_size=diagnostics_size)
    return MPPICore(
        config,
        ApproximateCTRModel(config),
        lumen_geometry=lumen_geometry_from_config(config),
        lumen_cost_weights=lumen_cost_weights_from_config(config),
        evaluation_diagnostics_enabled=diagnostics,
    )


def no_contact_snapshot():
    return snapshot_from_values(
        timestamp_s=1.0,
        frame_id="base_link",
        source="simulated",
        valid=True,
        contact=False,
        warning=False,
        stop=False,
        region=REGION_NO_CONTACT,
        force_magnitude_n=0.0,
    )


def assert_result_equal(test, reference, optimized, reference_core, optimized_core):
    np.testing.assert_array_equal(reference.command, optimized.command)
    np.testing.assert_array_equal(reference.nominal_sequence, optimized.nominal_sequence)
    np.testing.assert_array_equal(reference_core.last_candidate_sequences, optimized_core.last_candidate_sequences)
    np.testing.assert_array_equal(reference_core.last_costs, optimized_core.last_costs)
    np.testing.assert_array_equal(reference_core.last_normalized_weights, optimized_core.last_normalized_weights)
    np.testing.assert_array_equal(reference_core.last_rollout_final_q, optimized_core.last_rollout_final_q)
    test.assertEqual(reference.minimum_cost, optimized.minimum_cost)
    test.assertEqual(reference.mean_cost, optimized.mean_cost)
    test.assertEqual(reference.effective_sample_weight, optimized.effective_sample_weight)
    test.assertEqual(reference.command_saturated, optimized.command_saturated)
    test.assertEqual(reference.tactile_cost, optimized.tactile_cost)
    if np.isnan(reference.tactile_minimum_predicted_clearance_m):
        test.assertTrue(np.isnan(optimized.tactile_minimum_predicted_clearance_m))
    else:
        test.assertEqual(
            reference.tactile_minimum_predicted_clearance_m,
            optimized.tactile_minimum_predicted_clearance_m,
        )
    test.assertEqual(reference.evaluation_diagnostics.best_raw_terms, optimized.evaluation_diagnostics.best_raw_terms)
    test.assertEqual(reference.evaluation_diagnostics.best_weighted_terms, optimized.evaluation_diagnostics.best_weighted_terms)
    test.assertEqual(reference.evaluation_diagnostics.weighted_mean_terms, optimized.evaluation_diagnostics.weighted_mean_terms)


class MPPIPerformanceEquivalenceTest(unittest.TestCase):
    def test_fast_lumen_components_match_reference_cost_and_decisions(self):
        config = make_config(optimized=True)
        lumen = lumen_geometry_from_config(config)
        weights = lumen_cost_weights_from_config(config)
        rng = np.random.default_rng(181)
        model = ApproximateCTRModel(config)
        backbones = [model.forward_kinematics(np.zeros(6)).backbone_points]
        for _ in range(40):
            q = np.concatenate((rng.uniform(0.0, 0.1, 3), rng.uniform(-3.0, 3.0, 3)))
            backbones.append(model.forward_kinematics(q).backbone_points)
        backbones.extend(
            [
                lumen.centerline_points[:50],
                lumen.centerline_points[-50:],
            ]
        )
        for backbone in backbones:
            reference = compute_lumen_cost_breakdown(
                lumen=lumen, weights=weights, backbone_points=backbone, terminal=True
            )
            optimized = compute_lumen_cost_breakdown(
                lumen=lumen,
                weights=weights,
                backbone_points=backbone,
                terminal=True,
                optimized=True,
            )
            self.assertEqual(reference, optimized)
            clearance = lumen.backbone_clearance(backbone)
            physical, wall, inlet, outlet = lumen.cost_clearance_components(backbone)
            np.testing.assert_array_equal(clearance.physical_clearances, physical)
            np.testing.assert_array_equal(clearance.wall_penetrations, wall)
            np.testing.assert_array_equal(clearance.inlet_penetrations, inlet)
            np.testing.assert_array_equal(clearance.outlet_penetrations, outlet)
            np.testing.assert_array_equal(clearance.radial_collision_mask, physical < 0.0)
            np.testing.assert_array_equal(
                clearance.safety_margin_violation_mask,
                (physical < lumen.safety_margin) | (inlet > 0.0) | (outlet > 0.0),
            )

    def test_reference_and_optimized_paths_are_bitwise_equivalent_across_horizon(self):
        reference_core = make_core(optimized=False)
        optimized_core = make_core(optimized=True)
        target = np.array([0.0166457424, 0.00397477634, 0.102231139], dtype=float)
        snapshot = no_contact_snapshot()
        for index in range(8):
            q = np.array(
                [0.001 * index, 0.0004 * index, 0.0002 * index, 0.02 * index, -0.01 * index, 0.015 * index],
                dtype=float,
            )
            q_dot = np.array([0.0001, 0.0002, 0.0001, 0.01, -0.01, 0.005], dtype=float)
            reference = reference_core.solve(q=q, q_dot=q_dot, target_tip=target, tactile_snapshot=snapshot)
            optimized = optimized_core.solve(q=q, q_dot=q_dot, target_tip=target, tactile_snapshot=snapshot)
            assert_result_equal(self, reference, optimized, reference_core, optimized_core)
            self.assertEqual(
                json.dumps(reference_core.rng.bit_generator.state, sort_keys=True),
                json.dumps(optimized_core.rng.bit_generator.state, sort_keys=True),
            )

    def test_diagnostics_do_not_change_optimized_commands_or_rng(self):
        enabled = make_core(optimized=True, diagnostics=True)
        disabled = make_core(optimized=True, diagnostics=False)
        target = np.array([0.0166457424, 0.00397477634, 0.102231139], dtype=float)
        for index in range(5):
            q = np.array([0.0002 * index] * 3 + [0.01 * index, -0.005 * index, 0.002 * index])
            q_dot = np.zeros(6)
            with_diagnostics = enabled.solve(q=q, q_dot=q_dot, target_tip=target)
            without_diagnostics = disabled.solve(q=q, q_dot=q_dot, target_tip=target)
            np.testing.assert_array_equal(with_diagnostics.command, without_diagnostics.command)
            np.testing.assert_array_equal(enabled.last_costs, disabled.last_costs)
            self.assertEqual(
                json.dumps(enabled.rng.bit_generator.state, sort_keys=True),
                json.dumps(disabled.rng.bit_generator.state, sort_keys=True),
            )

    def test_reference_cache_is_detached_and_revalidates_new_objects(self):
        core = make_core(optimized=True)
        sequence = np.tile(np.array([0.0166457424, 0.00397477634, 0.102231139]), (core.horizon, 1))
        original = sequence.copy()
        core.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip_sequence=sequence)
        cached = core._validated_reference_sequence_cache
        self.assertIsNot(cached, sequence)
        self.assertFalse(np.shares_memory(cached, sequence))
        sequence[:] = 1.0
        np.testing.assert_array_equal(cached, original)
        with self.assertRaisesRegex(ValueError, "outside selected lumen"):
            core.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip_sequence=sequence)

    def test_optimized_path_has_bounded_memory_growth(self):
        core = make_core(optimized=True)
        target = np.array([0.0166457424, 0.00397477634, 0.102231139], dtype=float)
        for _ in range(3):
            core.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        gc.collect()
        tracemalloc.start()
        before, _ = tracemalloc.get_traced_memory()
        for _ in range(20):
            core.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        gc.collect()
        after, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        self.assertLess(after - before, 2_000_000)
        self.assertLess(peak, 16_000_000)

    def test_optimization_flag_requires_exact_boolean(self):
        config = make_config(optimized=False)
        config["mppi"]["behavior_preserving_optimization_enabled"] = 1
        with self.assertRaisesRegex(ValueError, "must be boolean"):
            MPPICore(config, ApproximateCTRModel(config))

    def test_baseline_normalized_profile_is_bitwise_equivalent(self):
        raw = load_parameter_files(CONFIG_FILES)
        reference_config = config_with_lumen_overrides(
            raw,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type="circular_arc",
            cylinder_profile="cylinder_fast",
            random_seed=23,
        )
        normalized_config = config_with_lumen_overrides(
            raw,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type="circular_arc",
            cylinder_profile="optimization_baseline",
            random_seed=23,
        )
        reference = MPPICore(
            reference_config,
            ApproximateCTRModel(reference_config),
            lumen_geometry=lumen_geometry_from_config(reference_config),
            lumen_cost_weights=lumen_cost_weights_from_config(reference_config),
            evaluation_diagnostics_enabled=True,
        )
        normalized = MPPICore(
            normalized_config,
            ApproximateCTRModel(normalized_config),
            lumen_geometry=lumen_geometry_from_config(normalized_config),
            lumen_cost_weights=lumen_cost_weights_from_config(normalized_config),
            evaluation_diagnostics_enabled=True,
        )
        self.assertEqual(reference.weights, normalized.weights)
        self.assertTrue(normalized.cost_normalization["enabled"])
        target = np.array([0.0166457424, 0.00397477634, 0.102231139], dtype=float)
        for _ in range(5):
            reference_result = reference.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
            normalized_result = normalized.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
            assert_result_equal(self, reference_result, normalized_result, reference, normalized)

    def test_normalized_candidate_changes_only_declared_tunable_weights(self):
        raw = load_parameter_files(CONFIG_FILES)
        baseline_config = config_with_mppi_profile(raw, "optimization_baseline")
        candidate_config = config_with_mppi_profile(raw, "optimization_c11")
        baseline = MPPICore(baseline_config, ApproximateCTRModel(baseline_config))
        candidate = MPPICore(candidate_config, ApproximateCTRModel(candidate_config))
        expected = {"tip": 0.5, "terminal": 8.0, "control": 0.25, "smoothness": 0.25}
        for key, value in baseline.weights.items():
            multiplier = expected.get(key, 1.0)
            self.assertEqual(candidate.weights[key], value * multiplier)
        self.assertEqual(
            baseline_config["cylindrical_lumen_cost"],
            candidate_config["cylindrical_lumen_cost"],
        )

    def test_normalization_rejects_non_tunable_or_missing_entries(self):
        config = make_config(optimized=True)
        config["mppi"]["cost_normalization"] = {
            "enabled": True,
            "reference_scales": {
                "tip": 1.0,
                "terminal": 1.0,
                "control": 1.0,
                "smoothness": 1.0,
                "wall_collision": 1.0,
            },
            "multipliers": {
                "tip": 1.0,
                "terminal": 1.0,
                "control": 1.0,
                "smoothness": 1.0,
            },
        }
        with self.assertRaisesRegex(ValueError, "must contain exactly"):
            MPPICore(config, ApproximateCTRModel(config))


if __name__ == "__main__":
    unittest.main()
