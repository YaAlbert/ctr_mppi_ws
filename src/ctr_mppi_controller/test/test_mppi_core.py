import copy
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_model"))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_sim"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_model.approximate_model import ApproximateCTRModel  # noqa: E402
from ctr_mppi_controller.mppi_core import MPPICore  # noqa: E402
from ctr_sim.simulation_core import CTRSimulationCore  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def make_test_config():
    config = load_parameter_files(CONFIG_FILES)
    validate_or_raise(config)
    config = copy.deepcopy(config)
    tube_count = config["robot"]["number_of_tubes"]
    config["mppi"]["horizon"] = 6
    config["mppi"]["num_samples"] = 160
    config["mppi"]["lambda"] = 0.001
    config["mppi"]["random_seed"] = 7
    config["mppi"]["noise_std"]["insertion"] = [0.002] * tube_count
    config["mppi"]["noise_std"]["rotation"] = [0.05] * tube_count
    config["mppi"]["weights"]["control"] = 0.05
    config["mppi"]["weights"]["smoothness"] = 0.1
    return config


def make_controller():
    config = make_test_config()
    model = ApproximateCTRModel(config)
    return config, model, MPPICore(config, model)


class FirstThreeJointTipModel:
    def forward_kinematics(self, q):
        tip = np.asarray(q, dtype=float)[:3].copy()
        return SimpleNamespace(
            backbone_points=np.vstack((np.zeros(3), tip)),
            tip_position=tip,
        )


def make_cost_indexing_controller():
    config = make_test_config()
    config["mppi"]["dt"] = 1.0
    config["mppi"]["horizon"] = 3
    config["mppi"]["num_samples"] = 2
    config["mppi"]["weights"]["tip"] = 1.0
    config["mppi"]["weights"]["control"] = 0.0
    config["mppi"]["weights"]["smoothness"] = 0.0
    config["mppi"]["weights"]["terminal"] = 0.0
    config["robot"]["limits"]["insertion_max"] = [10.0, 10.0, 10.0]
    config["robot"]["limits"]["rotation_min"] = [-10.0, -10.0, -10.0]
    config["robot"]["limits"]["rotation_max"] = [10.0, 10.0, 10.0]
    config["robot"]["limits"]["insertion_velocity_max"] = [10.0, 10.0, 10.0]
    config["robot"]["limits"]["rotation_velocity_max"] = [10.0, 10.0, 10.0]
    config["mppi"]["noise_std"]["insertion"] = [0.0, 0.0, 0.0]
    config["mppi"]["noise_std"]["rotation"] = [0.0, 0.0, 0.0]
    return MPPICore(config, FirstThreeJointTipModel())


class MPPICoreTest(unittest.TestCase):
    def test_configuration_loader_derives_dimensions(self):
        config, _, controller = make_controller()
        self.assertEqual(6, controller.control_dimension)
        self.assertEqual(2 * config["robot"]["number_of_tubes"], controller.control_dimension)
        self.assertEqual((config["mppi"]["horizon"], controller.control_dimension), controller.nominal_sequence.shape)
        self.assertEqual((controller.control_dimension,), controller.noise_std.shape)
        self.assertEqual((controller.control_dimension,), controller.q_min.shape)
        self.assertEqual((controller.control_dimension,), controller.q_max.shape)
        self.assertEqual((controller.control_dimension,), controller.velocity_max.shape)

    def test_sample_dimensions_and_finiteness(self):
        config, _, controller = make_controller()
        samples = controller.sample_control_noise()
        self.assertEqual(
            (config["mppi"]["num_samples"], config["mppi"]["horizon"], controller.control_dimension),
            samples.shape,
        )
        self.assertTrue(np.all(np.isfinite(samples)))
        self.assertTrue(np.allclose(0.0, samples[0]))

    def test_candidate_sequences_are_velocity_clipped(self):
        _, _, controller = make_controller()
        perturbations = np.ones((controller.num_samples, controller.horizon, controller.control_dimension)) * 1.0e6
        candidates = controller.candidate_sequences(perturbations)
        self.assertEqual((controller.num_samples, controller.horizon, controller.control_dimension), candidates.shape)
        self.assertTrue(np.all(np.isfinite(candidates)))
        self.assertTrue(np.all(candidates <= controller.velocity_max))
        self.assertTrue(np.all(candidates >= -controller.velocity_max))

    def test_rollout_dimensions_and_finite_cost(self):
        _, model, controller = make_controller()
        target = model.forward_kinematics(np.zeros(controller.control_dimension)).tip_position
        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=np.zeros((controller.horizon, controller.control_dimension)),
            previous_command=np.zeros(controller.control_dimension),
            target_tip=target,
        )
        self.assertEqual((controller.control_dimension,), rollout.final_q.shape)
        self.assertEqual((3,), rollout.final_tip.shape)
        self.assertTrue(np.all(np.isfinite(rollout.final_q)))
        self.assertTrue(np.all(np.isfinite(rollout.final_tip)))
        self.assertTrue(np.isfinite(rollout.cost))

    def test_normalized_importance_weights(self):
        _, _, controller = make_controller()
        costs = np.linspace(0.0, 10.0, controller.num_samples)
        weights = controller.normalized_importance_weights(costs)
        self.assertEqual((controller.num_samples,), weights.shape)
        self.assertTrue(np.all(np.isfinite(weights)))
        self.assertAlmostEqual(1.0, float(np.sum(weights)))
        self.assertGreater(weights[0], weights[-1])

    def test_zero_error_target_has_zero_minimum_cost(self):
        _, model, controller = make_controller()
        target = model.forward_kinematics(np.zeros(controller.control_dimension)).tip_position
        result = controller.solve(
            q=np.zeros(controller.control_dimension),
            q_dot=np.zeros(controller.control_dimension),
            target_tip=target,
        )
        self.assertAlmostEqual(0.0, result.minimum_cost)
        self.assertTrue(np.all(np.isfinite(result.command)))
        self.assertTrue(np.all(np.abs(result.command) <= controller.velocity_max))

    def test_fixed_target_reference_sequence_is_tiled(self):
        _, _, controller = make_controller()
        target = np.array([0.01, 0.02, 0.03])
        reference_sequence = controller._reference_sequence(target_tip=target, target_tip_sequence=None)
        self.assertEqual((controller.horizon, 3), reference_sequence.shape)
        self.assertTrue(np.allclose(np.tile(target, (controller.horizon, 1)), reference_sequence))

    def test_fixed_target_matches_equivalent_tiled_sequence(self):
        config, model, fixed_controller = make_controller()
        sequence_controller = MPPICore(config, model)
        target_q = np.zeros(fixed_controller.control_dimension)
        target_q[: config["robot"]["number_of_tubes"]] = 0.003
        target = model.forward_kinematics(target_q).tip_position
        target_sequence = np.tile(target, (fixed_controller.horizon, 1))

        fixed_result = fixed_controller.solve(
            q=np.zeros(fixed_controller.control_dimension),
            q_dot=np.zeros(fixed_controller.control_dimension),
            target_tip=target,
        )
        sequence_result = sequence_controller.solve(
            q=np.zeros(sequence_controller.control_dimension),
            q_dot=np.zeros(sequence_controller.control_dimension),
            target_tip_sequence=target_sequence,
        )

        self.assertTrue(np.allclose(fixed_result.command, sequence_result.command))
        self.assertTrue(np.allclose(fixed_result.nominal_sequence, sequence_result.nominal_sequence))
        self.assertTrue(np.allclose(fixed_controller.last_costs, sequence_controller.last_costs))

    def test_solve_returns_bounded_command_and_metrics(self):
        config, model, controller = make_controller()
        target_q = np.zeros(controller.control_dimension)
        target_q[: config["robot"]["number_of_tubes"]] = 0.003
        target = model.forward_kinematics(target_q).tip_position

        result = controller.solve(
            q=np.zeros(controller.control_dimension),
            q_dot=np.zeros(controller.control_dimension),
            target_tip=target,
        )
        self.assertEqual((controller.control_dimension,), result.command.shape)
        self.assertEqual((config["mppi"]["horizon"], controller.control_dimension), result.nominal_sequence.shape)
        self.assertLessEqual(abs(result.command[0]), controller.velocity_max[0])
        self.assertLessEqual(abs(result.command[3]), controller.velocity_max[3])
        self.assertGreaterEqual(result.effective_sample_weight, 1.0)
        self.assertTrue(np.isfinite(result.minimum_cost))
        self.assertTrue(np.isfinite(result.mean_cost))
        self.assertEqual((config["mppi"]["num_samples"],), controller.last_costs.shape)
        self.assertEqual((config["mppi"]["num_samples"],), controller.last_normalized_weights.shape)
        self.assertEqual((config["mppi"]["num_samples"], controller.control_dimension), controller.last_rollout_final_q.shape)
        self.assertEqual(
            (config["mppi"]["num_samples"], config["mppi"]["horizon"], controller.control_dimension),
            controller.last_candidate_sequences.shape,
        )
        self.assertTrue(np.all(np.isfinite(controller.last_costs)))
        self.assertTrue(np.all(np.isfinite(controller.last_normalized_weights)))
        self.assertAlmostEqual(1.0, float(np.sum(controller.last_normalized_weights)))

    def test_deterministic_seed_repeats_first_command(self):
        config = make_test_config()
        model = ApproximateCTRModel(config)
        control_dimension = 2 * config["robot"]["number_of_tubes"]
        target_q = np.zeros(control_dimension)
        target_q[: config["robot"]["number_of_tubes"]] = 0.003
        target = model.forward_kinematics(target_q).tip_position
        first = MPPICore(config, model).solve(
            q=np.zeros(control_dimension),
            q_dot=np.zeros(control_dimension),
            target_tip=target,
        ).command
        second = MPPICore(config, model).solve(
            q=np.zeros(control_dimension),
            q_dot=np.zeros(control_dimension),
            target_tip=target,
        ).command
        self.assertTrue(np.allclose(first, second))

    def test_advanced_costs_must_remain_disabled(self):
        config = make_test_config()
        config["mppi"]["weights"]["obstacle"] = 1.0
        with self.assertRaises(NotImplementedError):
            MPPICore(config, ApproximateCTRModel(config))

    def test_rejects_bad_inputs(self):
        _, _, controller = make_controller()
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension - 1),
                q_dot=np.zeros(controller.control_dimension),
                target_tip=np.zeros(3),
            )
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.zeros(controller.control_dimension),
                target_tip=np.zeros(2),
            )
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.full(controller.control_dimension, np.nan),
                q_dot=np.zeros(controller.control_dimension),
                target_tip=np.zeros(3),
            )
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.full(controller.control_dimension, np.inf),
                target_tip=np.zeros(3),
            )
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.zeros(controller.control_dimension),
                target_tip=np.full(3, np.inf),
            )

    def test_both_reference_inputs_are_rejected(self):
        _, model, controller = make_controller()
        target = model.forward_kinematics(np.zeros(controller.control_dimension)).tip_position
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.zeros(controller.control_dimension),
                target_tip=target,
                target_tip_sequence=np.tile(target, (controller.horizon, 1)),
            )

    def test_missing_reference_input_is_rejected(self):
        _, _, controller = make_controller()
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.zeros(controller.control_dimension),
            )

    def test_rejects_bad_target_sequence(self):
        _, _, controller = make_controller()
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.zeros(controller.control_dimension),
                target_tip_sequence=np.zeros((controller.horizon - 1, 3)),
            )
        with self.assertRaises(ValueError):
            controller.solve(
                q=np.zeros(controller.control_dimension),
                q_dot=np.zeros(controller.control_dimension),
                target_tip_sequence=np.full((controller.horizon, 3), np.inf),
            )

    def test_per_step_trajectory_references_are_used_by_running_cost(self):
        controller = make_cost_indexing_controller()
        sequence = np.zeros((controller.horizon, controller.control_dimension))
        sequence[0, :3] = [1.0, 0.0, 0.0]
        sequence[1, :3] = [0.0, 1.0, 0.0]
        sequence[2, :3] = [0.0, 0.0, 1.0]
        target_sequence = np.array(
            [
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [9.0, 9.0, 9.0],
            ]
        )

        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=sequence,
            previous_command=np.zeros(controller.control_dimension),
            target_tip_sequence=target_sequence,
        )

        self.assertAlmostEqual(192.0, rollout.cost)

    def test_final_sequence_point_is_used_by_terminal_cost(self):
        controller = make_cost_indexing_controller()
        controller.weights["tip"] = 0.0
        controller.weights["terminal"] = 1.0
        sequence = np.zeros((controller.horizon, controller.control_dimension))
        sequence[0, :3] = [1.0, 0.0, 0.0]
        sequence[1, :3] = [0.0, 1.0, 0.0]
        sequence[2, :3] = [0.0, 0.0, 1.0]
        target_sequence = np.array(
            [
                [9.0, 9.0, 9.0],
                [8.0, 8.0, 8.0],
                [1.0, 1.0, 1.0],
            ]
        )

        rollout = controller.rollout_candidate(
            q0=np.zeros(controller.control_dimension),
            sequence=sequence,
            previous_command=np.zeros(controller.control_dimension),
            target_tip_sequence=target_sequence,
        )

        self.assertAlmostEqual(0.0, rollout.cost)


class ClosedLoopFixedTargetIntegrationTest(unittest.TestCase):
    def test_fixed_target_error_decreases(self):
        config = make_test_config()
        model = ApproximateCTRModel(config)
        simulation = CTRSimulationCore(config)
        controller = MPPICore(config, model)

        target_q = np.zeros(controller.control_dimension)
        target_q[: config["robot"]["number_of_tubes"]] = 0.004
        target_tip = model.forward_kinematics(target_q).tip_position
        initial_error = np.linalg.norm(model.forward_kinematics(simulation.q).tip_position - target_tip)

        for _ in range(45):
            result = controller.solve(q=simulation.q, q_dot=simulation.q_dot, target_tip=target_tip)
            simulation.step(result.command, config["mppi"]["dt"])

        final_error = np.linalg.norm(model.forward_kinematics(simulation.q).tip_position - target_tip)
        self.assertLess(final_error, initial_error)


if __name__ == "__main__":
    unittest.main()
