import copy
import sys
import unittest
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


def test_config():
    config = load_parameter_files(CONFIG_FILES)
    validate_or_raise(config)
    config = copy.deepcopy(config)
    config["mppi"]["horizon"] = 6
    config["mppi"]["num_samples"] = 160
    config["mppi"]["lambda"] = 0.001
    config["mppi"]["random_seed"] = 7
    config["mppi"]["noise_std"]["insertion"] = [0.002, 0.002, 0.002]
    config["mppi"]["noise_std"]["rotation"] = [0.05, 0.05, 0.05]
    config["mppi"]["weights"]["control"] = 0.05
    config["mppi"]["weights"]["smoothness"] = 0.1
    return config


class MPPICoreTest(unittest.TestCase):
    def test_solve_returns_bounded_command_and_metrics(self):
        config = test_config()
        model = ApproximateCTRModel(config)
        controller = MPPICore(config, model)
        target = model.forward_kinematics(np.array([0.003, 0.003, 0.003, 0.0, 0.0, 0.0])).tip_position

        result = controller.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target)
        self.assertEqual((6,), result.command.shape)
        self.assertEqual((config["mppi"]["horizon"], 6), result.nominal_sequence.shape)
        self.assertLessEqual(abs(result.command[0]), 0.002)
        self.assertLessEqual(abs(result.command[3]), 0.10)
        self.assertGreaterEqual(result.effective_sample_weight, 1.0)
        self.assertTrue(np.isfinite(result.minimum_cost))
        self.assertTrue(np.isfinite(result.mean_cost))

    def test_deterministic_seed_repeats_first_command(self):
        config = test_config()
        model = ApproximateCTRModel(config)
        target = model.forward_kinematics(np.array([0.003, 0.003, 0.003, 0.0, 0.0, 0.0])).tip_position
        first = MPPICore(config, model).solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target).command
        second = MPPICore(config, model).solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=target).command
        self.assertTrue(np.allclose(first, second))

    def test_advanced_costs_must_remain_disabled(self):
        config = test_config()
        config["mppi"]["weights"]["obstacle"] = 1.0
        with self.assertRaises(NotImplementedError):
            MPPICore(config, ApproximateCTRModel(config))

    def test_rejects_bad_inputs(self):
        config = test_config()
        controller = MPPICore(config, ApproximateCTRModel(config))
        with self.assertRaises(ValueError):
            controller.solve(q=np.zeros(5), q_dot=np.zeros(6), target_tip=np.zeros(3))
        with self.assertRaises(ValueError):
            controller.solve(q=np.zeros(6), q_dot=np.zeros(6), target_tip=np.zeros(2))


class ClosedLoopFixedTargetIntegrationTest(unittest.TestCase):
    def test_fixed_target_error_decreases(self):
        config = test_config()
        model = ApproximateCTRModel(config)
        simulation = CTRSimulationCore(config)
        controller = MPPICore(config, model)

        target_q = np.array([0.004, 0.004, 0.004, 0.0, 0.0, 0.0])
        target_tip = model.forward_kinematics(target_q).tip_position
        initial_error = np.linalg.norm(model.forward_kinematics(simulation.q).tip_position - target_tip)

        for _ in range(45):
            result = controller.solve(q=simulation.q, q_dot=simulation.q_dot, target_tip=target_tip)
            simulation.step(result.command, config["mppi"]["dt"])

        final_error = np.linalg.norm(model.forward_kinematics(simulation.q).tip_position - target_tip)
        self.assertLess(final_error, initial_error)


if __name__ == "__main__":
    unittest.main()
