import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
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


class CTRSimulationCoreTest(unittest.TestCase):
    def setUp(self):
        self.config = load_parameter_files(CONFIG_FILES)
        validate_or_raise(self.config)

    def test_zero_command_keeps_initial_configuration(self):
        core = CTRSimulationCore(self.config)
        q0 = core.q.copy()
        result = core.step(np.zeros(6), 0.01)
        self.assertTrue(np.allclose(q0, result.q))
        self.assertTrue(np.allclose(np.zeros(6), result.q_dot))
        self.assertFalse(result.command_saturated)

    def test_velocity_command_updates_q(self):
        core = CTRSimulationCore(self.config)
        result = core.step([0.001, 0.0, 0.0, 0.0, 0.0, 0.0], 0.1)
        self.assertGreater(result.q[0], 0.0)
        self.assertAlmostEqual(0.001, result.q_dot[0])

    def test_command_clips_to_velocity_limits(self):
        core = CTRSimulationCore(self.config)
        result = core.step([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], 0.1)
        self.assertTrue(result.command_saturated)
        self.assertLessEqual(result.q_dot[0], 0.002)
        self.assertLessEqual(result.q_dot[3], 0.10)

    def test_rejects_bad_command_shape(self):
        core = CTRSimulationCore(self.config)
        with self.assertRaises(ValueError):
            core.step(np.zeros(5), 0.1)


if __name__ == "__main__":
    unittest.main()
