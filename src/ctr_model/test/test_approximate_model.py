import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise  # noqa: E402
from ctr_model.approximate_model import ApproximateCTRModel  # noqa: E402


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


class ApproximateCTRModelTest(unittest.TestCase):
    def setUp(self):
        self.config = load_parameter_files(CONFIG_FILES)
        validate_or_raise(self.config)

    def test_forward_kinematics_shapes_are_finite(self):
        model = ApproximateCTRModel(self.config)
        result = model.forward_kinematics(np.zeros(6))
        self.assertEqual((50, 3), result.backbone_points.shape)
        self.assertEqual((3,), result.tip_position.shape)
        self.assertTrue(np.all(np.isfinite(result.backbone_points)))
        self.assertTrue(np.all(np.isfinite(result.tip_position)))

    def test_forward_kinematics_rejects_bad_shape(self):
        model = ApproximateCTRModel(self.config)
        with self.assertRaises(ValueError):
            model.forward_kinematics(np.zeros(5))

    def test_forward_kinematics_rejects_nonfinite(self):
        model = ApproximateCTRModel(self.config)
        q = np.zeros(6)
        q[0] = np.nan
        with self.assertRaises(ValueError):
            model.forward_kinematics(q)


if __name__ == "__main__":
    unittest.main()
