import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_bringup.parameter_validation import (  # noqa: E402
    ParameterValidationError,
    UNRESOLVED_TODO_IDS,
    load_parameter_files,
    project_config_with_overrides,
    validate_project_config,
    validate_or_raise,
)


CONFIG_FILES = [
    REPO_ROOT / "config" / "robot_params.yaml",
    REPO_ROOT / "config" / "model_params.yaml",
    REPO_ROOT / "config" / "mppi_params.yaml",
    REPO_ROOT / "config" / "simulation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


class ParameterValidationTest(unittest.TestCase):
    def test_current_project_config_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        validate_or_raise(config)
        self.assertEqual([], validate_project_config(config))

    def test_duplicate_sections_are_rejected(self):
        with self.assertRaises(ParameterValidationError):
            load_parameter_files([CONFIG_FILES[0], CONFIG_FILES[0]])

    def test_invalid_robot_tube_count_is_rejected(self):
        config = load_parameter_files(CONFIG_FILES)
        config["robot"]["number_of_tubes"] = 2
        errors = validate_project_config(config)
        self.assertTrue(any("number_of_tubes" in error for error in errors))

    def test_runtime_overrides_do_not_mutate_base_config(self):
        config = load_parameter_files(CONFIG_FILES)
        overridden = project_config_with_overrides(
            config,
            runtime_mode="mock_hardware",
            hardware_implementation="mock",
        )
        self.assertNotIn("runtime", config)
        self.assertEqual("mock_hardware", overridden["runtime"]["mode"])
        self.assertEqual("mock", overridden["hardware"]["implementation"])

    def test_critical_todo_ids_are_registered(self):
        for todo_id in ("ROS-001", "ROS-002", "MODEL-004", "HW-003", "SNS-001", "SAFE-001"):
            self.assertIn(todo_id, UNRESOLVED_TODO_IDS)


if __name__ == "__main__":
    unittest.main()
