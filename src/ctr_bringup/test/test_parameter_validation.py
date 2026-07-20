import importlib.util
import copy
import os
import sys
import tempfile
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
    validate_config_paths,
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

    def test_config_paths_reject_previous_concatenated_scalar_failure(self):
        concatenated = f"{CONFIG_FILES[0]}{CONFIG_FILES[1]}"
        with self.assertRaises(ParameterValidationError) as context:
            validate_config_paths(concatenated)
        self.assertIn("string array", str(context.exception))

    def test_config_paths_preserve_separate_ordered_string_entries(self):
        paths = [str(CONFIG_FILES[0]), str(CONFIG_FILES[1]), str(CONFIG_FILES[2])]
        validated = validate_config_paths(paths)
        self.assertEqual([str(path.resolve()) for path in CONFIG_FILES[:3]], validated)
        self.assertEqual(3, len(validated))

    def test_simulation_launch_config_paths_are_separate_string_entries(self):
        launch_path = REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py"
        spec = importlib.util.spec_from_file_location("simulation_launch_under_test", launch_path)
        module = importlib.util.module_from_spec(spec)
        self.assertIsNotNone(spec.loader)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as temp_dir:
            package_share = Path(temp_dir)
            config_dir = package_share / "config"
            config_dir.mkdir()
            for source_path in CONFIG_FILES:
                (config_dir / source_path.name).write_text("placeholder: true\n", encoding="utf-8")

            module.get_package_share_directory = lambda package_name: str(package_share)
            paths = module._config_paths()

        self.assertEqual([str((config_dir / path.name).resolve()) for path in CONFIG_FILES], paths)
        self.assertTrue(all(isinstance(path, str) for path in paths))
        self.assertEqual(len(CONFIG_FILES), len(paths))

    def test_simulation_launch_declares_reference_manager_arguments(self):
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node

        launch_path = REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py"
        spec = importlib.util.spec_from_file_location("simulation_launch_reference_args_under_test", launch_path)
        module = importlib.util.module_from_spec(spec)
        self.assertIsNotNone(spec.loader)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as temp_dir:
            package_share = Path(temp_dir)
            previous_ros_log_dir = os.environ.get("ROS_LOG_DIR")
            os.environ["ROS_LOG_DIR"] = str(package_share / "ros_log")
            config_dir = package_share / "config"
            config_dir.mkdir()
            for source_path in CONFIG_FILES:
                (config_dir / source_path.name).write_text("placeholder: true\n", encoding="utf-8")

            module.get_package_share_directory = lambda package_name: str(package_share)
            try:
                launch_description = module.generate_launch_description()
            finally:
                if previous_ros_log_dir is None:
                    os.environ.pop("ROS_LOG_DIR", None)
                else:
                    os.environ["ROS_LOG_DIR"] = previous_ros_log_dir

        launch_arguments = {
            entity.name
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("start_reference_manager", launch_arguments)
        self.assertIn("reference_mode", launch_arguments)
        self.assertIn("reference_type", launch_arguments)

        node_actions = [entity for entity in launch_description.entities if isinstance(entity, Node)]
        executables = {getattr(node, "_Node__node_executable", None) for node in node_actions}
        self.assertIn("reference_manager_node", executables)

    def test_config_paths_reject_empty_required_array(self):
        with self.assertRaises(ParameterValidationError) as context:
            validate_config_paths([])
        self.assertIn("at least one", str(context.exception))

    def test_config_paths_reject_non_string_items(self):
        with self.assertRaises(ParameterValidationError) as context:
            validate_config_paths([str(CONFIG_FILES[0]), 42])
        self.assertIn("config_paths[1]", str(context.exception))

    def test_config_paths_reject_missing_files(self):
        missing = CONFIG_FILES[0].with_name("missing_params.yaml")
        with self.assertRaises(FileNotFoundError):
            validate_config_paths([str(missing)])

    def test_config_paths_reject_duplicates(self):
        with self.assertRaises(ParameterValidationError) as context:
            validate_config_paths([str(CONFIG_FILES[0]), str(CONFIG_FILES[0])])
        self.assertIn("Duplicate parameter path", str(context.exception))

    def test_duplicate_sections_are_rejected(self):
        with self.assertRaises(ParameterValidationError):
            load_parameter_files([CONFIG_FILES[0], CONFIG_FILES[0]])

    def test_reference_yaml_section_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        self.assertEqual([], [error for error in validate_project_config(config) if "reference" in error])

    def test_reference_invalid_mode_is_rejected(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["reference"]["mode"] = "invalid"
        errors = validate_project_config(config)
        self.assertTrue(any("reference.mode" in error for error in errors))

    def test_reference_zero_helix_height_is_rejected(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["reference"]["helix"]["height"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("reference.helix.height" in error for error in errors))

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
