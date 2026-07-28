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
    REPO_ROOT / "config" / "evaluation_params.yaml",
    REPO_ROOT / "config" / "safety_params.yaml",
    REPO_ROOT / "config" / "tactile_params.yaml",
    REPO_ROOT / "config" / "hardware_params.yaml",
]


def load_launch_module(file_name: str, module_name: str):
    launch_path = REPO_ROOT / "src" / "ctr_bringup" / "launch" / file_name
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise AssertionError(f"could not load launch module {file_name}")
    spec.loader.exec_module(module)
    return module


def temporary_config_paths(module):
    with tempfile.TemporaryDirectory() as temp_dir:
        package_share = Path(temp_dir)
        config_dir = package_share / "config"
        config_dir.mkdir()
        for source_path in CONFIG_FILES:
            (config_dir / source_path.name).write_text("placeholder: true\n", encoding="utf-8")
        module.get_package_share_directory = lambda package_name: str(package_share)
        paths = module._config_paths()
    return paths


def launch_description_with_temp_share(module):
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
            return module.generate_launch_description()
        finally:
            if previous_ros_log_dir is None:
                os.environ.pop("ROS_LOG_DIR", None)
            else:
                os.environ["ROS_LOG_DIR"] = previous_ros_log_dir


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
        module = load_launch_module("simulation.launch.py", "simulation_launch_under_test")
        paths = temporary_config_paths(module)
        self.assertEqual([path.name for path in CONFIG_FILES], [Path(path).name for path in paths])
        self.assertTrue(all(isinstance(path, str) for path in paths))
        self.assertEqual(len(CONFIG_FILES), len(paths))

    def test_all_launch_config_paths_include_evaluation_params_in_order(self):
        for file_name in (
            "simulation.launch.py",
            "mock_hardware.launch.py",
            "physical_hardware.launch.py",
            "evaluation_reference.launch.py",
            "evaluation_mppi_controller.launch.py",
        ):
            with self.subTest(file_name=file_name):
                module = load_launch_module(file_name, f"{file_name.replace('.', '_')}_config_paths_under_test")
                paths = temporary_config_paths(module)
                self.assertEqual([path.name for path in CONFIG_FILES], [Path(path).name for path in paths])
                self.assertIn("evaluation_params.yaml", [Path(path).name for path in paths])
                self.assertTrue(all(isinstance(path, str) for path in paths))

    def test_simulation_launch_declares_reference_manager_arguments(self):
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node

        module = load_launch_module("simulation.launch.py", "simulation_launch_reference_args_under_test")

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
        self.assertIn("start_evaluation", launch_arguments)
        self.assertIn("evaluation_experiment_group", launch_arguments)
        self.assertIn("evaluation_controller_label", launch_arguments)
        self.assertIn("evaluation_baseline_result_dir", launch_arguments)
        self.assertIn("evaluation_output_root", launch_arguments)
        self.assertIn("enable_cylindrical_lumen", launch_arguments)
        self.assertIn("cylinder_profile", launch_arguments)
        self.assertIn("cylinder_target_x", launch_arguments)
        self.assertIn("cylinder_target_y", launch_arguments)
        self.assertIn("cylinder_target_z", launch_arguments)
        self.assertIn("mppi_random_seed", launch_arguments)
        self.assertIn("run_role", launch_arguments)

        node_actions = [entity for entity in launch_description.entities if isinstance(entity, Node)]
        executables = {getattr(node, "_Node__node_executable", None) for node in node_actions}
        self.assertIn("reference_manager_node", executables)
        self.assertIn("evaluation_node", executables)

    def test_evaluation_reference_launch_static_compatibility(self):
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node

        module = load_launch_module("evaluation_reference.launch.py", "evaluation_reference_launch_under_test")
        paths = temporary_config_paths(module)
        self.assertEqual([path.name for path in CONFIG_FILES], [Path(path).name for path in paths])

        launch_description = launch_description_with_temp_share(module)
        launch_arguments = {
            entity.name
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("trajectory_start_policy", launch_arguments)
        self.assertIn("scheduled_reference_epoch", launch_arguments)
        self.assertIn("enable_cylindrical_lumen", launch_arguments)
        self.assertIn("cylinder_profile", launch_arguments)
        self.assertIn("cylinder_target_x", launch_arguments)
        self.assertIn("cylinder_target_y", launch_arguments)
        self.assertIn("cylinder_target_z", launch_arguments)
        self.assertIn("mppi_random_seed", launch_arguments)
        nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
        self.assertEqual(["reference_manager_node"], [getattr(node, "_Node__node_executable", None) for node in nodes])

    def test_evaluation_mppi_controller_launch_static_compatibility(self):
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node

        module = load_launch_module("evaluation_mppi_controller.launch.py", "evaluation_mppi_controller_launch_under_test")
        paths = temporary_config_paths(module)
        self.assertEqual([path.name for path in CONFIG_FILES], [Path(path).name for path in paths])

        launch_description = launch_description_with_temp_share(module)
        launch_arguments = {
            entity.name
            for entity in launch_description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        self.assertIn("publish_safe_command_for_simulation", launch_arguments)
        self.assertIn("enable_cylindrical_lumen", launch_arguments)
        self.assertIn("cylinder_profile", launch_arguments)
        self.assertIn("cylinder_target_x", launch_arguments)
        self.assertIn("cylinder_target_y", launch_arguments)
        self.assertIn("cylinder_target_z", launch_arguments)
        self.assertIn("mppi_random_seed", launch_arguments)
        nodes = [entity for entity in launch_description.entities if isinstance(entity, Node)]
        self.assertEqual(["mppi_controller_node"], [getattr(node, "_Node__node_executable", None) for node in nodes])

    def test_cylinder_target_node_parameters_are_double_arrays(self):
        node_files = (
            REPO_ROOT / "src" / "ctr_sim" / "ctr_sim" / "nodes" / "simulator_node.py",
            REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "mppi_controller_node.py",
            REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "reference_manager_node.py",
            REPO_ROOT / "src" / "ctr_evaluation" / "ctr_evaluation" / "nodes" / "evaluation_node.py",
        )
        for path in node_files:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn('declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)', source)

    def test_cylinder_target_launch_parameters_are_typed_double_arrays(self):
        launch_files = (
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py",
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "evaluation_reference.launch.py",
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "evaluation_mppi_controller.launch.py",
        )
        for path in launch_files:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn("ParameterValue", source)
                self.assertIn("value_type=list[float]", source)

    def test_mppi_seed_node_parameters_are_integer_with_negative_default(self):
        node_files = (
            REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "mppi_controller_node.py",
            REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "reference_manager_node.py",
            REPO_ROOT / "src" / "ctr_evaluation" / "ctr_evaluation" / "nodes" / "evaluation_node.py",
        )
        for path in node_files:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn('declare_parameter("mppi_random_seed", -1)', source)
                self.assertIn("def _optional_seed", source)

    def test_mppi_seed_launch_defaults_are_negative_integer(self):
        launch_files = (
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py",
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "evaluation_reference.launch.py",
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "evaluation_mppi_controller.launch.py",
        )
        for path in launch_files:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn('"mppi_random_seed"', source)
                self.assertIn('default_value="-1"', source)

    def test_evaluation_launch_helpers_start_no_hardware(self):
        from launch_ros.actions import Node

        for file_name in ("evaluation_reference.launch.py", "evaluation_mppi_controller.launch.py"):
            with self.subTest(file_name=file_name):
                module = load_launch_module(file_name, f"{file_name.replace('.', '_')}_no_hw_under_test")
                launch_description = launch_description_with_temp_share(module)
                executables = {
                    getattr(entity, "_Node__node_executable", None)
                    for entity in launch_description.entities
                    if isinstance(entity, Node)
                }
                self.assertNotIn("physical_hardware_node", executables)
                self.assertNotIn("mock_hardware_node", executables)

    def test_evaluation_nodes_are_disabled_by_default_in_launch_files(self):
        from launch.actions import DeclareLaunchArgument
        from launch_ros.actions import Node

        for file_name in ("simulation.launch.py", "mock_hardware.launch.py", "physical_hardware.launch.py"):
            with self.subTest(file_name=file_name):
                launch_path = REPO_ROOT / "src" / "ctr_bringup" / "launch" / file_name
                source = launch_path.read_text(encoding="utf-8")
                self.assertRegex(
                    source,
                    r'(?s)DeclareLaunchArgument\(\s*"start_evaluation",\s*default_value="false"',
                )
                module = load_launch_module(file_name, f"{file_name.replace('.', '_')}_eval_disabled_under_test")
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
                self.assertIn("start_evaluation", launch_arguments)
                eval_nodes = [
                    entity
                    for entity in launch_description.entities
                    if isinstance(entity, Node)
                    and getattr(entity, "_Node__node_executable", None) == "evaluation_node"
                ]
                self.assertEqual(1, len(eval_nodes))
                condition = getattr(eval_nodes[0], "condition", None)
                if condition is None:
                    condition = getattr(eval_nodes[0], "_Action__condition", None)
                self.assertIsNotNone(condition)

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

    def test_tracking_metrics_yaml_section_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        self.assertEqual([], [error for error in validate_project_config(config) if "tracking_metrics" in error])

    def test_tracking_metrics_invalid_stable_cycles_is_rejected(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["tracking_metrics"]["stable_cycles"] = 0
        errors = validate_project_config(config)
        self.assertTrue(any("tracking_metrics.stable_cycles" in error for error in errors))

    def test_cylindrical_lumen_yaml_section_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        errors = validate_project_config(config)
        self.assertEqual([], [error for error in errors if "cylindrical_lumen" in error])
        self.assertEqual([], [error for error in errors if "goal." in error])
        self.assertEqual([], [error for error in errors if "mppi_profiles" in error])

    def test_cylindrical_lumen_rejects_invalid_axis(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["cylindrical_lumen"]["axis_direction"] = [0.0, 0.0, 0.0]
        errors = validate_project_config(config)
        self.assertTrue(any("axis_direction" in error for error in errors))

    def test_cylindrical_lumen_rejects_unusable_radius(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["cylindrical_lumen"]["radius"] = config["cylindrical_lumen"]["ctr_outer_radius"]
        errors = validate_project_config(config)
        self.assertTrue(any("radius" in error for error in errors))

    def test_goal_rejects_bad_position_and_tolerance(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["goal"]["position"] = [0.0, 0.0]
        config["goal"]["tolerance"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("goal.position" in error for error in errors))
        self.assertTrue(any("goal.tolerance" in error for error in errors))

    def test_mppi_profile_rejects_nonpositive_control_period(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["mppi_profiles"]["cylinder_fast"]["control_period"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("mppi_profiles.cylinder_fast.control_period" in error for error in errors))

    def test_mppi_profile_rejects_negative_weight_override(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["mppi_profiles"]["cylinder_fast"]["weights"]["tip"] = -1.0
        errors = validate_project_config(config)
        self.assertTrue(any("mppi_profiles.cylinder_fast.weights.tip" in error for error in errors))

    def test_evaluation_yaml_section_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        self.assertEqual([], [error for error in validate_project_config(config) if "evaluation" in error])

    def test_evaluation_invalid_alignment_gap_is_rejected(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["evaluation"]["maximum_reference_alignment_gap"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("evaluation.maximum_reference_alignment_gap" in error for error in errors))

    def test_evaluation_orchestration_yaml_section_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        errors = validate_project_config(config)
        self.assertEqual([], [error for error in errors if "evaluation.orchestration" in error])

    def test_evaluation_invalid_orchestration_timeout_is_rejected(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["evaluation"]["orchestration"]["startup_timeout"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("evaluation.orchestration.startup_timeout" in error for error in errors))

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
