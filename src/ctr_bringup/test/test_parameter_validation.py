import importlib.util
import copy
import math
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
    parse_launch_bool,
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
    REPO_ROOT / "config" / "slice_7g_runtime_params.yaml",
]


def curved_lumen_config(lumen_type: str = "circular_arc"):
    config = load_parameter_files(CONFIG_FILES)
    config = copy.deepcopy(config)
    config["cylindrical_lumen"]["enabled"] = False
    config["curved_lumen"]["enabled"] = True
    config["curved_lumen"]["type"] = lumen_type
    return config


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
    def test_safety_tactile_defaults_and_watchdog_constraints(self):
        config = load_parameter_files(CONFIG_FILES)
        safety = config["safety"]
        self.assertFalse(safety["tactile_enabled"])
        self.assertEqual([], [error for error in validate_project_config(config) if error.startswith("`safety")])
        invalid_cases = (
            ("tactile_startup_grace_s", -0.01),
            ("tactile_future_skew_s", float("nan")),
            ("watchdog_period_s", 0.0),
            ("watchdog_period_s", 0.2),
        )
        for key, value in invalid_cases:
            with self.subTest(key=key, value=value):
                invalid = copy.deepcopy(config)
                invalid["safety"][key] = value
                self.assertTrue(any(f"safety.{key}" in error for error in validate_project_config(invalid)))

    def test_enabled_tactile_safety_requires_timeout_stop(self):
        config = load_parameter_files(CONFIG_FILES)
        config["safety"]["tactile_enabled"] = True
        config["safety"]["stop_on_tactile_timeout"] = False
        errors = validate_project_config(config)
        self.assertTrue(any("stop_on_tactile_timeout" in error for error in errors), errors)

    def test_current_project_config_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        validate_or_raise(config)
        self.assertEqual([], validate_project_config(config))

    def test_tactile_7b_defaults_and_runtime_constraints_validate(self):
        config = load_parameter_files(CONFIG_FILES)
        self.assertFalse(config["tactile"]["enabled"])
        self.assertEqual("simulated", config["tactile"]["mode"])
        self.assertEqual([], [error for error in validate_project_config(config) if "tactile" in error])

        invalid_cases = (
            ("mode", "hardware"),
            ("enabled", "false"),
        )
        for key, value in invalid_cases:
            with self.subTest(key=key, value=value):
                invalid = copy.deepcopy(config)
                invalid["tactile"][key] = value
                self.assertTrue(any("tactile." + key in error for error in validate_project_config(invalid)))

        for section, key, values in (
            ("calibration", "scale", (0.0, -1.0, math.nan, math.inf)),
            ("simulation", "force_saturation_n", (0.0, -1.0, math.nan, math.inf)),
        ):
            for value in values:
                with self.subTest(section=section, key=key, value=value):
                    invalid = copy.deepcopy(config)
                    invalid["tactile"][section][key] = value
                    self.assertTrue(
                        any(f"tactile.{section}.{key}" in error for error in validate_project_config(invalid)),
                        validate_project_config(invalid),
                    )

        for value in (0.0, -0.1, math.nan, math.inf):
            invalid = copy.deepcopy(config)
            invalid["tactile"]["filter"]["alpha"] = value
            self.assertTrue(any("tactile.filter.alpha" in error for error in validate_project_config(invalid)))

        for key in ("contact_off", "warning_off", "stop_off"):
            invalid = copy.deepcopy(config)
            invalid["tactile"]["thresholds"][key] = math.nan
            self.assertTrue(any(f"tactile.thresholds.{key}" in error for error in validate_project_config(invalid)))

        invalid = copy.deepcopy(config)
        invalid["tactile"]["thresholds"]["stop_off"] = invalid["tactile"]["thresholds"]["stop"]
        self.assertTrue(any("stop_off" in error for error in validate_project_config(invalid)))

        valid_equalities = copy.deepcopy(config)
        valid_equalities["tactile"]["thresholds"]["warning_off"] = valid_equalities["tactile"]["thresholds"]["contact"]
        valid_equalities["tactile"]["thresholds"]["stop_off"] = valid_equalities["tactile"]["thresholds"]["warning"]
        valid_equalities["tactile"]["simulation"]["force_saturation_n"] = valid_equalities["tactile"]["thresholds"]["stop"]
        self.assertEqual([], [error for error in validate_project_config(valid_equalities) if "tactile" in error])

    def test_simulation_visualization_defaults_validate(self):
        config = load_parameter_files(CONFIG_FILES)
        visualization = config["simulation"]["visualization"]
        self.assertIs(True, visualization["publish_lumen_markers"])
        self.assertIs(True, visualization["publish_lumen_diagnostics"])
        self.assertIs(False, visualization["publish_lumen_surface"])
        self.assertEqual(1, visualization["centerline_stride"])
        self.assertEqual(4, visualization["ring_stride"])
        self.assertEqual(20, visualization["ring_segments"])
        self.assertEqual(5.0, visualization["marker_publish_rate"])
        self.assertEqual(0.20, visualization["surface_alpha"])
        self.assertEqual(500, visualization["actual_tip_history_max_points"])
        self.assertEqual(0.05, visualization["actual_tip_history_min_interval"])
        self.assertEqual([], [error for error in validate_project_config(config) if "simulation.visualization" in error])

    def test_development_target_selection_limits_are_configured_and_strict(self):
        config = load_parameter_files(CONFIG_FILES)
        selection = config["simulation"]["development_target_selection"]
        self.assertEqual(0.035, selection["projection_limit"])
        self.assertEqual(5.0, selection["candidate_max_age"])
        self.assertEqual(0.5, selection["candidate_future_tolerance"])
        self.assertEqual([], validate_project_config(config))
        for key, invalid_values in {
            "projection_limit": (0.0, -1.0, True, float("nan")),
            "candidate_max_age": (0.0, -1.0, True, float("inf")),
            "candidate_future_tolerance": (-1.0, True, float("nan")),
        }.items():
            for value in invalid_values:
                with self.subTest(key=key, value=value):
                    invalid = copy.deepcopy(config)
                    invalid["simulation"]["development_target_selection"][key] = value
                    self.assertTrue(any(key in error for error in validate_project_config(invalid)))

    def test_mppi_tactile_defaults_and_cross_field_validation(self):
        config = load_parameter_files(CONFIG_FILES)
        self.assertEqual([], [error for error in validate_project_config(config) if "mppi.tactile" in error])
        invalid = copy.deepcopy(config)
        invalid["mppi"]["tactile"]["enabled"] = True
        self.assertTrue(any("positive `mppi.weights.force`" in error for error in validate_project_config(invalid)))
        invalid = copy.deepcopy(config)
        invalid["mppi"]["tactile"]["warning_multiplier"] = -1.0
        self.assertTrue(any("warning_multiplier" in error for error in validate_project_config(invalid)))
        invalid = copy.deepcopy(config)
        invalid["mppi"]["tactile"]["force_saturation_n"] = 0.0
        self.assertTrue(any("force_saturation_n" in error for error in validate_project_config(invalid)))

    def test_simulation_visualization_publish_lumen_markers_requires_bool(self):
        for value in ("true", 1, 0, None):
            with self.subTest(value=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["publish_lumen_markers"] = value
                errors = validate_project_config(config)
                self.assertTrue(
                    any("simulation.visualization.publish_lumen_markers" in error for error in errors),
                    errors,
                )

    def test_simulation_visualization_publish_lumen_diagnostics_requires_bool(self):
        for value in (True, False):
            with self.subTest(valid=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["publish_lumen_diagnostics"] = value
                self.assertEqual(
                    [],
                    [
                        error
                        for error in validate_project_config(config)
                        if "simulation.visualization.publish_lumen_diagnostics" in error
                    ],
                )
        for value in ("true", "false", 1, 0, 1.0, None):
            with self.subTest(invalid=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["publish_lumen_diagnostics"] = value
                errors = validate_project_config(config)
                self.assertTrue(
                    any("simulation.visualization.publish_lumen_diagnostics" in error for error in errors),
                    errors,
                )

    def test_development_visualization_surface_and_history_values_are_strictly_bounded(self):
        for value in (True, False):
            config = load_parameter_files(CONFIG_FILES)
            config["simulation"]["visualization"]["publish_lumen_surface"] = value
            self.assertEqual([], validate_project_config(config))
        for value in ("true", 1, 0, None):
            config = load_parameter_files(CONFIG_FILES)
            config["simulation"]["visualization"]["publish_lumen_surface"] = value
            self.assertTrue(any("publish_lumen_surface" in error for error in validate_project_config(config)))

        invalid_values = {
            "surface_alpha": (-0.1, 1.1, True, float("nan")),
            "actual_tip_history_max_points": (1, 5001, True, 5.0),
            "actual_tip_history_min_interval": (0.0, -1.0, True, float("inf")),
        }
        for key, values in invalid_values.items():
            for value in values:
                with self.subTest(key=key, value=value):
                    config = load_parameter_files(CONFIG_FILES)
                    config["simulation"]["visualization"][key] = value
                    self.assertTrue(any(key in error for error in validate_project_config(config)))

    def test_simulation_visualization_stride_values_require_exact_positive_ints(self):
        cases = (
            ("centerline_stride", 1, (0, -1, True, 1.0, "1")),
            ("ring_stride", 1, (0, -1, True, 1.0, "1")),
        )
        for key, valid_value, invalid_values in cases:
            config = load_parameter_files(CONFIG_FILES)
            config["simulation"]["visualization"][key] = valid_value
            self.assertEqual(
                [],
                [error for error in validate_project_config(config) if f"simulation.visualization.{key}" in error],
            )
            for value in invalid_values:
                with self.subTest(key=key, value=value):
                    config = load_parameter_files(CONFIG_FILES)
                    config["simulation"]["visualization"][key] = value
                    errors = validate_project_config(config)
                    self.assertTrue(any(f"simulation.visualization.{key}" in error for error in errors), errors)

    def test_simulation_visualization_ring_segments_bounds(self):
        for value in (8, 20, 128):
            with self.subTest(valid=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["ring_segments"] = value
                self.assertEqual(
                    [],
                    [
                        error
                        for error in validate_project_config(config)
                        if "simulation.visualization.ring_segments" in error
                    ],
                )
        for value in (7, 129, True, 20.0, "20"):
            with self.subTest(invalid=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["ring_segments"] = value
                errors = validate_project_config(config)
                self.assertTrue(any("simulation.visualization.ring_segments" in error for error in errors), errors)

    def test_simulation_visualization_marker_publish_rate_must_be_positive_and_finite(self):
        for value in (0.1, 5.0, 100):
            with self.subTest(valid=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["marker_publish_rate"] = value
                self.assertEqual(
                    [],
                    [
                        error
                        for error in validate_project_config(config)
                        if "simulation.visualization.marker_publish_rate" in error
                    ],
                )
        for value in (0.0, -1.0, float("nan"), float("inf"), True):
            with self.subTest(invalid=value):
                config = load_parameter_files(CONFIG_FILES)
                config["simulation"]["visualization"]["marker_publish_rate"] = value
                errors = validate_project_config(config)
                self.assertTrue(
                    any("simulation.visualization.marker_publish_rate" in error for error in errors),
                    errors,
                )

    def test_visualization_disabled_still_validates_supplied_sampling_values(self):
        config = load_parameter_files(CONFIG_FILES)
        config["simulation"]["visualization"]["publish_lumen_markers"] = False
        config["simulation"]["visualization"]["ring_segments"] = 7
        errors = validate_project_config(config)
        self.assertTrue(any("simulation.visualization.ring_segments" in error for error in errors), errors)

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
                self.assertEqual(list(module.CONFIG_NAMES), [Path(path).name for path in paths])
                self.assertIn("evaluation_params.yaml", [Path(path).name for path in paths])
                if file_name in {
                    "simulation.launch.py",
                    "evaluation_reference.launch.py",
                    "evaluation_mppi_controller.launch.py",
                }:
                    self.assertEqual("slice_7g_runtime_params.yaml", Path(paths[-1]).name)
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
        self.assertIn("enable_curved_lumen", launch_arguments)
        self.assertIn("curved_lumen_type", launch_arguments)
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
        self.assertIn("enable_curved_lumen", launch_arguments)
        self.assertIn("curved_lumen_type", launch_arguments)
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
        self.assertIn("enable_curved_lumen", launch_arguments)
        self.assertIn("curved_lumen_type", launch_arguments)
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

    def test_curved_lumen_node_parameters_are_declared_for_runtime_nodes(self):
        node_files = (
            REPO_ROOT / "src" / "ctr_sim" / "ctr_sim" / "nodes" / "simulator_node.py",
            REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "mppi_controller_node.py",
            REPO_ROOT / "src" / "ctr_mppi_controller" / "ctr_mppi_controller" / "nodes" / "reference_manager_node.py",
        )
        for path in node_files:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn('declare_parameter("enable_curved_lumen", False)', source)
                self.assertIn('declare_parameter("curved_lumen_type", "")', source)

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

    def test_cylinder_target_launch_values_are_finite_float_arrays(self):
        from launch import LaunchContext
        from launch_ros.actions import Node
        from launch_ros.parameter_descriptions import ParameterValue

        launch_files = (
            "simulation.launch.py",
            "evaluation_reference.launch.py",
            "evaluation_mppi_controller.launch.py",
        )
        cases = (
            (
                "s_curve_near_outlet_target",
                ["-0.008145713438018419", "0", "0.10901925297449416"],
                [-0.008145713438018419, 0.0, 0.10901925297449416],
            ),
            (
                "s_curve_middle_target",
                ["0.00000000000000002233456475320139", "0.0", "0.05999999999999997"],
                [2.233456475320139e-17, 0.0, 0.05999999999999997],
            ),
            ("circular_arc", ["0.015", "0.005", "0.100"], [0.015, 0.005, 0.1]),
            ("default", ["0.015", "0.005", "0.100"], [0.015, 0.005, 0.1]),
            ("integer-looking", ["1", "0", "2"], [1.0, 0.0, 2.0]),
            ("negative-zero", ["-1.0", "-0.0", "2.0"], [-1.0, -0.0, 2.0]),
            (
                "scientific",
                ["-8.145713438018419e-3", "0.0", "1.0901925297449416e-1"],
                [-0.008145713438018419, 0.0, 0.10901925297449416],
            ),
            ("nonnumeric", ["not-a-number", "0", "1"], None),
        )

        for launch_file in launch_files:
            module = load_launch_module(launch_file, f"{launch_file.replace('.', '_')}_target_array_under_test")
            for case_name, launch_values, expected in cases:
                with self.subTest(launch_file=launch_file, case=case_name):
                    launch_description = launch_description_with_temp_share(module)
                    target_values = []
                    for entity in launch_description.entities:
                        if not isinstance(entity, Node):
                            continue
                        for parameter_dict in getattr(entity, "_Node__parameters", ()):
                            for value in parameter_dict.values():
                                if isinstance(value, ParameterValue) and value.value_type == list[float]:
                                    target_values.append(value)
                    self.assertTrue(target_values, launch_file)
                    context = LaunchContext()
                    context.launch_configurations.update(
                        {
                            "cylinder_target_x": launch_values[0],
                            "cylinder_target_y": launch_values[1],
                            "cylinder_target_z": launch_values[2],
                            "target_x": launch_values[0],
                            "target_y": launch_values[1],
                            "target_z": launch_values[2],
                        }
                    )
                    for parameter_value in target_values:
                        if expected is None:
                            with self.assertRaises(ValueError):
                                parameter_value.evaluate(context)
                            continue
                        result = parameter_value.evaluate(context)
                        self.assertEqual(3, len(result))
                        self.assertTrue(all(isinstance(item, float) for item in result))
                        self.assertTrue(all(math.isfinite(item) for item in result))
                        self.assertEqual(expected, result)

    def test_curved_launch_arguments_are_forwarded_without_scalar_overrides(self):
        launch_files = (
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py",
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "evaluation_reference.launch.py",
            REPO_ROOT / "src" / "ctr_bringup" / "launch" / "evaluation_mppi_controller.launch.py",
        )
        forbidden_scalar_args = (
            '"lumen_radius"',
            '"ctr_outer_radius"',
            '"safety_margin"',
            '"centerline_sample_spacing"',
            '"curvature_radius"',
            '"arc_angle"',
            '"lateral_amplitude"',
        )
        for path in launch_files:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn('"enable_curved_lumen"', source)
                self.assertIn('"curved_lumen_type"', source)
                for scalar_arg in forbidden_scalar_args:
                    self.assertNotIn(f"DeclareLaunchArgument(\n                {scalar_arg}", source)

    def test_simulation_launch_reference_manager_condition_handles_curved_fixed_target(self):
        source = (REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py").read_text(encoding="utf-8")
        self.assertIn("reference_manager_condition", source)
        self.assertIn("enable_curved_lumen", source)
        self.assertIn("' == 'fixed_target'", source)

    def test_launch_bool_parser_accepts_only_textual_booleans(self):
        accepted = (
            (True, True),
            (False, False),
            ("true", True),
            ("false", False),
            ("True", True),
            ("False", False),
        )
        for value, expected in accepted:
            with self.subTest(value=value):
                self.assertIs(expected, parse_launch_bool(value, "enable_curved_lumen"))

        for value in ("1", "0", "", "yes", 1, 0, [], {}):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ParameterValidationError, "enable_curved_lumen"):
                    parse_launch_bool(value, "enable_curved_lumen")

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

    def test_reference_modes_are_explicitly_supported(self):
        for mode in ("fixed_target", "trajectory", "external_target"):
            with self.subTest(mode=mode):
                config = load_parameter_files(CONFIG_FILES)
                config = copy.deepcopy(config)
                config["reference"]["mode"] = mode
                errors = validate_project_config(config)
                self.assertEqual([], [error for error in errors if "reference.mode" in error])

    def test_reference_empty_or_non_string_mode_is_rejected(self):
        for mode in ("", None, 1, True):
            with self.subTest(mode=mode):
                config = load_parameter_files(CONFIG_FILES)
                config = copy.deepcopy(config)
                config["reference"]["mode"] = mode
                errors = validate_project_config(config)
                self.assertTrue(any("reference.mode" in error for error in errors), errors)

    def test_invalid_placeholder_goal_does_not_block_external_or_trajectory_modes(self):
        for mode in ("external_target", "trajectory"):
            with self.subTest(mode=mode):
                config = load_parameter_files(CONFIG_FILES)
                config = copy.deepcopy(config)
                config["reference"]["mode"] = mode
                config["goal"]["position"] = [float("nan"), 0.0]
                errors = validate_project_config(config)
                self.assertEqual([], [error for error in errors if "goal.position" in error])

    def test_fixed_target_mode_still_requires_valid_goal_position(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["reference"]["mode"] = "fixed_target"
        config["goal"]["position"] = [float("nan"), 0.0]
        errors = validate_project_config(config)
        self.assertTrue(any("goal.position" in error for error in errors), errors)

    def test_reference_mode_override_is_applied_before_goal_validation(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["reference"]["mode"] = "fixed_target"
        config["goal"]["position"] = [float("nan"), 0.0]

        fixed_errors = validate_project_config(project_config_with_overrides(config, reference_mode="fixed_target"))
        self.assertTrue(any("goal.position" in error for error in fixed_errors), fixed_errors)

        for mode in ("external_target", "trajectory"):
            with self.subTest(mode=mode):
                effective = project_config_with_overrides(config, reference_mode=mode)
                validate_or_raise(effective)
                self.assertEqual(mode, effective["reference"]["mode"])

    def test_lumen_goal_frame_mismatch_is_ignored_only_when_goal_is_placeholder(self):
        config = curved_lumen_config()
        config["reference"]["mode"] = "external_target"
        config["goal"]["frame_id"] = "placeholder_frame"
        errors = validate_project_config(config)
        self.assertEqual([], [error for error in errors if "goal.frame_id" in error and "selected lumen frame" in error])

    def test_simulation_launch_rejects_external_target_with_forced_reference_manager(self):
        from launch import LaunchContext

        module = load_launch_module("simulation.launch.py", "simulation_launch_external_conflict_under_test")
        context = LaunchContext()
        context.launch_configurations["reference_mode"] = "external_target"
        context.launch_configurations["start_reference_manager"] = "true"
        with self.assertRaisesRegex(RuntimeError, "external_target"):
            module._validate_reference_launch_arguments(context)

    def test_simulation_launch_accepts_external_target_without_reference_manager(self):
        from launch import LaunchContext

        module = load_launch_module("simulation.launch.py", "simulation_launch_external_ok_under_test")
        context = LaunchContext()
        context.launch_configurations["reference_mode"] = "external_target"
        context.launch_configurations["start_reference_manager"] = "false"
        self.assertEqual([], module._validate_reference_launch_arguments(context))

    def test_simulation_launch_rejects_invalid_reference_mode(self):
        from launch import LaunchContext

        module = load_launch_module("simulation.launch.py", "simulation_launch_invalid_reference_mode_under_test")
        context = LaunchContext()
        context.launch_configurations["reference_mode"] = "invalid"
        context.launch_configurations["start_reference_manager"] = "false"
        with self.assertRaisesRegex(RuntimeError, "reference_mode"):
            module._validate_reference_launch_arguments(context)

    def test_simulation_launch_forwards_effective_reference_mode_to_parameter_validator(self):
        source = (REPO_ROOT / "src" / "ctr_bringup" / "launch" / "simulation.launch.py").read_text(encoding="utf-8")
        validator_index = source.index('executable="parameter_validator_node"')
        simulator_index = source.index('executable="simulator_node"')
        validator_block = source[validator_index:simulator_index]
        self.assertIn('"reference_mode": ParameterValue(reference_mode, value_type=str)', validator_block)
        self.assertIn('"reference_type": ParameterValue(reference_type, value_type=str)', validator_block)
        self.assertIn(
            '"enable_cylindrical_lumen": ParameterValue(enable_cylindrical_lumen, value_type=str)',
            validator_block,
        )
        self.assertIn(
            '"enable_curved_lumen": ParameterValue(enable_curved_lumen, value_type=str)',
            validator_block,
        )
        self.assertIn('"cylinder_target_position": cylinder_target_position', validator_block)

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

    def test_default_curved_lumen_disabled_schema_validates(self):
        config = load_parameter_files(CONFIG_FILES)
        errors = validate_project_config(config)
        self.assertEqual([], [error for error in errors if "curved_lumen" in error])

    def test_disabled_curved_lumen_minimal_schema_is_accepted(self):
        config = load_parameter_files(CONFIG_FILES)
        config = copy.deepcopy(config)
        config["curved_lumen"] = {"enabled": False}
        errors = validate_project_config(config)
        self.assertEqual([], [error for error in errors if "curved_lumen" in error])

    def test_valid_curved_circular_arc_is_accepted(self):
        self.assertEqual([], [error for error in validate_project_config(curved_lumen_config("circular_arc")) if "curved_lumen" in error])

    def test_valid_curved_s_curve_is_accepted(self):
        self.assertEqual([], [error for error in validate_project_config(curved_lumen_config("s_curve")) if "curved_lumen" in error])

    def test_curved_lumen_matching_authoritative_frames_is_accepted(self):
        config = curved_lumen_config()
        config["curved_lumen"]["frame_id"] = "lumen_frame"
        config["robot"]["frames"]["base"] = "lumen_frame"
        config["goal"]["frame_id"] = "lumen_frame"
        config["reference"]["frame_id"] = "lumen_frame"
        errors = validate_project_config(config)
        self.assertEqual([], [error for error in errors if "selected lumen frame" in error])

    def test_curved_lumen_frame_mismatch_is_rejected(self):
        cases = (
            ("robot.frames.base", lambda config: config["robot"]["frames"].__setitem__("base", "robot_base")),
            ("goal.frame_id", lambda config: config["goal"].__setitem__("frame_id", "goal_frame")),
            ("reference.frame_id", lambda config: config["reference"].__setitem__("frame_id", "reference_frame")),
        )
        for label, mutate in cases:
            with self.subTest(label=label):
                config = curved_lumen_config()
                mutate(config)
                errors = validate_project_config(config)
                self.assertTrue(
                    any("selected lumen frame" in error and label in error for error in errors),
                    errors,
                )

    def test_zero_amplitude_s_curve_is_accepted(self):
        config = curved_lumen_config("s_curve")
        config["curved_lumen"]["s_curve"]["lateral_amplitude"] = 0.0
        self.assertEqual([], [error for error in validate_project_config(config) if "curved_lumen" in error])

    def test_curved_enabled_must_be_bool(self):
        config = curved_lumen_config()
        config["curved_lumen"]["enabled"] = "true"
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.enabled" in error for error in errors))

    def test_curved_invalid_type_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["type"] = "spiral"
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.type" in error for error in errors))

    def test_curved_empty_frame_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["frame_id"] = ""
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.frame_id" in error for error in errors))

    def test_curved_invalid_radius_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["lumen_radius"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.lumen_radius" in error for error in errors))

    def test_curved_invalid_ctr_outer_radius_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["ctr_outer_radius"] = -0.001
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.ctr_outer_radius" in error for error in errors))

    def test_curved_invalid_safety_margin_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["safety_margin"] = -0.001
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.safety_margin" in error for error in errors))

    def test_curved_unusable_physical_radius_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["lumen_radius"] = config["curved_lumen"]["ctr_outer_radius"]
        errors = validate_project_config(config)
        self.assertTrue(any("lumen_radius" in error and "ctr_outer_radius" in error for error in errors))

    def test_curved_unusable_safety_radius_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["lumen_radius"] = (
            config["curved_lumen"]["ctr_outer_radius"] + config["curved_lumen"]["safety_margin"]
        )
        errors = validate_project_config(config)
        self.assertTrue(any("usable radius" in error for error in errors))

    def test_curved_invalid_centerline_spacing_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["centerline_sample_spacing"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("centerline_sample_spacing" in error for error in errors))

    def test_curved_malformed_inlet_position_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["inlet_position"] = [0.0, 0.0]
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.circular_arc.inlet_position" in error for error in errors))

    def test_curved_non_finite_inlet_position_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["inlet_position"] = [0.0, float("nan"), 0.0]
        errors = validate_project_config(config)
        self.assertTrue(any("curved_lumen.circular_arc.inlet_position" in error for error in errors))

    def test_curved_zero_tangent_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["initial_tangent"] = [0.0, 0.0, 0.0]
        errors = validate_project_config(config)
        self.assertTrue(any("initial_tangent" in error and "non-zero" in error for error in errors))

    def test_curved_zero_bend_normal_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["bend_normal"] = [0.0, 0.0, 0.0]
        errors = validate_project_config(config)
        self.assertTrue(any("bend_normal" in error and "non-zero" in error for error in errors))

    def test_curved_parallel_tangent_normal_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["bend_normal"] = [0.0, 0.0, 2.0]
        errors = validate_project_config(config)
        self.assertTrue(any("must not be parallel" in error for error in errors))

    def test_curved_invalid_curvature_radius_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["curvature_radius"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("curvature_radius" in error for error in errors))

    def test_curved_zero_arc_angle_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["arc_angle"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("arc_angle" in error and "non-zero" in error for error in errors))

    def test_curved_non_finite_arc_angle_is_rejected(self):
        config = curved_lumen_config()
        config["curved_lumen"]["circular_arc"]["arc_angle"] = float("inf")
        errors = validate_project_config(config)
        self.assertTrue(any("arc_angle" in error for error in errors))

    def test_curved_invalid_s_curve_total_length_is_rejected(self):
        config = curved_lumen_config("s_curve")
        config["curved_lumen"]["s_curve"]["total_length"] = 0.0
        errors = validate_project_config(config)
        self.assertTrue(any("s_curve.total_length" in error for error in errors))

    def test_curved_non_finite_s_curve_amplitude_is_rejected(self):
        config = curved_lumen_config("s_curve")
        config["curved_lumen"]["s_curve"]["lateral_amplitude"] = float("nan")
        errors = validate_project_config(config)
        self.assertTrue(any("s_curve.lateral_amplitude" in error for error in errors))

    def test_cylinder_plus_curved_conflict_is_rejected(self):
        config = curved_lumen_config()
        config["cylindrical_lumen"]["enabled"] = True
        errors = validate_project_config(config)
        self.assertTrue(any("cannot both be true" in error for error in errors))

    def test_curved_generator_basis_failure_is_validation_failure(self):
        config = curved_lumen_config("s_curve")
        config["curved_lumen"]["s_curve"]["bend_plane_normal"] = config["curved_lumen"]["s_curve"]["initial_tangent"]
        errors = validate_project_config(config)
        self.assertTrue(any("s_curve.initial_tangent" in error and "parallel" in error for error in errors))

    def test_curved_error_message_contains_key_path(self):
        config = curved_lumen_config()
        config["curved_lumen"]["centerline_sample_spacing"] = -1.0
        errors = validate_project_config(config)
        self.assertTrue(any("`curved_lumen.centerline_sample_spacing`" in error for error in errors))

    def test_curved_validation_does_not_change_hardware_configuration(self):
        config = curved_lumen_config()
        original_hardware = copy.deepcopy(config["hardware"])
        validate_project_config(config)
        self.assertEqual(original_hardware, config["hardware"])

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
