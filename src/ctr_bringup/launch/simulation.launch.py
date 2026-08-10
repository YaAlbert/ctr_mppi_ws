from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import parse_launch_bool, validate_config_paths
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIG_NAMES = (
    "robot_params.yaml",
    "model_params.yaml",
    "mppi_params.yaml",
    "simulation_params.yaml",
    "evaluation_params.yaml",
    "safety_params.yaml",
    "tactile_params.yaml",
    "hardware_params.yaml",
)
REFERENCE_MODES = ("fixed_target", "trajectory", "external_target")


def _config_paths():
    config_dir = Path(get_package_share_directory("ctr_bringup")) / "config"
    return validate_config_paths([str(config_dir / name) for name in CONFIG_NAMES])


def _validate_reference_launch_arguments(context, *args, **kwargs):
    mode = LaunchConfiguration("reference_mode").perform(context)
    if mode not in REFERENCE_MODES:
        raise RuntimeError(f"reference_mode must be one of {REFERENCE_MODES}")
    start_manager = parse_launch_bool(
        LaunchConfiguration("start_reference_manager").perform(context),
        "start_reference_manager",
    )
    if mode == "external_target" and start_manager:
        raise RuntimeError("reference_mode=external_target cannot be combined with start_reference_manager=true")
    return []


def generate_launch_description():
    runtime_mode = LaunchConfiguration("runtime_mode")
    start_manual_command_publisher = LaunchConfiguration("start_manual_command_publisher")
    start_mppi_controller = LaunchConfiguration("start_mppi_controller")
    start_reference_manager = LaunchConfiguration("start_reference_manager")
    reference_mode = LaunchConfiguration("reference_mode")
    reference_type = LaunchConfiguration("reference_type")
    mppi_publish_safe_for_simulation = LaunchConfiguration("mppi_publish_safe_for_simulation")
    start_evaluation = LaunchConfiguration("start_evaluation")
    evaluation_experiment_group = LaunchConfiguration("evaluation_experiment_group")
    evaluation_controller_label = LaunchConfiguration("evaluation_controller_label")
    evaluation_baseline_result_dir = LaunchConfiguration("evaluation_baseline_result_dir")
    evaluation_output_root = LaunchConfiguration("evaluation_output_root")
    enable_cylindrical_lumen = LaunchConfiguration("enable_cylindrical_lumen")
    enable_curved_lumen = LaunchConfiguration("enable_curved_lumen")
    curved_lumen_type = LaunchConfiguration("curved_lumen_type")
    cylinder_profile = LaunchConfiguration("cylinder_profile")
    cylinder_target_x = LaunchConfiguration("cylinder_target_x")
    cylinder_target_y = LaunchConfiguration("cylinder_target_y")
    cylinder_target_z = LaunchConfiguration("cylinder_target_z")
    mppi_random_seed = LaunchConfiguration("mppi_random_seed")
    run_role = LaunchConfiguration("run_role")
    cylinder_target_position = ParameterValue(
        [
            [cylinder_target_x],
            [cylinder_target_y],
            [cylinder_target_z],
        ],
        value_type=list[float],
    )
    reference_manager_condition = IfCondition(
        PythonExpression(
            [
                "'",
                reference_mode,
                "' != 'external_target' and (",
                "'",
                start_reference_manager,
                "'.lower() == 'true' or '",
                reference_mode,
                "' == 'trajectory' or '",
                reference_mode,
                "' == 'fixed_target' and ('",
                enable_cylindrical_lumen,
                "'.lower() == 'true' or '",
                enable_curved_lumen,
                "'.lower() == 'true'))",
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_mode",
                default_value="simulation",
                description="Runtime mode for the CTR simulation loop.",
            ),
            DeclareLaunchArgument(
                "start_manual_command_publisher",
                default_value="false",
                description="Start a fixed safe-command publisher for simulation smoke tests.",
            ),
            DeclareLaunchArgument(
                "start_mppi_controller",
                default_value="false",
                description="Start the Milestone 4 MPPI ROS2 wrapper.",
            ),
            DeclareLaunchArgument(
                "start_reference_manager",
                default_value="false",
                description="Start the Milestone 5 reference manager. Trajectory mode starts it automatically.",
            ),
            DeclareLaunchArgument(
                "reference_mode",
                default_value="fixed_target",
                description="Reference operating mode: fixed_target, trajectory, or external_target.",
            ),
            DeclareLaunchArgument(
                "reference_type",
                default_value="circle",
                description="Trajectory type for reference manager trajectory mode: circle, ellipse, or helix.",
            ),
            DeclareLaunchArgument(
                "mppi_publish_safe_for_simulation",
                default_value="false",
                description="Simulation-only bypass that publishes MPPI output to /ctr/safe_command.",
            ),
            DeclareLaunchArgument(
                "start_evaluation",
                default_value="false",
                description="Start the observation-only evaluation recorder.",
            ),
            DeclareLaunchArgument(
                "evaluation_experiment_group",
                default_value="",
                description="Optional override for evaluation.experiment_group.",
            ),
            DeclareLaunchArgument(
                "evaluation_controller_label",
                default_value="",
                description="Optional override for evaluation.controller_label.",
            ),
            DeclareLaunchArgument(
                "evaluation_baseline_result_dir",
                default_value="",
                description="Optional baseline result directory for automatic comparison.",
            ),
            DeclareLaunchArgument(
                "evaluation_output_root",
                default_value="",
                description="Optional override for evaluation.output_root.",
            ),
            DeclareLaunchArgument(
                "enable_cylindrical_lumen",
                default_value="false",
                description="Enable simulation-only straight cylindrical-lumen point-goal navigation.",
            ),
            DeclareLaunchArgument(
                "enable_curved_lumen",
                default_value="false",
                description="Enable simulation-only curved-lumen point-goal runtime wiring.",
            ),
            DeclareLaunchArgument(
                "curved_lumen_type",
                default_value="",
                description="Optional curved-lumen type override: circular_arc or s_curve.",
            ),
            DeclareLaunchArgument(
                "cylinder_profile",
                default_value="",
                description="Optional MPPI profile name, for example cylinder_fast.",
            ),
            DeclareLaunchArgument(
                "cylinder_target_x",
                default_value="0.015",
                description="Simulation-only cylinder point-goal x coordinate in meters.",
            ),
            DeclareLaunchArgument(
                "cylinder_target_y",
                default_value="0.005",
                description="Simulation-only cylinder point-goal y coordinate in meters.",
            ),
            DeclareLaunchArgument(
                "cylinder_target_z",
                default_value="0.100",
                description="Simulation-only cylinder point-goal z coordinate in meters.",
            ),
            DeclareLaunchArgument(
                "mppi_random_seed",
                default_value="-1",
                description="Optional MPPI random seed override.",
            ),
            DeclareLaunchArgument(
                "run_role",
                default_value="",
                description="Optional evaluation run role metadata.",
            ),
            OpaqueFunction(function=_validate_reference_launch_arguments),
            Node(
                package="ctr_bringup",
                executable="parameter_validator_node",
                name="parameter_validator",
                output="screen",
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": ParameterValue(runtime_mode, value_type=str),
                        "enable_hardware_io": False,
                        "reference_mode": ParameterValue(reference_mode, value_type=str),
                        "reference_type": ParameterValue(reference_type, value_type=str),
                        "enable_cylindrical_lumen": ParameterValue(enable_cylindrical_lumen, value_type=str),
                        "enable_curved_lumen": ParameterValue(enable_curved_lumen, value_type=str),
                        "curved_lumen_type": ParameterValue(curved_lumen_type, value_type=str),
                        "cylinder_profile": ParameterValue(cylinder_profile, value_type=str),
                        "cylinder_target_position": cylinder_target_position,
                        "mppi_random_seed": ParameterValue(mppi_random_seed, value_type=str),
                    }
                ],
            ),
            Node(
                package="ctr_sim",
                executable="simulator_node",
                name="ctr_simulator",
                output="screen",
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "target_position": [0.0, 0.0, 0.08],
                        "command_timeout": 0.25,
                        "enable_cylindrical_lumen": enable_cylindrical_lumen,
                        "enable_curved_lumen": enable_curved_lumen,
                        "curved_lumen_type": curved_lumen_type,
                        "cylinder_target_position": cylinder_target_position,
                    }
                ],
            ),
            Node(
                package="ctr_sim",
                executable="manual_command_publisher",
                name="manual_command_publisher",
                output="screen",
                condition=IfCondition(start_manual_command_publisher),
                parameters=[
                    {
                        "q_dot": [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "publish_rate": 20.0,
                        "duration": 5.0,
                        "repeat": False,
                    }
                ],
            ),
            Node(
                package="ctr_mppi_controller",
                executable="mppi_controller_node",
                name="mppi_controller",
                output="screen",
                condition=IfCondition(start_mppi_controller),
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "target_position": [0.0, 0.0, 0.08],
                        "reference_mode": reference_mode,
                        "reference_type": reference_type,
                        "publish_safe_command_for_simulation": mppi_publish_safe_for_simulation,
                        "enable_cylindrical_lumen": enable_cylindrical_lumen,
                        "enable_curved_lumen": enable_curved_lumen,
                        "curved_lumen_type": curved_lumen_type,
                        "cylinder_profile": cylinder_profile,
                        "cylinder_target_position": cylinder_target_position,
                        "mppi_random_seed": mppi_random_seed,
                    }
                ],
            ),
            Node(
                package="ctr_mppi_controller",
                executable="reference_manager_node",
                name="reference_manager",
                output="screen",
                condition=reference_manager_condition,
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "reference_mode": reference_mode,
                        "reference_type": reference_type,
                        "enable_cylindrical_lumen": enable_cylindrical_lumen,
                        "enable_curved_lumen": enable_curved_lumen,
                        "curved_lumen_type": curved_lumen_type,
                        "cylinder_profile": cylinder_profile,
                        "cylinder_target_position": cylinder_target_position,
                        "mppi_random_seed": mppi_random_seed,
                    }
                ],
            ),
            Node(
                package="ctr_evaluation",
                executable="evaluation_node",
                name="evaluation_node",
                output="screen",
                condition=IfCondition(start_evaluation),
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "experiment_group": evaluation_experiment_group,
                        "controller_label": evaluation_controller_label,
                        "baseline_result_dir": evaluation_baseline_result_dir,
                        "output_root": evaluation_output_root,
                        "enable_cylindrical_lumen": enable_cylindrical_lumen,
                        "enable_curved_lumen": enable_curved_lumen,
                        "curved_lumen_type": curved_lumen_type,
                        "cylinder_profile": cylinder_profile,
                        "cylinder_target_position": cylinder_target_position,
                        "mppi_random_seed": mppi_random_seed,
                        "run_role": run_role,
                    }
                ],
            ),
        ]
    )
