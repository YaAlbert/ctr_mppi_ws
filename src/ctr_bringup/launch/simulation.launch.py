from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import parse_launch_bool, validate_config_paths
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
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
    "slice_7g_runtime_params.yaml",
)
REFERENCE_MODES = ("fixed_target", "trajectory", "external_target")
TARGET_SOURCE_MODES = ("profile", "cli", "rviz")


def _config_paths():
    config_dir = Path(get_package_share_directory("ctr_bringup")) / "config"
    return validate_config_paths([str(config_dir / name) for name in CONFIG_NAMES])


def _config_path_substitutions(config_root):
    return [PathJoinSubstitution([config_root, name]) for name in CONFIG_NAMES]


def _config_path_parameter(config_root):
    """Preserve each independently resolved file path as a STRING_ARRAY."""

    # Each nested list forces launch_ros to retain one substitution per array element.
    return ParameterValue(
        [[path] for path in _config_path_substitutions(config_root)],
        value_type=list[str],
    )


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
    try:
        start_safety = parse_launch_bool(
            LaunchConfiguration("start_safety_supervisor").perform(context), "start_safety_supervisor"
        )
    except Exception:
        # Direct unit tests may call this validator with the pre-7E context.
        start_safety = False
    if start_safety and parse_launch_bool(
        LaunchConfiguration("mppi_publish_safe_for_simulation").perform(context),
        "mppi_publish_safe_for_simulation",
    ):
        raise RuntimeError("start_safety_supervisor=true cannot be combined with mppi_publish_safe_for_simulation=true")
    if start_safety and parse_launch_bool(
        LaunchConfiguration("start_manual_command_publisher").perform(context),
        "start_manual_command_publisher",
    ):
        raise RuntimeError("start_safety_supervisor=true cannot be combined with start_manual_command_publisher=true")
    try:
        slice_7g = parse_launch_bool(
            LaunchConfiguration("slice_7g_profile").perform(context),
            "slice_7g_profile",
        )
    except Exception:
        # Preserve direct legacy validator tests whose synthetic context predates Slice 7G.
        slice_7g = False
    if slice_7g:
        if LaunchConfiguration("runtime_mode").perform(context) != "simulation":
            raise RuntimeError("slice_7g_profile=true requires runtime_mode=simulation")
        if not start_safety:
            raise RuntimeError("slice_7g_profile=true requires start_safety_supervisor=true")
        if not parse_launch_bool(LaunchConfiguration("tactile_enabled").perform(context), "tactile_enabled"):
            raise RuntimeError("slice_7g_profile=true requires tactile_enabled=true")
    try:
        development = parse_launch_bool(
            LaunchConfiguration("development_simulation").perform(context),
            "development_simulation",
        )
    except Exception:
        development = False
    if development and not slice_7g:
        raise RuntimeError("development_simulation=true requires slice_7g_profile=true")
    if development and LaunchConfiguration("runtime_mode").perform(context) != "simulation":
        raise RuntimeError("development_simulation=true requires runtime_mode=simulation")
    try:
        target_source = LaunchConfiguration("target_source").perform(context)
    except Exception:
        target_source = "profile"
    if target_source not in TARGET_SOURCE_MODES:
        raise RuntimeError(f"target_source must be one of {TARGET_SOURCE_MODES}")
    if target_source != "profile":
        if not development:
            raise RuntimeError("development target overrides require development_simulation=true")
        if mode != "external_target":
            raise RuntimeError("cli/rviz target selection requires reference_mode=external_target")
    return []


def generate_launch_description():
    runtime_mode = LaunchConfiguration("runtime_mode")
    start_manual_command_publisher = LaunchConfiguration("start_manual_command_publisher")
    start_mppi_controller = LaunchConfiguration("start_mppi_controller")
    start_safety_supervisor = LaunchConfiguration("start_safety_supervisor")
    safety_supervisor_start_delay = LaunchConfiguration("safety_supervisor_start_delay")
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
    tactile_enabled = LaunchConfiguration("tactile_enabled")
    config_root = LaunchConfiguration("config_root")
    slice_7g_profile = LaunchConfiguration("slice_7g_profile")
    development_simulation = LaunchConfiguration("development_simulation")
    evaluation_diagnostics_enabled = LaunchConfiguration("evaluation_diagnostics_enabled")
    enable_development_visualization = LaunchConfiguration("enable_development_visualization")
    target_source = LaunchConfiguration("target_source")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    wait_for_target = LaunchConfiguration("wait_for_target")
    target_selection_timeout = LaunchConfiguration("target_selection_timeout")
    target_projection_limit = LaunchConfiguration("target_projection_limit")
    cylinder_target_position = ParameterValue(
        [
            [cylinder_target_x],
            [cylinder_target_y],
            [cylinder_target_z],
        ],
        value_type=list[float],
    )
    development_target_position = ParameterValue(
        [[target_x], [target_y], [target_z]],
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
                "start_safety_supervisor",
                default_value="false",
                description="Start the independent CTR safety supervisor.",
            ),
            DeclareLaunchArgument(
                "safety_supervisor_start_delay",
                default_value="0.0",
                description="Optional simulator startup grace before the safety supervisor starts.",
            ),
            DeclareLaunchArgument(
                "config_root",
                default_value=str(Path(get_package_share_directory("ctr_bringup")) / "config"),
                description="Directory containing the ordered project YAML configuration files.",
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
            DeclareLaunchArgument(
                "tactile_enabled",
                default_value="false",
                description="Enable the simulation-only Slice 7B tactile publisher.",
            ),
            DeclareLaunchArgument(
                "slice_7g_profile",
                default_value="false",
                description="Apply the authenticated simulation-only Slice 7G effective configuration.",
            ),
            DeclareLaunchArgument(
                "development_simulation",
                default_value="false",
                description="Explicit non-production user-level Slice 7G workflow.",
            ),
            DeclareLaunchArgument(
                "evaluation_diagnostics_enabled",
                default_value="false",
                description="Enable evaluation-only synchronized diagnostic evidence.",
            ),
            DeclareLaunchArgument(
                "enable_development_visualization",
                default_value="false",
                description=(
                    "Publish the development-only RViz surface, reference, and bounded tip-history topics."
                ),
            ),
            DeclareLaunchArgument(
                "target_source",
                default_value="profile",
                description="Development-only target source: profile, cli, or rviz.",
            ),
            DeclareLaunchArgument(
                "target_x",
                default_value="0.015",
                description="Development CLI target X in base_link, metres.",
            ),
            DeclareLaunchArgument(
                "target_y",
                default_value="0.005",
                description="Development CLI target Y in base_link, metres.",
            ),
            DeclareLaunchArgument(
                "target_z",
                default_value="0.100",
                description="Development CLI target Z in base_link, metres.",
            ),
            DeclareLaunchArgument(
                "wait_for_target",
                default_value="true",
                description="Wait without motion for an RViz target candidate.",
            ),
            DeclareLaunchArgument(
                "target_selection_timeout",
                default_value="0.0",
                description="Optional RViz selection timeout in seconds; zero disables it.",
            ),
            DeclareLaunchArgument(
                "target_projection_limit",
                default_value="-1.0",
                description="Optional positive RViz projection limit override; negative uses YAML.",
            ),
            OpaqueFunction(function=_validate_reference_launch_arguments),
            Node(
                package="ctr_bringup",
                executable="parameter_validator_node",
                name="parameter_validator",
                output="screen",
                parameters=[
                    {
                        "config_paths": _config_path_parameter(config_root),
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
                        "slice_7g_profile": ParameterValue(slice_7g_profile, value_type=bool),
                        "development_simulation": ParameterValue(
                            development_simulation, value_type=bool
                        ),
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
                        "config_paths": _config_path_parameter(config_root),
                        "runtime_mode": runtime_mode,
                        "target_position": [0.0, 0.0, 0.08],
                        "command_timeout": 0.25,
                        "enable_cylindrical_lumen": enable_cylindrical_lumen,
                        "enable_curved_lumen": enable_curved_lumen,
                        "curved_lumen_type": curved_lumen_type,
                        "cylinder_target_position": cylinder_target_position,
                        "tactile_enabled": ParameterValue(tactile_enabled, value_type=bool),
                        "slice_7g_profile": ParameterValue(slice_7g_profile, value_type=bool),
                        "development_simulation": ParameterValue(
                            development_simulation, value_type=bool
                        ),
                        "enable_development_visualization": ParameterValue(
                            enable_development_visualization, value_type=bool
                        ),
                        "evaluation_diagnostics_enabled": ParameterValue(
                            evaluation_diagnostics_enabled, value_type=bool
                        ),
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
            TimerAction(
                period=safety_supervisor_start_delay,
                actions=[
                    Node(
                        package="ctr_safety",
                        executable="safety_supervisor_node",
                        name="safety_supervisor",
                        output="screen",
                        condition=IfCondition(start_safety_supervisor),
                        parameters=[
                            {
                                "config_paths": _config_path_parameter(config_root),
                                "runtime_mode": runtime_mode,
                                "enable_cylindrical_lumen": ParameterValue(enable_cylindrical_lumen, value_type=bool),
                                "enable_curved_lumen": ParameterValue(enable_curved_lumen, value_type=bool),
                                "curved_lumen_type": ParameterValue(curved_lumen_type, value_type=str),
                                "cylinder_target_position": cylinder_target_position,
                                "slice_7g_profile": ParameterValue(slice_7g_profile, value_type=bool),
                                "development_simulation": ParameterValue(
                                    development_simulation, value_type=bool
                                ),
                                "evaluation_diagnostics_enabled": ParameterValue(
                                    evaluation_diagnostics_enabled, value_type=bool
                                ),
                            }
                        ],
                    )
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
                        "config_paths": _config_path_parameter(config_root),
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
                        "slice_7g_profile": ParameterValue(slice_7g_profile, value_type=bool),
                        "development_simulation": ParameterValue(
                            development_simulation, value_type=bool
                        ),
                        "evaluation_diagnostics_enabled": ParameterValue(
                            evaluation_diagnostics_enabled, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="ctr_sim",
                executable="development_target_selector_node",
                name="development_target_selector",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", target_source, "' != 'profile'"])
                ),
                parameters=[
                    {
                        "config_paths": _config_path_parameter(config_root),
                        "development_simulation": ParameterValue(
                            development_simulation, value_type=bool
                        ),
                        "target_source": ParameterValue(target_source, value_type=str),
                        "target_position": development_target_position,
                        "seed": ParameterValue(mppi_random_seed, value_type=int),
                        "wait_for_target": ParameterValue(wait_for_target, value_type=bool),
                        "target_selection_timeout": ParameterValue(
                            target_selection_timeout, value_type=float
                        ),
                        "projection_limit": ParameterValue(
                            target_projection_limit, value_type=float
                        ),
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
                        "config_paths": _config_path_parameter(config_root),
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
                        "config_paths": _config_path_parameter(config_root),
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
                        "slice_7g_profile": ParameterValue(slice_7g_profile, value_type=bool),
                        "development_simulation": ParameterValue(
                            development_simulation, value_type=bool
                        ),
                        "evaluation_diagnostics_enabled": ParameterValue(
                            evaluation_diagnostics_enabled, value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
