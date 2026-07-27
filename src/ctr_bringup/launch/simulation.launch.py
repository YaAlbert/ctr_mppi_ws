from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import validate_config_paths
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def _config_paths():
    config_dir = Path(get_package_share_directory("ctr_bringup")) / "config"
    return validate_config_paths([str(config_dir / name) for name in CONFIG_NAMES])


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
    reference_manager_condition = IfCondition(
        PythonExpression(
            [
                "'",
                start_reference_manager,
                "' == 'true' or '",
                reference_mode,
                "' == 'trajectory'",
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
                description="Reference operating mode: fixed_target or trajectory.",
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
            Node(
                package="ctr_bringup",
                executable="parameter_validator_node",
                name="parameter_validator",
                output="screen",
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "enable_hardware_io": False,
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
                    }
                ],
            ),
        ]
    )
