from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import validate_config_paths
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    reference_mode = LaunchConfiguration("reference_mode")
    reference_type = LaunchConfiguration("reference_type")
    publish_safe_command_for_simulation = LaunchConfiguration("publish_safe_command_for_simulation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_mode",
                default_value="simulation",
                description="Runtime mode for simulation-only evaluation controller startup.",
            ),
            DeclareLaunchArgument(
                "reference_mode",
                default_value="trajectory",
                description="Reference mode for the evaluation MPPI controller.",
            ),
            DeclareLaunchArgument(
                "reference_type",
                default_value="circle",
                description="Trajectory type for the evaluation MPPI controller.",
            ),
            DeclareLaunchArgument(
                "publish_safe_command_for_simulation",
                default_value="false",
                description="Simulation-only bypass from /ctr/mppi_command to /ctr/safe_command.",
            ),
            Node(
                package="ctr_mppi_controller",
                executable="mppi_controller_node",
                name="mppi_controller",
                output="screen",
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "target_position": [0.0, 0.0, 0.08],
                        "reference_mode": reference_mode,
                        "reference_type": reference_type,
                        "publish_safe_command_for_simulation": publish_safe_command_for_simulation,
                    }
                ],
            ),
        ]
    )
