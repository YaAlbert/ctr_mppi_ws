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
    trajectory_start_policy = LaunchConfiguration("trajectory_start_policy")
    scheduled_reference_epoch = LaunchConfiguration("scheduled_reference_epoch")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_mode",
                default_value="simulation",
                description="Runtime mode for simulation-only evaluation reference startup.",
            ),
            DeclareLaunchArgument(
                "reference_mode",
                default_value="trajectory",
                description="Reference mode for the evaluation reference manager.",
            ),
            DeclareLaunchArgument(
                "reference_type",
                default_value="circle",
                description="Trajectory type for the evaluation reference manager.",
            ),
            DeclareLaunchArgument(
                "trajectory_start_policy",
                default_value="node_start",
                description="Reference trajectory start policy: node_start or scheduled_time.",
            ),
            DeclareLaunchArgument(
                "scheduled_reference_epoch",
                default_value="0.0",
                description="Absolute ROS-clock reference epoch in seconds for scheduled_time.",
            ),
            Node(
                package="ctr_mppi_controller",
                executable="reference_manager_node",
                name="reference_manager",
                output="screen",
                parameters=[
                    {
                        "config_paths": _config_paths(),
                        "runtime_mode": runtime_mode,
                        "reference_mode": reference_mode,
                        "reference_type": reference_type,
                        "trajectory_start_policy": trajectory_start_policy,
                        "scheduled_reference_epoch": scheduled_reference_epoch,
                    }
                ],
            ),
        ]
    )
