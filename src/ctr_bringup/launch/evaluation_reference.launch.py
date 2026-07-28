from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import validate_config_paths
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
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


def _config_paths():
    config_dir = Path(get_package_share_directory("ctr_bringup")) / "config"
    return validate_config_paths([str(config_dir / name) for name in CONFIG_NAMES])


def generate_launch_description():
    runtime_mode = LaunchConfiguration("runtime_mode")
    reference_mode = LaunchConfiguration("reference_mode")
    reference_type = LaunchConfiguration("reference_type")
    trajectory_start_policy = LaunchConfiguration("trajectory_start_policy")
    scheduled_reference_epoch = LaunchConfiguration("scheduled_reference_epoch")
    enable_cylindrical_lumen = LaunchConfiguration("enable_cylindrical_lumen")
    enable_curved_lumen = LaunchConfiguration("enable_curved_lumen")
    curved_lumen_type = LaunchConfiguration("curved_lumen_type")
    cylinder_profile = LaunchConfiguration("cylinder_profile")
    cylinder_target_x = LaunchConfiguration("cylinder_target_x")
    cylinder_target_y = LaunchConfiguration("cylinder_target_y")
    cylinder_target_z = LaunchConfiguration("cylinder_target_z")
    mppi_random_seed = LaunchConfiguration("mppi_random_seed")
    cylinder_target_position = ParameterValue(
        PythonExpression(["[", cylinder_target_x, ", ", cylinder_target_y, ", ", cylinder_target_z, "]"]),
        value_type=list[float],
    )

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
            DeclareLaunchArgument(
                "enable_cylindrical_lumen",
                default_value="false",
                description="Enable simulation-only fixed point goal from the cylindrical-lumen config.",
            ),
            DeclareLaunchArgument(
                "enable_curved_lumen",
                default_value="false",
                description="Enable simulation-only fixed point goal from the curved-lumen config.",
            ),
            DeclareLaunchArgument(
                "curved_lumen_type",
                default_value="",
                description="Optional curved-lumen type override: circular_arc or s_curve.",
            ),
            DeclareLaunchArgument(
                "cylinder_profile",
                default_value="",
                description="Optional MPPI profile name used to size fixed/trajectory horizons.",
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
                description="Optional MPPI random seed override for shared metadata.",
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
                        "enable_cylindrical_lumen": enable_cylindrical_lumen,
                        "enable_curved_lumen": enable_curved_lumen,
                        "curved_lumen_type": curved_lumen_type,
                        "cylinder_profile": cylinder_profile,
                        "cylinder_target_position": cylinder_target_position,
                        "mppi_random_seed": mppi_random_seed,
                    }
                ],
            ),
        ]
    )
