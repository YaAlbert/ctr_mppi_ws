from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import validate_config_paths
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONFIG_NAMES = (
    "robot_params.yaml",
    "model_params.yaml",
    "mppi_params.yaml",
    "simulation_params.yaml",
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
    mppi_publish_safe_for_simulation = LaunchConfiguration("mppi_publish_safe_for_simulation")

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
                "mppi_publish_safe_for_simulation",
                default_value="false",
                description="Simulation-only bypass that publishes MPPI output to /ctr/safe_command.",
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
                        "publish_safe_command_for_simulation": mppi_publish_safe_for_simulation,
                    }
                ],
            ),
        ]
    )
