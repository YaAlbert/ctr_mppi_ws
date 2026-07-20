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
    "safety_params.yaml",
    "tactile_params.yaml",
    "hardware_params.yaml",
)


def _config_paths():
    config_dir = Path(get_package_share_directory("ctr_bringup")) / "config"
    return validate_config_paths([str(config_dir / name) for name in CONFIG_NAMES])


def _placeholder(package, executable, name, runtime_mode):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=[
            {
                "config_paths": _config_paths(),
                "runtime_mode": runtime_mode,
                "enable_hardware_io": False,
            }
        ],
    )


def generate_launch_description():
    runtime_mode = LaunchConfiguration("runtime_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_mode",
                default_value="physical_hardware",
                description=(
                    "Physical hardware placeholder startup. TODO-HW-003 keeps hardware I/O disabled "
                    "until a driver and safety commissioning procedure exist."
                ),
            ),
            _placeholder("ctr_bringup", "parameter_validator_node", "parameter_validator", runtime_mode),
            _placeholder("ctr_model", "model_placeholder_node", "ctr_model_placeholder", runtime_mode),
            _placeholder("ctr_hardware", "physical_hardware_node", "ctr_physical_hardware", runtime_mode),
            _placeholder("ctr_state_estimator", "state_estimator_node", "ctr_state_estimator", runtime_mode),
            _placeholder("ctr_tactile", "tactile_placeholder_node", "ctr_tactile", runtime_mode),
            _placeholder("ctr_mppi_controller", "mppi_controller_placeholder_node", "ctr_mppi_controller", runtime_mode),
            _placeholder("ctr_safety", "safety_supervisor_node", "ctr_safety_supervisor", runtime_mode),
            _placeholder("ctr_viz", "visualization_node", "ctr_visualization", runtime_mode),
            _placeholder("ctr_evaluation", "evaluation_node", "ctr_evaluation", runtime_mode),
        ]
    )
