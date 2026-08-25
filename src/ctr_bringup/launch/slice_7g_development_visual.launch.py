"""Explicit simulator-only Slice 7G development visualization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import parse_launch_bool
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require_development_opt_in(context, *args, **kwargs):
    enabled = parse_launch_bool(
        LaunchConfiguration("development_simulation").perform(context),
        "development_simulation",
    )
    if not enabled:
        raise RuntimeError(
            "slice_7g_development_visual.launch.py requires development_simulation:=true"
        )
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("ctr_bringup"))
    seed = LaunchConfiguration("seed")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "simulation.launch.py")),
        launch_arguments={
            "runtime_mode": "simulation",
            "slice_7g_profile": "true",
            "development_simulation": "true",
            "enable_development_visualization": "true",
            "tactile_enabled": "true",
            "start_safety_supervisor": "true",
            "safety_supervisor_start_delay": "1.0",
            "start_mppi_controller": "true",
            "start_reference_manager": "true",
            "reference_mode": "fixed_target",
            "enable_cylindrical_lumen": "false",
            "enable_curved_lumen": "true",
            "curved_lumen_type": "circular_arc",
            "cylinder_profile": "cylinder_fast",
            "mppi_random_seed": seed,
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="slice_7g_development_rviz",
        output="screen",
        arguments=["-d", str(share / "config" / "slice_7g_development.rviz")],
    )
    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="slice_7g_world_to_fixed_base",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world", "--child-frame-id", "base_link",
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "development_simulation",
                default_value="false",
                description="Required explicit opt-in for the non-production simulation view.",
            ),
            DeclareLaunchArgument("seed", default_value="11"),
            OpaqueFunction(function=_require_development_opt_in),
            world_to_base,
            simulation,
            rviz,
        ]
    )
