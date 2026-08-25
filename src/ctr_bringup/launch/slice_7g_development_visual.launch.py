"""Explicit simulator-only Slice 7G development visualization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ctr_bringup.parameter_validation import parse_launch_bool
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    target_source = LaunchConfiguration("target_source").perform(context)
    if target_source not in ("profile", "cli", "rviz"):
        raise RuntimeError("target_source must be profile, cli, or rviz")
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("ctr_bringup"))
    seed = LaunchConfiguration("seed")
    target_source = LaunchConfiguration("target_source")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    wait_for_target = LaunchConfiguration("wait_for_target")
    target_selection_timeout = LaunchConfiguration("target_selection_timeout")
    reference_mode = PythonExpression(
        ["'fixed_target' if '", target_source, "' == 'profile' else 'external_target'"]
    )
    start_reference_manager = PythonExpression(
        ["'true' if '", target_source, "' == 'profile' else 'false'"]
    )
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
            "start_reference_manager": start_reference_manager,
            "reference_mode": reference_mode,
            "enable_cylindrical_lumen": "false",
            "enable_curved_lumen": "true",
            "curved_lumen_type": "circular_arc",
            "cylinder_profile": "cylinder_fast",
            "mppi_random_seed": seed,
            "target_source": target_source,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "wait_for_target": wait_for_target,
            "target_selection_timeout": target_selection_timeout,
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
            DeclareLaunchArgument(
                "target_source",
                default_value="profile",
                description="Target source: profile (default), cli, or rviz.",
            ),
            DeclareLaunchArgument("target_x", default_value="0.015", description="CLI target X (m) in base_link."),
            DeclareLaunchArgument("target_y", default_value="0.005", description="CLI target Y (m) in base_link."),
            DeclareLaunchArgument("target_z", default_value="0.100", description="CLI target Z (m) in base_link."),
            DeclareLaunchArgument(
                "wait_for_target",
                default_value="true",
                description="In rviz mode, hold motion until a valid point is accepted.",
            ),
            DeclareLaunchArgument(
                "target_selection_timeout",
                default_value="0.0",
                description="Optional target-selection timeout in seconds; zero disables it.",
            ),
            OpaqueFunction(function=_require_development_opt_in),
            world_to_base,
            simulation,
            rviz,
        ]
    )
