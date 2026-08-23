"""Runtime parameter validator node."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ctr_bringup.parameter_validation import (
    load_parameter_files,
    project_config_with_overrides,
    validate_config_paths,
    validate_or_raise,
)
from ctr_bringup.placeholder_node import run_node_until_shutdown


REQUIRED_SECTIONS = ("robot", "model", "mppi", "simulation", "safety", "tactile", "hardware")


class ParameterValidatorNode(Node):
    """Validate project YAML after launch-level runtime overrides."""

    def __init__(self):
        super().__init__("parameter_validator_node")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("enable_hardware_io", False)
        self.declare_parameter("reference_mode", "")
        self.declare_parameter("reference_type", "")
        self.declare_parameter("enable_cylindrical_lumen", "")
        self.declare_parameter("enable_curved_lumen", "")
        self.declare_parameter("curved_lumen_type", "")
        self.declare_parameter("cylinder_profile", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("mppi_random_seed", "")
        self.declare_parameter("slice_7g_profile", False)
        self.declare_parameter("development_simulation", False)

        config_paths = validate_config_paths(self.get_parameter("config_paths").value, required=True)
        runtime_mode = self.get_parameter("runtime_mode").value
        enable_hardware_io = bool(self.get_parameter("enable_hardware_io").value)

        self.get_logger().info(
            "ctr_bringup: Validates project YAML files and exits only when the ROS2 process is stopped."
        )
        self.get_logger().info(f"runtime_mode={runtime_mode}; enable_hardware_io={enable_hardware_io}")

        config = load_parameter_files(config_paths)
        config = project_config_with_overrides(
            config,
            runtime_mode=runtime_mode,
            reference_mode=self.get_parameter("reference_mode").value,
            reference_type=self.get_parameter("reference_type").value,
            enable_cylindrical_lumen=self.get_parameter("enable_cylindrical_lumen").value,
            enable_curved_lumen=self.get_parameter("enable_curved_lumen").value,
            curved_lumen_type=self.get_parameter("curved_lumen_type").value,
            cylinder_target_position=self.get_parameter("cylinder_target_position").value,
            mppi_profile=self.get_parameter("cylinder_profile").value,
            mppi_random_seed=self.get_parameter("mppi_random_seed").value,
            slice_7g_profile=self.get_parameter("slice_7g_profile").value,
            development_simulation=self.get_parameter("development_simulation").value,
        )
        validate_or_raise(config)
        missing_sections = sorted(set(REQUIRED_SECTIONS) - set(config.keys()))
        if missing_sections:
            raise RuntimeError(f"Missing required config sections for ctr_bringup: {missing_sections}")
        self.get_logger().info(f"Loaded and validated {len(config_paths)} parameter files for ctr_bringup.")
        self.create_timer(30.0, self._heartbeat)

    def _heartbeat(self):
        self.get_logger().debug("ctr_bringup parameter validator alive.")


def main(args=None):
    run_node_until_shutdown(rclpy, ParameterValidatorNode, args=args)


if __name__ == "__main__":
    main()
