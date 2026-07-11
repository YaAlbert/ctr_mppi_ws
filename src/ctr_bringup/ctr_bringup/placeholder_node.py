"""Reusable ROS2 placeholder node for Milestone 1 packages."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from .parameter_validation import load_parameter_files, validate_or_raise


def create_placeholder_main(
    *,
    package_name: str,
    node_name: str,
    required_sections: Iterable[str] = (),
    note: str = "Placeholder node. Runtime behavior is intentionally not implemented in Milestone 1.",
):
    """Create a ROS2 main function for a package placeholder node."""

    def main(args=None):
        import rclpy
        from rclpy.node import Node

        class PlaceholderNode(Node):
            def __init__(self):
                super().__init__(node_name)
                self.declare_parameter("config_paths", [])
                self.declare_parameter("runtime_mode", "simulation")
                self.declare_parameter("placeholder_scope", package_name)
                self.declare_parameter("enable_hardware_io", False)

                config_paths = [str(path) for path in self.get_parameter("config_paths").value]
                runtime_mode = self.get_parameter("runtime_mode").value
                enable_hardware_io = bool(self.get_parameter("enable_hardware_io").value)

                self.get_logger().info(f"{package_name}: {note}")
                self.get_logger().info(f"runtime_mode={runtime_mode}; enable_hardware_io={enable_hardware_io}")

                if config_paths:
                    config = load_parameter_files(config_paths)
                    validate_or_raise(config)
                    missing_sections = sorted(set(required_sections) - set(config.keys()))
                    if missing_sections:
                        raise RuntimeError(f"Missing required config sections for {package_name}: {missing_sections}")
                    self.get_logger().info(
                        f"Loaded and validated {len(config_paths)} parameter files for {package_name}."
                    )
                else:
                    self.get_logger().warn(
                        f"TODO-ROS-001: {package_name} started without `config_paths`; "
                        "launch files should provide project YAML paths."
                    )

                if package_name == "ctr_hardware" and enable_hardware_io:
                    self.get_logger().warn(
                        "TODO-HW-003: hardware I/O is not implemented in Milestone 1."
                    )

                self.create_timer(30.0, self._heartbeat)

            def _heartbeat(self):
                self.get_logger().debug(f"{package_name} placeholder alive.")

        rclpy.init(args=args)
        node = PlaceholderNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    return main


def default_config_paths_from_repo(repo_root: str | Path) -> list[str]:
    """Return the standard Milestone 1 project YAML files from a repo root."""

    config_dir = Path(repo_root) / "config"
    return [
        str(config_dir / "robot_params.yaml"),
        str(config_dir / "model_params.yaml"),
        str(config_dir / "mppi_params.yaml"),
        str(config_dir / "simulation_params.yaml"),
        str(config_dir / "safety_params.yaml"),
        str(config_dir / "tactile_params.yaml"),
        str(config_dir / "hardware_params.yaml"),
    ]
