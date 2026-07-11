"""ROS2 wrapper for the ROS-independent MPPI core."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise
from ctr_interfaces.msg import CtrControllerMetrics, CtrJointCommand, CtrState
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.mppi_core import MPPICore
from rclpy.node import Node


class MPPIControllerNode(Node):
    """Thin ROS2 adapter around `MPPICore`."""

    def __init__(self):
        super().__init__("mppi_controller_node")
        self.declare_parameter("config_paths", [])
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("target_position", [0.0, 0.0, 0.08])
        self.declare_parameter("publish_safe_command_for_simulation", False)

        config_paths = [str(path) for path in self.get_parameter("config_paths").value]
        if not config_paths:
            raise RuntimeError("TODO-ROS-001: mppi_controller_node requires `config_paths`.")

        self.config = load_parameter_files(config_paths)
        validate_or_raise(self.config)

        self.model = ApproximateCTRModel(self.config)
        self.core = MPPICore(self.config, self.model)
        self.target_tip = _vector3(self.get_parameter("target_position").value, "target_position")
        self.latest_state: CtrState | None = None
        self.frame_id = self.config["robot"]["frames"]["base"]
        self.publish_safe_for_sim = bool(self.get_parameter("publish_safe_command_for_simulation").value)

        self.state_sub = self.create_subscription(CtrState, "/ctr/state", self._on_state, 10)
        self.target_sub = self.create_subscription(PoseStamped, "/ctr/reference/tip", self._on_target, 10)
        self.command_pub = self.create_publisher(CtrJointCommand, "/ctr/mppi_command", 10)
        self.metrics_pub = self.create_publisher(CtrControllerMetrics, "/ctr/controller/metrics", 10)
        self.safe_command_pub = None
        if self.publish_safe_for_sim:
            self.safe_command_pub = self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)
            self.get_logger().warn(
                "publish_safe_command_for_simulation is enabled. Use only for simulation; hardware still requires safety supervisor."
            )

        control_frequency = float(self.config["mppi"]["control_frequency"])
        self.timer = self.create_timer(1.0 / control_frequency, self._on_timer)
        self.get_logger().info(
            "MPPI controller wrapper started. Enabled costs: tip, control, smoothness, terminal; advanced costs disabled."
        )

    def _on_state(self, msg: CtrState) -> None:
        if msg.valid:
            self.latest_state = msg
        else:
            self.get_logger().warn("Ignoring invalid /ctr/state message.")

    def _on_target(self, msg: PoseStamped) -> None:
        self.target_tip = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )

    def _on_timer(self) -> None:
        if self.latest_state is None:
            return

        try:
            result = self.core.solve(
                q=np.asarray(self.latest_state.q, dtype=float),
                q_dot=np.asarray(self.latest_state.q_dot, dtype=float),
                target_tip=self.target_tip,
            )
        except Exception as exc:
            self.get_logger().error(f"MPPI solve failed: {exc}")
            return

        stamp = self.get_clock().now().to_msg()
        command_msg = CtrJointCommand()
        command_msg.header.stamp = stamp
        command_msg.header.frame_id = self.frame_id
        command_msg.q_dot = [float(value) for value in result.command]
        command_msg.valid = True
        command_msg.diagnostic_status = result.diagnostic_status
        self.command_pub.publish(command_msg)
        if self.safe_command_pub is not None:
            self.safe_command_pub.publish(command_msg)

        metrics = CtrControllerMetrics()
        metrics.header.stamp = stamp
        metrics.header.frame_id = self.frame_id
        metrics.solve_time = float(result.solve_time)
        metrics.minimum_cost = float(result.minimum_cost)
        metrics.mean_cost = float(result.mean_cost)
        metrics.effective_sample_weight = float(result.effective_sample_weight)
        metrics.command_magnitude = float(result.command_magnitude)
        metrics.command_saturated = bool(result.command_saturated)
        metrics.valid = True
        metrics.diagnostic_status = result.diagnostic_status
        self.metrics_pub.publish(metrics)


def _vector3(values, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array


def main(args=None):
    rclpy.init(args=args)
    node = MPPIControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
