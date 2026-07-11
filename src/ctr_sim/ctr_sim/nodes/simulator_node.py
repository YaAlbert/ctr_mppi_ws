"""ROS2 simulation loop for Milestone 3."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise
from ctr_interfaces.msg import CtrBackbone, CtrJointCommand, CtrJointState, CtrState
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_sim.simulation_core import CTRSimulationCore
from rclpy.node import Node


class CTRSimulatorNode(Node):
    """Receive safe q_dot commands, update q, and publish simulated CTR state."""

    def __init__(self):
        super().__init__("simulator_node")
        self.declare_parameter("config_paths", [])
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("target_position", [0.0, 0.0, 0.08])
        self.declare_parameter("command_timeout", 0.25)

        config_paths = [str(path) for path in self.get_parameter("config_paths").value]
        if not config_paths:
            raise RuntimeError("TODO-ROS-001: simulator_node requires `config_paths`.")

        self.config = load_parameter_files(config_paths)
        validate_or_raise(self.config)

        self.core = CTRSimulationCore(self.config)
        self.model = ApproximateCTRModel(self.config)

        simulation = self.config["simulation"]
        self.update_frequency = float(simulation["update_frequency"])
        self.dt = 1.0 / self.update_frequency
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.frame_id = self.config["robot"]["frames"]["base"]
        self.world_frame_id = self.config["robot"]["frames"]["world"]
        self.tip_frame_id = self.config["robot"]["frames"]["tip"]
        self.target_position = _vector3(self.get_parameter("target_position").value, "target_position")

        self.latest_command = np.zeros(6, dtype=float)
        self.command_valid = False
        self.command_saturated = False
        self.last_command_time = self.get_clock().now()
        self.last_diagnostic_status = "Initialized without command"

        self.command_sub = self.create_subscription(
            CtrJointCommand,
            "/ctr/safe_command",
            self._on_safe_command,
            10,
        )
        self.target_sub = self.create_subscription(
            PoseStamped,
            "/ctr/reference/tip",
            self._on_target,
            10,
        )

        self.joint_pub = self.create_publisher(CtrJointState, "/ctr/joint_state", 10)
        self.standard_joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.backbone_pub = self.create_publisher(CtrBackbone, "/ctr/backbone", 10)
        self.tip_pub = self.create_publisher(PoseStamped, "/ctr/tip", 10)
        self.state_pub = self.create_publisher(CtrState, "/ctr/state", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ctr/visualization", 10)

        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            "CTR simulator started: /ctr/safe_command -> /ctr/joint_state, /ctr/backbone, /ctr/tip, /ctr/state."
        )

    def _on_safe_command(self, msg: CtrJointCommand) -> None:
        try:
            command = np.asarray(msg.q_dot, dtype=float)
            if command.shape != (6,) or not np.all(np.isfinite(command)):
                raise ValueError("safe command must contain 6 finite values")
            if not msg.valid:
                raise ValueError("safe command validity flag is false")
        except ValueError as exc:
            self.command_valid = False
            self.latest_command = np.zeros(6, dtype=float)
            self.last_diagnostic_status = f"Rejected command: {exc}"
            self.get_logger().warn(self.last_diagnostic_status)
            return

        self.latest_command = command
        self.command_valid = True
        self.last_command_time = self.get_clock().now()
        self.last_diagnostic_status = msg.diagnostic_status or "Safe command accepted"

    def _on_target(self, msg: PoseStamped) -> None:
        self.target_position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        command_age = (now - self.last_command_time).nanoseconds * 1e-9
        command = self.latest_command if self.command_valid and command_age <= self.command_timeout else np.zeros(6)
        if self.command_valid and command_age > self.command_timeout:
            self.last_diagnostic_status = "Command timed out; applying zero velocity"

        step = self.core.step(command, self.dt)
        self.command_saturated = step.command_saturated
        model_result = self.model.forward_kinematics(step.q)

        stamp = now.to_msg()
        backbone_points = [_point_from_array(point) for point in model_result.backbone_points]
        tip_pose = self._tip_pose(stamp, model_result.tip_position)

        self.joint_pub.publish(self._joint_state_msg(stamp, step.q, step.q_dot))
        self.standard_joint_pub.publish(self._standard_joint_state_msg(stamp, step.q, step.q_dot))
        self.backbone_pub.publish(self._backbone_msg(stamp, backbone_points))
        self.tip_pub.publish(tip_pose)
        self.state_pub.publish(self._state_msg(stamp, step.q, step.q_dot, backbone_points, tip_pose))
        self.diagnostics_pub.publish(self._diagnostics_msg(stamp, command_age, model_result.diagnostic_status))
        self.marker_pub.publish(self._marker_array_msg(stamp, backbone_points))

    def _joint_state_msg(self, stamp, q: np.ndarray, q_dot: np.ndarray) -> CtrJointState:
        msg = CtrJointState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.insertion_position = [float(value) for value in q[:3]]
        msg.rotation_position = [float(value) for value in q[3:]]
        msg.joint_velocity = [float(value) for value in q_dot]
        msg.valid = True
        msg.diagnostic_status = self.last_diagnostic_status
        return msg

    def _standard_joint_state_msg(self, stamp, q: np.ndarray, q_dot: np.ndarray) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "rho1",
            "rho2",
            "rho3",
            "theta1",
            "theta2",
            "theta3",
        ]
        msg.position = [float(value) for value in q]
        msg.velocity = [float(value) for value in q_dot]
        return msg

    def _backbone_msg(self, stamp, points: list[Point]) -> CtrBackbone:
        msg = CtrBackbone()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.points = points
        msg.valid = True
        msg.diagnostic_status = self.last_diagnostic_status
        return msg

    def _tip_pose(self, stamp, tip_position: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position = _point_from_array(tip_position)
        msg.pose.orientation.w = 1.0
        return msg

    def _state_msg(
        self,
        stamp,
        q: np.ndarray,
        q_dot: np.ndarray,
        backbone_points: list[Point],
        tip_pose: PoseStamped,
    ) -> CtrState:
        msg = CtrState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.q = [float(value) for value in q]
        msg.q_dot = [float(value) for value in q_dot]
        msg.backbone = backbone_points
        msg.tip_pose = tip_pose.pose
        msg.tactile_force.x = 0.0
        msg.tactile_force.y = 0.0
        msg.tactile_force.z = 0.0
        msg.contact = False
        msg.valid = True
        msg.diagnostic_status = self.last_diagnostic_status
        return msg

    def _diagnostics_msg(self, stamp, command_age: float, model_status: str) -> DiagnosticArray:
        status = DiagnosticStatus()
        status.name = "ctr_sim/simulator_node"
        status.hardware_id = "simulation"
        status.level = DiagnosticStatus.OK if self.command_valid else DiagnosticStatus.WARN
        status.message = self.last_diagnostic_status
        status.values = [
            KeyValue(key="runtime_mode", value=str(self.get_parameter("runtime_mode").value)),
            KeyValue(key="command_age_s", value=f"{command_age:.6f}"),
            KeyValue(key="command_saturated", value=str(self.command_saturated)),
            KeyValue(key="model_status", value=model_status),
            KeyValue(key="TODO-SIM-001", value="Actuator nonidealities are not implemented."),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = stamp
        msg.status = [status]
        return msg

    def _marker_array_msg(self, stamp, backbone_points: list[Point]) -> MarkerArray:
        backbone = Marker()
        backbone.header.stamp = stamp
        backbone.header.frame_id = self.frame_id
        backbone.ns = "ctr_backbone"
        backbone.id = 0
        backbone.type = Marker.LINE_STRIP
        backbone.action = Marker.ADD
        backbone.scale.x = 0.002
        backbone.color = ColorRGBA(r=0.1, g=0.45, b=1.0, a=1.0)
        backbone.points = backbone_points

        tip = Marker()
        tip.header.stamp = stamp
        tip.header.frame_id = self.frame_id
        tip.ns = "ctr_tip"
        tip.id = 1
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.scale.x = 0.008
        tip.scale.y = 0.008
        tip.scale.z = 0.008
        tip.color = ColorRGBA(r=0.0, g=0.8, b=0.3, a=1.0)
        if backbone_points:
            tip.pose.position = backbone_points[-1]
        tip.pose.orientation.w = 1.0

        target = Marker()
        target.header.stamp = stamp
        target.header.frame_id = self.frame_id
        target.ns = "ctr_target"
        target.id = 2
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.scale.x = 0.01
        target.scale.y = 0.01
        target.scale.z = 0.01
        target.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=1.0)
        target.pose.position = _point_from_array(self.target_position)
        target.pose.orientation.w = 1.0

        return MarkerArray(markers=[backbone, tip, target])


def _point_from_array(values: Iterable[float]) -> Point:
    x, y, z = [float(value) for value in values]
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def _vector3(values, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array


def main(args=None):
    rclpy.init(args=args)
    node = CTRSimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
