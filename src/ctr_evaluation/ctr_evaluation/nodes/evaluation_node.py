"""Observation-only ROS2 evaluation node."""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath

from ctr_bringup.parameter_validation import load_parameter_files, validate_config_paths, validate_or_raise
from ctr_evaluation.experiment_recorder import EvaluationRecorderConfig, ExperimentRecorder, STATE_RECORDING
from ctr_interfaces.msg import CtrControllerMetrics, CtrJointCommand, CtrState
from ctr_interfaces.srv import StartExperiment, StopExperiment
from rclpy.node import Node
from rclpy.parameter import Parameter


ACTUATOR_COMMAND_TOPICS = ("/ctr/mppi_command", "/ctr/safe_command")
EVALUATION_COMMAND_PUBLISHERS: tuple[str, ...] = ()


class EvaluationNode(Node):
    """Record evaluation data without publishing actuator commands."""

    def __init__(self):
        super().__init__("evaluation_node")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("experiment_group", "")
        self.declare_parameter("controller_label", "")
        self.declare_parameter("baseline_result_dir", "")

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)
        self.project_config = load_parameter_files(config_paths)
        runtime_mode = str(self.get_parameter("runtime_mode").value)
        self.project_config.setdefault("runtime", {})["mode"] = runtime_mode
        validate_or_raise(self.project_config)

        self.recorder = ExperimentRecorder(
            config=EvaluationRecorderConfig.from_project_config(
                self.project_config,
                overrides={
                    "experiment_group": self.get_parameter("experiment_group").value,
                    "controller_label": self.get_parameter("controller_label").value,
                    "baseline_result_dir": self.get_parameter("baseline_result_dir").value,
                },
            ),
            project_config=self.project_config,
        )
        if not self.recorder.config.enabled:
            self.get_logger().warn("Evaluation node started with evaluation.enabled=false; services remain available.")

        self.state_sub = self.create_subscription(CtrState, "/ctr/state", self._on_state, 10)
        self.tip_sub = self.create_subscription(PoseStamped, "/ctr/tip", self._on_tip, 10)
        self.reference_tip_sub = self.create_subscription(PoseStamped, "/ctr/reference/tip", self._on_reference_tip, 10)
        self.reference_horizon_sub = self.create_subscription(NavPath, "/ctr/reference/horizon", self._on_horizon, 10)
        self.reference_path_sub = self.create_subscription(NavPath, "/ctr/reference/path", self._on_path, 10)
        self.mppi_command_sub = self.create_subscription(
            CtrJointCommand,
            "/ctr/mppi_command",
            lambda msg: self._on_command(msg, "mppi_command"),
            10,
        )
        self.safe_command_sub = self.create_subscription(
            CtrJointCommand,
            "/ctr/safe_command",
            lambda msg: self._on_command(msg, "safe_command"),
            10,
        )
        self.metrics_sub = self.create_subscription(
            CtrControllerMetrics,
            "/ctr/controller/metrics",
            self._on_controller_metrics,
            10,
        )
        self.trajectory_metrics_sub = self.create_subscription(
            DiagnosticArray,
            "/ctr/controller/trajectory_metrics",
            lambda msg: self.recorder.record_topic("/ctr/controller/trajectory_metrics"),
            10,
        )
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            lambda msg: self.recorder.record_topic("/diagnostics"),
            10,
        )

        self.start_srv = self.create_service(StartExperiment, "/ctr/start_experiment", self._start_experiment)
        self.stop_srv = self.create_service(StopExperiment, "/ctr/stop_experiment", self._stop_experiment)
        self.get_logger().info("Evaluation node ready; observation-only subscriptions and Start/StopExperiment services active.")

    def _start_experiment(self, request, response):
        if self.recorder.lifecycle_state == STATE_RECORDING:
            response.accepted = False
            response.message = "experiment already recording"
            return response
        try:
            metadata = parse_metadata(request.metadata)
            run_id = self.recorder.start(
                experiment_name=request.experiment_name,
                metadata=metadata,
                monotonic_time=self._now_seconds(),
            )
        except Exception as exc:
            response.accepted = False
            response.message = f"failed to start experiment: {exc}"
            return response
        response.accepted = True
        response.message = f"started evaluation run {run_id}"
        self.get_logger().info(response.message)
        return response

    def _stop_experiment(self, request, response):
        if self.recorder.lifecycle_state != STATE_RECORDING:
            response.accepted = False
            response.message = "no experiment is recording"
            return response
        try:
            result = self.recorder.stop(monotonic_time=self._now_seconds(), interrupted=False)
        except Exception as exc:
            response.accepted = False
            response.message = f"failed to finalize experiment: {exc}"
            self.get_logger().error(response.message)
            return response
        response.accepted = True
        response.message = f"completed evaluation run {result.run_id}: {result.run_dir}"
        self.get_logger().info(response.message)
        return response

    def _on_state(self, msg: CtrState) -> None:
        self.recorder.record_state(
            timestamp=stamp_seconds(msg.header.stamp),
            q=msg.q,
            q_dot=msg.q_dot,
            tip_position=[msg.tip_pose.position.x, msg.tip_pose.position.y, msg.tip_pose.position.z],
        )

    def _on_tip(self, msg: PoseStamped) -> None:
        self.recorder.record_tip(
            timestamp=stamp_seconds(msg.header.stamp),
            position=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        )

    def _on_reference_tip(self, msg: PoseStamped) -> None:
        self.recorder.record_reference(
            timestamp=stamp_seconds(msg.header.stamp),
            position=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        )

    def _on_horizon(self, msg: NavPath) -> None:
        if not msg.poses:
            self.recorder.record_horizon(timestamp=stamp_seconds(msg.header.stamp), count=0, first_point=[math.nan] * 3, final_point=[math.nan] * 3)
            return
        first = msg.poses[0].pose.position
        final = msg.poses[-1].pose.position
        self.recorder.record_horizon(
            timestamp=stamp_seconds(msg.header.stamp),
            count=len(msg.poses),
            first_point=[first.x, first.y, first.z],
            final_point=[final.x, final.y, final.z],
        )

    def _on_path(self, msg: NavPath) -> None:
        self.recorder.record_path(timestamp=stamp_seconds(msg.header.stamp), count=len(msg.poses))

    def _on_command(self, msg: CtrJointCommand, source: str) -> None:
        self.recorder.record_command(
            timestamp=stamp_seconds(msg.header.stamp),
            command=msg.q_dot,
            saturated="saturated" in (msg.diagnostic_status or "").lower(),
            source=source,
        )

    def _on_controller_metrics(self, msg: CtrControllerMetrics) -> None:
        self.recorder.record_solve_timing(
            timestamp=stamp_seconds(msg.header.stamp),
            solve_time=msg.solve_time,
            saturated=bool(msg.command_saturated),
        )

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1.0e-9

    def destroy_node(self) -> bool:
        if (
            getattr(self, "recorder", None) is not None
            and self.recorder.lifecycle_state == STATE_RECORDING
            and self.recorder.config.auto_finalize_on_shutdown
        ):
            elapsed = self._now_seconds() - (self.recorder.start_monotonic_time or self._now_seconds())
            interrupted = elapsed < self.recorder.config.configured_duration
            try:
                result = self.recorder.stop(monotonic_time=self._now_seconds(), interrupted=interrupted)
                self.get_logger().info(f"Auto-finalized evaluation run on shutdown: {result.run_dir}")
            except Exception as exc:
                self.get_logger().error(f"Evaluation auto-finalization failed: {exc}")
        return super().destroy_node()


def parse_metadata(text: str) -> dict[str, Any]:
    if not text:
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        data = yaml.safe_load(text)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError("experiment metadata must decode to a mapping")
    return data


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def point_from_pose(msg: PoseStamped) -> np.ndarray:
    return np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)


def run_evaluation_node_until_shutdown(rclpy_module, node_factory, *, args=None) -> None:
    """Run the evaluator and tolerate one known rclpy inactive-context shutdown race."""

    node = None
    rclpy_module.init(args=args)
    try:
        node = node_factory()
        try:
            rclpy_module.spin(node)
        except KeyboardInterrupt:
            pass
        except RuntimeError as exc:
            if not _is_inactive_context_message_conversion_error(rclpy_module, exc):
                raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy_module.ok():
            rclpy_module.shutdown()


def _is_inactive_context_message_conversion_error(rclpy_module, exc: RuntimeError) -> bool:
    if "Unable to convert call argument to Python object" not in str(exc):
        return False
    try:
        return not bool(rclpy_module.ok())
    except Exception:
        return False


def main(args=None):
    run_evaluation_node_until_shutdown(rclpy, EvaluationNode, args=args)
