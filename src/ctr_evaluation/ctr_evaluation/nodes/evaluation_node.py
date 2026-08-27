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
from ctr_bringup.slice_7g_profile import (
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
)
from ctr_evaluation.experiment_recorder import EvaluationRecorderConfig, ExperimentRecorder, STATE_RECORDING
from ctr_interfaces.msg import (
    CtrControllerMetrics,
    CtrJointCommand,
    CtrSafetyStatus,
    CtrState,
    CtrTactileState,
)
from ctr_interfaces.srv import StartExperiment, StopExperiment
from ctr_mppi_controller.lumen_factory import config_with_lumen_overrides
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
        self.declare_parameter("output_root", "")
        self.declare_parameter("enable_cylindrical_lumen", False)
        self.declare_parameter("enable_curved_lumen", False)
        self.declare_parameter("curved_lumen_type", "")
        self.declare_parameter("cylinder_profile", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("mppi_random_seed", -1)
        self.declare_parameter("run_role", "")
        self.declare_parameter("slice_7g_profile", False)
        self.declare_parameter("development_simulation", False)
        self.declare_parameter("evaluation_diagnostics_enabled", False)

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)
        raw_config = load_parameter_files(config_paths)
        self.project_config = config_with_lumen_overrides(
            raw_config,
            enable_cylindrical_lumen=_bool_value(self.get_parameter("enable_cylindrical_lumen").value),
            enable_curved_lumen=_bool_value(self.get_parameter("enable_curved_lumen").value),
            curved_lumen_type=str(self.get_parameter("curved_lumen_type").value or ""),
            target=_optional_vector3_parameter(self.get_parameter("cylinder_target_position").value),
            cylinder_profile=str(self.get_parameter("cylinder_profile").value or ""),
            random_seed=_optional_seed(self.get_parameter("mppi_random_seed").value),
        )
        runtime_mode = str(self.get_parameter("runtime_mode").value)
        self.project_config.setdefault("runtime", {})["mode"] = runtime_mode
        development_enabled = _bool_value(
            self.get_parameter("development_simulation").value
        )
        diagnostics_enabled = _bool_value(
            self.get_parameter("evaluation_diagnostics_enabled").value
        )
        if diagnostics_enabled and not development_enabled:
            raise ValueError("evaluation diagnostics require explicit development_simulation mode")
        self.project_config = (
            apply_slice_7g_development_simulation_profile(
                self.project_config, enabled=True
            )
            if development_enabled
            else apply_slice_7g_simulation_profile(
                self.project_config,
                enabled=_bool_value(self.get_parameter("slice_7g_profile").value),
            )
        )
        output_root = str(self.get_parameter("output_root").value or "")
        if output_root:
            self.project_config.setdefault("evaluation", {})["output_root"] = output_root
        self.project_config.setdefault("evaluation", {})[
            "diagnostic_data_collection"
        ] = diagnostics_enabled
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
        self.mppi_evaluation_diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            "/ctr/evaluation/mppi_diagnostics",
            self._on_mppi_evaluation_diagnostics,
            10,
        )
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            lambda msg: self.recorder.record_topic("/diagnostics"),
            10,
        )
        self.tactile_sub = self.create_subscription(
            CtrTactileState,
            "/ctr/tactile/state",
            self._on_tactile,
            10,
        )
        self.safety_status_sub = self.create_subscription(
            CtrSafetyStatus,
            "/ctr/safety/status",
            self._on_safety_status,
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
        self.recorder.record_diagnostic_event("stop_service_callback", phase="start", status="entered")
        if self.recorder.lifecycle_state != STATE_RECORDING:
            response.accepted = False
            response.message = "no experiment is recording"
            self.recorder.record_diagnostic_event("stop_service_callback", phase="end", status="rejected")
            return response
        try:
            self.recorder.record_diagnostic_event("recorder_finalization", phase="start", status="started")
            result = self.recorder.stop(monotonic_time=self._now_seconds(), interrupted=False)
            self.recorder.record_diagnostic_event("recorder_finalization", phase="end", status="ok")
        except Exception as exc:
            self.recorder.record_diagnostic_event(
                "stop_service_callback",
                phase="error",
                status="error",
                error_type=type(exc).__name__,
                error_message=str(exc),
            )
            response.accepted = False
            response.message = f"failed to finalize experiment: {exc}"
            self.get_logger().error(response.message)
            return response
        self.recorder.record_diagnostic_event("stop_response_construction", phase="start", status="started")
        response.accepted = True
        response.message = f"completed evaluation run {result.run_id}: {result.run_dir}"
        self.recorder.record_diagnostic_event("stop_response_construction", phase="end", status="ok")
        self.get_logger().info(response.message)
        self.recorder.record_diagnostic_event("stop_service_callback", phase="end", status="ok")
        return response

    def _on_state(self, msg: CtrState) -> None:
        self.recorder.record_state(
            timestamp=stamp_seconds(msg.header.stamp),
            q=msg.q,
            q_dot=msg.q_dot,
            tip_position=[msg.tip_pose.position.x, msg.tip_pose.position.y, msg.tip_pose.position.z],
            backbone_points=[[point.x, point.y, point.z] for point in msg.backbone],
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

    def _on_mppi_evaluation_diagnostics(self, msg: DiagnosticArray) -> None:
        if len(msg.status) != 1:
            self.recorder.record_invalid_mppi_diagnostic()
            return
        status = msg.status[0]
        if status.name != "ctr_mppi_evaluation_iteration_v1":
            self.recorder.record_invalid_mppi_diagnostic()
            return
        values = {item.key: item.value for item in status.values}
        if len(values) != len(status.values):
            self.recorder.record_invalid_mppi_diagnostic()
            return
        self.recorder.record_mppi_diagnostic(
            timestamp=stamp_seconds(msg.header.stamp),
            values=values,
        )

    def _on_tactile(self, msg: CtrTactileState) -> None:
        self.recorder.record_slice_7g_tactile(valid=bool(msg.valid), source=str(msg.source))
        self.recorder.record_tactile_evidence(
            timestamp=stamp_seconds(msg.header.stamp),
            received_timestamp=self._now_seconds(),
            frame_id=str(msg.header.frame_id),
            source=str(msg.source),
            raw_values=list(msg.raw_values),
            filtered_values=list(msg.filtered_values),
            force_magnitude=float(msg.force_magnitude),
            clearance_m=float(msg.clearance_m),
            contact=bool(msg.contact),
            warning=bool(msg.warning),
            stop=bool(msg.stop),
            valid=bool(msg.valid),
            region=int(msg.region),
        )

    def _on_safety_status(self, msg: CtrSafetyStatus) -> None:
        self.recorder.record_slice_7g_safety(
            valid=bool(msg.valid),
            fault=bool(msg.fault),
            emergency_stop=bool(msg.emergency_stop),
        )
        self.recorder.record_safety_evidence(
            timestamp=stamp_seconds(msg.header.stamp),
            state=int(msg.state),
            state_name=str(msg.state_name),
            command_allowed=bool(msg.command_allowed),
            emergency_stop=bool(msg.emergency_stop),
            fault=bool(msg.fault),
            valid=bool(msg.valid),
            diagnostic_status=str(msg.diagnostic_status),
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


def _optional_vector3_parameter(values) -> list[float] | None:
    if values is None or values == "":
        return None
    if isinstance(values, (list, tuple)) and len(values) == 0:
        return None
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError("cylinder_target_position must contain 3 finite values")
    return [float(value) for value in array]


def _bool_value(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() == "true"
    return bool(value)


def _optional_seed(value) -> int | None:
    seed = int(value)
    return None if seed < 0 else seed


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
