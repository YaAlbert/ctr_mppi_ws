"""ROS2 wrapper for the ROS-independent MPPI core."""

from __future__ import annotations

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath

from ctr_bringup.parameter_validation import load_parameter_files, validate_config_paths, validate_or_raise
from ctr_bringup.placeholder_node import run_node_until_shutdown
from ctr_interfaces.msg import CtrControllerMetrics, CtrJointCommand, CtrState
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.cylindrical_lumen import config_with_cylinder_overrides, goal_position_from_config
from ctr_mppi_controller.mppi_core import MPPICore
from ctr_mppi_controller.trajectory_metrics import TrajectoryMetricsAccumulator, TrajectoryMetricsConfig
from rclpy.node import Node
from rclpy.parameter import Parameter


REFERENCE_MODES = ("fixed_target", "trajectory")


class MPPIControllerNode(Node):
    """Thin ROS2 adapter around `MPPICore`."""

    def __init__(self):
        super().__init__("mppi_controller_node")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("target_position", [0.0, 0.0, 0.08])
        self.declare_parameter("reference_mode", "")
        self.declare_parameter("reference_type", "")
        self.declare_parameter("publish_safe_command_for_simulation", False)
        self.declare_parameter("enable_cylindrical_lumen", False)
        self.declare_parameter("cylinder_profile", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("mppi_random_seed", -1)

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)

        raw_config = load_parameter_files(config_paths)
        enable_lumen = _bool_value(self.get_parameter("enable_cylindrical_lumen").value)
        self.config = config_with_cylinder_overrides(
            raw_config,
            enabled=enable_lumen,
            target_position=_optional_vector3_parameter(self.get_parameter("cylinder_target_position").value),
            mppi_profile=str(self.get_parameter("cylinder_profile").value or ""),
            random_seed=_optional_seed(self.get_parameter("mppi_random_seed").value),
        )
        validate_or_raise(self.config)

        self.model = ApproximateCTRModel(self.config)
        self.core = MPPICore(self.config, self.model)
        self.target_tip = (
            goal_position_from_config(self.config)
            if enable_lumen
            else _vector3(self.get_parameter("target_position").value, "target_position")
        )
        self.reference_mode = reference_mode_from_config(self.config, self.get_parameter("reference_mode").value)
        self.trajectory_type = reference_type_from_config(self.config, self.get_parameter("reference_type").value)
        self.latest_state: CtrState | None = None
        self.frame_id = str(self.config["robot"]["frames"]["base"])
        self.reference_frame_id = str(self.config.get("reference", {}).get("frame_id", self.frame_id))
        self.reference_stale_timeout = float(self.config.get("reference", {}).get("stale_timeout", 0.20))
        self.latest_reference_horizon: np.ndarray | None = None
        self.latest_reference_horizon_stamp_s: float | None = None
        self._warned_missing_horizon = False
        self.publish_safe_for_sim = bool(self.get_parameter("publish_safe_command_for_simulation").value)
        self.tracking_metrics_config = TrajectoryMetricsConfig.from_project_config(self.config)
        self.trajectory_metrics = TrajectoryMetricsAccumulator(
            config=self.tracking_metrics_config,
            command_dimension=self.core.control_dimension,
            trajectory_type=self.trajectory_type if self.reference_mode == "trajectory" else "fixed_target",
        )
        self.last_trajectory_metrics_publish_s: float | None = None

        self.state_sub = self.create_subscription(CtrState, "/ctr/state", self._on_state, 10)
        self.target_sub = self.create_subscription(PoseStamped, "/ctr/reference/tip", self._on_target, 10)
        self.horizon_sub = self.create_subscription(NavPath, "/ctr/reference/horizon", self._on_reference_horizon, 10)
        self.command_pub = self.create_publisher(CtrJointCommand, "/ctr/mppi_command", 10)
        self.metrics_pub = self.create_publisher(CtrControllerMetrics, "/ctr/controller/metrics", 10)
        self.trajectory_metrics_pub = self.create_publisher(
            DiagnosticArray,
            "/ctr/controller/trajectory_metrics",
            10,
        )
        self.safe_command_pub = None
        if self.publish_safe_for_sim:
            self.safe_command_pub = self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)
            self.get_logger().warn(
                "publish_safe_command_for_simulation is enabled. Use only for simulation; hardware still requires safety supervisor."
            )

        control_frequency = float(self.config["mppi"]["control_frequency"])
        self.timer = self.create_timer(1.0 / control_frequency, self._on_timer)
        self.get_logger().info(
            "MPPI controller wrapper started. Enabled costs: tip, control, smoothness, terminal; "
            f"advanced costs disabled; reference_mode={self.reference_mode}; trajectory_type={self.trajectory_type}."
        )

    def _on_state(self, msg: CtrState) -> None:
        if msg.valid:
            self.latest_state = msg
        else:
            self.get_logger().warn("Ignoring invalid /ctr/state message.")

    def _on_target(self, msg: PoseStamped) -> None:
        try:
            self.target_tip = _vector3(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                "/ctr/reference/tip",
            )
        except ValueError as exc:
            self.get_logger().warn(f"Ignoring malformed /ctr/reference/tip message: {exc}")

    def _on_reference_horizon(self, msg: NavPath) -> None:
        if self.reference_mode != "trajectory":
            return
        now_s = ros_time_seconds(self.get_clock().now())
        try:
            sequence, stamp_s = target_sequence_from_path(
                msg,
                expected_horizon=self.core.horizon,
                expected_frame_id=self.reference_frame_id,
                current_time_s=now_s,
                stale_timeout=self.reference_stale_timeout,
            )
        except ValueError as exc:
            self.latest_reference_horizon = None
            self.latest_reference_horizon_stamp_s = None
            self.get_logger().warn(f"Ignoring malformed /ctr/reference/horizon message: {exc}")
            return

        self.latest_reference_horizon = sequence
        self.latest_reference_horizon_stamp_s = stamp_s
        self._warned_missing_horizon = False

    def _on_timer(self) -> None:
        if self.latest_state is None:
            return

        try:
            current_time_s = ros_time_seconds(self.get_clock().now())
            reference_kwargs = solve_reference_kwargs(
                reference_mode=self.reference_mode,
                target_tip=self.target_tip,
                target_tip_sequence=self.latest_reference_horizon,
                horizon_stamp_s=self.latest_reference_horizon_stamp_s,
                current_time_s=current_time_s,
                stale_timeout=self.reference_stale_timeout,
            )
        except ValueError as exc:
            if not self._warned_missing_horizon:
                self.get_logger().warn(f"MPPI reference unavailable: {exc}")
                self._warned_missing_horizon = True
            return

        try:
            result = self.core.solve(
                q=np.asarray(self.latest_state.q, dtype=float),
                q_dot=np.asarray(self.latest_state.q_dot, dtype=float),
                **reference_kwargs,
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

        self._update_and_publish_trajectory_metrics(
            current_time_s=current_time_s,
            stamp=stamp,
            result=result,
            reference_kwargs=reference_kwargs,
        )

    def _update_and_publish_trajectory_metrics(self, *, current_time_s: float, stamp, result, reference_kwargs) -> None:
        if not self.tracking_metrics_config.enabled:
            return

        try:
            reference_point = active_reference_point(reference_kwargs)
            self.trajectory_metrics.add_sample(
                timestamp=current_time_s,
                tip_position=tip_position_from_state(self.latest_state),
                reference_position=reference_point,
                command=result.command,
                dt=self.core.dt,
                solve_time=result.solve_time,
                command_saturated=result.command_saturated,
                control_period=1.0 / float(self.config["mppi"]["control_frequency"]),
            )
        except ValueError as exc:
            self.trajectory_metrics.record_invalid_sample()
            self.get_logger().warn(f"Trajectory metric sample rejected: {exc}")
            return

        if not should_publish_metrics(
            last_publish_time_s=self.last_trajectory_metrics_publish_s,
            current_time_s=current_time_s,
            publish_frequency=self.tracking_metrics_config.publish_frequency,
        ):
            return
        self.last_trajectory_metrics_publish_s = current_time_s
        self.trajectory_metrics_pub.publish(
            trajectory_metrics_diagnostic_array(
                self.trajectory_metrics.snapshot(),
                frame_id=self.frame_id,
                stamp=stamp,
            )
        )


def reference_mode_from_config(config: dict, override) -> str:
    configured = config.get("reference", {}).get("mode", "fixed_target")
    mode = configured if override is None or override == "" else override
    if mode not in REFERENCE_MODES:
        raise ValueError(f"reference_mode must be one of {REFERENCE_MODES}")
    return str(mode)


def reference_type_from_config(config: dict, override) -> str:
    configured = config.get("reference", {}).get("trajectory_type", "circle")
    trajectory_type = configured if override is None or override == "" else override
    allowed = ("circle", "ellipse", "helix")
    if trajectory_type not in allowed:
        raise ValueError(f"reference_type must be one of {allowed}")
    return str(trajectory_type)


def target_sequence_from_path(
    msg: NavPath,
    *,
    expected_horizon: int,
    expected_frame_id: str,
    current_time_s: float,
    stale_timeout: float,
) -> tuple[np.ndarray, float]:
    if expected_horizon <= 0:
        raise ValueError("expected_horizon must be positive")
    if not expected_frame_id:
        raise ValueError("expected_frame_id must be non-empty")
    now_s = _finite_float(current_time_s, "current_time_s")
    timeout = _positive_float(stale_timeout, "stale_timeout")
    if msg.header.frame_id != expected_frame_id:
        raise ValueError(f"horizon frame_id must be `{expected_frame_id}`")
    if len(msg.poses) != expected_horizon:
        raise ValueError(f"horizon must contain exactly {expected_horizon} poses")

    stamp_s = ros_time_seconds(msg.header.stamp)
    age_s = now_s - stamp_s
    if age_s < -1.0e-9:
        raise ValueError("horizon timestamp is in the future")
    if age_s > timeout:
        raise ValueError(f"horizon is stale: age_s={age_s:.6f}")

    points = []
    for index, pose_stamped in enumerate(msg.poses):
        pose_frame = pose_stamped.header.frame_id
        if pose_frame and pose_frame != expected_frame_id:
            raise ValueError(f"horizon pose {index} frame_id must be `{expected_frame_id}`")
        points.append(
            [
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
            ]
        )
    array = np.asarray(points, dtype=float)
    if array.shape != (expected_horizon, 3) or not np.all(np.isfinite(array)):
        raise ValueError(f"horizon positions must have shape ({expected_horizon}, 3) and contain finite values")
    return array, stamp_s


def solve_reference_kwargs(
    *,
    reference_mode: str,
    target_tip: np.ndarray,
    target_tip_sequence: np.ndarray | None,
    horizon_stamp_s: float | None,
    current_time_s: float,
    stale_timeout: float,
) -> dict[str, np.ndarray]:
    if reference_mode == "fixed_target":
        return {"target_tip": _vector3(target_tip, "target_tip")}
    if reference_mode != "trajectory":
        raise ValueError(f"reference_mode must be one of {REFERENCE_MODES}")
    if target_tip_sequence is None or horizon_stamp_s is None:
        raise ValueError("trajectory mode requires a valid /ctr/reference/horizon message")
    now_s = _finite_float(current_time_s, "current_time_s")
    stamp_s = _finite_float(horizon_stamp_s, "horizon_stamp_s")
    timeout = _positive_float(stale_timeout, "stale_timeout")
    age_s = now_s - stamp_s
    if age_s < -1.0e-9:
        raise ValueError("trajectory horizon timestamp is in the future")
    if age_s > timeout:
        raise ValueError(f"trajectory horizon is stale: age_s={age_s:.6f}")
    sequence = np.asarray(target_tip_sequence, dtype=float)
    if sequence.ndim != 2 or sequence.shape[1] != 3 or not np.all(np.isfinite(sequence)):
        raise ValueError("trajectory horizon must have shape (H, 3) and contain finite values")
    return {"target_tip_sequence": sequence.copy()}


def active_reference_point(reference_kwargs: dict[str, np.ndarray]) -> np.ndarray:
    if "target_tip" in reference_kwargs:
        return _vector3(reference_kwargs["target_tip"], "target_tip")
    if "target_tip_sequence" in reference_kwargs:
        sequence = np.asarray(reference_kwargs["target_tip_sequence"], dtype=float)
        if sequence.ndim != 2 or sequence.shape[1] != 3 or sequence.shape[0] < 1 or not np.all(np.isfinite(sequence)):
            raise ValueError("target_tip_sequence must contain at least one finite 3D point")
        return sequence[0].copy()
    raise ValueError("reference kwargs must contain target_tip or target_tip_sequence")


def tip_position_from_state(state: CtrState | None) -> np.ndarray:
    if state is None:
        raise ValueError("state is unavailable")
    return _vector3(
        [state.tip_pose.position.x, state.tip_pose.position.y, state.tip_pose.position.z],
        "state.tip_pose.position",
    )


def should_publish_metrics(
    *,
    last_publish_time_s: float | None,
    current_time_s: float,
    publish_frequency: float,
) -> bool:
    now_s = _finite_float(current_time_s, "current_time_s")
    frequency = _positive_float(publish_frequency, "publish_frequency")
    if last_publish_time_s is None:
        return True
    previous = _finite_float(last_publish_time_s, "last_publish_time_s")
    if now_s < previous:
        return True
    return (now_s - previous) + 1.0e-12 >= (1.0 / frequency)


def trajectory_metrics_diagnostic_array(snapshot, *, frame_id: str, stamp) -> DiagnosticArray:
    status = DiagnosticStatus()
    status.name = "ctr_mppi_controller/trajectory_metrics"
    status.hardware_id = "simulation"
    status.level = DiagnosticStatus.OK if snapshot.has_valid_samples else DiagnosticStatus.WARN
    status.message = snapshot.completion_state
    status.values = [
        KeyValue(key="trajectory_type", value=str(snapshot.trajectory_type)),
        KeyValue(key="sample_count", value=str(snapshot.sample_count)),
        KeyValue(key="invalid_sample_count", value=str(snapshot.invalid_sample_count)),
        KeyValue(key="rmse", value=_format_float(snapshot.rmse)),
        KeyValue(key="mean_error", value=_format_float(snapshot.mean_error)),
        KeyValue(key="max_error", value=_format_float(snapshot.max_error)),
        KeyValue(key="control_effort", value=_format_float(snapshot.control_effort)),
        KeyValue(key="transient_duration", value=_format_float(snapshot.transient_duration)),
        KeyValue(key="mean_solve_time", value=_format_float(snapshot.mean_solve_time)),
        KeyValue(key="max_solve_time", value=_format_float(snapshot.max_solve_time)),
        KeyValue(key="min_solve_time", value=_format_float(snapshot.min_solve_time)),
        KeyValue(key="control_period_overrun_count", value=str(snapshot.control_period_overrun_count)),
        KeyValue(key="command_saturation_count", value=str(snapshot.command_saturation_count)),
        KeyValue(
            key="maximum_command_per_joint",
            value=",".join(_format_float(value) for value in snapshot.maximum_command_per_joint),
        ),
        KeyValue(key="experiment_elapsed_time", value=_format_float(snapshot.experiment_elapsed_time)),
        KeyValue(key="completion_state", value=str(snapshot.completion_state)),
    ]
    msg = DiagnosticArray()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status = [status]
    return msg


def ros_time_seconds(time_value) -> float:
    if hasattr(time_value, "nanoseconds"):
        seconds = float(time_value.nanoseconds) * 1.0e-9
    elif hasattr(time_value, "sec") and hasattr(time_value, "nanosec"):
        seconds = float(time_value.sec) + float(time_value.nanosec) * 1.0e-9
    else:
        raise ValueError("ROS time value must provide nanoseconds or sec/nanosec fields")
    if not np.isfinite(seconds):
        raise ValueError("ROS time must be finite")
    return seconds


def _vector3(values, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array


def _optional_vector3_parameter(values) -> list[float] | None:
    if values is None or values == "":
        return None
    if isinstance(values, (list, tuple)) and len(values) == 0:
        return None
    return [float(value) for value in _vector3(values, "cylinder_target_position")]


def _bool_value(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() == "true"
    return bool(value)


def _optional_seed(value) -> int | None:
    seed = int(value)
    return None if seed < 0 else seed


def _finite_float(value, label: str) -> float:
    numeric = float(value)
    if not np.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _positive_float(value, label: str) -> float:
    numeric = _finite_float(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _format_float(value) -> str:
    return f"{float(value):.9g}"


def main(args=None):
    run_node_until_shutdown(rclpy, MPPIControllerNode, args=args)


if __name__ == "__main__":
    main()
