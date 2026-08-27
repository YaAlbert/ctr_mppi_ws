"""ROS2 simulation loop for Milestone 3."""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Iterable

import numpy as np

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ctr_bringup.parameter_validation import (
    load_parameter_files,
    parse_launch_bool,
    validate_config_paths,
    validate_or_raise,
)
from ctr_bringup.slice_7g_profile import (
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
)
from ctr_bringup.placeholder_node import run_node_until_shutdown
from ctr_interfaces.msg import CtrBackbone, CtrJointCommand, CtrJointState, CtrState, CtrTactileState
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_mppi_controller.cylindrical_lumen import CylindricalLumen, goal_position_from_config
from ctr_mppi_controller.lumen_factory import (
    config_with_lumen_overrides,
    lumen_geometry_fingerprint,
    lumen_geometry_log_line,
    lumen_geometry_from_config,
    lumen_mode_from_config,
)
from ctr_mppi_controller.nodes.reference_manager_node import reference_path_qos_profile
from ctr_sim.lumen_markers import (
    BoundedTipTrajectory,
    build_actual_tip_path_marker,
    build_dynamic_lumen_delete_markers,
    build_dynamic_lumen_diagnostic_markers,
    LumenMarkerConfig,
    build_curved_static_lumen_markers,
    build_reference_path_markers,
    build_static_lumen_delete_markers,
    marker_keys,
    markers_with_stamp,
    static_lumen_cache_key,
)
from ctr_sim.lumen_diagnostics import LumenRuntimeDiagnostic, build_lumen_runtime_diagnostic
from ctr_sim.simulation_core import CTRSimulationCore
from ctr_tactile.simulated_tactile import SimulatedTactileParameters, simulate_tactile
from ctr_tactile.tactile_processing import TactileProcessor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


_STATIC_DEVELOPMENT_MARKER_NAMESPACES = frozenset(
    {"lumen_surface", "lumen_wireframe", "lumen_centerline"}
)


def development_marker_qos_profile(namespace: str) -> QoSProfile:
    """Keep static visual geometry available to RViz without periodic large republishes."""

    static = namespace in _STATIC_DEVELOPMENT_MARKER_NAMESPACES
    return QoSProfile(
        depth=1 if static else 10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE
        ),
    )


class CTRSimulatorNode(Node):
    """Receive safe q_dot commands, update q, and publish simulated CTR state."""

    def __init__(self):
        super().__init__("simulator_node")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("target_position", [0.0, 0.0, 0.08])
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("enable_cylindrical_lumen", False)
        self.declare_parameter("enable_curved_lumen", False)
        self.declare_parameter("curved_lumen_type", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("tactile_enabled", False)
        self.declare_parameter("slice_7g_profile", False)
        self.declare_parameter("development_simulation", False)
        self.declare_parameter("enable_development_visualization", False)
        self.declare_parameter("evaluation_diagnostics_enabled", False)

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)

        raw_config = load_parameter_files(config_paths)
        enable_lumen = parse_launch_bool(
            self.get_parameter("enable_cylindrical_lumen").value,
            "enable_cylindrical_lumen",
        )
        enable_curved_lumen = parse_launch_bool(
            self.get_parameter("enable_curved_lumen").value,
            "enable_curved_lumen",
        )
        self.config = config_with_lumen_overrides(
            raw_config,
            enable_cylindrical_lumen=enable_lumen,
            enable_curved_lumen=enable_curved_lumen,
            curved_lumen_type=str(self.get_parameter("curved_lumen_type").value or ""),
            target=_optional_vector3_parameter(self.get_parameter("cylinder_target_position").value),
        )
        self.config.setdefault("runtime", {})["mode"] = str(
            self.get_parameter("runtime_mode").value
        )
        slice_7g_enabled = parse_launch_bool(
            self.get_parameter("slice_7g_profile").value,
            "slice_7g_profile",
        )
        development_enabled = parse_launch_bool(
            self.get_parameter("development_simulation").value,
            "development_simulation",
        )
        development_visualization_enabled = parse_launch_bool(
            self.get_parameter("enable_development_visualization").value,
            "enable_development_visualization",
        )
        evaluation_diagnostics_enabled = parse_launch_bool(
            self.get_parameter("evaluation_diagnostics_enabled").value,
            "evaluation_diagnostics_enabled",
        )
        if development_visualization_enabled and not development_enabled:
            raise ValueError(
                "enable_development_visualization requires development_simulation=true"
            )
        self.development_simulation = development_enabled
        self.development_visualization = development_visualization_enabled
        self.evaluation_diagnostics_enabled = evaluation_diagnostics_enabled
        self.config = (
            apply_slice_7g_development_simulation_profile(self.config, enabled=True)
            if development_enabled
            else apply_slice_7g_simulation_profile(self.config, enabled=slice_7g_enabled)
        )
        validate_or_raise(self.config)
        self.lumen_mode = lumen_mode_from_config(self.config)

        self.core = CTRSimulationCore(self.config)
        self.model = ApproximateCTRModel(self.config)

        simulation = self.config["simulation"]
        self.update_frequency = float(simulation["update_frequency"])
        self.dt = 1.0 / self.update_frequency
        self.lumen_marker_config = LumenMarkerConfig.from_mapping(
            simulation.get("visualization", {})
        )
        if self.development_visualization:
            self.lumen_marker_config = replace(
                self.lumen_marker_config,
                publish_lumen_surface=True,
            )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.frame_id = self.config["robot"]["frames"]["base"]
        self.world_frame_id = self.config["robot"]["frames"]["world"]
        self.tip_frame_id = self.config["robot"]["frames"]["tip"]
        self.target_position = (
            goal_position_from_config(self.config)
            if self.lumen_mode != "none"
            else _vector3(self.get_parameter("target_position").value, "target_position")
        )
        self.lumen_geometry = lumen_geometry_from_config(self.config)
        self.lumen = self.lumen_geometry
        self.tactile_enabled = slice_7g_enabled or parse_launch_bool(
            self.get_parameter("tactile_enabled").value,
            "tactile_enabled",
        )
        self.tactile_parameters = (
            SimulatedTactileParameters.from_mapping(self.config) if self.tactile_enabled else None
        )
        self.tactile_processor = (
            TactileProcessor.from_mapping(self.config) if self.tactile_enabled else None
        )
        self._static_lumen_cache_key = None
        self._static_lumen_markers: list[Marker] = []
        self._static_lumen_marker_keys: tuple[tuple[str, int], ...] = ()
        self._static_lumen_marker_frame_id = self.frame_id
        self._last_static_lumen_publish_time_s: float | None = None
        self._last_development_visualization_publish_time_s: float | None = None
        self._last_runtime_marker_publish_time_s: float | None = None
        self._static_lumen_build_count = 0
        self._static_lumen_cache_hit_logged = False
        self._dynamic_lumen_marker_keys: tuple[tuple[str, int], ...] = ()
        self._dynamic_lumen_marker_frame_id = self.frame_id
        self._last_lumen_diagnostic_log_signature: tuple[str, ...] | None = None
        self._lumen_diagnostic_update_count = 0
        self._reference_path_points = np.empty((0, 3), dtype=np.float64)
        self._reference_path_frame_id = self.frame_id
        self._tip_trajectory = (
            BoundedTipTrajectory(
                max_points=self.lumen_marker_config.actual_tip_history_max_points,
                minimum_interval=self.lumen_marker_config.actual_tip_history_min_interval,
            )
            if self.development_visualization
            else None
        )

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
        self.reference_path_sub = (
            self.create_subscription(
                NavPath,
                "/ctr/reference/path",
                self._on_reference_path,
                reference_path_qos_profile(),
            )
            if self.development_visualization
            else None
        )

        self.joint_pub = self.create_publisher(CtrJointState, "/ctr/joint_state", 10)
        self.standard_joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.backbone_pub = self.create_publisher(CtrBackbone, "/ctr/backbone", 10)
        self.tip_pub = self.create_publisher(PoseStamped, "/ctr/tip", 10)
        self.state_pub = self.create_publisher(CtrState, "/ctr/state", 10)
        self.tactile_pub = (
            self.create_publisher(CtrTactileState, "/ctr/tactile/state", 10)
            if self.tactile_enabled
            else None
        )
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ctr/visualization", 10)
        self.development_marker_pubs = (
            {
                namespace: self.create_publisher(
                    MarkerArray,
                    f"/ctr/development_visualization/{namespace}",
                    development_marker_qos_profile(namespace),
                )
                for namespace in (
                    "lumen_surface",
                    "lumen_wireframe",
                    "lumen_centerline",
                    "ctr_backbone",
                    "reference_path",
                    "actual_tip_path",
                    "tip_marker",
                    "target_marker",
                )
            }
            if self.development_visualization
            else {}
        )

        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            "CTR simulator started: /ctr/safe_command -> /ctr/joint_state, /ctr/backbone, /ctr/tip, /ctr/state."
        )
        self.get_logger().info(lumen_geometry_log_line(self.config, role="simulator"))

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

    def _on_reference_path(self, msg: NavPath) -> None:
        """Retain only the exact finite reference data published by the controller path owner."""

        try:
            frame_id = str(msg.header.frame_id)
            if not frame_id:
                raise ValueError("reference path frame is empty")
            points = np.asarray(
                [
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                    for pose in msg.poses
                ],
                dtype=np.float64,
            )
            if points.ndim != 2 or points.shape[1:] != (3,) or points.shape[0] < 1:
                raise ValueError("reference path must contain at least one 3D pose")
            if not np.all(np.isfinite(points)):
                raise ValueError("reference path contains non-finite coordinates")
            pose_frames = {pose.header.frame_id for pose in msg.poses if pose.header.frame_id}
            if pose_frames and pose_frames != {frame_id}:
                raise ValueError("reference path pose frames do not match its header")
        except (TypeError, ValueError) as exc:
            self._reference_path_points = np.empty((0, 3), dtype=np.float64)
            self.get_logger().warn(f"Reference path visualization rejected: {exc}")
            return
        self._reference_path_points = points.copy()
        self._reference_path_frame_id = frame_id

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
        if self._tip_trajectory is not None:
            self._tip_trajectory.append(model_result.tip_position, _stamp_seconds(stamp))

        self.joint_pub.publish(self._joint_state_msg(stamp, step.q, step.q_dot))
        self.standard_joint_pub.publish(self._standard_joint_state_msg(stamp, step.q, step.q_dot))
        self.backbone_pub.publish(self._backbone_msg(stamp, backbone_points))
        self.tip_pub.publish(tip_pose)
        self.state_pub.publish(self._state_msg(stamp, step.q, step.q_dot, backbone_points, tip_pose))
        if self.tactile_pub is not None:
            self.tactile_pub.publish(self._tactile_msg(stamp, model_result.tip_position))
        self.diagnostics_pub.publish(self._diagnostics_msg(stamp, command_age, model_result.diagnostic_status))
        if self._runtime_marker_publication_due(stamp):
            publish_development_visualization = self._development_visualization_publish_due(stamp)
            marker_array = self._marker_array_msg(
                stamp,
                backbone_points,
                model_result.backbone_points,
                include_development=publish_development_visualization,
            )
            self.marker_pub.publish(
                MarkerArray(
                    markers=[marker for marker in marker_array.markers if marker.ns != "lumen_surface"]
                )
            )
            if publish_development_visualization:
                self._publish_development_marker_topics(marker_array)

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

    def _tactile_msg(self, stamp, tip_position: np.ndarray) -> CtrTactileState:
        if self.lumen is None:
            sample = simulate_tactile(None, self.tactile_parameters)
        else:
            clearance = self.lumen.point_clearance(tip_position).physical_clearance
            sample = simulate_tactile(clearance, self.tactile_parameters)
        processed = self.tactile_processor.process(
            [sample.raw_signal] if sample.valid else None,
            clearance_m=sample.clearance_m,
            geometric_contact=sample.contact,
            timestamp_s=stamp.sec + stamp.nanosec * 1e-9,
        )

        msg = CtrTactileState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.raw_values = [float(sample.raw_signal)]
        msg.filtered_values = [float(processed.filtered_signal)]
        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = 0.0
        msg.force_magnitude = float(processed.force_n)
        msg.contact = bool(processed.contact)
        msg.warning = bool(processed.warning)
        msg.stop = bool(processed.stop)
        msg.valid = bool(processed.valid)
        msg.diagnostic_status = processed.diagnostic_status
        msg.clearance_m = float(processed.clearance_m)
        msg.source = "simulated"
        msg.region = int(processed.region)
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

    def _marker_array_msg(
        self,
        stamp,
        backbone_points: list[Point],
        backbone_array: np.ndarray,
        *,
        include_development: bool | None = None,
    ) -> MarkerArray:
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
        tip.ns = "tip_marker"
        tip.id = 1
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.scale.x = 0.008
        tip.scale.y = 0.008
        tip.scale.z = 0.008
        tip.color = ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0)
        if backbone_points:
            tip.pose.position = backbone_points[-1]
        tip.pose.orientation.w = 1.0

        target = Marker()
        target.header.stamp = stamp
        target.header.frame_id = self.frame_id
        target.ns = "target_marker"
        target.id = 2
        target.type = Marker.LINE_STRIP
        target.action = Marker.ADD
        target.scale.x = 0.0015
        target.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
        target.pose.orientation.w = 1.0
        target.points = [
            _point_from_array(
                self.target_position
                + 0.006
                * np.array(
                    [
                        math.cos(2.0 * math.pi * index / 32.0),
                        math.sin(2.0 * math.pi * index / 32.0),
                        0.0,
                    ],
                    dtype=float,
                )
            )
            for index in range(33)
        ]

        markers = [backbone, tip, target]
        markers.extend(self._static_lumen_markers_for_publish(stamp))
        markers.extend(self._dynamic_lumen_markers_for_publish(stamp, backbone_array))
        if self.lumen_mode == "cylindrical" and isinstance(self.lumen, CylindricalLumen):
            markers.extend(self._lumen_markers(stamp, backbone_array))
        if include_development is None:
            include_development = self.development_visualization
        if include_development and self._reference_path_points.shape[0] > 0:
            markers.extend(
                build_reference_path_markers(
                    self._reference_path_points,
                    self._reference_path_frame_id,
                    stamp,
                )
            )
        if include_development and self._tip_trajectory is not None:
            actual_path = build_actual_tip_path_marker(
                self._tip_trajectory.points(),
                self.frame_id,
                stamp,
            )
            if actual_path is not None:
                markers.append(actual_path)
        return MarkerArray(markers=markers)

    def _development_visualization_publish_due(self, stamp) -> bool:
        if not self.development_visualization:
            return False
        stamp_s = _stamp_seconds(stamp)
        last_publish = self._last_development_visualization_publish_time_s
        period = 1.0 / self.lumen_marker_config.marker_publish_rate
        if last_publish is None or stamp_s < last_publish or stamp_s - last_publish >= period - 1.0e-12:
            self._last_development_visualization_publish_time_s = stamp_s
            return True
        return False

    def _runtime_marker_publication_due(self, stamp) -> bool:
        """Rate-limit non-control visualization work in explicit diagnostic runs.

        The normal simulator behavior is unchanged.  Paper diagnostics do not
        consume live marker data, so honoring the configured marker rate avoids
        making 100 Hz physics/tactile publication compete with large MarkerArray
        construction while still retaining an inspectable marker stream.
        """

        if not self.evaluation_diagnostics_enabled:
            return True
        stamp_s = _stamp_seconds(stamp)
        last_publish = self._last_runtime_marker_publish_time_s
        period = 1.0 / self.lumen_marker_config.marker_publish_rate
        if (
            last_publish is None
            or stamp_s < last_publish
            or stamp_s - last_publish >= period - 1.0e-12
        ):
            self._last_runtime_marker_publish_time_s = stamp_s
            return True
        return False

    def _publish_development_marker_topics(self, marker_array: MarkerArray) -> None:
        if not self.development_marker_pubs:
            return
        by_namespace: dict[str, list[Marker]] = {
            namespace: [] for namespace in self.development_marker_pubs
        }
        for marker in marker_array.markers:
            if marker.ns in by_namespace:
                by_namespace[marker.ns].append(marker)
        for namespace, publisher in self.development_marker_pubs.items():
            if by_namespace[namespace]:
                publisher.publish(MarkerArray(markers=by_namespace[namespace]))

    def _static_lumen_markers_for_publish(self, stamp) -> list[Marker]:
        config = getattr(self, "lumen_marker_config", LumenMarkerConfig())
        if self.lumen_mode != "curved" or not isinstance(self.lumen, CurvedLumen):
            return self._clear_static_lumen_markers(stamp, reason=f"mode_{self.lumen_mode}")
        if not config.publish_lumen_markers:
            return self._clear_static_lumen_markers(stamp, reason="visualization_disabled")

        fingerprint = lumen_geometry_fingerprint(getattr(self, "config", self.lumen))
        cache_key = static_lumen_cache_key(fingerprint, config)
        if cache_key != self._static_lumen_cache_key:
            deletes = self._clear_static_lumen_markers(stamp, reason="cache_rebuild")
            try:
                markers = build_curved_static_lumen_markers(
                    self.lumen,
                    fingerprint,
                    self.lumen.frame_id,
                    config,
                    stamp,
                )
            except ValueError as exc:
                self._static_lumen_cache_key = None
                self._static_lumen_markers = []
                self._static_lumen_marker_keys = ()
                self._static_lumen_marker_frame_id = self.lumen.frame_id
                self._log_lumen_markers(
                    mode="curved",
                    frame=self.lumen.frame_id,
                    fingerprint=fingerprint,
                    centerline_points=int(self.lumen.centerline_points.shape[0]),
                    rings=0,
                    segments=config.ring_segments,
                    cached=False,
                    reason=f"generation_failed:{exc}",
                )
                return deletes
            self._static_lumen_cache_key = cache_key
            self._static_lumen_markers = markers
            self._static_lumen_marker_keys = marker_keys(markers)
            self._static_lumen_marker_frame_id = self.lumen.frame_id
            self._static_lumen_build_count += 1
            self._static_lumen_cache_hit_logged = False
            self._last_static_lumen_publish_time_s = _stamp_seconds(stamp)
            self._log_lumen_markers(
                mode="curved",
                frame=self.lumen.frame_id,
                fingerprint=fingerprint,
                centerline_points=int(self.lumen.centerline_points.shape[0]),
                rings=self._static_lumen_ring_count(config),
                segments=config.ring_segments,
                cached=False,
                reason="built",
            )
            return deletes + markers_with_stamp(markers, stamp)

        if not self._static_lumen_publish_due(stamp, config):
            return []
        if not self._static_lumen_cache_hit_logged:
            self._log_lumen_markers(
                mode="curved",
                frame=self.lumen.frame_id,
                fingerprint=fingerprint,
                centerline_points=int(self.lumen.centerline_points.shape[0]),
                rings=self._static_lumen_ring_count(config),
                segments=config.ring_segments,
                cached=True,
                reason="cache_hit",
            )
            self._static_lumen_cache_hit_logged = True
        return markers_with_stamp(self._static_lumen_markers, stamp)

    def _static_lumen_publish_due(self, stamp, config: LumenMarkerConfig) -> bool:
        if self.development_visualization:
            # Static development topics are transient-local, so one authenticated
            # publication is sufficient even when RViz joins after the simulator.
            return False
        stamp_s = _stamp_seconds(stamp)
        last_publish = self._last_static_lumen_publish_time_s
        period = 1.0 / config.marker_publish_rate
        if last_publish is None or stamp_s < last_publish or stamp_s - last_publish >= period - 1.0e-12:
            self._last_static_lumen_publish_time_s = stamp_s
            return True
        return False

    def _clear_static_lumen_markers(self, stamp, *, reason: str) -> list[Marker]:
        keys = tuple(getattr(self, "_static_lumen_marker_keys", ()))
        if not keys:
            self._static_lumen_cache_key = None
            self._static_lumen_markers = []
            self._static_lumen_marker_keys = ()
            self._last_static_lumen_publish_time_s = None
            return []
        frame = getattr(self, "_static_lumen_marker_frame_id", self.frame_id)
        deletes = build_static_lumen_delete_markers(keys, frame, stamp)
        self._log_lumen_markers(
            mode=self.lumen_mode,
            frame=frame,
            fingerprint="none",
            centerline_points=0,
            rings=0,
            segments=getattr(getattr(self, "lumen_marker_config", None), "ring_segments", 0),
            cached=False,
            reason=reason,
        )
        self._static_lumen_cache_key = None
        self._static_lumen_markers = []
        self._static_lumen_marker_keys = ()
        self._last_static_lumen_publish_time_s = None
        self._static_lumen_cache_hit_logged = False
        return deletes

    def _static_lumen_ring_count(self, config: LumenMarkerConfig) -> int:
        if not isinstance(self.lumen, CurvedLumen):
            return 0
        point_count = int(self.lumen.centerline_points.shape[0])
        return len(tuple(dict.fromkeys(list(range(0, point_count, config.ring_stride)) + [point_count - 1])))

    def _log_lumen_markers(
        self,
        *,
        mode: str,
        frame: str,
        fingerprint: str,
        centerline_points: int,
        rings: int,
        segments: int,
        cached: bool,
        reason: str,
    ) -> None:
        try:
            logger = self.get_logger()
        except Exception:
            return
        logger.info(
            "LUMEN_MARKERS "
            f"mode={mode} "
            f"frame={frame} "
            f"fingerprint={fingerprint} "
            f"centerline_points={centerline_points} "
            f"rings={rings} "
            f"segments={segments} "
            f"cached={str(cached).lower()} "
            f"build_count={self._static_lumen_build_count} "
            f"reason={reason}"
        )

    def _dynamic_lumen_markers_for_publish(self, stamp, backbone_array: np.ndarray) -> list[Marker]:
        config = getattr(self, "lumen_marker_config", LumenMarkerConfig())
        if not config.publish_lumen_markers:
            return self._clear_dynamic_lumen_markers(stamp, reason="visualization_disabled")
        if not config.publish_lumen_diagnostics:
            return self._clear_dynamic_lumen_markers(stamp, reason="diagnostics_disabled")
        if self.lumen_mode != "curved" or not isinstance(self.lumen, CurvedLumen):
            return self._clear_dynamic_lumen_markers(stamp, reason=f"mode_{self.lumen_mode}")

        try:
            diagnostic = build_lumen_runtime_diagnostic(self.lumen, backbone_array, self.lumen_mode)
            markers = build_dynamic_lumen_diagnostic_markers(diagnostic, stamp)
        except ValueError as exc:
            self._log_lumen_diagnostic_unavailable(
                mode=self.lumen_mode,
                frame=getattr(self.lumen, "frame_id", self.frame_id),
                reason=f"generation_failed:{exc}",
            )
            return self._clear_dynamic_lumen_markers(stamp, reason="generation_failed")

        new_keys = marker_keys(markers)
        old_keys = tuple(getattr(self, "_dynamic_lumen_marker_keys", ()))
        obsolete_keys = tuple(key for key in old_keys if key not in new_keys)
        deletes: list[Marker] = []
        if obsolete_keys:
            deletes = build_dynamic_lumen_delete_markers(
                obsolete_keys,
                getattr(self, "_dynamic_lumen_marker_frame_id", diagnostic.frame_id),
                stamp,
            )
        self._dynamic_lumen_marker_keys = new_keys
        self._dynamic_lumen_marker_frame_id = diagnostic.frame_id
        self._lumen_diagnostic_update_count += 1
        self._log_lumen_diagnostic(diagnostic, reason="updated")
        return deletes + markers

    def _clear_dynamic_lumen_markers(self, stamp, *, reason: str) -> list[Marker]:
        keys = tuple(getattr(self, "_dynamic_lumen_marker_keys", ()))
        if not keys:
            return []
        frame = getattr(self, "_dynamic_lumen_marker_frame_id", self.frame_id)
        deletes = build_dynamic_lumen_delete_markers(keys, frame, stamp)
        self._dynamic_lumen_marker_keys = ()
        self._log_lumen_diagnostic_unavailable(mode=self.lumen_mode, frame=frame, reason=reason)
        return deletes

    def _log_lumen_diagnostic(self, diagnostic: LumenRuntimeDiagnostic, *, reason: str) -> None:
        signature = (
            diagnostic.geometry_mode,
            diagnostic.constraint_type,
            diagnostic.status,
        )
        if signature == self._last_lumen_diagnostic_log_signature:
            return
        self._last_lumen_diagnostic_log_signature = signature
        try:
            logger = self.get_logger()
        except Exception:
            return
        logger.info(
            "LUMEN_DIAGNOSTIC "
            f"mode={diagnostic.geometry_mode} "
            f"constraint={diagnostic.constraint_type} "
            f"state={diagnostic.status} "
            f"backbone_index={diagnostic.backbone_index} "
            f"physical_clearance={diagnostic.physical_clearance:.9f} "
            f"safety_clearance={diagnostic.safety_clearance:.9f} "
            f"collision={str(diagnostic.physical_collision).lower()} "
            f"margin_violation={str(diagnostic.safety_margin_violation).lower()} "
            f"frame={diagnostic.frame_id} "
            f"reason={reason}"
        )

    def _log_lumen_diagnostic_unavailable(self, *, mode: str, frame: str, reason: str) -> None:
        signature = (str(mode), "unavailable", str(reason))
        if signature == self._last_lumen_diagnostic_log_signature:
            return
        self._last_lumen_diagnostic_log_signature = signature
        try:
            logger = self.get_logger()
        except Exception:
            return
        logger.info(
            "LUMEN_DIAGNOSTIC "
            f"mode={mode} "
            "constraint=unavailable "
            "state=UNAVAILABLE "
            "backbone_index=-1 "
            "physical_clearance=unavailable "
            "safety_clearance=unavailable "
            "collision=false "
            "margin_violation=false "
            f"frame={frame} "
            f"reason={reason}"
        )

    def _lumen_markers(self, stamp, backbone_array: np.ndarray) -> list[Marker]:
        assert self.lumen is not None
        clearance = self.lumen.backbone_clearance(backbone_array)
        center = self.lumen.axis_origin + 0.5 * self.lumen.length * self.lumen.axis_direction
        orientation = _quaternion_from_z_axis(self.lumen.axis_direction)

        wall = Marker()
        wall.header.stamp = stamp
        wall.header.frame_id = self.lumen.frame_id
        wall.ns = "cylindrical_lumen"
        wall.id = 10
        wall.type = Marker.CYLINDER
        wall.action = Marker.ADD
        wall.pose.position = _point_from_array(center)
        wall.pose.orientation.x = orientation[0]
        wall.pose.orientation.y = orientation[1]
        wall.pose.orientation.z = orientation[2]
        wall.pose.orientation.w = orientation[3]
        wall.scale.x = 2.0 * self.lumen.radius
        wall.scale.y = 2.0 * self.lumen.radius
        wall.scale.z = self.lumen.length
        wall.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.16)

        safety = Marker()
        safety.header.stamp = stamp
        safety.header.frame_id = self.lumen.frame_id
        safety.ns = "cylindrical_lumen"
        safety.id = 11
        safety.type = Marker.CYLINDER
        safety.action = Marker.ADD
        safety.pose = wall.pose
        safety.scale.x = 2.0 * self.lumen.preferred_radius
        safety.scale.y = 2.0 * self.lumen.preferred_radius
        safety.scale.z = self.lumen.length
        safety.color = ColorRGBA(r=0.1, g=0.9, b=0.4, a=0.10)

        axis = Marker()
        axis.header.stamp = stamp
        axis.header.frame_id = self.lumen.frame_id
        axis.ns = "cylindrical_lumen"
        axis.id = 12
        axis.type = Marker.LINE_STRIP
        axis.action = Marker.ADD
        axis.scale.x = 0.001
        axis.color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)
        axis.points = [
            _point_from_array(self.lumen.axis_origin),
            _point_from_array(self.lumen.axis_origin + self.lumen.length * self.lumen.axis_direction),
        ]

        closest = Marker()
        closest.header.stamp = stamp
        closest.header.frame_id = self.lumen.frame_id
        closest.ns = "cylindrical_lumen"
        closest.id = 13
        closest.type = Marker.SPHERE
        closest.action = Marker.ADD
        closest.scale.x = 0.006
        closest.scale.y = 0.006
        closest.scale.z = 0.006
        if clearance.collision_count:
            closest.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif clearance.safety_margin_violation_count:
            closest.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
        else:
            closest.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)
        closest.pose.position = _point_from_array(backbone_array[clearance.closest_backbone_point_index])
        closest.pose.orientation.w = 1.0

        return [wall, safety, axis, closest]


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


def _optional_vector3_parameter(values) -> list[float] | None:
    if values is None or values == "":
        return None
    if isinstance(values, (list, tuple)) and len(values) == 0:
        return None
    return [float(value) for value in _vector3(values, "cylinder_target_position")]


def _stamp_seconds(stamp) -> float:
    return float(getattr(stamp, "sec", 0)) + 1.0e-9 * float(getattr(stamp, "nanosec", 0))


def _quaternion_from_z_axis(axis: np.ndarray) -> tuple[float, float, float, float]:
    target = np.asarray(axis, dtype=float)
    target = target / max(float(np.linalg.norm(target)), 1.0e-12)
    source = np.array([0.0, 0.0, 1.0], dtype=float)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if dot > 1.0 - 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    cross = np.cross(source, target)
    quat = np.array([cross[0], cross[1], cross[2], 1.0 + dot], dtype=float)
    quat /= np.linalg.norm(quat)
    return (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def main(args=None):
    run_node_until_shutdown(rclpy, CTRSimulatorNode, args=args)


if __name__ == "__main__":
    main()
