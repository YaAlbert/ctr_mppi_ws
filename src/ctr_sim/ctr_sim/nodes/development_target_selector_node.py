"""Development-only CLI and RViz target selection for Slice 7G.

This node owns the one-shot handoff from a user candidate to the existing
controller reference topics.  It never runs outside explicit development
simulation, never changes the configured profile target, and never accepts a
second target after the first reference has been published.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Any, Callable

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from ctr_bringup.parameter_validation import (
    load_parameter_files,
    parse_launch_bool,
    validate_config_paths,
    validate_or_raise,
)
from ctr_bringup.placeholder_node import run_node_until_shutdown
from ctr_bringup.slice_7g_profile import apply_slice_7g_development_simulation_profile
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_mppi_controller.lumen_factory import config_with_lumen_overrides, lumen_geometry_from_config
from ctr_mppi_controller.nodes.reference_manager_node import path_from_points, pose_from_point


TARGET_SOURCE_MODES = ("profile", "cli", "rviz")
TARGET_POINT_TOPIC = "/ctr/target_point_candidate"
TARGET_STATUS_TOPIC = "/ctr/target_selection/status"
TARGET_RECORD_TOPIC = "/ctr/target_selection/record"
TARGET_CANDIDATE_TOPIC = "/ctr/development_visualization/target_candidate"
TARGET_INVALID_TOPIC = "/ctr/development_visualization/target_candidate_invalid"
TARGET_STATUS_MARKER_TOPIC = "/ctr/development_visualization/target_status"
REFERENCE_STARTUP_BURST_COUNT = 20


def target_selection_qos_profile() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


@dataclass(frozen=True)
class TargetSelectionResult:
    target_source: str
    raw_input_point: tuple[float, float, float]
    raw_input_frame: str
    validated_target: tuple[float, float, float] | None
    controller_target_frame: str
    projection_distance: float
    projected: bool
    accepted: bool
    status: str
    orientation_used: bool
    accepted_target_timestamp: float | None
    reference_pose_count: int
    seed: int

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True, separators=(",", ":"), allow_nan=False)


def transform_candidate_to_controller_frame(
    point: Any,
    *,
    input_frame: str,
    controller_frame: str,
    world_frame: str,
) -> np.ndarray:
    """Apply the established fixed identity ``world -> base_link`` transform."""

    candidate = _vector3(point, "target candidate")
    frame = _non_empty_string(input_frame, "target candidate frame")
    controller = _non_empty_string(controller_frame, "controller frame")
    world = _non_empty_string(world_frame, "world frame")
    if frame == controller:
        return candidate
    if frame == world:
        # slice_7g_development_visual.launch.py establishes this exact physical
        # identity transform.  No other inferred/fake transform is accepted.
        return candidate
    raise ValueError("target_frame_invalid")


def validate_candidate_timestamp(
    *,
    stamp_seconds: float,
    now_seconds: float,
    maximum_age: float,
    future_tolerance: float,
) -> None:
    stamp = _nonnegative_number(stamp_seconds, "candidate timestamp")
    now = _nonnegative_number(now_seconds, "current ROS time")
    age_limit = _positive_number(maximum_age, "candidate maximum age")
    future_limit = _nonnegative_number(future_tolerance, "candidate future tolerance")
    # Zero is the explicit interactive-development "latest" timestamp.
    if stamp == 0.0:
        return
    age = now - stamp
    if age > age_limit:
        raise ValueError("target_timestamp_stale")
    if age < -future_limit:
        raise ValueError("target_timestamp_future")


def target_update_status(target_already_accepted: bool) -> str:
    if type(target_already_accepted) is not bool:
        raise ValueError("target_already_accepted must be a bool")
    return "target_update_rejected_motion_started" if target_already_accepted else "target_candidate_received"


def select_development_target(
    point: Any,
    *,
    input_frame: str,
    target_source: str,
    geometry: CurvedLumen,
    controller_frame: str,
    world_frame: str,
    projection_limit: float,
    reachable: Callable[[np.ndarray], bool],
    accepted_target_timestamp: float,
    seed: int,
) -> TargetSelectionResult:
    source = _choice(target_source, "target_source", ("cli", "rviz"))
    raw = _vector3(point, "target candidate")
    frame = _non_empty_string(input_frame, "target candidate frame")
    limit = _positive_number(projection_limit, "target projection limit")
    timestamp = _nonnegative_number(accepted_target_timestamp, "accepted target timestamp")
    target = transform_candidate_to_controller_frame(
        raw,
        input_frame=frame,
        controller_frame=controller_frame,
        world_frame=world_frame,
    )
    projection = geometry.project_point(target)
    clearance = geometry.point_clearance(target)

    if clearance.inlet_violation or clearance.outlet_violation:
        return _result(
            source, raw, frame, None, controller_frame, 0.0, False, False,
            "target_unreachable", timestamp, seed,
        )
    outside_lumen = projection.radial_distance > projection.local_radius + 1.0e-12
    if outside_lumen and source == "cli":
        return _result(
            source, raw, frame, None, controller_frame, 0.0, False, False,
            "target_outside_lumen", timestamp, seed,
        )
    if outside_lumen and projection.radial_distance > limit:
        return _result(
            source, raw, frame, None, controller_frame, 0.0, False, False,
            "target_projection_too_far", timestamp, seed,
        )

    validation = geometry.validate_target(
        target,
        frame_id=controller_frame,
        require_safety_margin=True,
    )
    projected = False
    projection_distance = 0.0
    selected = target
    if not validation.valid or outside_lumen:
        if source != "rviz":
            return _result(
                source, raw, frame, None, controller_frame, 0.0, False, False,
                "target_clearance_invalid", timestamp, seed,
            )
        if projection.radial_distance > limit:
            return _result(
                source, raw, frame, None, controller_frame, 0.0, False, False,
                "target_projection_too_far", timestamp, seed,
            )
        selected = np.asarray(projection.closest_point, dtype=np.float64)
        projected = not np.array_equal(selected, target)
        projection_distance = float(projection.radial_distance)
        projected_validation = geometry.validate_target(
            selected,
            frame_id=controller_frame,
            require_safety_margin=True,
        )
        if not projected_validation.valid:
            return _result(
                source, raw, frame, None, controller_frame, projection_distance,
                projected, False, "target_clearance_invalid", timestamp, seed,
            )

    if not bool(reachable(selected.copy())):
        return _result(
            source, raw, frame, None, controller_frame, projection_distance,
            projected, False, "target_unreachable", timestamp, seed,
        )
    return _result(
        source, raw, frame, selected, controller_frame, projection_distance,
        projected, True, "target_accepted", timestamp, seed,
    )


def build_sampled_reachability_cloud(
    model: ApproximateCTRModel,
    config: dict[str, Any],
) -> np.ndarray:
    """Reproduce the repository's deterministic approximate-model sanity set."""

    limits = config["robot"]["limits"]
    insertion_min = _vector3(limits["insertion_min"], "insertion_min")
    insertion_max = _vector3(limits["insertion_max"], "insertion_max")
    rotation_min = _vector3(limits["rotation_min"], "rotation_min")
    rotation_max = _vector3(limits["rotation_max"], "rotation_max")
    insertion_values = (
        insertion_min,
        insertion_max,
        0.5 * (insertion_min + insertion_max),
        np.array([insertion_max[0], insertion_min[1], insertion_max[2]]),
        np.array([insertion_min[0], insertion_max[1], insertion_max[2]]),
    )
    rotation_values = (
        np.zeros(3, dtype=np.float64),
        rotation_min,
        rotation_max,
        0.5 * (rotation_min + rotation_max),
    )
    goal = config["goal"]
    sample_count = _exact_nonnegative_int(goal["reachability_samples"], "reachability_samples")
    sample_seed = _exact_nonnegative_int(goal["reachability_seed"], "reachability_seed")
    rng = np.random.default_rng(sample_seed)
    random_insertions = rng.uniform(insertion_min, insertion_max, size=(sample_count, 3))
    random_rotations = rng.uniform(rotation_min, rotation_max, size=(sample_count, 3))
    configurations = [
        np.concatenate((insertion, rotation))
        for insertion in insertion_values
        for rotation in rotation_values
    ]
    configurations.extend(
        np.concatenate((insertion, rotation))
        for insertion, rotation in zip(random_insertions, random_rotations)
    )
    tips: list[np.ndarray] = []
    for configuration in configurations:
        try:
            tip = np.asarray(model.forward_kinematics(configuration).tip_position, dtype=np.float64)
        except Exception:
            continue
        if tip.shape == (3,) and np.all(np.isfinite(tip)):
            tips.append(tip.copy())
    if not tips:
        raise ValueError("target reachability model produced no finite samples")
    return np.asarray(tips, dtype=np.float64)


def sampled_reachability_predicate(points: Any, tolerance: float) -> Callable[[np.ndarray], bool]:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1:] != (3,) or cloud.shape[0] < 1:
        raise ValueError("reachability cloud must have shape (N, 3)")
    if not np.all(np.isfinite(cloud)):
        raise ValueError("reachability cloud must contain finite points")
    threshold = _positive_number(tolerance, "reachability tolerance")

    def reachable(target: np.ndarray) -> bool:
        candidate = _vector3(target, "reachability target")
        return float(np.min(np.linalg.norm(cloud - candidate, axis=1))) <= threshold

    return reachable


class DevelopmentTargetSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("development_target_selector")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("development_simulation", False)
        self.declare_parameter("target_source", "profile")
        self.declare_parameter("target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("seed", 11)
        self.declare_parameter("wait_for_target", True)
        self.declare_parameter("target_selection_timeout", 0.0)
        self.declare_parameter("projection_limit", -1.0)

        if not parse_launch_bool(
            self.get_parameter("development_simulation").value,
            "development_simulation",
        ):
            raise ValueError("development target selection requires development_simulation=true")
        self.target_source = _choice(
            self.get_parameter("target_source").value,
            "target_source",
            ("cli", "rviz"),
        )
        self.wait_for_target = parse_launch_bool(
            self.get_parameter("wait_for_target").value,
            "wait_for_target",
        )
        self.selection_timeout = _nonnegative_number(
            self.get_parameter("target_selection_timeout").value,
            "target_selection_timeout",
        )
        if self.target_source == "rviz" and not self.wait_for_target and self.selection_timeout == 0.0:
            raise ValueError("rviz target selection requires wait_for_target=true or a positive timeout")

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)
        raw_config = load_parameter_files(config_paths)
        config = config_with_lumen_overrides(
            raw_config,
            enable_cylindrical_lumen=False,
            enable_curved_lumen=True,
            curved_lumen_type="circular_arc",
            cylinder_profile="cylinder_fast",
            random_seed=self.get_parameter("seed").value,
        )
        self.config = apply_slice_7g_development_simulation_profile(config, enabled=True)
        validate_or_raise(self.config)
        geometry = lumen_geometry_from_config(self.config)
        if not isinstance(geometry, CurvedLumen):
            raise ValueError("development target selection requires CurvedLumen geometry")
        self.geometry = geometry
        self.controller_frame = str(self.config["reference"]["frame_id"])
        self.world_frame = str(self.config["robot"]["frames"]["world"])
        selection_config = self.config["simulation"]["development_target_selection"]
        override_limit = float(self.get_parameter("projection_limit").value)
        self.projection_limit = (
            _positive_number(override_limit, "projection_limit")
            if override_limit > 0.0
            else _positive_number(selection_config["projection_limit"], "projection_limit")
        )
        self.candidate_max_age = _positive_number(
            selection_config["candidate_max_age"], "candidate_max_age"
        )
        self.candidate_future_tolerance = _nonnegative_number(
            selection_config["candidate_future_tolerance"],
            "candidate_future_tolerance",
        )
        self.seed = _exact_nonnegative_int(self.get_parameter("seed").value, "seed")
        reachability_cloud = build_sampled_reachability_cloud(
            ApproximateCTRModel(self.config), self.config
        )
        self.reachable = sampled_reachability_predicate(
            reachability_cloud, float(self.config["goal"]["tolerance"])
        )

        qos = target_selection_qos_profile()
        self.reference_path_pub = self.create_publisher(NavPath, "/ctr/reference/path", qos)
        self.reference_tip_pub = self.create_publisher(PoseStamped, "/ctr/reference/tip", qos)
        self.status_pub = self.create_publisher(String, TARGET_STATUS_TOPIC, qos)
        self.record_pub = self.create_publisher(String, TARGET_RECORD_TOPIC, qos)
        self.candidate_marker_pub = self.create_publisher(MarkerArray, TARGET_CANDIDATE_TOPIC, qos)
        self.invalid_marker_pub = self.create_publisher(MarkerArray, TARGET_INVALID_TOPIC, qos)
        self.status_marker_pub = self.create_publisher(MarkerArray, TARGET_STATUS_MARKER_TOPIC, qos)
        self.candidate_sub = (
            self.create_subscription(PointStamped, TARGET_POINT_TOPIC, self._on_candidate, 10)
            if self.target_source == "rviz"
            else None
        )
        self.accepted: TargetSelectionResult | None = None
        self.last_result: TargetSelectionResult | None = None
        self.selection_expired = False
        self.reference_burst_remaining = 0
        self.started_at = self._now_seconds()
        self.timer = self.create_timer(0.1, self._on_timer)
        self._publish_status("waiting_for_target")
        if self.target_source == "cli":
            self._process_candidate(
                self.get_parameter("target_position").value,
                self.controller_frame,
                stamp_seconds=self._now_seconds(),
            )

    def _on_candidate(self, msg: PointStamped) -> None:
        raw = [msg.point.x, msg.point.y, msg.point.z]
        if self.selection_expired:
            self._publish_status("target_selection_timeout")
            self.get_logger().warn("Rejected target after the selection timeout.")
            return
        if self.accepted is not None:
            self._publish_status(target_update_status(True))
            self.get_logger().warn("Rejected target update after navigation start.")
            return
        self._publish_status(target_update_status(False))
        try:
            stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
            validate_candidate_timestamp(
                stamp_seconds=stamp_s,
                now_seconds=self._now_seconds(),
                maximum_age=self.candidate_max_age,
                future_tolerance=self.candidate_future_tolerance,
            )
        except (TypeError, ValueError) as exc:
            self._reject_without_transform(raw, str(msg.header.frame_id), str(exc))
            return
        self._process_candidate(raw, str(msg.header.frame_id), stamp_seconds=stamp_s or self._now_seconds())

    def _process_candidate(self, point: Any, frame_id: str, *, stamp_seconds: float) -> None:
        try:
            transformed = transform_candidate_to_controller_frame(
                point,
                input_frame=frame_id,
                controller_frame=self.controller_frame,
                world_frame=self.world_frame,
            )
        except (TypeError, ValueError) as exc:
            self._reject_without_transform(point, frame_id, str(exc))
            return
        self.candidate_marker_pub.publish(
            MarkerArray(markers=[_sphere_marker(
                transformed,
                self.controller_frame,
                self.get_clock().now().to_msg(),
                namespace="target_candidate",
                marker_id=0,
                color=ColorRGBA(r=1.0, g=0.45, b=0.0, a=0.75),
            )])
        )
        try:
            result = select_development_target(
                point,
                input_frame=frame_id,
                target_source=self.target_source,
                geometry=self.geometry,
                controller_frame=self.controller_frame,
                world_frame=self.world_frame,
                projection_limit=self.projection_limit,
                reachable=self.reachable,
                accepted_target_timestamp=stamp_seconds,
                seed=self.seed,
            )
        except (TypeError, ValueError) as exc:
            self._reject_without_transform(point, frame_id, str(exc))
            return
        self.last_result = result
        self._publish_record(result)
        if not result.accepted:
            self.candidate_marker_pub.publish(MarkerArray(markers=[_delete_marker(
                "target_candidate", 0, self.controller_frame, self.get_clock().now().to_msg()
            )]))
            self.invalid_marker_pub.publish(
                MarkerArray(markers=[_sphere_marker(
                    transformed,
                    self.controller_frame,
                    self.get_clock().now().to_msg(),
                    namespace="target_candidate_invalid",
                    marker_id=0,
                    color=ColorRGBA(r=1.0, g=0.05, b=0.05, a=1.0),
                )])
            )
            self._publish_status(result.status)
            self.get_logger().warn(result.status)
            return
        self.accepted = result
        # Preserve the raw clicked point after projection, but recolor it from
        # pending orange to accepted green so the user can see the adjustment
        # relative to the yellow controller target.
        self.candidate_marker_pub.publish(
            MarkerArray(markers=[_sphere_marker(
                transformed,
                self.controller_frame,
                self.get_clock().now().to_msg(),
                namespace="target_candidate",
                marker_id=0,
                color=ColorRGBA(r=0.2, g=1.0, b=0.25, a=0.85),
            )])
        )
        self.invalid_marker_pub.publish(MarkerArray(markers=[_delete_marker(
            "target_candidate_invalid", 0, self.controller_frame, self.get_clock().now().to_msg()
        )]))
        self._publish_status("target_accepted")
        self.get_logger().info(f"target_accepted {result.to_json()}")
        self.reference_burst_remaining = REFERENCE_STARTUP_BURST_COUNT - 1
        self._publish_reference()

    def _reject_without_transform(self, point: Any, frame_id: str, status: str) -> None:
        try:
            raw = tuple(float(value) for value in _vector3(point, "target candidate"))
        except (TypeError, ValueError):
            raw = (0.0, 0.0, 0.0)
        stable_status = status if status.startswith("target_") else "target_candidate_invalid"
        result = _result(
            self.target_source,
            raw,
            str(frame_id),
            None,
            self.controller_frame,
            0.0,
            False,
            False,
            stable_status,
            self._now_seconds(),
            self.seed,
        )
        self.last_result = result
        self._publish_record(result)
        self._publish_status(stable_status)
        self.get_logger().warn(stable_status)

    def _on_timer(self) -> None:
        if self.accepted is not None:
            if self.reference_burst_remaining > 0:
                self._publish_reference()
                self.reference_burst_remaining -= 1
            return
        if self.selection_timeout > 0.0 and self._now_seconds() - self.started_at >= self.selection_timeout:
            if not self.selection_expired:
                self.selection_expired = True
                self._publish_status("target_selection_timeout")
                self.get_logger().warn("Target selection timed out; restart to select another target.")

    def _publish_reference(self) -> None:
        if self.accepted is None or self.accepted.validated_target is None:
            return
        stamp = self.get_clock().now().to_msg()
        target = self.accepted.validated_target
        self.reference_path_pub.publish(path_from_points([target], self.controller_frame, stamp))
        self.reference_tip_pub.publish(pose_from_point(target, self.controller_frame, stamp))

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = str(status)
        self.status_pub.publish(message)
        stamp = self.get_clock().now().to_msg()
        position = np.asarray(self.geometry.centerline_points[0], dtype=np.float64) + np.array([0.0, 0.0, 0.012])
        self.status_marker_pub.publish(MarkerArray(markers=[_text_marker(
            position,
            self.controller_frame,
            stamp,
            str(status),
        )]))

    def _publish_record(self, result: TargetSelectionResult) -> None:
        message = String()
        message.data = result.to_json()
        self.record_pub.publish(message)

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1.0e-9


def _result(
    source: str,
    raw: Any,
    frame: str,
    target: Any | None,
    controller_frame: str,
    projection_distance: float,
    projected: bool,
    accepted: bool,
    status: str,
    timestamp: float,
    seed: int,
) -> TargetSelectionResult:
    raw_tuple = tuple(float(value) for value in _vector3(raw, "raw target"))
    target_tuple = None if target is None else tuple(float(value) for value in _vector3(target, "validated target"))
    return TargetSelectionResult(
        target_source=source,
        raw_input_point=raw_tuple,
        raw_input_frame=str(frame),
        validated_target=target_tuple,
        controller_target_frame=str(controller_frame),
        projection_distance=float(projection_distance),
        projected=bool(projected),
        accepted=bool(accepted),
        status=str(status),
        orientation_used=False,
        accepted_target_timestamp=float(timestamp) if accepted else None,
        reference_pose_count=1 if accepted else 0,
        seed=int(seed),
    )


def _sphere_marker(position, frame_id, stamp, *, namespace: str, marker_id: int, color: ColorRGBA) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.007
    marker.color = color
    return marker


def _text_marker(position, frame_id, stamp, text: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "target_status"
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.006
    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    marker.text = text
    return marker


def _delete_marker(namespace: str, marker_id: int, frame_id: str, stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.action = Marker.DELETE
    return marker


def _vector3(value: Any, label: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain exactly 3 finite numbers") from exc
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain exactly 3 finite numbers")
    return result.copy()


def _choice(value: Any, label: str, choices: tuple[str, ...]) -> str:
    if type(value) is not str or value not in choices:
        raise ValueError(f"{label} must be one of {choices}")
    return value


def _non_empty_string(value: Any, label: str) -> str:
    if type(value) is not str or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _positive_number(value: Any, label: str) -> float:
    number = _finite_number(value, label)
    if number <= 0.0:
        raise ValueError(f"{label} must be positive")
    return number


def _nonnegative_number(value: Any, label: str) -> float:
    number = _finite_number(value, label)
    if number < 0.0:
        raise ValueError(f"{label} must be nonnegative")
    return number


def _finite_number(value: Any, label: str) -> float:
    if type(value) not in (int, float) or isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number")
    return number


def _exact_nonnegative_int(value: Any, label: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{label} must be a nonnegative integer")
    return value


def main(args=None) -> None:
    run_node_until_shutdown(rclpy, DevelopmentTargetSelectorNode, args=args)


if __name__ == "__main__":
    main()
