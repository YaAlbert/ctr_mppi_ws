"""ROS2 reference manager for fixed-target and trajectory MPPI modes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath

from ctr_bringup.parameter_validation import load_parameter_files, validate_config_paths, validate_or_raise
from ctr_bringup.placeholder_node import run_node_until_shutdown
from ctr_mppi_controller.reference_trajectory import (
    ReferenceTrajectory,
    generate_circle,
    generate_ellipse,
    generate_helix,
)
from rclpy.node import Node
from rclpy.parameter import Parameter


REFERENCE_MODES = ("fixed_target", "trajectory")
TRAJECTORY_TYPES = ("circle", "ellipse", "helix")
COMPLETION_BEHAVIORS = ("loop", "hold_final")


@dataclass(frozen=True)
class ReferenceSettings:
    mode: str
    trajectory_type: str
    frame_id: str
    completion_behavior: str
    sample_period: float
    duration: float
    publish_frequency: float
    stale_timeout: float
    fixed_target: np.ndarray
    horizon: int


class ReferenceManagerNode(Node):
    """Publish fixed or time-varying tip references for the MPPI node."""

    def __init__(self):
        super().__init__("reference_manager_node")
        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("reference_mode", "")
        self.declare_parameter("reference_type", "")

        config_paths = validate_config_paths(self.get_parameter("config_paths").value)
        self.config = load_parameter_files(config_paths)
        validate_or_raise(self.config)

        self.settings = reference_settings_from_config(
            self.config,
            mode_override=self.get_parameter("reference_mode").value,
            type_override=self.get_parameter("reference_type").value,
        )
        self.trajectory = (
            build_reference_trajectory(self.config, settings=self.settings)
            if self.settings.mode == "trajectory"
            else None
        )

        self.path_pub = self.create_publisher(NavPath, "/ctr/reference/path", 10)
        self.horizon_pub = self.create_publisher(NavPath, "/ctr/reference/horizon", 10)
        self.tip_pub = self.create_publisher(PoseStamped, "/ctr/reference/tip", 10)

        now_s = ros_time_seconds(self.get_clock().now())
        self.trajectory_start_time_s = now_s
        self.last_time_s = now_s
        self.timer = self.create_timer(1.0 / self.settings.publish_frequency, self._on_timer)
        self.get_logger().info(
            f"Reference manager started in {self.settings.mode} mode; "
            f"trajectory_type={self.settings.trajectory_type}."
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        now_s = ros_time_seconds(now)
        next_start_time_s = adjusted_trajectory_start_time(
            previous_time_s=self.last_time_s,
            current_time_s=now_s,
            start_time_s=self.trajectory_start_time_s,
        )
        if next_start_time_s != self.trajectory_start_time_s:
            self.trajectory_start_time_s = next_start_time_s
            self.get_logger().warn("ROS time moved backward; restarting reference trajectory timing.")
        self.last_time_s = now_s
        stamp = now.to_msg()

        if self.settings.mode == "fixed_target":
            self._publish_fixed_target(stamp)
            return

        if self.trajectory is None:
            raise RuntimeError("trajectory mode requires a generated reference trajectory")
        horizon = self.trajectory.horizon_at_time(
            current_time=now_s,
            start_time=self.trajectory_start_time_s,
            horizon_length=self.settings.horizon,
        )
        validate_reference_points(horizon.points, expected_count=self.settings.horizon)

        self.path_pub.publish(path_from_points(self.trajectory.points, self.settings.frame_id, stamp))
        self.horizon_pub.publish(path_from_points(horizon.points, self.settings.frame_id, stamp))
        self.tip_pub.publish(pose_from_point(horizon.current_point, self.settings.frame_id, stamp))

    def _publish_fixed_target(self, stamp) -> None:
        self.path_pub.publish(path_from_points([self.settings.fixed_target], self.settings.frame_id, stamp))
        self.tip_pub.publish(pose_from_point(self.settings.fixed_target, self.settings.frame_id, stamp))


def reference_settings_from_config(
    config: dict[str, Any],
    *,
    mode_override: Any = None,
    type_override: Any = None,
) -> ReferenceSettings:
    reference = _reference_config(config)
    mode = _choice(_override_or_config(mode_override, reference["mode"]), "reference.mode", REFERENCE_MODES)
    trajectory_type = _choice(
        _override_or_config(type_override, reference["trajectory_type"]),
        "reference.trajectory_type",
        TRAJECTORY_TYPES,
    )
    completion_behavior = _choice(reference["completion_behavior"], "reference.completion_behavior", COMPLETION_BEHAVIORS)
    if bool(reference["loop"]) != (completion_behavior == "loop"):
        raise ValueError("reference.loop must match reference.completion_behavior")

    return ReferenceSettings(
        mode=mode,
        trajectory_type=trajectory_type,
        frame_id=_non_empty_string(reference["frame_id"], "reference.frame_id"),
        completion_behavior=completion_behavior,
        sample_period=_positive_number(reference["sample_period"], "reference.sample_period"),
        duration=_positive_number(reference["duration"], "reference.duration"),
        publish_frequency=_positive_number(reference["publish_frequency"], "reference.publish_frequency"),
        stale_timeout=_positive_number(reference["stale_timeout"], "reference.stale_timeout"),
        fixed_target=_vector3(reference["fixed_target"], "reference.fixed_target"),
        horizon=_positive_int(config["mppi"]["horizon"], "mppi.horizon"),
    )


def build_reference_trajectory(
    config: dict[str, Any],
    *,
    settings: ReferenceSettings | None = None,
    mode_override: Any = None,
    type_override: Any = None,
) -> ReferenceTrajectory:
    settings = settings or reference_settings_from_config(
        config,
        mode_override=mode_override,
        type_override=type_override,
    )
    reference = _reference_config(config)
    common = {
        "duration": settings.duration,
        "sample_period": settings.sample_period,
        "frame_id": settings.frame_id,
        "completion_behavior": settings.completion_behavior,
    }
    if settings.trajectory_type == "circle":
        circle = reference["circle"]
        return generate_circle(
            center=circle["center"],
            radius=circle["radius"],
            angular_velocity=circle["angular_velocity"],
            phase=circle["phase"],
            **common,
        )
    if settings.trajectory_type == "ellipse":
        ellipse = reference["ellipse"]
        return generate_ellipse(
            center=ellipse["center"],
            radii=ellipse["radii"],
            angular_velocity=ellipse["angular_velocity"],
            phase=ellipse["phase"],
            **common,
        )
    if settings.trajectory_type == "helix":
        helix = reference["helix"]
        return generate_helix(
            center=helix["center"],
            radius=helix["radius"],
            height=helix["height"],
            angular_velocity=helix["angular_velocity"],
            phase=helix["phase"],
            **common,
        )
    raise ValueError(f"unsupported reference trajectory type: {settings.trajectory_type}")


def path_from_points(points: Any, frame_id: str, stamp=None) -> NavPath:
    point_array = validate_reference_points(points)
    frame = _non_empty_string(frame_id, "frame_id")
    msg = NavPath()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.poses = [pose_from_point(point, frame, stamp) for point in point_array]
    return msg


def pose_from_point(point: Any, frame_id: str, stamp=None) -> PoseStamped:
    point_array = _vector3(point, "point")
    frame = _non_empty_string(frame_id, "frame_id")
    msg = PoseStamped()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x = float(point_array[0])
    msg.pose.position.y = float(point_array[1])
    msg.pose.position.z = float(point_array[2])
    msg.pose.orientation.w = 1.0
    return msg


def validate_reference_points(points: Any, *, expected_count: int | None = None) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("reference points must have shape (N, 3)")
    if expected_count is not None and array.shape[0] != expected_count:
        raise ValueError(f"reference points must contain exactly {expected_count} points")
    if not np.all(np.isfinite(array)):
        raise ValueError("reference points must contain finite values")
    return array.copy()


def ros_time_seconds(time_value: Any) -> float:
    if hasattr(time_value, "nanoseconds"):
        seconds = float(time_value.nanoseconds) * 1.0e-9
    elif hasattr(time_value, "sec") and hasattr(time_value, "nanosec"):
        seconds = float(time_value.sec) + float(time_value.nanosec) * 1.0e-9
    else:
        raise ValueError("ROS time value must provide nanoseconds or sec/nanosec fields")
    if not np.isfinite(seconds):
        raise ValueError("ROS time must be finite")
    return seconds


def adjusted_trajectory_start_time(*, previous_time_s: float, current_time_s: float, start_time_s: float) -> float:
    previous = _number(previous_time_s, "previous_time_s")
    current = _number(current_time_s, "current_time_s")
    start = _number(start_time_s, "start_time_s")
    if current < previous:
        return current
    return start


def _reference_config(config: dict[str, Any]) -> dict[str, Any]:
    reference = config.get("reference")
    if not isinstance(reference, dict):
        raise ValueError("configuration must contain a reference map")
    return reference


def _override_or_config(override: Any, configured: Any) -> Any:
    if override is None:
        return configured
    if isinstance(override, str) and override == "":
        return configured
    return override


def _choice(value: Any, label: str, allowed: tuple[str, ...]) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be one of {allowed}")
    if value not in allowed:
        raise ValueError(f"{label} must be one of {allowed}")
    return value


def _non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _vector3(values: Any, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain 3 finite values")
    return array.copy()


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    numeric = float(value)
    if not np.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _positive_number(value: Any, label: str) -> float:
    numeric = _number(value, label)
    if numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    return numeric


def _positive_int(value: Any, label: str) -> int:
    numeric = _positive_number(value, label)
    integer = int(numeric)
    if integer != numeric:
        raise ValueError(f"{label} must be an integer")
    return integer


def main(args=None):
    run_node_until_shutdown(rclpy, ReferenceManagerNode, args=args)


if __name__ == "__main__":
    main()
