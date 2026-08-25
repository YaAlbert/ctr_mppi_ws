"""Static RViz marker construction for lumen geometry."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import copy
import math
from typing import Any, Iterable

import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_sim.lumen_diagnostics import (
    LumenRuntimeDiagnostic,
    STATUS_COLLISION,
    STATUS_MARGIN,
    STATUS_SAFE,
    STATUS_UNAVAILABLE,
)


MIN_RING_SEGMENTS = 8
MAX_RING_SEGMENTS = 128
FRAME_TOLERANCE = 1.0e-12
STATIC_LUMEN_STYLE_VERSION = 2

LUMEN_CENTERLINE_KEY = ("lumen_centerline", 0)
LUMEN_SURFACE_KEY = ("lumen_surface", 0)
LUMEN_PHYSICAL_BOUNDARY_KEY = ("lumen_wireframe", 0)
LUMEN_SAFETY_BOUNDARY_KEY = ("lumen_wireframe", 1)
LUMEN_INLET_KEY = ("lumen_wireframe", 2)
LUMEN_OUTLET_KEY = ("lumen_wireframe", 3)
CURVED_STATIC_LUMEN_MARKER_KEYS = (
    LUMEN_CENTERLINE_KEY,
    LUMEN_PHYSICAL_BOUNDARY_KEY,
    LUMEN_SAFETY_BOUNDARY_KEY,
    LUMEN_INLET_KEY,
    LUMEN_OUTLET_KEY,
)
CURVED_STATIC_LUMEN_MARKER_KEYS_WITH_SURFACE = (
    LUMEN_SURFACE_KEY,
    *CURVED_STATIC_LUMEN_MARKER_KEYS,
)

LUMEN_CLOSEST_LINE_KEY = ("lumen_closest_pair", 0)
LUMEN_BACKBONE_WITNESS_KEY = ("lumen_closest_pair", 1)
LUMEN_BOUNDARY_WITNESS_KEY = ("lumen_closest_pair", 2)
LUMEN_STATUS_KEY = ("lumen_status", 0)
DYNAMIC_LUMEN_MARKER_KEYS = (
    LUMEN_CLOSEST_LINE_KEY,
    LUMEN_BACKBONE_WITNESS_KEY,
    LUMEN_BOUNDARY_WITNESS_KEY,
    LUMEN_STATUS_KEY,
)


@dataclass(frozen=True)
class LumenMarkerConfig:
    publish_lumen_markers: bool = True
    publish_lumen_diagnostics: bool = True
    centerline_stride: int = 1
    ring_stride: int = 4
    ring_segments: int = 20
    marker_publish_rate: float = 5.0
    publish_lumen_surface: bool = False
    surface_alpha: float = 0.20
    actual_tip_history_max_points: int = 500
    actual_tip_history_min_interval: float = 0.05

    @classmethod
    def from_mapping(cls, values: Any) -> "LumenMarkerConfig":
        if values is None:
            return cls()
        if not isinstance(values, dict):
            raise ValueError("simulation.visualization must be a map")
        return cls(
            publish_lumen_markers=_bool_value(
                values.get("publish_lumen_markers", cls.publish_lumen_markers),
                "simulation.visualization.publish_lumen_markers",
            ),
            publish_lumen_diagnostics=_bool_value(
                values.get("publish_lumen_diagnostics", cls.publish_lumen_diagnostics),
                "simulation.visualization.publish_lumen_diagnostics",
            ),
            centerline_stride=_int_value(
                values.get("centerline_stride", cls.centerline_stride),
                "simulation.visualization.centerline_stride",
                minimum=1,
            ),
            ring_stride=_int_value(
                values.get("ring_stride", cls.ring_stride),
                "simulation.visualization.ring_stride",
                minimum=1,
            ),
            ring_segments=_int_value(
                values.get("ring_segments", cls.ring_segments),
                "simulation.visualization.ring_segments",
                minimum=MIN_RING_SEGMENTS,
                maximum=MAX_RING_SEGMENTS,
            ),
            marker_publish_rate=_positive_number(
                values.get("marker_publish_rate", cls.marker_publish_rate),
                "simulation.visualization.marker_publish_rate",
            ),
            publish_lumen_surface=_bool_value(
                values.get("publish_lumen_surface", cls.publish_lumen_surface),
                "simulation.visualization.publish_lumen_surface",
            ),
            surface_alpha=_unit_interval(
                values.get("surface_alpha", cls.surface_alpha),
                "simulation.visualization.surface_alpha",
            ),
            actual_tip_history_max_points=_int_value(
                values.get("actual_tip_history_max_points", cls.actual_tip_history_max_points),
                "simulation.visualization.actual_tip_history_max_points",
                minimum=2,
                maximum=5000,
            ),
            actual_tip_history_min_interval=_positive_number(
                values.get("actual_tip_history_min_interval", cls.actual_tip_history_min_interval),
                "simulation.visualization.actual_tip_history_min_interval",
            ),
        )


@dataclass(frozen=True)
class TransportFrames:
    tangents: np.ndarray
    normals: np.ndarray
    binormals: np.ndarray


class BoundedTipTrajectory:
    """Bounded, time-decimated development-only tip history."""

    def __init__(self, *, max_points: int, minimum_interval: float) -> None:
        self.max_points = _int_value(max_points, "max_points", minimum=2, maximum=5000)
        self.minimum_interval = _positive_number(minimum_interval, "minimum_interval")
        self._points: deque[np.ndarray] = deque(maxlen=self.max_points)
        self._last_time: float | None = None

    def append(self, point: Any, timestamp: Any) -> bool:
        sample = _vector3(point, "tip trajectory point")
        time_value = _finite_number(timestamp, "tip trajectory timestamp")
        if self._last_time is not None and time_value >= self._last_time:
            if time_value - self._last_time < self.minimum_interval:
                return False
        elif self._last_time is not None:
            self.clear()
        self._points.append(sample)
        self._last_time = time_value
        return True

    def clear(self) -> None:
        self._points.clear()
        self._last_time = None

    def points(self) -> np.ndarray:
        if not self._points:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(tuple(self._points)).astype(np.float64, copy=True)


def compute_parallel_transport_frames(centerline_points: Any) -> TransportFrames:
    """Return deterministic right-handed transport frames for sampled points."""

    points = _points_array(centerline_points, "centerline_points")
    tangents = _estimate_tangents(points)
    normals = np.empty_like(tangents)
    binormals = np.empty_like(tangents)

    normal = _initial_normal(tangents[0])
    binormal = _unit(np.cross(tangents[0], normal), "initial binormal")
    normal, binormal = _orthonormal_pair(tangents[0], normal, binormal)
    normals[0] = normal
    binormals[0] = binormal

    for index in range(1, tangents.shape[0]):
        transported = _transport_normal(tangents[index - 1], tangents[index], normals[index - 1])
        normal, binormal = _orthonormal_pair(tangents[index], transported, binormals[index - 1])
        if float(np.dot(normal, normals[index - 1])) < 0.0 and float(np.dot(tangents[index - 1], tangents[index])) > 0.0:
            normal = -normal
            binormal = -binormal
        normals[index] = normal
        binormals[index] = binormal

    _make_readonly(tangents)
    _make_readonly(normals)
    _make_readonly(binormals)
    return TransportFrames(tangents=tangents, normals=normals, binormals=binormals)


def sample_ring(
    center: Any,
    normal: Any,
    binormal: Any,
    radius: Any,
    segments: int,
) -> np.ndarray:
    """Sample a ring as LINE_LIST endpoint pairs."""

    center_array = _vector3(center, "ring center")
    normal_array = _unit(_vector3(normal, "ring normal"), "ring normal")
    binormal_array = _unit(_vector3(binormal, "ring binormal"), "ring binormal")
    radius_value = _positive_number(radius, "ring radius")
    segment_count = _int_value(segments, "ring_segments", minimum=MIN_RING_SEGMENTS, maximum=MAX_RING_SEGMENTS)
    points = np.empty((2 * segment_count, 3), dtype=np.float64)
    samples = np.empty((segment_count, 3), dtype=np.float64)
    for index in range(segment_count):
        theta = 2.0 * math.pi * float(index) / float(segment_count)
        samples[index] = center_array + radius_value * (
            math.cos(theta) * normal_array + math.sin(theta) * binormal_array
        )
    for index in range(segment_count):
        points[2 * index] = samples[index]
        points[2 * index + 1] = samples[(index + 1) % segment_count]
    if not np.all(np.isfinite(points)):
        raise ValueError("ring points must be finite")
    return points


def build_curved_static_lumen_markers(
    geometry: CurvedLumen,
    geometry_fingerprint: str,
    frame_id: str,
    visualization_config: LumenMarkerConfig | dict[str, Any],
    stamp: Any,
) -> list[Marker]:
    """Build static C3 markers for a sampled curved lumen."""

    if not isinstance(geometry, CurvedLumen):
        raise ValueError("curved static lumen markers require CurvedLumen geometry")
    if not isinstance(geometry_fingerprint, str) or not geometry_fingerprint:
        raise ValueError("geometry_fingerprint must be a non-empty string")
    marker_config = _marker_config(visualization_config)
    if not marker_config.publish_lumen_markers:
        return []
    frame = _non_empty_string(frame_id, "frame_id")
    if frame != geometry.frame_id:
        raise ValueError(f"frame_id `{frame}` does not match curved lumen frame_id `{geometry.frame_id}`")

    centerline = _points_array(geometry.centerline_points, "curved_lumen.centerline_points")
    radius_profile = _radius_profile(geometry, centerline.shape[0])
    safety_radius = radius_profile - float(geometry.ctr_outer_radius) - float(geometry.safety_margin)
    if not np.all(np.isfinite(safety_radius)) or np.any(safety_radius <= 0.0):
        raise ValueError("curved lumen safety boundary radius must be finite and positive")

    frames = compute_parallel_transport_frames(centerline)
    centerline_indices = _stride_indices(centerline.shape[0], marker_config.centerline_stride)
    ring_indices = _stride_indices(centerline.shape[0], marker_config.ring_stride)

    markers = [
        _triangle_surface_marker(
            ns=LUMEN_SURFACE_KEY[0],
            marker_id=LUMEN_SURFACE_KEY[1],
            frame_id=frame,
            stamp=stamp,
            centerline=centerline,
            normals=frames.normals,
            binormals=frames.binormals,
            radii=radius_profile,
            segments=marker_config.ring_segments,
            color=ColorRGBA(r=0.72, g=0.34, b=0.34, a=marker_config.surface_alpha),
        ) if marker_config.publish_lumen_surface else None,
        _line_strip_marker(
            ns=LUMEN_CENTERLINE_KEY[0],
            marker_id=LUMEN_CENTERLINE_KEY[1],
            frame_id=frame,
            stamp=stamp,
            points=centerline[list(centerline_indices)],
            scale=0.0015,
            color=ColorRGBA(r=0.25, g=0.75, b=1.0, a=1.0),
        ),
        _ring_list_marker(
            ns=LUMEN_PHYSICAL_BOUNDARY_KEY[0],
            marker_id=LUMEN_PHYSICAL_BOUNDARY_KEY[1],
            frame_id=frame,
            stamp=stamp,
            centerline=centerline,
            normals=frames.normals,
            binormals=frames.binormals,
            radii=radius_profile,
            indices=ring_indices,
            segments=marker_config.ring_segments,
            scale=0.0010,
            color=ColorRGBA(r=0.0, g=0.9, b=1.0, a=0.70),
        ),
        _ring_list_marker(
            ns=LUMEN_SAFETY_BOUNDARY_KEY[0],
            marker_id=LUMEN_SAFETY_BOUNDARY_KEY[1],
            frame_id=frame,
            stamp=stamp,
            centerline=centerline,
            normals=frames.normals,
            binormals=frames.binormals,
            radii=safety_radius,
            indices=ring_indices,
            segments=marker_config.ring_segments,
            scale=0.0010,
            color=ColorRGBA(r=0.1, g=0.9, b=0.4, a=0.65),
        ),
        _single_ring_marker(
            ns=LUMEN_INLET_KEY[0],
            marker_id=LUMEN_INLET_KEY[1],
            frame_id=frame,
            stamp=stamp,
            center=centerline[0],
            normal=frames.normals[0],
            binormal=frames.binormals[0],
            radius=radius_profile[0],
            segments=marker_config.ring_segments,
            scale=0.0014,
            color=ColorRGBA(r=1.0, g=0.85, b=0.15, a=1.0),
        ),
        _single_ring_marker(
            ns=LUMEN_OUTLET_KEY[0],
            marker_id=LUMEN_OUTLET_KEY[1],
            frame_id=frame,
            stamp=stamp,
            center=centerline[-1],
            normal=frames.normals[-1],
            binormal=frames.binormals[-1],
            radius=radius_profile[-1],
            segments=marker_config.ring_segments,
            scale=0.0014,
            color=ColorRGBA(r=1.0, g=0.25, b=0.15, a=1.0),
        ),
    ]
    markers = [marker for marker in markers if marker is not None]
    _validate_markers(markers)
    return markers


def build_reference_path_markers(
    points: Any,
    frame_id: str,
    stamp: Any,
) -> list[Marker]:
    """Render only the points contained in the authoritative reference Path."""

    path = _path_points_array(points, "reference path points")
    frame = _non_empty_string(frame_id, "frame_id")
    color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
    markers: list[Marker] = []
    if path.shape[0] >= 2:
        markers.append(
            _line_strip_marker(
                ns="reference_path",
                marker_id=0,
                frame_id=frame,
                stamp=stamp,
                points=path,
                scale=0.004,
                color=color,
            )
        )
    points_marker = _base_marker(
        "reference_path", 1, frame, stamp, Marker.SPHERE_LIST, 0.005, color
    )
    points_marker.scale.y = points_marker.scale.x
    points_marker.scale.z = points_marker.scale.x
    points_marker.points = [_point_from_array(point) for point in path]
    markers.append(points_marker)
    _validate_markers(markers)
    return markers


def build_actual_tip_path_marker(
    points: Any,
    frame_id: str,
    stamp: Any,
) -> Marker | None:
    """Build the bright-green executed path once two real samples exist."""

    path = _path_points_array(points, "actual tip path points", allow_empty=True)
    if path.shape[0] < 2:
        return None
    marker = _line_strip_marker(
        ns="actual_tip_path",
        marker_id=0,
        frame_id=frame_id,
        stamp=stamp,
        points=path,
        scale=0.0035,
        color=ColorRGBA(r=0.15, g=1.0, b=0.15, a=1.0),
    )
    _validate_markers([marker])
    return marker


def build_static_lumen_delete_markers(
    previous_marker_keys: Iterable[tuple[str, int]],
    frame_id: str,
    stamp: Any,
) -> list[Marker]:
    markers: list[Marker] = []
    frame = _non_empty_string(frame_id, "frame_id")
    seen: set[tuple[str, int]] = set()
    for namespace, marker_id in previous_marker_keys:
        key = (str(namespace), int(marker_id))
        if key in seen:
            continue
        seen.add(key)
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = key[0]
        marker.id = key[1]
        marker.action = Marker.DELETE
        markers.append(marker)
    return markers


def build_dynamic_lumen_diagnostic_markers(
    diagnostic: LumenRuntimeDiagnostic,
    stamp: Any,
) -> list[Marker]:
    """Build dynamic C4 markers from one validated runtime diagnostic."""

    if not isinstance(diagnostic, LumenRuntimeDiagnostic):
        raise ValueError("diagnostic must be a LumenRuntimeDiagnostic")
    if not diagnostic.valid or diagnostic.status == STATUS_UNAVAILABLE:
        return []
    color = _diagnostic_color(diagnostic.status)
    markers: list[Marker] = []
    if diagnostic.witness_available:
        markers.append(
            _line_list_marker(
                ns=LUMEN_CLOSEST_LINE_KEY[0],
                marker_id=LUMEN_CLOSEST_LINE_KEY[1],
                frame_id=diagnostic.frame_id,
                stamp=stamp,
                points=np.asarray([diagnostic.ctr_surface_point, diagnostic.lumen_boundary_point], dtype=np.float64),
                scale=0.0020,
                color=color,
            )
        )
        markers.append(
            _sphere_marker(
                ns=LUMEN_BACKBONE_WITNESS_KEY[0],
                marker_id=LUMEN_BACKBONE_WITNESS_KEY[1],
                frame_id=diagnostic.frame_id,
                stamp=stamp,
                position=diagnostic.ctr_surface_point,
                diameter=0.0060,
                color=color,
            )
        )
        markers.append(
            _sphere_marker(
                ns=LUMEN_BOUNDARY_WITNESS_KEY[0],
                marker_id=LUMEN_BOUNDARY_WITNESS_KEY[1],
                frame_id=diagnostic.frame_id,
                stamp=stamp,
                position=diagnostic.lumen_boundary_point,
                diameter=0.0060,
                color=color,
            )
        )
    markers.append(
        _status_text_marker(
            ns=LUMEN_STATUS_KEY[0],
            marker_id=LUMEN_STATUS_KEY[1],
            frame_id=diagnostic.frame_id,
            stamp=stamp,
            position=diagnostic.backbone_center_point + np.array([0.0, 0.0, 0.014], dtype=np.float64),
            text=_diagnostic_text(diagnostic),
            color=color,
        )
    )
    _validate_markers(markers)
    return markers


def build_dynamic_lumen_delete_markers(
    previous_marker_keys: Iterable[tuple[str, int]],
    frame_id: str,
    stamp: Any,
) -> list[Marker]:
    return build_static_lumen_delete_markers(previous_marker_keys, frame_id, stamp)


def static_lumen_cache_key(
    geometry_fingerprint: str,
    visualization_config: LumenMarkerConfig | dict[str, Any],
) -> tuple[Any, ...]:
    config = _marker_config(visualization_config)
    if not isinstance(geometry_fingerprint, str) or not geometry_fingerprint:
        raise ValueError("geometry_fingerprint must be a non-empty string")
    return (
        "static_lumen_markers",
        STATIC_LUMEN_STYLE_VERSION,
        geometry_fingerprint,
        config.publish_lumen_markers,
        config.centerline_stride,
        config.ring_stride,
        config.ring_segments,
        config.publish_lumen_surface,
        config.surface_alpha,
    )


def marker_keys(markers: Iterable[Marker]) -> tuple[tuple[str, int], ...]:
    return tuple((str(marker.ns), int(marker.id)) for marker in markers)


def markers_with_stamp(markers: Iterable[Marker], stamp: Any) -> list[Marker]:
    stamped: list[Marker] = []
    for marker in markers:
        copied = copy.deepcopy(marker)
        copied.header.stamp = stamp
        stamped.append(copied)
    return stamped


def _line_strip_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    points: np.ndarray,
    scale: float,
    color: ColorRGBA,
) -> Marker:
    if points.shape[0] < 2:
        raise ValueError(f"{ns} requires at least two points")
    marker = _base_marker(ns, marker_id, frame_id, stamp, Marker.LINE_STRIP, scale, color)
    marker.points = [_point_from_array(point) for point in points]
    return marker


def _line_list_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    points: np.ndarray,
    scale: float,
    color: ColorRGBA,
) -> Marker:
    point_array = np.asarray(points, dtype=np.float64)
    if point_array.shape != (2, 3):
        raise ValueError(f"{ns} requires exactly two line endpoint points")
    marker = _base_marker(ns, marker_id, frame_id, stamp, Marker.LINE_LIST, scale, color)
    marker.points = [_point_from_array(point) for point in point_array]
    return marker


def _ring_list_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    centerline: np.ndarray,
    normals: np.ndarray,
    binormals: np.ndarray,
    radii: np.ndarray,
    indices: tuple[int, ...],
    segments: int,
    scale: float,
    color: ColorRGBA,
) -> Marker:
    marker = _base_marker(ns, marker_id, frame_id, stamp, Marker.LINE_LIST, scale, color)
    ring_points = [
        sample_ring(centerline[index], normals[index], binormals[index], radii[index], segments)
        for index in indices
    ]
    combined = np.vstack(ring_points) if ring_points else np.empty((0, 3), dtype=np.float64)
    marker.points = [_point_from_array(point) for point in combined]
    return marker


def _triangle_surface_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    centerline: np.ndarray,
    normals: np.ndarray,
    binormals: np.ndarray,
    radii: np.ndarray,
    segments: int,
    color: ColorRGBA,
) -> Marker:
    segment_count = _int_value(
        segments, "ring_segments", minimum=MIN_RING_SEGMENTS, maximum=MAX_RING_SEGMENTS
    )
    angles = 2.0 * math.pi * np.arange(segment_count, dtype=np.float64) / float(segment_count)
    rings = (
        centerline[:, None, :]
        + radii[:, None, None]
        * (
            np.cos(angles)[None, :, None] * normals[:, None, :]
            + np.sin(angles)[None, :, None] * binormals[:, None, :]
        )
    )
    triangles = np.empty(((rings.shape[0] - 1) * segment_count * 6, 3), dtype=np.float64)
    cursor = 0
    for ring_index in range(rings.shape[0] - 1):
        for segment_index in range(segment_count):
            next_segment = (segment_index + 1) % segment_count
            current = rings[ring_index, segment_index]
            current_next = rings[ring_index, next_segment]
            following = rings[ring_index + 1, segment_index]
            following_next = rings[ring_index + 1, next_segment]
            triangles[cursor : cursor + 6] = (
                current,
                following_next,
                following,
                current,
                current_next,
                following_next,
            )
            cursor += 6
    if not np.all(np.isfinite(triangles)):
        raise ValueError("lumen surface points must be finite")
    marker = _base_marker(ns, marker_id, frame_id, stamp, Marker.TRIANGLE_LIST, 1.0, color)
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.points = [_point_from_array(point) for point in triangles]
    return marker


def _single_ring_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    center: np.ndarray,
    normal: np.ndarray,
    binormal: np.ndarray,
    radius: float,
    segments: int,
    scale: float,
    color: ColorRGBA,
) -> Marker:
    marker = _base_marker(ns, marker_id, frame_id, stamp, Marker.LINE_LIST, scale, color)
    marker.points = [_point_from_array(point) for point in sample_ring(center, normal, binormal, radius, segments)]
    return marker


def _sphere_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    position: np.ndarray,
    diameter: float,
    color: ColorRGBA,
) -> Marker:
    marker = _base_marker(ns, marker_id, frame_id, stamp, Marker.SPHERE, diameter, color)
    marker.scale.y = marker.scale.x
    marker.scale.z = marker.scale.x
    marker.pose.position = _point_from_array(position)
    return marker


def _status_text_marker(
    *,
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    position: np.ndarray,
    text: str,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = _non_empty_string(frame_id, "frame_id")
    marker.ns = str(ns)
    marker.id = int(marker_id)
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = _point_from_array(position)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.0060
    marker.color = color
    marker.text = str(text)
    if not marker.text:
        raise ValueError(f"{ns}.text must be non-empty")
    return marker


def _base_marker(
    ns: str,
    marker_id: int,
    frame_id: str,
    stamp: Any,
    marker_type: int,
    scale: float,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = _non_empty_string(frame_id, "frame_id")
    marker.ns = str(ns)
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = _positive_number(scale, f"{ns}.scale.x")
    marker.color = color
    return marker


def _diagnostic_color(status: str) -> ColorRGBA:
    if status == STATUS_SAFE:
        return ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)
    if status == STATUS_MARGIN:
        return ColorRGBA(r=1.0, g=0.62, b=0.0, a=1.0)
    if status == STATUS_COLLISION:
        return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    raise ValueError(f"unsupported diagnostic status {status}")


def _diagnostic_text(diagnostic: LumenRuntimeDiagnostic) -> str:
    return (
        f"state={diagnostic.status}\n"
        f"physical_clearance={diagnostic.physical_clearance:.6f} m\n"
        f"safety_clearance={diagnostic.safety_clearance:.6f} m\n"
        f"constraint={diagnostic.constraint_type}\n"
        f"backbone_index={diagnostic.backbone_index}"
    )


def _estimate_tangents(points: np.ndarray) -> np.ndarray:
    tangents = np.empty_like(points)
    for index in range(points.shape[0]):
        direction = _sample_direction(points, index)
        tangents[index] = _unit(direction, f"centerline tangent[{index}]")
    return tangents


def _sample_direction(points: np.ndarray, index: int) -> np.ndarray:
    previous_index = _nearest_distinct_index(points, index, step=-1)
    next_index = _nearest_distinct_index(points, index, step=1)
    if previous_index is not None and next_index is not None:
        return points[next_index] - points[previous_index]
    if next_index is not None:
        return points[next_index] - points[index]
    if previous_index is not None:
        return points[index] - points[previous_index]
    raise ValueError("centerline_points must contain at least two distinct points")


def _nearest_distinct_index(points: np.ndarray, index: int, *, step: int) -> int | None:
    cursor = index + step
    while 0 <= cursor < points.shape[0]:
        if float(np.linalg.norm(points[cursor] - points[index])) > FRAME_TOLERANCE:
            return cursor
        cursor += step
    return None


def _initial_normal(tangent: np.ndarray) -> np.ndarray:
    axes = np.eye(3, dtype=np.float64)
    axis = axes[int(np.argmin(np.abs(axes @ tangent)))]
    normal = axis - float(np.dot(axis, tangent)) * tangent
    return _unit(normal, "initial normal")


def _transport_normal(previous_tangent: np.ndarray, tangent: np.ndarray, normal: np.ndarray) -> np.ndarray:
    dot = float(np.clip(np.dot(previous_tangent, tangent), -1.0, 1.0))
    cross = np.cross(previous_tangent, tangent)
    cross_norm = float(np.linalg.norm(cross))
    if cross_norm <= FRAME_TOLERANCE:
        return normal.copy()
    axis = cross / cross_norm
    angle = math.atan2(cross_norm, dot)
    return _rotate_vector(normal, axis, angle)


def _rotate_vector(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    return (
        vector * math.cos(angle)
        + np.cross(axis, vector) * math.sin(angle)
        + axis * float(np.dot(axis, vector)) * (1.0 - math.cos(angle))
    )


def _orthonormal_pair(
    tangent: np.ndarray,
    normal_candidate: np.ndarray,
    previous_binormal: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    normal = normal_candidate - float(np.dot(normal_candidate, tangent)) * tangent
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= FRAME_TOLERANCE:
        if previous_binormal is not None and float(np.linalg.norm(previous_binormal)) > FRAME_TOLERANCE:
            normal = np.cross(previous_binormal, tangent)
        else:
            normal = _initial_normal(tangent)
    normal = _unit(normal, "transport normal")
    binormal = _unit(np.cross(tangent, normal), "transport binormal")
    if float(np.dot(np.cross(normal, binormal), tangent)) < 0.0:
        binormal = -binormal
    return normal, binormal


def _stride_indices(point_count: int, stride: int) -> tuple[int, ...]:
    stride_value = _int_value(stride, "stride", minimum=1)
    indices = list(range(0, point_count, stride_value))
    if indices[-1] != point_count - 1:
        indices.append(point_count - 1)
    return tuple(dict.fromkeys(indices))


def _radius_profile(geometry: CurvedLumen, expected_count: int) -> np.ndarray:
    profile = np.asarray(getattr(geometry, "radius_profile", geometry.lumen_radius), dtype=np.float64)
    if profile.shape == ():
        profile = np.full(expected_count, float(profile), dtype=np.float64)
    if profile.shape != (expected_count,):
        raise ValueError("curved_lumen.lumen_radius profile must have one value per centerline point")
    if not np.all(np.isfinite(profile)) or np.any(profile <= 0.0):
        raise ValueError("curved_lumen.lumen_radius profile values must be finite and positive")
    return profile.copy()


def _validate_markers(markers: Iterable[Marker]) -> None:
    for marker in markers:
        _non_empty_string(marker.header.frame_id, f"{marker.ns}.header.frame_id")
        if marker.action != Marker.ADD:
            raise ValueError(f"{marker.ns}.action must be Marker.ADD")
        for label, value in (
            ("scale.x", marker.scale.x),
            ("scale.y", marker.scale.y),
            ("scale.z", marker.scale.z),
        ):
            if value != 0.0 and not math.isfinite(float(value)):
                raise ValueError(f"{marker.ns}.{label} must be finite")
        for label, value in (
            ("color.r", marker.color.r),
            ("color.g", marker.color.g),
            ("color.b", marker.color.b),
            ("color.a", marker.color.a),
        ):
            numeric = float(value)
            if not math.isfinite(numeric) or numeric < 0.0 or numeric > 1.0:
                raise ValueError(f"{marker.ns}.{label} must be finite and in [0, 1]")
        point_marker_types = {
            Marker.LINE_LIST,
            Marker.LINE_STRIP,
            Marker.TRIANGLE_LIST,
            Marker.SPHERE_LIST,
        }
        if marker.type in point_marker_types and not marker.points:
            raise ValueError(f"{marker.ns} must contain marker points")
        if marker.type == Marker.TRIANGLE_LIST and len(marker.points) % 3:
            raise ValueError(f"{marker.ns} must contain complete triangles")
        for point_index, point in enumerate(marker.points):
            coordinates = (float(point.x), float(point.y), float(point.z))
            if not all(math.isfinite(value) for value in coordinates):
                raise ValueError(f"{marker.ns}.points[{point_index}] must be finite")
        if marker.type in {Marker.SPHERE, Marker.TEXT_VIEW_FACING}:
            coordinates = (
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
            )
            if not all(math.isfinite(value) for value in coordinates):
                raise ValueError(f"{marker.ns}.pose.position must be finite")
        if marker.type == Marker.TEXT_VIEW_FACING and not marker.text:
            raise ValueError(f"{marker.ns}.text must be non-empty")


def _points_array(values: Any, label: str) -> np.ndarray:
    if values is None:
        raise ValueError(f"{label} is required")
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric with shape (N, 3)") from exc
    if array.ndim != 2:
        raise ValueError(f"{label} must have rank 2")
    if array.shape[0] < 2:
        raise ValueError(f"{label} must contain at least two points")
    if array.shape[1] != 3:
        raise ValueError(f"{label} must have shape (N, 3)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    return array.astype(np.float64, copy=True)


def _path_points_array(values: Any, label: str, *, allow_empty: bool = False) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric with shape (N, 3)") from exc
    if allow_empty and array.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if array.ndim != 2 or array.shape[1:] != (3,) or array.shape[0] < 1:
        raise ValueError(f"{label} must have shape (N, 3) with at least one point")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain finite values")
    return array.astype(np.float64, copy=True)


def _vector3(values: Any, label: str) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain three finite values") from exc
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain three finite values")
    return array.copy()


def _unit(values: np.ndarray, label: str) -> np.ndarray:
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= FRAME_TOLERANCE:
        raise ValueError(f"{label} must be non-zero and finite")
    return values / norm


def _point_from_array(values: Iterable[float]) -> Point:
    x, y, z = [float(value) for value in values]
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def _marker_config(values: LumenMarkerConfig | dict[str, Any]) -> LumenMarkerConfig:
    if isinstance(values, LumenMarkerConfig):
        return values
    return LumenMarkerConfig.from_mapping(values)


def _bool_value(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be a boolean")
    return value


def _int_value(value: Any, label: str, *, minimum: int, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{label} must be an integer")
    if value < minimum:
        raise ValueError(f"{label} must be >= {minimum}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{label} must be <= {maximum}")
    return int(value)


def _positive_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite and positive") from exc
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return numeric


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be numeric, not boolean")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    return numeric


def _unit_interval(value: Any, label: str) -> float:
    numeric = _finite_number(value, label)
    if numeric < 0.0 or numeric > 1.0:
        raise ValueError(f"{label} must be in [0, 1]")
    return numeric


def _non_empty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string")
    return value


def _make_readonly(array: np.ndarray) -> None:
    array.setflags(write=False)
