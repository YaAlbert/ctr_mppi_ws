"""Independent command safety enforcement for the CTR simulation path."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import threading
import time
from typing import Any

import numpy as np
import rclpy
from ctr_bringup.parameter_validation import (
    load_parameter_files,
    parse_launch_bool,
    project_config_with_overrides,
    validate_config_paths,
    validate_or_raise,
)
from ctr_bringup.development_physical_evidence import (
    PhysicalEvidenceError,
    PhysicalEvidenceReader,
    PhysicalEvidenceRecord,
    PRODUCTION_HARDWARE_FRESHNESS_TIMEOUT_S,
    SIMULATOR_PAPER_EVALUATION_FRESHNESS_TIMEOUT_S,
    TRANSPORT_AUTHENTICATED_SHARED_MEMORY,
    TRANSPORT_ROS,
    TRANSPORT_VALUES,
    selected_transport,
)
from ctr_bringup.slice_7g_profile import (
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
)
from ctr_interfaces.msg import (
    CtrJointCommand,
    CtrJointState,
    CtrSafetyStatus,
    CtrState,
    CtrTactileState,
)
from ctr_interfaces.srv import ClearFault
from ctr_model.approximate_model import ApproximateCTRModel
from geometry_msgs.msg import Point
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ctr_safety.geometry_adapter import GeometryAdapter


ZERO_STATE = 0
READY_STATE = 1
WARNING_STATE = 2
STOP_STATE = 3
FAULT_STATE = 4
VALID_REGIONS = {
    CtrTactileState.REGION_NO_CONTACT,
    CtrTactileState.REGION_CONTACT,
    CtrTactileState.REGION_WARNING,
    CtrTactileState.REGION_STOP,
}


@dataclass(frozen=True)
class TactileSnapshot:
    stamp_ns: int
    frame_id: str
    valid: bool
    clearance_m: float
    force_magnitude: float
    contact: bool
    warning: bool
    stop: bool
    region: int


@dataclass(frozen=True)
class SafetyDecision:
    command: tuple[float, ...]
    allowed: bool
    emergency_stop: bool
    fault: bool
    state_name: str
    reason: str
    warning: bool = False


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _zero_command() -> tuple[float, ...]:
    return (0.0,) * 6


def _point(values: Any) -> Point:
    x, y, z = (float(value) for value in values)
    result = Point()
    result.x = x
    result.y = y
    result.z = z
    return result


class SafetySupervisorNode(Node):
    """Timer-driven composition of command, state, geometry and tactile safety.

    TODO-SAFE-010 was the former placeholder for this independent supervisor;
    the retained identifier documents its source-compatible resolution.
    """

    def __init__(self) -> None:
        super().__init__("safety_supervisor_node")
        self._lock = threading.RLock()
        self._start_mono = time.monotonic()
        self._last_tactile_stamp_ns = 0
        self._tactile: TactileSnapshot | None = None
        self._tactile_received_mono: float | None = None
        self._tactile_status = "startup_unavailable"
        self._tactile_snapshot: tuple[TactileSnapshot | None, float | None, str] = (
            None,
            None,
            "startup_unavailable",
        )
        self._stop_latched = False
        self._fault_latched = False
        self._latched_fault_reason = ""
        self._last_reason = "startup_unavailable"
        self._last_safe_command = _zero_command()
        self._raw_command: CtrJointCommand | None = None
        self._raw_command_received_mono: float | None = None
        self._state: CtrState | None = None
        self._state_received_mono: float | None = None
        self._state_snapshot: tuple[CtrState, float] | None = None
        self._last_state_source_sequence = 0
        self._last_state_source_stamp_s: float | None = None
        self._last_state_receipt_mono: float | None = None
        self._state_timing_trace: dict[str, float | int | bool] = {}
        self._last_tactile_source_sequence = 0
        self._last_tactile_source_stamp_s: float | None = None
        self._last_tactile_receipt_mono: float | None = None
        self._tactile_timing_trace: dict[str, float | int | bool] = {}

        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("enable_cylindrical_lumen", False)
        self.declare_parameter("enable_curved_lumen", False)
        self.declare_parameter("curved_lumen_type", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("slice_7g_profile", False)
        self.declare_parameter("development_simulation", False)
        self.declare_parameter("evaluation_diagnostics_enabled", False)
        self.declare_parameter("physical_evidence_transport", TRANSPORT_ROS)
        self.declare_parameter("simulator_paper_evaluation_profile", False)
        paths = validate_config_paths(self.get_parameter("config_paths").value)
        config = project_config_with_overrides(
            load_parameter_files(paths),
            runtime_mode=self.get_parameter("runtime_mode").value,
            enable_cylindrical_lumen=self.get_parameter("enable_cylindrical_lumen").value,
            enable_curved_lumen=self.get_parameter("enable_curved_lumen").value,
            curved_lumen_type=self.get_parameter("curved_lumen_type").value,
            cylinder_target_position=self.get_parameter("cylinder_target_position").value,
        )
        development_enabled = parse_launch_bool(
            self.get_parameter("development_simulation").value,
            "development_simulation",
        )
        evaluation_diagnostics_enabled = parse_launch_bool(
            self.get_parameter("evaluation_diagnostics_enabled").value,
            "evaluation_diagnostics_enabled",
        )
        if evaluation_diagnostics_enabled and not development_enabled:
            raise ValueError("evaluation diagnostics require explicit development_simulation mode")
        physical_evidence_transport = str(
            self.get_parameter("physical_evidence_transport").value
        )
        if physical_evidence_transport not in TRANSPORT_VALUES:
            raise ValueError("physical_evidence_transport is invalid")
        if (
            physical_evidence_transport == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
            and (not development_enabled or not evaluation_diagnostics_enabled)
        ):
            raise ValueError(
                "authenticated shared physical evidence requires explicit development diagnostics"
            )
        if physical_evidence_transport != selected_transport():
            raise ValueError(
                "physical_evidence_transport differs from the runner-bound environment"
            )
        self.evaluation_diagnostics_enabled = evaluation_diagnostics_enabled
        self.physical_evidence_transport = physical_evidence_transport
        simulator_paper_evaluation_profile = parse_launch_bool(
            self.get_parameter("simulator_paper_evaluation_profile").value,
            "simulator_paper_evaluation_profile",
        )
        if simulator_paper_evaluation_profile and (
            not development_enabled
            or not evaluation_diagnostics_enabled
            or physical_evidence_transport
            != TRANSPORT_AUTHENTICATED_SHARED_MEMORY
        ):
            raise ValueError(
                "simulator paper evaluation profile requires explicit development "
                "diagnostics and authenticated shared physical evidence"
            )
        self.simulator_paper_evaluation_profile = simulator_paper_evaluation_profile
        config = (
            apply_slice_7g_development_simulation_profile(config, enabled=True)
            if development_enabled
            else apply_slice_7g_simulation_profile(
                config,
                enabled=parse_launch_bool(
                    self.get_parameter("slice_7g_profile").value,
                    "slice_7g_profile",
                ),
            )
        )
        validate_or_raise(config)
        self.config = config
        safety = config["safety"]
        self.frame_id = str(config["robot"]["frames"]["base"])
        self.safety_enabled = bool(safety["enabled"])
        self.tactile_enabled = bool(safety["tactile_enabled"])
        self.state_timeout = float(safety["state_timeout"])
        self.command_timeout = float(safety["command_timeout"])
        self.tactile_timeout = float(safety["tactile_timeout"])
        if simulator_paper_evaluation_profile and (
            not math.isclose(
                self.state_timeout,
                PRODUCTION_HARDWARE_FRESHNESS_TIMEOUT_S,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
            or not math.isclose(
                self.tactile_timeout,
                PRODUCTION_HARDWARE_FRESHNESS_TIMEOUT_S,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            )
        ):
            raise ValueError(
                "simulator paper evaluation profile requires the unchanged 0.10 s "
                "production/hardware source contract"
            )
        self.physical_evidence_freshness_timeout = (
            SIMULATOR_PAPER_EVALUATION_FRESHNESS_TIMEOUT_S
            if simulator_paper_evaluation_profile
            else min(self.state_timeout, self.tactile_timeout)
        )
        self.tactile_startup_grace = float(safety["tactile_startup_grace_s"])
        self.tactile_future_skew = float(safety["tactile_future_skew_s"])
        self.watchdog_period = float(safety["watchdog_period_s"])
        self.stop_on_state_timeout = bool(safety["stop_on_state_timeout"])
        self.stop_on_tactile_timeout = bool(safety["stop_on_tactile_timeout"])
        self.stop_on_invalid_value = bool(safety["stop_on_invalid_value"])
        self.soft_contact_velocity_scale = float(safety["soft_contact"]["velocity_scale"])
        if not 0.0 < self.soft_contact_velocity_scale <= 1.0:
            raise ValueError("safety.soft_contact.velocity_scale must be in (0, 1]")
        self.geometry = GeometryAdapter(config)
        self._state_model = ApproximateCTRModel(config) if development_enabled else None
        self._physical_evidence_reader = (
            PhysicalEvidenceReader.from_environment()
            if physical_evidence_transport
            == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
            else None
        )
        self._last_shared_record: PhysicalEvidenceRecord | None = None
        self._last_shared_sequence = 0
        self._last_shared_source_stamp_ns = 0
        self._last_shared_safety_read_mono = 0.0
        self._last_ros_state_evidence: tuple[int, int, tuple[float, ...], tuple[float, ...]] | None = None
        self._last_ros_tactile_evidence: tuple[int, int, tuple[float, ...]] | None = None
        self._shared_ros_equivalence_error = ""
        self._command_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_callback_group = MutuallyExclusiveCallbackGroup()
        self._tactile_callback_group = MutuallyExclusiveCallbackGroup()
        self._watchdog_callback_group = MutuallyExclusiveCallbackGroup()
        # Safety decisions need the newest authenticated physical evidence,
        # never a historical reliable backlog.  Depth ten allowed state and
        # tactile samples to queue for longer than their unchanged 0.10 s
        # freshness contract under controller/evaluation load.  Reliable
        # keep-last depth one preserves delivery while superseding stale
        # queued evidence with a genuinely newer source sample. Reliable
        # delivery is now safe because source generation and state/tactile ROS
        # publication are separate processes with one-slot handoffs; DDS
        # backpressure cannot delay the authoritative physical clock. The
        # safety consumer therefore need not trade away delivery evidence to
        # protect source cadence.
        latest_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_tactile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_sub = self.create_subscription(
            CtrJointCommand, "/ctr/mppi_command", self._on_command, 10,
            callback_group=self._command_callback_group,
        )
        self.state_sub = (
            self.create_subscription(
                CtrJointState,
                "/ctr/safety/joint_state",
                self._on_compact_state,
                latest_state_qos,
                callback_group=self._state_callback_group,
            )
            if development_enabled
            else self.create_subscription(
                CtrState,
                "/ctr/state",
                self._on_state,
                latest_state_qos,
                callback_group=self._state_callback_group,
            )
        )
        self.tactile_sub = self.create_subscription(
            CtrTactileState, "/ctr/tactile/state", self._on_tactile, latest_tactile_qos,
            callback_group=self._tactile_callback_group,
        )
        self.safe_command_pub = self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)
        self.status_pub = self.create_publisher(CtrSafetyStatus, "/ctr/safety/status", 10)
        self.clear_fault_srv = self.create_service(
            ClearFault, "/ctr/safety/clear_fault", self._on_clear_fault,
            callback_group=self._command_callback_group,
        )
        self.watchdog_timer = self.create_timer(
            self.watchdog_period,
            self._on_watchdog,
            callback_group=self._watchdog_callback_group,
        )
        self.get_logger().info(
            f"Safety supervisor active: tactile_enabled={self.tactile_enabled}, frame={self.frame_id}, "
            f"watchdog_period_s={self.watchdog_period}"
        )

    @staticmethod
    def _stamp_ns(message: Any) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _monotonic(self) -> float:
        return time.monotonic()

    def _on_command(self, message: CtrJointCommand) -> None:
        with self._lock:
            self._raw_command = message
            self._raw_command_received_mono = self._monotonic()

    def _on_state(
        self,
        message: CtrState,
        *,
        receipt_mono: float | None = None,
    ) -> None:
        receipt_mono = self._monotonic() if receipt_mono is None else receipt_mono
        stamp_ns = self._stamp_ns(message)
        source_timing = _parse_evaluation_timing(
            str(message.diagnostic_status), "ctr_state_timing_v1"
        )
        source_sequence = int(source_timing.get("sequence", 0))
        source_stamp_s = stamp_ns * 1.0e-9
        self._state_timing_trace = {
            "source_sequence": source_sequence,
            "source_stamp_s": source_stamp_s,
            "safety_receipt_monotonic_s": receipt_mono,
            "safety_receipt_gap_s": (
                0.0
                if getattr(self, "_last_state_receipt_mono", None) is None
                else receipt_mono - self._last_state_receipt_mono
            ),
            "source_stamp_gap_s": (
                0.0
                if getattr(self, "_last_state_source_stamp_s", None) is None
                else source_stamp_s - self._last_state_source_stamp_s
            ),
            "sequence_gap": (
                0
                if not source_sequence
                or not getattr(self, "_last_state_source_sequence", 0)
                else source_sequence - self._last_state_source_sequence
            ),
            "queued_age_s": max(0.0, self._now_ns() * 1.0e-9 - source_stamp_s),
            "publisher_queued_age_s": source_timing.get("queued_age_s", 0.0),
            "publisher_mailbox_overwrites": source_timing.get(
                "mailbox_overwrites", 0
            ),
            "publisher_pid": source_timing.get("publisher_pid", 0),
            "pid": os.getpid(),
            "thread_id": threading.get_native_id(),
        }
        self._last_state_receipt_mono = receipt_mono
        self._last_state_source_stamp_s = source_stamp_s
        if source_sequence:
            self._last_state_source_sequence = source_sequence
        # Store the message and receipt clock as one atomic Python reference.
        # State delivery must never wait behind the watchdog's geometry check;
        # the immutable callback snapshot is consumed consistently below.
        self._state_snapshot = (message, receipt_mono)
        self._state = message
        self._state_received_mono = receipt_mono

    def _on_compact_state(self, message: CtrJointState) -> None:
        """Reconstruct the deterministic safety backbone from exact source q."""

        receipt_mono = self._monotonic()
        if (
            getattr(self, "physical_evidence_transport", TRANSPORT_ROS)
            == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
        ):
            timing = _parse_evaluation_timing(
                str(message.diagnostic_status), "ctr_state_timing_v1"
            )
            sequence = int(timing.get("sequence", 0))
            stamp_ns = self._stamp_ns(message)
            q = tuple(float(value) for value in message.insertion_position) + tuple(
                float(value) for value in message.rotation_position
            )
            q_dot = tuple(float(value) for value in message.joint_velocity)
            if sequence <= 0 or len(q) != 6 or len(q_dot) != 6:
                self._shared_ros_equivalence_error = (
                    "physical_evidence_ros_state_identity_invalid"
                )
                return
            self._last_ros_state_evidence = (sequence, stamp_ns, q, q_dot)
            self._reconcile_shared_ros_evidence()
            return
        reconstructed = CtrState()
        reconstructed.header.stamp = message.header.stamp
        reconstructed.header.frame_id = message.header.frame_id
        reconstructed.diagnostic_status = str(message.diagnostic_status)
        q = tuple(float(value) for value in message.insertion_position) + tuple(
            float(value) for value in message.rotation_position
        )
        q_dot = tuple(float(value) for value in message.joint_velocity)
        valid = (
            bool(message.valid)
            and len(q) == 6
            and len(q_dot) == 6
            and all(math.isfinite(value) for value in (*q, *q_dot))
            and self._state_model is not None
        )
        if valid:
            try:
                model_result = self._state_model.forward_kinematics(q)
            except (TypeError, ValueError, RuntimeError):
                valid = False
            else:
                reconstructed.q = list(q)
                reconstructed.q_dot = list(q_dot)
                reconstructed.backbone = [
                    _point(point) for point in model_result.backbone_points
                ]
                reconstructed.tip_pose.position = _point(model_result.tip_position)
                reconstructed.tip_pose.orientation.w = 1.0
        reconstructed.valid = bool(valid)
        if not valid:
            reconstructed.diagnostic_status = (
                f"{reconstructed.diagnostic_status}|compact_state_invalid"
            )
        self._on_state(reconstructed, receipt_mono=receipt_mono)

    def _tactile_invalid_reason(self, message: CtrTactileState, now_ns: int) -> str | None:
        stamp_ns = self._stamp_ns(message)
        if stamp_ns <= 0:
            return "tactile_zero_timestamp"
        if not message.header.frame_id or message.header.frame_id != self.frame_id:
            return "tactile_frame_incompatible"
        if stamp_ns > now_ns + int(self.tactile_future_skew * 1e9):
            return "tactile_future_dated"
        if not bool(message.valid):
            return "tactile_invalid"
        if message.region not in VALID_REGIONS:
            return "tactile_unknown_region"
        values = [message.clearance_m, message.force_magnitude, message.force.x, message.force.y, message.force.z]
        values.extend(message.raw_values)
        values.extend(message.filtered_values)
        if not all(_finite(value) for value in values):
            return "tactile_nonfinite"
        if message.clearance_m < 0.0:
            return "tactile_negative_clearance"
        expected_warning = message.region in (CtrTactileState.REGION_WARNING, CtrTactileState.REGION_STOP)
        expected_stop = message.region == CtrTactileState.REGION_STOP
        if bool(message.warning) != expected_warning or bool(message.stop) != expected_stop:
            return "tactile_flag_region_inconsistent"
        if message.region == CtrTactileState.REGION_NO_CONTACT and bool(message.stop):
            return "tactile_no_contact_stop_inconsistent"
        return None

    def _on_tactile(self, message: CtrTactileState) -> None:
        now_ns = self._now_ns()
        stamp_ns = self._stamp_ns(message)
        receipt_mono = self._monotonic()
        source_timing = _parse_evaluation_timing(
            str(message.diagnostic_status), "ctr_tactile_timing_v1"
        )
        source_sequence = int(source_timing.get("sequence", 0))
        source_stamp_s = stamp_ns * 1.0e-9
        if (
            getattr(self, "physical_evidence_transport", TRANSPORT_ROS)
            == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
        ):
            raw = float(message.raw_values[0]) if message.raw_values else math.nan
            filtered = (
                float(message.filtered_values[0])
                if message.filtered_values
                else math.nan
            )
            values = (
                raw,
                filtered,
                float(message.force_magnitude),
                float(message.clearance_m),
                float(message.region),
                float(bool(message.valid)),
                float(bool(message.contact)),
                float(bool(message.warning)),
                float(bool(message.stop)),
            )
            if source_sequence <= 0 or not all(math.isfinite(value) for value in values):
                self._shared_ros_equivalence_error = (
                    "physical_evidence_ros_tactile_identity_invalid"
                )
                return
            self._last_ros_tactile_evidence = (
                source_sequence,
                stamp_ns,
                values,
            )
            self._reconcile_shared_ros_evidence()
            return
        receipt_gap_s = (
            0.0
            if self._last_tactile_receipt_mono is None
            else receipt_mono - self._last_tactile_receipt_mono
        )
        source_gap_s = (
            0.0
            if self._last_tactile_source_stamp_s is None
            else source_stamp_s - self._last_tactile_source_stamp_s
        )
        duplicate_sequence = bool(
            source_sequence and source_sequence == self._last_tactile_source_sequence
        )
        out_of_order_sequence = bool(
            source_sequence and source_sequence < self._last_tactile_source_sequence
        )
        sequence_gap = (
            0
            if not source_sequence or not self._last_tactile_source_sequence
            else source_sequence - self._last_tactile_source_sequence
        )
        self._tactile_timing_trace = {
            "source_sequence": source_sequence,
            "source_stamp_s": source_stamp_s,
            "safety_receipt_monotonic_s": receipt_mono,
            "safety_receipt_gap_s": receipt_gap_s,
            "source_stamp_gap_s": source_gap_s,
            "sequence_gap": sequence_gap,
            "duplicate_sequence": duplicate_sequence,
            "out_of_order_sequence": out_of_order_sequence,
            "queued_age_s": max(0.0, self._now_ns() * 1.0e-9 - source_stamp_s),
            "pid": os.getpid(),
            "thread_id": threading.get_native_id(),
        }
        self._last_tactile_receipt_mono = receipt_mono
        self._last_tactile_source_stamp_s = source_stamp_s
        if source_sequence:
            self._last_tactile_source_sequence = source_sequence
        # Do not wait behind the watchdog's whole-backbone geometry check.
        # A tuple assignment is one atomic Python reference update, so the
        # watchdog observes either the previous complete sample or this new
        # complete sample.  The previous global-lock path could turn an on-time
        # sensor callback into evidence older than the unchanged 0.10 s safety
        # contract while the watchdog itself held the lock.
        if stamp_ns <= self._last_tactile_stamp_ns:
            self._tactile_status = "tactile_duplicate_or_out_of_order"
            self._tactile_snapshot = (
                None,
                receipt_mono,
                "tactile_duplicate_or_out_of_order",
            )
            return
        reason = self._tactile_invalid_reason(message, now_ns)
        self._last_tactile_stamp_ns = stamp_ns
        self._tactile_received_mono = receipt_mono
        if reason is not None:
            self._tactile = None
            self._tactile_status = reason
            self._tactile_snapshot = (None, receipt_mono, reason)
            return
        tactile = TactileSnapshot(
            stamp_ns=stamp_ns,
            frame_id=message.header.frame_id,
            valid=bool(message.valid),
            clearance_m=float(message.clearance_m),
            force_magnitude=float(message.force_magnitude),
            contact=bool(message.contact),
            warning=bool(message.warning),
            stop=bool(message.stop),
            region=int(message.region),
        )
        status = "eligible_stop" if message.stop else (
            "eligible_warning" if message.warning else "eligible_no_contact"
        )
        self._tactile = tactile
        self._tactile_status = status
        self._tactile_snapshot = (tactile, receipt_mono, status)
        if tactile.stop:
            # Preserve immediate stop latching; only the evidence snapshot is
            # lock-free.  Boolean/string assignments are atomic under CPython.
            self._stop_latched = True
            self._fault_latched = True
            self._latched_fault_reason = "tactile_stop"

    def _command_values(self) -> tuple[float, ...] | None:
        message = self._raw_command
        if message is None or not bool(message.valid):
            return None
        values = tuple(float(value) for value in message.q_dot)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            return None
        return values

    def _state_valid(
        self,
        now_ns: int,
        now_mono: float,
        snapshot: tuple[CtrState, float] | None = None,
    ) -> tuple[bool, str]:
        snapshot = snapshot or getattr(self, "_state_snapshot", None)
        if snapshot is None:
            if self._state is None or self._state_received_mono is None:
                return False, "state_unavailable"
            snapshot = (self._state, self._state_received_mono)
        state, received_mono = snapshot
        if state is None or received_mono is None:
            return False, "state_unavailable"
        if now_mono - received_mono > self.state_timeout:
            return False, "state_stale"
        stamp_ns = self._stamp_ns(state)
        if stamp_ns <= 0 or stamp_ns > now_ns + int(self.tactile_future_skew * 1e9):
            return False, "state_timestamp_invalid"
        if state.header.frame_id != self.frame_id or not bool(state.valid):
            return False, "state_invalid"
        points = [(point.x, point.y, point.z) for point in state.backbone]
        if not all(_finite(value) for point in points for value in point):
            return False, "state_nonfinite"
        if now_ns - stamp_ns > int(self.state_timeout * 1e9):
            return False, "state_stale"
        return True, "state_valid"

    def _tactile_eligibility(
        self,
        now_ns: int,
        now_mono: float,
        snapshot: tuple[TactileSnapshot | None, float | None, str] | None = None,
    ) -> tuple[bool, str]:
        if not self.tactile_enabled:
            return True, "tactile_disabled"
        tactile, received_mono, status = snapshot or self._tactile_snapshot
        if status not in {
            "startup_unavailable",
            "eligible_no_contact",
            "eligible_warning",
            "eligible_stop",
        }:
            return False, status
        if tactile is None or received_mono is None:
            if now_mono - self._start_mono <= self.tactile_startup_grace:
                return False, "startup_unavailable"
            return False, "tactile_timeout"
        if status not in {"eligible_no_contact", "eligible_warning", "eligible_stop"}:
            return False, status
        if now_mono - received_mono > self.tactile_timeout:
            return False, "tactile_stale"
        if now_ns - tactile.stamp_ns > int(self.tactile_timeout * 1e9):
            return False, "tactile_stale"
        return True, status

    def _decision(self) -> SafetyDecision:
        now_ns = self._now_ns()
        now_mono = self._monotonic()
        with self._lock:
            reason, values, tactile_reason, warning = self._active_fault_reason(now_ns, now_mono)
            if reason == "waiting_for_command":
                return SafetyDecision(_zero_command(), False, False, False, "ready", "waiting_for_command")
            if reason is not None:
                # Initial absence is a readiness gate, not a transient fault
                # that permanently prevents the node from reaching READY.
                latch = reason not in {"state_unavailable", "startup_unavailable"}
                return self._fault(reason, latch=latch)
            if self._fault_latched or self._stop_latched:
                latched = self._latched_fault_reason or "fault_latched"
                return SafetyDecision(
                    _zero_command(), False, self._stop_latched, True,
                    "tactile_stop" if self._stop_latched else "fault_latched", latched,
                )
            supervised = (
                tuple(value * self.soft_contact_velocity_scale for value in values)
                if warning
                else values
            )
            return SafetyDecision(
                supervised,
                True,
                False,
                False,
                "warning" if warning else "ready",
                tactile_reason,
                warning,
            )

    def _active_fault_reason(
        self, now_ns: int, now_mono: float,
    ) -> tuple[str | None, tuple[float, ...] | None, str, bool]:
        if (
            getattr(self, "physical_evidence_transport", TRANSPORT_ROS)
            == TRANSPORT_AUTHENTICATED_SHARED_MEMORY
        ):
            return self._active_shared_physical_fault_reason(now_ns, now_mono)
        state_snapshot = getattr(self, "_state_snapshot", None)
        if state_snapshot is None and self._state is not None and self._state_received_mono is not None:
            state_snapshot = (self._state, self._state_received_mono)
        state_ok, state_reason = self._state_valid(now_ns, now_mono, state_snapshot)
        if not state_ok:
            return state_reason, None, "", False
        state = state_snapshot[0]
        points = [(point.x, point.y, point.z) for point in state.backbone]
        try:
            geometry_ok, geometry_reason, _ = self.geometry.check_backbone(points)
        except Exception:
            return "geometry_invalid", None, "", False
        if not geometry_ok:
            return geometry_reason or "geometry_invalid", None, "", False
        tactile_snapshot = getattr(
            self,
            "_tactile_snapshot",
            (self._tactile, self._tactile_received_mono, self._tactile_status),
        )
        tactile, _, _ = tactile_snapshot
        tactile_ok, tactile_reason = self._tactile_eligibility(
            now_ns,
            now_mono,
            tactile_snapshot,
        )
        if self.tactile_enabled and not tactile_ok:
            return tactile_reason, None, tactile_reason, False
        if tactile is not None and tactile.stop:
            self._stop_latched = True
            self._fault_latched = True
            self._latched_fault_reason = "tactile_stop"
            return "tactile_stop", None, tactile_reason, False
        if self._raw_command is None or self._raw_command_received_mono is None:
            return "waiting_for_command", None, tactile_reason, False
        values = self._command_values()
        if values is None:
            return "command_invalid", None, tactile_reason, False
        if now_mono - self._raw_command_received_mono > self.command_timeout:
            return "command_stale", None, tactile_reason, False
        warning = tactile is not None and tactile.warning and tactile_ok
        return None, values, tactile_reason, warning

    def _active_shared_physical_fault_reason(
        self,
        now_ns: int,
        now_mono: float,
    ) -> tuple[str | None, tuple[float, ...] | None, str, bool]:
        """Evaluate one authenticated physical record without ROS fallback."""

        reader = self._physical_evidence_reader
        if reader is None:
            return "physical_evidence_unavailable", None, "", False
        try:
            record = reader.read()
        except PhysicalEvidenceError as exc:
            return str(exc), None, "", False

        # A newer immutable record can be committed while read() performs its
        # stable seqlock copy.  Sample the clocks after that copy so a genuine
        # new record cannot be misclassified as future-dated by a pre-read
        # watchdog timestamp.
        now_ns = self._now_ns()
        now_mono = self._monotonic()

        previous_sequence = self._last_shared_sequence
        previous_stamp_ns = self._last_shared_source_stamp_ns
        duplicate = record.generated_sequence == previous_sequence
        sequence_gap = (
            0
            if previous_sequence == 0
            else record.generated_sequence - previous_sequence
        )
        source_stamp_gap_s = (
            0.0
            if previous_stamp_ns == 0
            else (record.source_stamp_ns - previous_stamp_ns) * 1.0e-9
        )
        safety_receipt_gap_s = (
            0.0
            if self._last_shared_safety_read_mono == 0.0
            else now_mono - self._last_shared_safety_read_mono
        )
        self._last_shared_safety_read_mono = now_mono
        if record.generated_sequence > previous_sequence:
            self._last_shared_sequence = record.generated_sequence
            self._last_shared_source_stamp_ns = record.source_stamp_ns
        self._last_shared_record = record
        self._reconcile_shared_ros_evidence()

        now_mono_ns = int(now_mono * 1_000_000_000)
        wall_age_ns = now_ns - record.source_stamp_ns
        monotonic_age_ns = now_mono_ns - record.source_monotonic_ns
        valid_age_ns = max(0, wall_age_ns, monotonic_age_ns)
        self._tactile_timing_trace = {
            "source_sequence": record.generated_sequence,
            "source_stamp_s": record.source_stamp_ns * 1.0e-9,
            "safety_receipt_monotonic_s": now_mono,
            "safety_receipt_gap_s": safety_receipt_gap_s,
            "source_stamp_gap_s": source_stamp_gap_s,
            "sequence_gap": sequence_gap,
            "duplicate_sequence": duplicate,
            "out_of_order_sequence": False,
            "queued_age_s": valid_age_ns * 1.0e-9,
            "evidence_transport_code": 1,
            "freshness_timeout_s": self.physical_evidence_freshness_timeout,
            "producer_pid": record.producer_pid,
            "pid": os.getpid(),
            "thread_id": threading.get_native_id(),
        }

        future_skew_ns = int(self.tactile_future_skew * 1_000_000_000)
        if wall_age_ns < -future_skew_ns or monotonic_age_ns < -future_skew_ns:
            return "physical_evidence_future_dated", None, "", False
        freshness_limit_ns = int(
            self.physical_evidence_freshness_timeout * 1_000_000_000
        )
        if wall_age_ns >= freshness_limit_ns or monotonic_age_ns >= freshness_limit_ns:
            return "state_stale", None, "", False
        if self._shared_ros_equivalence_error:
            return self._shared_ros_equivalence_error, None, "", False
        if not record.source_valid or not record.frame_valid or not record.simulation:
            return "physical_evidence_source_invalid", None, "", False
        if not all(math.isfinite(value) for value in (*record.q, *record.q_dot)):
            return "state_nonfinite", None, "", False
        if (
            record.physical_collision
            or record.safety_margin_violation
            or record.whole_backbone_safety_clearance_m < 0.0
        ):
            return "whole_backbone_safety_margin", None, "", False
        if (
            not record.tactile_valid
            or record.tactile_region not in VALID_REGIONS
            or record.tactile_clearance_m < 0.0
        ):
            return "tactile_invalid", None, "tactile_invalid", False
        expected_warning = record.tactile_region in (
            CtrTactileState.REGION_WARNING,
            CtrTactileState.REGION_STOP,
        )
        expected_stop = record.tactile_region == CtrTactileState.REGION_STOP
        if record.warning != expected_warning or record.stop != expected_stop:
            return (
                "tactile_flag_region_inconsistent",
                None,
                "tactile_flag_region_inconsistent",
                False,
            )
        if record.stop:
            self._stop_latched = True
            self._fault_latched = True
            self._latched_fault_reason = "tactile_stop"
            return "tactile_stop", None, "eligible_stop", False
        if self._raw_command is None or self._raw_command_received_mono is None:
            return "waiting_for_command", None, "eligible_no_contact", False
        values = self._command_values()
        if values is None:
            return "command_invalid", None, "eligible_no_contact", False
        if now_mono - self._raw_command_received_mono > self.command_timeout:
            return "command_stale", None, "eligible_no_contact", False
        tactile_reason = "eligible_warning" if record.warning else "eligible_no_contact"
        return None, values, tactile_reason, bool(record.warning)

    def _reconcile_shared_ros_evidence(self) -> None:
        """Fail closed if common-sequence ROS values differ from the memfd record."""

        record = self._last_shared_record
        if record is None or self._shared_ros_equivalence_error:
            return
        state = self._last_ros_state_evidence
        if state is not None and state[0] == record.generated_sequence:
            if (
                state[1] != record.source_stamp_ns
                or state[2] != record.q
                or state[3] != record.q_dot
            ):
                self._shared_ros_equivalence_error = (
                    "physical_evidence_ros_state_mismatch"
                )
                return
        tactile = self._last_ros_tactile_evidence
        if tactile is not None and tactile[0] == record.generated_sequence:
            expected = (
                record.raw_tactile,
                record.filtered_tactile,
                record.tactile_force_n,
                record.tactile_clearance_m,
                float(record.tactile_region),
                float(record.tactile_valid),
                float(record.contact),
                float(record.warning),
                float(record.stop),
            )
            if tactile[1] != record.source_stamp_ns or tactile[2] != expected:
                self._shared_ros_equivalence_error = (
                    "physical_evidence_ros_tactile_mismatch"
                )

    def _fault(self, reason: str, *, latch: bool = True) -> SafetyDecision:
        self._last_reason = reason
        if latch:
            self._fault_latched = True
            if not self._latched_fault_reason:
                self._latched_fault_reason = reason
        emergency = self._stop_latched or reason == "tactile_stop"
        return SafetyDecision(
            _zero_command(), False, emergency, True,
            "tactile_stop" if emergency else "fault", reason,
        )

    def _on_watchdog(self) -> None:
        decision = self._decision()
        message = CtrJointCommand()
        now = self.get_clock().now().to_msg()
        message.header.stamp = now
        message.header.frame_id = self.frame_id
        message.q_dot = list(decision.command)
        message.valid = True
        message.diagnostic_status = decision.reason
        self.safe_command_pub.publish(message)
        with self._lock:
            self._last_safe_command = decision.command
            self._last_reason = decision.reason
        status = CtrSafetyStatus()
        status.header.stamp = now
        status.header.frame_id = self.frame_id
        status.state_name = decision.state_name
        status.state = (
            STOP_STATE
            if decision.emergency_stop
            else FAULT_STATE
            if decision.fault
            else WARNING_STATE
            if decision.warning
            else READY_STATE
        )
        status.command_allowed = decision.allowed
        status.emergency_stop = decision.emergency_stop
        status.fault = decision.fault
        status.valid = True
        status.diagnostic_status = decision.reason
        if self.evaluation_diagnostics_enabled and self._tactile_timing_trace:
            status.diagnostic_status = _with_evaluation_timing(
                status.diagnostic_status,
                "ctr_safety_timing_v1",
                self._tactile_timing_trace,
            )
        self.status_pub.publish(status)

    def _on_clear_fault(self, request: ClearFault.Request, response: ClearFault.Response) -> ClearFault.Response:
        del request
        with self._lock:
            now_ns = self._now_ns()
            reason, values, _, _ = self._active_fault_reason(now_ns, self._monotonic())
            if reason is None and values is not None:
                self._stop_latched = False
                self._fault_latched = False
                self._latched_fault_reason = ""
                response.accepted = True
                response.message = "fault cleared"
                return response
            response.accepted = False
            response.message = f"fault not clearable: {reason}"
            return response

    def destroy_node(self) -> bool:
        if getattr(self, "watchdog_timer", None) is not None:
            self.watchdog_timer.cancel()
        reader = getattr(self, "_physical_evidence_reader", None)
        if reader is not None:
            reader.close()
            self._physical_evidence_reader = None
        return super().destroy_node()


def _parse_evaluation_timing(status: str, schema: str) -> dict[str, float]:
    marker = f"|{schema}|"
    if marker not in status:
        return {}
    payload = status.rsplit(marker, 1)[1]
    result: dict[str, float] = {}
    for field in payload.split(";"):
        if "=" not in field:
            continue
        key, value = field.split("=", 1)
        try:
            result[key] = float(value)
        except ValueError:
            continue
    return result


def _with_evaluation_timing(
    status: str,
    schema: str,
    values: dict[str, float | int | bool],
) -> str:
    fields = ";".join(
        f"{key}={int(value) if type(value) is bool else value}"
        for key, value in sorted(values.items())
    )
    return f"{status}|{schema}|{fields}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = SafetySupervisorNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # ROS 2 Humble can surface a subscription-conversion RuntimeError when
        # SIGINT invalidates the context while the executor is taking a
        # message.  Treat that race as normal shutdown only after the context
        # is no longer active; a RuntimeError during normal operation remains
        # a real failure and must retain its traceback.
        if rclpy.ok():
            raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
