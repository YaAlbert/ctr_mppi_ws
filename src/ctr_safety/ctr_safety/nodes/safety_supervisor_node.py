"""Independent command safety enforcement for the CTR simulation path."""

from __future__ import annotations

from dataclasses import dataclass
import math
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
from ctr_bringup.slice_7g_profile import (
    apply_slice_7g_development_simulation_profile,
    apply_slice_7g_simulation_profile,
)
from ctr_interfaces.msg import CtrJointCommand, CtrSafetyStatus, CtrState, CtrTactileState
from ctr_interfaces.srv import ClearFault
from rclpy.node import Node
from rclpy.parameter import Parameter

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
        self._stop_latched = False
        self._fault_latched = False
        self._latched_fault_reason = ""
        self._last_reason = "startup_unavailable"
        self._last_safe_command = _zero_command()
        self._raw_command: CtrJointCommand | None = None
        self._raw_command_received_mono: float | None = None
        self._state: CtrState | None = None
        self._state_received_mono: float | None = None

        self.declare_parameter("config_paths", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("runtime_mode", "simulation")
        self.declare_parameter("enable_cylindrical_lumen", False)
        self.declare_parameter("enable_curved_lumen", False)
        self.declare_parameter("curved_lumen_type", "")
        self.declare_parameter("cylinder_target_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("slice_7g_profile", False)
        self.declare_parameter("development_simulation", False)
        self.declare_parameter("evaluation_diagnostics_enabled", False)
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

        self.command_sub = self.create_subscription(CtrJointCommand, "/ctr/mppi_command", self._on_command, 10)
        self.state_sub = self.create_subscription(CtrState, "/ctr/state", self._on_state, 10)
        self.tactile_sub = self.create_subscription(CtrTactileState, "/ctr/tactile/state", self._on_tactile, 10)
        self.safe_command_pub = self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)
        self.status_pub = self.create_publisher(CtrSafetyStatus, "/ctr/safety/status", 10)
        self.clear_fault_srv = self.create_service(ClearFault, "/ctr/safety/clear_fault", self._on_clear_fault)
        self.watchdog_timer = self.create_timer(self.watchdog_period, self._on_watchdog)
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

    def _on_state(self, message: CtrState) -> None:
        with self._lock:
            self._state = message
            self._state_received_mono = self._monotonic()

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
        with self._lock:
            if stamp_ns <= self._last_tactile_stamp_ns:
                self._tactile_status = "tactile_duplicate_or_out_of_order"
                return
            reason = self._tactile_invalid_reason(message, now_ns)
            self._last_tactile_stamp_ns = stamp_ns
            self._tactile_received_mono = self._monotonic()
            if reason is not None:
                self._tactile_status = reason
                return
            self._tactile = TactileSnapshot(
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
            self._tactile_status = "eligible_stop" if message.stop else (
                "eligible_warning" if message.warning else "eligible_no_contact"
            )
            if message.stop:
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

    def _state_valid(self, now_ns: int, now_mono: float) -> tuple[bool, str]:
        if self._state is None or self._state_received_mono is None:
            return False, "state_unavailable"
        if now_mono - self._state_received_mono > self.state_timeout:
            return False, "state_stale"
        stamp_ns = self._stamp_ns(self._state)
        if stamp_ns <= 0 or stamp_ns > now_ns + int(self.tactile_future_skew * 1e9):
            return False, "state_timestamp_invalid"
        if self._state.header.frame_id != self.frame_id or not bool(self._state.valid):
            return False, "state_invalid"
        points = [(point.x, point.y, point.z) for point in self._state.backbone]
        if not all(_finite(value) for point in points for value in point):
            return False, "state_nonfinite"
        if now_ns - stamp_ns > int(self.state_timeout * 1e9):
            return False, "state_stale"
        return True, "state_valid"

    def _tactile_eligibility(self, now_ns: int, now_mono: float) -> tuple[bool, str]:
        if not self.tactile_enabled:
            return True, "tactile_disabled"
        if self._tactile_status not in {
            "startup_unavailable",
            "eligible_no_contact",
            "eligible_warning",
            "eligible_stop",
        }:
            return False, self._tactile_status
        if self._tactile is None or self._tactile_received_mono is None:
            if now_mono - self._start_mono <= self.tactile_startup_grace:
                return False, "startup_unavailable"
            return False, "tactile_timeout"
        if self._tactile_status not in {"eligible_no_contact", "eligible_warning", "eligible_stop"}:
            return False, self._tactile_status
        if now_mono - self._tactile_received_mono > self.tactile_timeout:
            return False, "tactile_stale"
        if now_ns - self._tactile.stamp_ns > int(self.tactile_timeout * 1e9):
            return False, "tactile_stale"
        return True, self._tactile_status

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
        state_ok, state_reason = self._state_valid(now_ns, now_mono)
        if not state_ok:
            return state_reason, None, "", False
        points = [(point.x, point.y, point.z) for point in self._state.backbone]
        try:
            geometry_ok, geometry_reason, _ = self.geometry.check_backbone(points)
        except Exception:
            return "geometry_invalid", None, "", False
        if not geometry_ok:
            return geometry_reason or "geometry_invalid", None, "", False
        tactile_ok, tactile_reason = self._tactile_eligibility(now_ns, now_mono)
        if self.tactile_enabled and not tactile_ok:
            return tactile_reason, None, tactile_reason, False
        if self._tactile is not None and self._tactile.stop:
            return "tactile_stop", None, tactile_reason, False
        if self._raw_command is None or self._raw_command_received_mono is None:
            return "waiting_for_command", None, tactile_reason, False
        values = self._command_values()
        if values is None:
            return "command_invalid", None, tactile_reason, False
        if now_mono - self._raw_command_received_mono > self.command_timeout:
            return "command_stale", None, tactile_reason, False
        warning = self._tactile is not None and self._tactile.warning and tactile_ok
        return None, values, tactile_reason, warning

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
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SafetySupervisorNode()
        rclpy.spin(node)
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
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
