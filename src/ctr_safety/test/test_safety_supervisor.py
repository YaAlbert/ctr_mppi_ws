import math
import sys
import threading
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


try:
    from ctr_interfaces.msg import (
        CtrJointCommand,
        CtrJointState,
        CtrSafetyStatus,
        CtrState,
        CtrTactileState,
    )
    from ctr_interfaces.srv import ClearFault
except ImportError:
    from geometry_msgs.msg import Point, Vector3

    def _header():
        return SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0),
            frame_id="",
        )

    class CtrJointCommand:
        def __init__(self):
            self.header = _header()
            self.q_dot = []
            self.valid = False
            self.diagnostic_status = ""

    class CtrJointState:
        def __init__(self):
            self.header = _header()
            self.insertion_position = []
            self.rotation_position = []
            self.joint_velocity = []
            self.valid = False
            self.diagnostic_status = ""

    class CtrState:
        def __init__(self):
            self.header = _header()
            self.valid = False
            self.backbone = []
            self.tip_pose = SimpleNamespace(
                position=Point(),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            )

    class CtrTactileState:
        REGION_NO_CONTACT = 0
        REGION_CONTACT = 1
        REGION_WARNING = 2
        REGION_STOP = 3

        def __init__(self):
            self.header = _header()
            self.raw_values = []
            self.filtered_values = []
            self.force = Vector3()
            self.force_magnitude = 0.0
            self.contact = False
            self.warning = False
            self.stop = False
            self.valid = False
            self.diagnostic_status = ""
            self.clearance_m = 0.0
            self.source = "simulated"
            self.region = self.REGION_NO_CONTACT

    class CtrSafetyStatus:
        def __init__(self):
            self.header = _header()

    class ClearFault:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.accepted = False
                self.message = ""

    interface_package = types.ModuleType("ctr_interfaces")
    message_module = types.ModuleType("ctr_interfaces.msg")
    service_module = types.ModuleType("ctr_interfaces.srv")
    for name, value in {
        "CtrJointCommand": CtrJointCommand,
        "CtrJointState": CtrJointState,
        "CtrSafetyStatus": CtrSafetyStatus,
        "CtrState": CtrState,
        "CtrTactileState": CtrTactileState,
    }.items():
        setattr(message_module, name, value)
    service_module.ClearFault = ClearFault
    sys.modules["ctr_interfaces"] = interface_package
    sys.modules["ctr_interfaces.msg"] = message_module
    sys.modules["ctr_interfaces.srv"] = service_module

from ctr_safety.nodes import safety_supervisor_node as supervisor_module
from ctr_safety.nodes.safety_supervisor_node import SafetySupervisorNode, _zero_command


class FakeGeometry:
    def __init__(self, safe=True):
        self.safe = safe

    def check_backbone(self, points):
        return self.safe, "geometry_safe" if self.safe else "whole_backbone_safety_margin", 0.001


class SafetySupervisorMainShutdownTest(unittest.TestCase):
    def test_shutdown_context_normalizes_executor_runtime_error(self):
        node = mock.Mock()
        executor = mock.Mock()
        executor.spin.side_effect = RuntimeError("Unable to convert call argument to Python object")
        with (
            mock.patch.object(supervisor_module.rclpy, "init"),
            mock.patch.object(supervisor_module, "SafetySupervisorNode", return_value=node),
            mock.patch.object(supervisor_module, "MultiThreadedExecutor", return_value=executor),
            mock.patch.object(supervisor_module.rclpy, "ok", return_value=False),
            mock.patch.object(supervisor_module.rclpy, "shutdown") as shutdown,
        ):
            supervisor_module.main()

        executor.add_node.assert_called_once_with(node)
        executor.shutdown.assert_called_once_with()
        node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()

    def test_active_context_preserves_executor_runtime_error(self):
        node = mock.Mock()
        failure = RuntimeError("active executor failure")
        executor = mock.Mock()
        executor.spin.side_effect = failure
        with (
            mock.patch.object(supervisor_module.rclpy, "init"),
            mock.patch.object(supervisor_module, "SafetySupervisorNode", return_value=node),
            mock.patch.object(supervisor_module, "MultiThreadedExecutor", return_value=executor),
            mock.patch.object(supervisor_module.rclpy, "ok", return_value=True),
            mock.patch.object(supervisor_module.rclpy, "shutdown") as shutdown,
        ):
            with self.assertRaisesRegex(RuntimeError, "active executor failure"):
                supervisor_module.main()

        node.destroy_node.assert_called_once_with()
        executor.shutdown.assert_called_once_with()
        shutdown.assert_called_once_with()

    def test_source_separates_evidence_and_watchdog_callback_groups(self):
        source = Path(supervisor_module.__file__).read_text(encoding="utf-8")
        self.assertIn("MultiThreadedExecutor(num_threads=4)", source)
        self.assertIn("callback_group=self._state_callback_group", source)
        self.assertIn("callback_group=self._tactile_callback_group", source)
        self.assertIn("callback_group=self._command_callback_group", source)
        self.assertIn("callback_group=self._watchdog_callback_group", source)

    def test_development_state_and_tactile_use_closed_latest_evidence_qos(self):
        source = Path(supervisor_module.__file__).read_text(encoding="utf-8")
        self.assertIn("history=HistoryPolicy.KEEP_LAST", source)
        self.assertIn("depth=1", source)
        self.assertIn("reliability=ReliabilityPolicy.RELIABLE", source)
        self.assertIn("durability=DurabilityPolicy.VOLATILE", source)
        self.assertIn(
            '"/ctr/safety/joint_state"',
            source,
        )
        self.assertIn("latest_state_qos,", source)
        self.assertIn("latest_tactile_qos,", source)

    def test_compact_state_reconstructs_the_exact_deterministic_backbone(self):
        node = make_node()
        node._state_model = SimpleNamespace(
            forward_kinematics=lambda q: SimpleNamespace(
                backbone_points=[(q[0], q[1], q[2]), (q[3], q[4], q[5])],
                tip_position=(q[3], q[4], q[5]),
            )
        )
        captured = []
        node._monotonic = lambda: 12.5
        node._on_state = lambda message, **kwargs: captured.append((message, kwargs))
        message = CtrJointState()
        stamp(message, 2_000_000_000)
        message.insertion_position = [0.01, 0.02, 0.03]
        message.rotation_position = [0.1, 0.2, 0.3]
        message.joint_velocity = [0.0] * 6
        message.valid = True
        message.diagnostic_status = "source"

        node._on_compact_state(message)

        self.assertEqual(1, len(captured))
        reconstructed, receipt = captured[0]
        self.assertEqual({"receipt_mono": 12.5}, receipt)
        self.assertTrue(reconstructed.valid)
        self.assertEqual(
            [0.01, 0.02, 0.03, 0.1, 0.2, 0.3],
            list(reconstructed.q),
        )
        self.assertEqual(2, len(reconstructed.backbone))
        self.assertEqual(0.1, reconstructed.tip_pose.position.x)


def stamp(message, nanoseconds):
    message.header.stamp.sec = nanoseconds // 1_000_000_000
    message.header.stamp.nanosec = nanoseconds % 1_000_000_000
    message.header.frame_id = "base_link"


def make_node(*, tactile_enabled=True, geometry_safe=True):
    node = SafetySupervisorNode.__new__(SafetySupervisorNode)
    node._lock = threading.RLock()
    node._start_mono = 0.0
    node._last_tactile_stamp_ns = 0
    node._tactile = None
    node._tactile_received_mono = None
    node._tactile_status = "startup_unavailable"
    node._tactile_snapshot = (None, None, "startup_unavailable")
    node._stop_latched = False
    node._fault_latched = False
    node._latched_fault_reason = ""
    node._last_reason = "startup_unavailable"
    node._last_safe_command = _zero_command()
    node._raw_command = None
    node._raw_command_received_mono = None
    node._state = None
    node._state_received_mono = None
    node._state_snapshot = None
    node._last_tactile_source_sequence = 0
    node._last_tactile_source_stamp_s = None
    node._last_tactile_receipt_mono = None
    node._tactile_timing_trace = {}
    node.evaluation_diagnostics_enabled = False
    node.physical_evidence_transport = supervisor_module.TRANSPORT_ROS
    node.simulator_paper_evaluation_profile = False
    node.physical_evidence_freshness_timeout = 0.10
    node._physical_evidence_reader = None
    node._last_shared_record = None
    node._last_shared_sequence = 0
    node._last_shared_source_stamp_ns = 0
    node._last_shared_safety_read_mono = 0.0
    node._last_ros_state_evidence = None
    node._last_ros_tactile_evidence = None
    node._shared_ros_equivalence_error = ""
    node.frame_id = "base_link"
    node.safety_enabled = True
    node.tactile_enabled = tactile_enabled
    node.state_timeout = 0.10
    node.command_timeout = 0.10
    node.tactile_timeout = 0.10
    node.tactile_startup_grace = 0.10
    node.tactile_future_skew = 0.02
    node.soft_contact_velocity_scale = 0.30
    node.geometry = FakeGeometry(geometry_safe)
    node._test_now_ns = 1_000_000_000
    node._test_now_mono = 0.05
    node._now_ns = lambda: node._test_now_ns
    node._monotonic = lambda: node._test_now_mono
    return node


class FakePhysicalEvidenceReader:
    def __init__(self, value):
        self.value = value

    def read(self):
        if isinstance(self.value, BaseException):
            raise self.value
        return self.value


def shared_record(node, sequence=1, *, age_s=0.0):
    now_ns = node._test_now_ns
    now_mono_ns = int(node._test_now_mono * 1_000_000_000)
    return supervisor_module.PhysicalEvidenceRecord(
        session_id="ab" * 32,
        producer_pid=123,
        producer_uid=1000,
        generated_sequence=sequence,
        source_monotonic_ns=now_mono_ns - int(age_s * 1_000_000_000),
        source_stamp_ns=now_ns - int(age_s * 1_000_000_000),
        command_sequence=1,
        q=(0.0,) * 6,
        q_dot=(0.0,) * 6,
        tip_position=(0.0, 0.0, 0.08),
        whole_backbone_physical_clearance_m=0.02,
        whole_backbone_safety_clearance_m=0.01,
        raw_tactile=0.0,
        filtered_tactile=0.0,
        tactile_force_n=0.0,
        tactile_clearance_m=0.02,
        tactile_region=CtrTactileState.REGION_NO_CONTACT,
        source_valid=True,
        simulation=True,
        frame_valid=True,
        physical_collision=False,
        safety_margin_violation=False,
        tactile_valid=True,
        contact=False,
        warning=False,
        stop=False,
    )


def valid_state():
    message = CtrState()
    stamp(message, 1_000_000_000)
    message.valid = True
    message.backbone = []
    for _ in range(2):
        point = message.tip_pose.position
        message.backbone.append(type(point)())
    return message


def valid_command(values=None):
    message = CtrJointCommand()
    stamp(message, 1_000_000_000)
    message.q_dot = list(values or (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    message.valid = True
    return message


def tactile(region, stamp_ns=1_000_000_000, *, valid=True, warning=None, stop=None, clearance=0.01):
    message = CtrTactileState()
    stamp(message, stamp_ns)
    message.valid = valid
    message.region = region
    message.warning = region in (CtrTactileState.REGION_WARNING, CtrTactileState.REGION_STOP) if warning is None else warning
    message.stop = region == CtrTactileState.REGION_STOP if stop is None else stop
    message.clearance_m = clearance
    message.force_magnitude = 0.0
    message.raw_values = [0.0]
    message.filtered_values = [0.0]
    return message


class SafetySupervisorLogicTest(unittest.TestCase):
    def _ready(self, *, tactile_enabled=True):
        node = make_node(tactile_enabled=tactile_enabled)
        node._raw_command = valid_command()
        node._raw_command_received_mono = 0.05
        node._state = valid_state()
        node._state_received_mono = 0.05
        if tactile_enabled:
            node._on_tactile(tactile(CtrTactileState.REGION_NO_CONTACT))
        return node

    def _shared_ready(self, *, age_s=0.0):
        node = make_node(tactile_enabled=True)
        node._test_now_mono = 1.0
        node.physical_evidence_transport = (
            supervisor_module.TRANSPORT_AUTHENTICATED_SHARED_MEMORY
        )
        node._physical_evidence_reader = FakePhysicalEvidenceReader(
            shared_record(node, age_s=age_s)
        )
        node._raw_command = valid_command()
        node._raw_command_received_mono = node._test_now_mono
        return node

    def test_authenticated_physical_evidence_is_the_direct_freshness_authority(self):
        node = self._shared_ready(age_s=0.02)
        decision = node._decision()
        self.assertTrue(decision.allowed)
        self.assertFalse(decision.fault)
        self.assertEqual("ready", decision.state_name)
        self.assertEqual(1, node._tactile_timing_trace["evidence_transport_code"])

    def test_delayed_ros_delivery_does_not_replace_fresh_direct_evidence(self):
        node = self._shared_ready(age_s=0.02)
        node._state = valid_state()
        node._state_received_mono = node._test_now_mono - 0.5
        node._tactile = None
        node._tactile_received_mono = node._test_now_mono - 0.5
        decision = node._decision()
        self.assertTrue(decision.allowed)
        self.assertFalse(decision.fault)

    def test_genuinely_stale_direct_evidence_stops_without_ros_fallback(self):
        node = self._shared_ready(age_s=0.101)
        node._state = valid_state()
        node._state_received_mono = node._test_now_mono
        decision = node._decision()
        self.assertFalse(decision.allowed)
        self.assertTrue(decision.fault)
        self.assertIn(decision.reason, {"state_stale", "tactile_stale"})

    def test_simulator_paper_profile_accepts_below_and_rejects_at_020_seconds(self):
        below = self._shared_ready(age_s=0.199)
        below.simulator_paper_evaluation_profile = True
        below.physical_evidence_freshness_timeout = 0.20
        self.assertTrue(below._decision().allowed)

        boundary = self._shared_ready(age_s=0.20)
        boundary.simulator_paper_evaluation_profile = True
        boundary.physical_evidence_freshness_timeout = 0.20
        decision = boundary._decision()
        self.assertFalse(decision.allowed)
        self.assertEqual("state_stale", decision.reason)

    def test_shared_reader_samples_clocks_after_stable_record_copy(self):
        node = self._shared_ready(age_s=0.0)

        class CommitDuringRead:
            def read(self_nonlocal):
                node._test_now_ns += 50_000_000
                node._test_now_mono += 0.05
                return shared_record(node, sequence=2, age_s=0.0)

        node._physical_evidence_reader = CommitDuringRead()
        decision = node._decision()
        self.assertTrue(decision.allowed)
        self.assertNotEqual("physical_evidence_future_dated", decision.reason)

    def test_invalid_direct_channel_never_falls_back_to_ros(self):
        node = self._shared_ready()
        node._physical_evidence_reader = FakePhysicalEvidenceReader(
            supervisor_module.PhysicalEvidenceError(
                "physical_evidence_integrity_invalid"
            )
        )
        node._state = valid_state()
        node._state_received_mono = node._test_now_mono
        decision = node._decision()
        self.assertFalse(decision.allowed)
        self.assertEqual("physical_evidence_integrity_invalid", decision.reason)

    def test_repeated_sequence_does_not_refresh_source_timestamp(self):
        node = self._shared_ready(age_s=0.02)
        self.assertTrue(node._decision().allowed)
        node._test_now_ns += 90_000_000
        node._test_now_mono += 0.09
        decision = node._decision()
        self.assertFalse(decision.allowed)
        self.assertTrue(decision.fault)
        self.assertIn(decision.reason, {"state_stale", "tactile_stale"})

    def test_disabled_tactile_preserves_safe_command(self):
        decision = self._ready(tactile_enabled=False)._decision()
        self.assertEqual((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), decision.command)
        self.assertTrue(decision.allowed)

    def test_no_contact_passes_and_warning_applies_source_configured_scale(self):
        node = self._ready()
        self.assertEqual((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), node._decision().command)
        node._on_tactile(tactile(CtrTactileState.REGION_WARNING, 1_010_000_000))
        decision = node._decision()
        self.assertEqual(6, len(decision.command))
        for observed, expected in zip(decision.command, (0.3, 0.6, 0.9, 1.2, 1.5, 1.8)):
            self.assertAlmostEqual(expected, observed)
        self.assertTrue(decision.warning)

    def test_stop_overrides_and_latches_until_clear_fault(self):
        node = self._ready()
        node._on_tactile(tactile(CtrTactileState.REGION_STOP, 1_010_000_000))
        decision = node._decision()
        self.assertEqual(_zero_command(), decision.command)
        self.assertTrue(decision.emergency_stop)
        node._on_tactile(tactile(CtrTactileState.REGION_NO_CONTACT, 1_020_000_000))
        self.assertEqual(_zero_command(), node._decision().command)
        response = node._on_clear_fault(ClearFault.Request(), ClearFault.Response())
        self.assertTrue(response.accepted)
        self.assertEqual((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), node._decision().command)

    def test_invalid_and_stale_tactile_gate_without_new_command(self):
        node = self._ready()
        node._test_now_mono = 0.20
        self.assertEqual(_zero_command(), node._decision().command)
        node = self._ready()
        invalid = tactile(CtrTactileState.REGION_NO_CONTACT, 1_010_000_000, valid=False)
        node._on_tactile(invalid)
        self.assertEqual(_zero_command(), node._decision().command)

    def test_geometric_failure_dominates_no_contact(self):
        node = self._ready()
        node.geometry = FakeGeometry(False)
        decision = node._decision()
        self.assertEqual(_zero_command(), decision.command)
        self.assertIn("whole_backbone", decision.reason)

    def test_invalid_command_never_propagates_nonfinite_values(self):
        node = self._ready()
        node._raw_command.q_dot[2] = math.nan
        decision = node._decision()
        self.assertEqual(_zero_command(), decision.command)
        self.assertTrue(all(math.isfinite(value) for value in decision.command))

    def test_older_message_does_not_clear_stop_or_refresh(self):
        node = self._ready()
        node._on_tactile(tactile(CtrTactileState.REGION_STOP, 1_010_000_000))
        node._on_tactile(tactile(CtrTactileState.REGION_NO_CONTACT, 1_005_000_000))
        self.assertTrue(node._stop_latched)
        self.assertEqual(_zero_command(), node._decision().command)

    def _assert_manual_clear_lifecycle(self, node, break_health, restore_health):
        break_health()
        first = node._decision()
        self.assertEqual(_zero_command(), first.command)
        self.assertTrue(first.fault)
        self.assertTrue(node._fault_latched)
        unhealthy = node._on_clear_fault(ClearFault.Request(), ClearFault.Response())
        self.assertFalse(unhealthy.accepted)
        restore_health()
        retained = node._decision()
        self.assertEqual(_zero_command(), retained.command)
        self.assertTrue(retained.fault)
        healthy = node._on_clear_fault(ClearFault.Request(), ClearFault.Response())
        self.assertTrue(healthy.accepted)
        resumed = node._decision()
        self.assertTrue(resumed.allowed)
        self.assertNotEqual(_zero_command(), resumed.command)

    def test_transient_geometry_fault_requires_healthy_manual_clear(self):
        node = self._ready()
        self._assert_manual_clear_lifecycle(
            node,
            lambda: setattr(node.geometry, "safe", False),
            lambda: setattr(node.geometry, "safe", True),
        )

    def test_stale_state_requires_healthy_manual_clear(self):
        node = self._ready()
        self._assert_manual_clear_lifecycle(
            node,
            lambda: setattr(node, "_state_received_mono", -1.0),
            lambda: setattr(node, "_state_received_mono", node._test_now_mono),
        )

    def test_stale_command_requires_healthy_manual_clear(self):
        node = self._ready()
        self._assert_manual_clear_lifecycle(
            node,
            lambda: setattr(node, "_raw_command_received_mono", -1.0),
            lambda: setattr(node, "_raw_command_received_mono", node._test_now_mono),
        )

    def test_stale_tactile_requires_healthy_manual_clear(self):
        node = self._ready()
        tactile = node._tactile_snapshot[0]
        self._assert_manual_clear_lifecycle(
            node,
            lambda: setattr(node, "_tactile_snapshot", (tactile, -1.0, "eligible_no_contact")),
            lambda: setattr(
                node,
                "_tactile_snapshot",
                (tactile, node._test_now_mono, "eligible_no_contact"),
            ),
        )

    def test_invalid_command_requires_healthy_manual_clear(self):
        node = self._ready()
        original = list(node._raw_command.q_dot)
        self._assert_manual_clear_lifecycle(
            node,
            lambda: node._raw_command.q_dot.__setitem__(2, math.nan),
            lambda: setattr(node._raw_command, "q_dot", list(original)),
        )

    def test_tactile_stop_requires_healthy_manual_clear(self):
        node = self._ready()

        def stop():
            node._on_tactile(tactile(CtrTactileState.REGION_STOP, 1_010_000_000))

        def restore():
            node._on_tactile(tactile(CtrTactileState.REGION_NO_CONTACT, 1_020_000_000))

        self._assert_manual_clear_lifecycle(node, stop, restore)


if __name__ == "__main__":
    unittest.main()
