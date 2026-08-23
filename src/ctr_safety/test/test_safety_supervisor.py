import math
import sys
import threading
import types
import unittest
from types import SimpleNamespace


try:
    from ctr_interfaces.msg import CtrJointCommand, CtrSafetyStatus, CtrState, CtrTactileState
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

    class CtrState:
        def __init__(self):
            self.header = _header()
            self.valid = False
            self.backbone = []
            self.tip_pose = SimpleNamespace(position=Point())

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
        "CtrSafetyStatus": CtrSafetyStatus,
        "CtrState": CtrState,
        "CtrTactileState": CtrTactileState,
    }.items():
        setattr(message_module, name, value)
    service_module.ClearFault = ClearFault
    sys.modules["ctr_interfaces"] = interface_package
    sys.modules["ctr_interfaces.msg"] = message_module
    sys.modules["ctr_interfaces.srv"] = service_module

from ctr_safety.nodes.safety_supervisor_node import SafetySupervisorNode, _zero_command


class FakeGeometry:
    def __init__(self, safe=True):
        self.safe = safe

    def check_backbone(self, points):
        return self.safe, "geometry_safe" if self.safe else "whole_backbone_safety_margin", 0.001


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
    node._stop_latched = False
    node._fault_latched = False
    node._latched_fault_reason = ""
    node._last_reason = "startup_unavailable"
    node._last_safe_command = _zero_command()
    node._raw_command = None
    node._raw_command_received_mono = None
    node._state = None
    node._state_received_mono = None
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
        self._assert_manual_clear_lifecycle(
            node,
            lambda: setattr(node, "_tactile_received_mono", -1.0),
            lambda: setattr(node, "_tactile_received_mono", node._test_now_mono),
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
