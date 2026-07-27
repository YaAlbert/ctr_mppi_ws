import inspect
import sys
import types
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src" / "ctr_bringup"))

if "ctr_interfaces" not in sys.modules:
    ctr_interfaces = types.ModuleType("ctr_interfaces")
    ctr_interfaces_msg = types.ModuleType("ctr_interfaces.msg")
    ctr_interfaces_srv = types.ModuleType("ctr_interfaces.srv")
    for name in ("CtrControllerMetrics", "CtrJointCommand", "CtrState"):
        setattr(ctr_interfaces_msg, name, type(name, (), {}))
    for name in ("StartExperiment", "StopExperiment"):
        setattr(ctr_interfaces_srv, name, type(name, (), {}))
    sys.modules["ctr_interfaces"] = ctr_interfaces
    sys.modules["ctr_interfaces.msg"] = ctr_interfaces_msg
    sys.modules["ctr_interfaces.srv"] = ctr_interfaces_srv

from ctr_evaluation.nodes import evaluation_node  # noqa: E402


class EvaluationNodeStaticTest(unittest.TestCase):
    def test_evaluation_node_is_observation_only(self):
        self.assertEqual((), evaluation_node.EVALUATION_COMMAND_PUBLISHERS)
        self.assertFalse(
            set(evaluation_node.ACTUATOR_COMMAND_TOPICS)
            & set(evaluation_node.EVALUATION_COMMAND_PUBLISHERS)
        )

    def test_evaluation_node_does_not_create_publishers(self):
        source = inspect.getsource(evaluation_node.EvaluationNode)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn('"/ctr/mppi_command"', source.split("create_subscription")[0])
        self.assertNotIn('"/ctr/safe_command"', source.split("create_subscription")[0])

    def test_parse_metadata_json(self):
        self.assertEqual({"case": "circle"}, evaluation_node.parse_metadata('{"case": "circle"}'))

    def test_parse_metadata_yaml(self):
        self.assertEqual({"case": "ellipse"}, evaluation_node.parse_metadata("case: ellipse"))

    def test_parse_metadata_rejects_non_mapping(self):
        with self.assertRaises(ValueError):
            evaluation_node.parse_metadata("[1, 2, 3]")

    def test_stamp_seconds(self):
        class Stamp:
            sec = 2
            nanosec = 500000000

        self.assertAlmostEqual(2.5, evaluation_node.stamp_seconds(Stamp()))

    def test_inactive_context_message_conversion_error_is_shutdown_only(self):
        rclpy = FakeRclpy(RuntimeError("Unable to convert call argument to Python object"), ok_after_spin=False)
        node = FakeNode()
        evaluation_node.run_evaluation_node_until_shutdown(rclpy, lambda: node)
        self.assertTrue(node.destroyed)
        self.assertEqual(0, rclpy.shutdown_count)

    def test_runtime_exception_is_re_raised(self):
        rclpy = FakeRclpy(RuntimeError("runtime failure"), ok_after_spin=True)
        with self.assertRaisesRegex(RuntimeError, "runtime failure"):
            evaluation_node.run_evaluation_node_until_shutdown(rclpy, FakeNode)
        self.assertEqual(1, rclpy.shutdown_count)

    def test_shutdown_auto_finalize_guard_is_recording_only(self):
        source = inspect.getsource(evaluation_node.EvaluationNode.destroy_node)
        self.assertIn("self.recorder.lifecycle_state == STATE_RECORDING", source)
        self.assertIn("auto_finalize_on_shutdown", source)


class FakeNode:
    def __init__(self):
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


class FakeRclpy:
    def __init__(self, spin_exception, *, ok_after_spin):
        self.spin_exception = spin_exception
        self.ok_after_spin = ok_after_spin
        self.shutdown_count = 0
        self._spun = False

    def init(self, args=None):
        self.args = args

    def spin(self, node):
        self._spun = True
        raise self.spin_exception

    def ok(self):
        return self.ok_after_spin if self._spun else True

    def shutdown(self):
        self.shutdown_count += 1


if __name__ == "__main__":
    unittest.main()
