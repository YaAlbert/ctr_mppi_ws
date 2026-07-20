import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ctr_bringup.placeholder_node import run_node_until_shutdown  # noqa: E402


class FakeRclpy:
    def __init__(self, *, spin_exception=None, shutdown_exception=None, ok=True):
        self.spin_exception = spin_exception
        self.shutdown_exception = shutdown_exception
        self._ok = ok
        self.init_calls = []
        self.spin_calls = 0
        self.shutdown_calls = 0

    def init(self, *, args=None):
        self.init_calls.append(args)

    def spin(self, node):
        self.spin_calls += 1
        if self.spin_exception is not None:
            raise self.spin_exception

    def ok(self):
        return self._ok

    def shutdown(self):
        self.shutdown_calls += 1
        self._ok = False
        if self.shutdown_exception is not None:
            raise self.shutdown_exception


class FakeNode:
    def __init__(self, *, destroy_exception=None):
        self.destroy_exception = destroy_exception
        self.destroy_calls = 0

    def destroy_node(self):
        self.destroy_calls += 1
        if self.destroy_exception is not None:
            raise self.destroy_exception


class ShutdownHandlingTest(unittest.TestCase):
    def test_normal_spin_destroys_node_and_shuts_down_once(self):
        rclpy = FakeRclpy()
        node = FakeNode()

        run_node_until_shutdown(rclpy, lambda: node, args=["--ros-args"])

        self.assertEqual([["--ros-args"]], rclpy.init_calls)
        self.assertEqual(1, rclpy.spin_calls)
        self.assertEqual(1, node.destroy_calls)
        self.assertEqual(1, rclpy.shutdown_calls)

    def test_destroy_node_called_once_after_spin_completion(self):
        rclpy = FakeRclpy()
        node = FakeNode()

        run_node_until_shutdown(rclpy, lambda: node)

        self.assertEqual(1, node.destroy_calls)

    def test_keyboard_interrupt_is_normal_shutdown(self):
        rclpy = FakeRclpy(spin_exception=KeyboardInterrupt())
        node = FakeNode()

        run_node_until_shutdown(rclpy, lambda: node)

        self.assertEqual(1, rclpy.spin_calls)
        self.assertEqual(1, node.destroy_calls)
        self.assertEqual(1, rclpy.shutdown_calls)

    def test_runtime_exception_stays_visible(self):
        rclpy = FakeRclpy(spin_exception=RuntimeError("runtime failure"))
        node = FakeNode()

        with self.assertRaisesRegex(RuntimeError, "runtime failure"):
            run_node_until_shutdown(rclpy, lambda: node)

        self.assertEqual(1, node.destroy_calls)
        self.assertEqual(1, rclpy.shutdown_calls)

    def test_startup_exception_stays_visible_and_context_shuts_down(self):
        rclpy = FakeRclpy()

        with self.assertRaisesRegex(RuntimeError, "startup failure"):
            run_node_until_shutdown(rclpy, lambda: (_ for _ in ()).throw(RuntimeError("startup failure")))

        self.assertEqual(0, rclpy.spin_calls)
        self.assertEqual(1, rclpy.shutdown_calls)

    def test_inactive_context_does_not_shutdown_again(self):
        rclpy = FakeRclpy(ok=False)
        node = FakeNode()

        run_node_until_shutdown(rclpy, lambda: node)

        self.assertEqual(1, node.destroy_calls)
        self.assertEqual(0, rclpy.shutdown_calls)

    def test_shutdown_called_at_most_once_when_shutdown_raises_keyboard_interrupt(self):
        rclpy = FakeRclpy(
            spin_exception=KeyboardInterrupt(),
            shutdown_exception=KeyboardInterrupt(),
        )
        node = FakeNode()

        run_node_until_shutdown(rclpy, lambda: node)

        self.assertEqual(1, node.destroy_calls)
        self.assertEqual(1, rclpy.shutdown_calls)


if __name__ == "__main__":
    unittest.main()
