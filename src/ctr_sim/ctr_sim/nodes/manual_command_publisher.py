"""Manual/scripted safe-command publisher for simulation testing."""

from __future__ import annotations

import numpy as np
import rclpy
from ctr_interfaces.msg import CtrJointCommand
from rclpy.node import Node


class ManualCommandPublisher(Node):
    """Publish a fixed six-dimensional q_dot command to `/ctr/safe_command`."""

    def __init__(self):
        super().__init__("manual_command_publisher")
        self.declare_parameter("q_dot", [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("duration", 5.0)
        self.declare_parameter("repeat", False)

        self.command = np.asarray(self.get_parameter("q_dot").value, dtype=float)
        if self.command.shape != (6,) or not np.all(np.isfinite(self.command)):
            raise ValueError("q_dot parameter must contain 6 finite values")

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0:
            raise ValueError("publish_rate must be positive")

        self.duration = float(self.get_parameter("duration").value)
        if self.duration <= 0:
            raise ValueError("duration must be positive")

        self.repeat = bool(self.get_parameter("repeat").value)
        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(CtrJointCommand, "/ctr/safe_command", 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_command)
        self.get_logger().info(
            "Publishing scripted safe command to /ctr/safe_command. TODO-SAFE-001/TODO-SAFE-002 limits still require hardware tests."
        )

    def _publish_command(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.duration:
            if self.repeat:
                self.start_time = self.get_clock().now()
            else:
                self.command = np.zeros(6, dtype=float)

        msg = CtrJointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.q_dot = [float(value) for value in self.command]
        msg.valid = True
        msg.diagnostic_status = "Manual scripted safe-command publisher"
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualCommandPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
