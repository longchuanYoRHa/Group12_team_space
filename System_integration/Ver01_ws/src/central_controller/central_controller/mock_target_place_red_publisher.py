#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs


class MockTargetPlaceRedPublisher(Node):
    """Publish mock /target_place/red points at 5 Hz for trigger testing."""

    def __init__(self):
        super().__init__("mock_target_place_red_publisher")

        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("start_z_m", 0.5)
        self.declare_parameter("stop_z_m", 0.265)
        self.declare_parameter("z_step_m", 0.01)
        self.declare_parameter("fixed_x_m", 0.0)
        self.declare_parameter("fixed_y_m", 0.0)

        self._publisher = self.create_publisher(geometry_msgs.Point, "/target_place/red", 10)
        self._current_z = float(self.get_parameter("start_z_m").value)
        self._reached_stop_z = False

        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._timer_cb)

        self.get_logger().info(
            "Mock PLACE publisher started: topic=/target_place/red, "
            f"z from {self._current_z:.3f}m to {float(self.get_parameter('stop_z_m').value):.3f}m "
            f"at {publish_rate_hz:.1f}Hz."
        )

    def _timer_cb(self):
        stop_z = float(self.get_parameter("stop_z_m").value)
        z_step = max(1e-4, abs(float(self.get_parameter("z_step_m").value)))
        x = float(self.get_parameter("fixed_x_m").value)
        y = float(self.get_parameter("fixed_y_m").value)

        if self._current_z > stop_z:
            self._current_z = max(stop_z, self._current_z - z_step)
        elif not self._reached_stop_z:
            self._reached_stop_z = True
            self.get_logger().info(
                f"Mock PLACE z reached stop value {stop_z:.3f}m. Holding this value."
            )

        msg = geometry_msgs.Point()
        msg.x = x
        msg.y = y
        msg.z = self._current_z
        self._publisher.publish(msg)

        self.get_logger().info(
            f"Publish /target_place/red: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockTargetPlaceRedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
