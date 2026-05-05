#!/usr/bin/env python3

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs


class MockTargetPlaceRedPublisher(Node):
    """Publish mock /target_place/red points at 5 Hz for trigger testing."""

    def __init__(self):
        super().__init__("mock_target_place_red_publisher")

        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("start_z_m", 0.6)
        self.declare_parameter("stop_z_m", 0.33)
        self.declare_parameter("approach_speed_mps", 0.07)
        self.declare_parameter("fixed_x_m", 0.0)
        self.declare_parameter("fixed_y_m", 0.0)

        self._publisher = self.create_publisher(geometry_msgs.Point, "/target_place/red", 10)
        self._current_z = float(self.get_parameter("start_z_m").value)
        self._stopped = False
        self._last_tick_time = time.monotonic()

        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._timer_cb)

        self.get_logger().info(
            "Mock PLACE publisher started: topic=/target_place/red, "
            f"z from {self._current_z:.3f}m to {float(self.get_parameter('stop_z_m').value):.3f}m "
            f"at {publish_rate_hz:.1f}Hz."
        )

    def _timer_cb(self):
        if self._stopped:
            return

        now = time.monotonic()
        dt = max(0.0, now - self._last_tick_time)
        self._last_tick_time = now

        stop_z = float(self.get_parameter("stop_z_m").value)
        speed = abs(float(self.get_parameter("approach_speed_mps").value))
        speed = max(0.0, speed)
        x = float(self.get_parameter("fixed_x_m").value)
        y = float(self.get_parameter("fixed_y_m").value)

        if self._current_z > stop_z:
            self._current_z = max(stop_z, self._current_z - speed * dt)
        else:
            self._stopped = True
            try:
                self._timer.cancel()
            except Exception:
                pass
            self.get_logger().info(
                f"Mock PLACE: reached z={stop_z:.3f}m -> stop publishing /target_place/red."
            )
            return

        msg = geometry_msgs.Point()
        msg.x = x
        msg.y = y
        msg.z = self._current_z
        self._publisher.publish(msg)

        self.get_logger().info(
            f"Publish /target_place/red: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f} "
            f"(v={speed:.3f}m/s, dt={dt:.3f}s)"
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
