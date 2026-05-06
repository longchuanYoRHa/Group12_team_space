#!/usr/bin/env python3
"""
Docking module test (V4-style):
- Startup: enter visual precision phase immediately
- Enter visual precision phase immediately (no DockRobot)
- Subscribe to /target_pick/{red,green,blue}; on each message update camera-frame point
- 10 Hz timer applies the same cmd_vel law as task_manager_v4_refactor/alignment.py
- When |x| and |z - target| within tolerance -> publish /arm/target_pick (camera frame, m)
  and wait for arm/gripper status like before
- After grasp success, enter place precision phase:
  subscribe to /target_place/{red,green,blue} and drive towards the camera-z
  trigger value (place_trigger_camera_z_m, default 36). When |z - trigger| is
  within tolerance, stop the chassis, briefly hold, then move forward
  forward_before_place_distance_m (default 0.10 m) and publish a FIXED
  /arm/target_place point (camera frame): (fixed_place_target_x/y/z).
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class TestState(Enum):
    INIT = "init"
    PRECISION_ALIGN_PICK = "precision_align_pick"
    GRASP = "grasp"
    BACKUP_BEFORE_PLACE = "backup_before_place"
    PRECISION_ALIGN_PLACE = "precision_align_place"
    FORWARD_BEFORE_PLACE = "forward_before_place"
    PLACE = "place"
    BACKUP_AFTER_PLACE = "backup_after_place"
    DONE = "done"


class ModuleTestDockingNode(Node):
    def __init__(self):
        super().__init__("module_test_docking")

        self.current_state = TestState.INIT
        self._visual_pick_last_point: Optional[geometry_msgs.Point] = None
        self._visual_place_last_point: Optional[geometry_msgs.Point] = None
        self._visual_align_deadline: Optional[float] = None
        self._backup_deadline: Optional[float] = None
        self._forward_deadline: Optional[float] = None
        self._forward_stop_hold_deadline: Optional[float] = None
        self._forward_start_time: Optional[float] = None
        self._forward_speed_mps: float = 0.0
        self._forward_distance_m: float = 0.0
        self._forward_last_log_time: Optional[float] = None

        self.arm_status = "idle"
        self.gripper_status = "unknown"
        self._arm_cmd_sent = False
        self._grasp_target_point: Optional[geometry_msgs.Point] = None
        self._place_target_point: Optional[geometry_msgs.Point] = None
        self._visual_pick_last_color: Optional[str] = None
        self._locked_pick_color: Optional[str] = None

        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("visual_align_timeout_sec", 120.0)

        # Same defaults as task_manager_node_v4 / alignment visual docking
        self.declare_parameter("docking_linear_speed_mps", 0.07)
        self.declare_parameter("docking_angular_speed_max_rps", 0.25)
        self.declare_parameter("visual_docking_x_kp", 1.5)
        self.declare_parameter("visual_docking_z_kp", 1.0)
        self.declare_parameter("visual_docking_x_tolerance_m", 0.05)
        self.declare_parameter("grasp_target_camera_z_m", 0.265)
        self.declare_parameter("grasp_target_camera_z_tolerance_m", 0.01)
        self.declare_parameter("backup_before_place_distance_m", 0.50)
        self.declare_parameter("backup_before_place_speed_mps", 0.10)
        self.declare_parameter("place_trigger_camera_z_m", 0.36)
        self.declare_parameter("place_trigger_camera_z_tolerance", 0.005)
        self.declare_parameter("forward_before_place_distance_m", 0.10)
        self.declare_parameter("forward_before_place_speed_mps", 0.10)
        self.declare_parameter("forward_before_place_stop_hold_sec", 0.2)
        self.declare_parameter("fixed_place_target_x", 0.0)
        self.declare_parameter("fixed_place_target_y", 0.0)
        self.declare_parameter("fixed_place_target_z", 0.275)
        self.declare_parameter("backup_after_place_distance_m", 0.30)
        self.declare_parameter("backup_after_place_speed_mps", 0.10)

        self.state_pub = self.create_publisher(std_msgs.String, "module_test_docking/state", 10)
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.Twist, "/cmd_vel", 10)
        self._arm_pick_pub = self.create_publisher(
            geometry_msgs.Point, "/arm/target_pick", 10
        )
        self._arm_place_pub = self.create_publisher(
            geometry_msgs.Point, "/arm/target_place", 10
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self._vision_subs = []
        for color, topic in [
            ("red", "/target_pick/red"),
            ("green", "/target_pick/green"),
            ("blue", "/target_pick/blue"),
        ]:
            sub = self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color, t=topic: self._pick_point_callback(msg, c, t),
                qos_profile_sensor_data,
            )
            self._vision_subs.append(sub)

        for color, topic in [
            ("red", "/target_place/red"),
            ("green", "/target_place/green"),
            ("blue", "/target_place/blue"),
        ]:
            sub = self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color, t=topic: self._place_point_callback(msg, c, t),
                qos_profile_sensor_data,
            )
            self._vision_subs.append(sub)

        self.arm_status_sub = self.create_subscription(
            std_msgs.String,
            "/arm/status",
            self._arm_status_callback,
            10,
        )
        self.arm_gripper_status_sub = self.create_subscription(
            std_msgs.String,
            "/arm/gripper_status",
            self._arm_gripper_status_callback,
            10,
        )

        self._last_vision_msg_time = None
        self._last_tf_ok_time = None

        self._control_timer = self.create_timer(0.1, self._visual_control_timer_cb)
        self._watchdog_timer = self.create_timer(0.5, self._watchdog_timer_cb)

        self._enter_visual_align_pick_phase()

        self.get_logger().info("module_test_docking initialized (V4 pure visual align + grasp).")
        self._publish_state()

    def _publish_state(self):
        state_msg = std_msgs.String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)

    def _set_state(self, new_state: TestState):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        self._publish_state()

    def _stop_cmd_vel(self):
        self.cmd_vel_pub.publish(geometry_msgs.Twist())

    def _enter_visual_align_pick_phase(self):
        self._stop_cmd_vel()
        self._visual_pick_last_point = None
        self._visual_pick_last_color = None
        self._grasp_target_point = None
        self._locked_pick_color = None
        now = time.monotonic()
        timeout = float(self.get_parameter("visual_align_timeout_sec").value)
        self._visual_align_deadline = now + max(5.0, timeout)
        self._set_state(TestState.PRECISION_ALIGN_PICK)
        self.get_logger().info(
            "PRECISION_ALIGN: V4-style visual servo (camera x/z -> cmd_vel); "
            f"phase timeout {timeout:.0f}s. Waiting for /target_pick/*."
        )

    def _pick_point_callback(self, msg: geometry_msgs.Point, color: str, topic: str):
        self._last_vision_msg_time = self.get_clock().now()
        if self.current_state != TestState.PRECISION_ALIGN_PICK:
            return

        self._visual_pick_last_point = msg
        self._visual_pick_last_color = color
        self.get_logger().debug(f"Vision pick {color} from {topic}: x={msg.x:.3f} z={msg.z:.3f}")

    def _enter_visual_align_place_phase(self):
        self._stop_cmd_vel()
        self._visual_place_last_point = None
        self._place_target_point = None
        self._backup_deadline = None
        self._forward_deadline = None
        now = time.monotonic()
        timeout = float(self.get_parameter("visual_align_timeout_sec").value)
        self._visual_align_deadline = now + max(5.0, timeout)
        self._set_state(TestState.PRECISION_ALIGN_PLACE)
        self.get_logger().info(
            "PRECISION_ALIGN_PLACE: V4-style visual servo (camera x/z -> cmd_vel); "
            f"phase timeout {timeout:.0f}s. Waiting for /target_place/*."
        )

    def _enter_backup_before_place_phase(self):
        self._stop_cmd_vel()
        self._visual_align_deadline = None
        self._visual_place_last_point = None
        self._place_target_point = None
        self._forward_deadline = None

        dist_m = abs(float(self.get_parameter("backup_before_place_distance_m").value))
        speed_mps = abs(float(self.get_parameter("backup_before_place_speed_mps").value))
        speed_mps = max(0.01, speed_mps)

        duration_s = dist_m / speed_mps
        self._backup_deadline = time.monotonic() + max(0.1, duration_s)
        self._set_state(TestState.BACKUP_BEFORE_PLACE)
        self.get_logger().info(
            f"BACKUP_BEFORE_PLACE: cmd_vel back {dist_m:.2f}m at {speed_mps:.2f}m/s "
            f"(~{duration_s:.1f}s), then enter PRECISION_ALIGN_PLACE."
        )

    def _enter_forward_before_place_phase(self):
        self._stop_cmd_vel()
        self._visual_align_deadline = None
        self._forward_deadline = None
        self._forward_stop_hold_deadline = None
        self._forward_start_time = None
        self._forward_last_log_time = None

        stop_hold_s = max(
            0.0, float(self.get_parameter("forward_before_place_stop_hold_sec").value)
        )
        self._forward_stop_hold_deadline = time.monotonic() + stop_hold_s
        self._set_state(TestState.FORWARD_BEFORE_PLACE)
        self.get_logger().info(
            "FORWARD_BEFORE_PLACE: z trigger reached, stop first then move forward 0.10m "
            f"(stop hold {stop_hold_s:.2f}s)."
        )

    def _enter_backup_after_place_phase(self):
        self._stop_cmd_vel()
        self._visual_align_deadline = None
        self._visual_place_last_point = None
        self._place_target_point = None
        self._forward_deadline = None

        dist_m = abs(float(self.get_parameter("backup_after_place_distance_m").value))
        speed_mps = abs(float(self.get_parameter("backup_after_place_speed_mps").value))
        speed_mps = max(0.01, speed_mps)

        duration_s = dist_m / speed_mps
        self._backup_deadline = time.monotonic() + max(0.1, duration_s)
        self._set_state(TestState.BACKUP_AFTER_PLACE)
        self.get_logger().info(
            f"BACKUP_AFTER_PLACE: cmd_vel back {dist_m:.2f}m at {speed_mps:.2f}m/s "
            f"(~{duration_s:.1f}s), then re-enter PRECISION_ALIGN_PICK."
        )

    def _place_point_callback(self, msg: geometry_msgs.Point, color: str, topic: str):
        self._last_vision_msg_time = self.get_clock().now()
        if self.current_state != TestState.PRECISION_ALIGN_PLACE:
            return
        if self._locked_pick_color is not None and color != self._locked_pick_color:
            self.get_logger().debug(
                f"Vision place {color} from {topic} ignored, locked color={self._locked_pick_color}."
            )
            return

        self._visual_place_last_point = msg
        self.get_logger().debug(
            f"Vision place {color} from {topic}: x={msg.x:.3f} z={msg.z:.3f}"
        )

    def _visual_control_timer_cb(self):
        if self.current_state == TestState.FORWARD_BEFORE_PLACE:
            now = time.monotonic()
            if self._forward_stop_hold_deadline is not None and now < self._forward_stop_hold_deadline:
                self._stop_cmd_vel()
                return

            if self._forward_deadline is None:
                dist_m = abs(float(self.get_parameter("forward_before_place_distance_m").value))
                speed_mps = abs(float(self.get_parameter("forward_before_place_speed_mps").value))
                speed_mps = max(0.01, speed_mps)
                duration_s = dist_m / speed_mps
                self._forward_deadline = now + max(0.1, duration_s)
                self._forward_start_time = now
                self._forward_speed_mps = speed_mps
                self._forward_distance_m = dist_m
                self._forward_last_log_time = None
                self.get_logger().info(
                    f"FORWARD_BEFORE_PLACE: start cmd_vel forward {dist_m:.2f}m at {speed_mps:.2f}m/s "
                    f"(~{duration_s:.1f}s)."
                )

            if self._forward_deadline is not None and now >= self._forward_deadline:
                self._stop_cmd_vel()
                if self._forward_start_time is not None:
                    elapsed = now - self._forward_start_time
                    est_dist = min(
                        self._forward_distance_m, elapsed * self._forward_speed_mps
                    )
                    self.get_logger().info(
                        f"FORWARD_BEFORE_PLACE: reached target, "
                        f"t={elapsed:.2f}s, v={self._forward_speed_mps:.3f}m/s (cmd=0.000m/s now), "
                        f"est_distance={est_dist:.3f}m / {self._forward_distance_m:.3f}m -> STOP."
                    )
                self._forward_start_time = None
                self._forward_last_log_time = None
                self._place_target_point = geometry_msgs.Point(
                    x=float(self.get_parameter("fixed_place_target_x").value),
                    y=float(self.get_parameter("fixed_place_target_y").value),
                    z=float(self.get_parameter("fixed_place_target_z").value),
                )
                self.get_logger().info(
                    "FORWARD_BEFORE_PLACE: done -> send fixed /arm/target_place and wait arm done."
                )
                self._publish_place_target_and_finish()
                return

            speed_mps = self._forward_speed_mps
            twist = geometry_msgs.Twist()
            twist.linear.x = speed_mps
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            if self._forward_start_time is not None:
                elapsed = now - self._forward_start_time
                est_dist = min(self._forward_distance_m, elapsed * speed_mps)
                if (
                    self._forward_last_log_time is None
                    or (now - self._forward_last_log_time) >= 0.2
                ):
                    self._forward_last_log_time = now
                    self.get_logger().info(
                        f"FORWARD_BEFORE_PLACE: progress "
                        f"t={elapsed:.2f}s, v={speed_mps:.3f}m/s, "
                        f"est_distance={est_dist:.3f}m / {self._forward_distance_m:.3f}m."
                    )
            return

        if self.current_state == TestState.BACKUP_BEFORE_PLACE:
            now = time.monotonic()
            if self._backup_deadline is not None and now >= self._backup_deadline:
                self._stop_cmd_vel()
                self.get_logger().info("BACKUP_BEFORE_PLACE: done -> enter PRECISION_ALIGN_PLACE.")
                self._enter_visual_align_place_phase()
                return

            speed_mps = abs(float(self.get_parameter("backup_before_place_speed_mps").value))
            speed_mps = max(0.01, speed_mps)
            twist = geometry_msgs.Twist()
            twist.linear.x = -speed_mps
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        if self.current_state == TestState.BACKUP_AFTER_PLACE:
            now = time.monotonic()
            if self._backup_deadline is not None and now >= self._backup_deadline:
                self._stop_cmd_vel()
                self.get_logger().info(
                    "BACKUP_AFTER_PLACE: done -> return to PRECISION_ALIGN_PICK."
                )
                self._enter_visual_align_pick_phase()
                return

            speed_mps = abs(float(self.get_parameter("backup_after_place_speed_mps").value))
            speed_mps = max(0.01, speed_mps)
            twist = geometry_msgs.Twist()
            twist.linear.x = -speed_mps
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        if self.current_state not in (
            TestState.PRECISION_ALIGN_PICK,
            TestState.PRECISION_ALIGN_PLACE,
        ):
            return

        now = time.monotonic()
        if self._visual_align_deadline is not None and now >= self._visual_align_deadline:
            self.get_logger().warn(
                "PRECISION_ALIGN: timeout; stopping cmd_vel and ending test."
            )
            self._stop_cmd_vel()
            self._set_state(TestState.DONE)
            return

        point = (
            self._visual_pick_last_point
            if self.current_state == TestState.PRECISION_ALIGN_PICK
            else self._visual_place_last_point
        )
        if point is None:
            return

        if self.current_state == TestState.PRECISION_ALIGN_PICK:
            target_z = float(self.get_parameter("grasp_target_camera_z_m").value)
            z_tol = abs(float(self.get_parameter("grasp_target_camera_z_tolerance_m").value))
        else:
            # Place state ONLY uses the z trigger (default 36) to switch to
            # FORWARD_BEFORE_PLACE; no other branch may publish /arm/target_place.
            target_z = float(self.get_parameter("place_trigger_camera_z_m").value)
            z_tol = abs(float(self.get_parameter("place_trigger_camera_z_tolerance").value))
        x_tol = abs(float(self.get_parameter("visual_docking_x_tolerance_m").value))

        max_w = abs(float(self.get_parameter("docking_angular_speed_max_rps").value))
        max_v = abs(float(self.get_parameter("docking_linear_speed_mps").value))

        kp_x = float(self.get_parameter("visual_docking_x_kp").value)
        kp_z = float(self.get_parameter("visual_docking_z_kp").value)

        # Same as task_manager_v4_refactor/alignment.py _visual_docking_control_step
        x_error = -float(point.x)
        z_error = float(point.z) - target_z

        if self.current_state == TestState.PRECISION_ALIGN_PLACE:
            if abs(z_error) <= z_tol:
                self.get_logger().info(
                    f"PRECISION_ALIGN_PLACE: z trigger reached "
                    f"(point.z={float(point.z):.3f}, target={target_z:.3f}) -> FORWARD_BEFORE_PLACE."
                )
                self._enter_forward_before_place_phase()
                return

            twist = geometry_msgs.Twist()
            twist.angular.z = _clamp(kp_x * x_error, -max_w, max_w)
            twist.linear.x = _clamp(kp_z * z_error, -max_v, max_v)
            self.cmd_vel_pub.publish(twist)
            return

        aligned = (abs(x_error) <= x_tol) and (abs(z_error) <= z_tol)
        if aligned:
            self._stop_cmd_vel()
            self._grasp_target_point = geometry_msgs.Point(
                x=point.x, y=point.y, z=point.z
            )
            self._locked_pick_color = self._visual_pick_last_color
            self.get_logger().info(
                "PRECISION_ALIGN_PICK: aligned within tolerance -> GRASP "
                f"(|x_err|={abs(x_error):.4f}, |z_err|={abs(z_error):.4f}, "
                f"pick_color={self._locked_pick_color})."
            )
            self._enter_grasp_state()
            return

        twist = geometry_msgs.Twist()
        twist.angular.z = _clamp(kp_x * x_error, -max_w, max_w)
        twist.linear.x = _clamp(kp_z * z_error, -max_v, max_v)
        self.cmd_vel_pub.publish(twist)

    def _watchdog_timer_cb(self):
        now = self.get_clock().now()

        if self._last_vision_msg_time is not None:
            if (now - self._last_vision_msg_time).nanoseconds > int(3.0 * 1e9):
                self.get_logger().warn(
                    "No /target_pick vision messages for >3s "
                    f"(current_state={self.current_state.value})."
                )
                self._last_vision_msg_time = now

        cam_frame = str(self.get_parameter("camera_frame_id").value)
        if self.tf_buffer.can_transform(
            "odom", cam_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
        ):
            self._last_tf_ok_time = now
        else:
            if self._last_tf_ok_time is None or (now - self._last_tf_ok_time).nanoseconds > int(
                3.0 * 1e9
            ):
                self.get_logger().warn(f"TF diagnostic: odom<-{cam_frame} not ready.")

    def _arm_status_callback(self, msg: std_msgs.String):
        self.arm_status = msg.data.lower()
        self._try_finish_arm_action_from_topics()

    def _arm_gripper_status_callback(self, msg: std_msgs.String):
        self.gripper_status = msg.data.lower()
        self._try_finish_arm_action_from_topics()

    def _enter_grasp_state(self):
        self._set_state(TestState.GRASP)
        self._arm_cmd_sent = False

        if self._grasp_target_point is None:
            self.get_logger().warn("GRASP: no aligned vision point; skipping arm command.")
            self._set_state(TestState.DONE)
            return

        target_pt = self._grasp_target_point
        self.get_logger().info(
            "GRASP: publishing /arm/target_pick in camera frame (meters) "
            f"({target_pt.x:.3f}, {target_pt.y:.3f}, {target_pt.z:.3f})"
        )
        self._arm_pick_pub.publish(target_pt)
        self._arm_cmd_sent = True

    def _publish_place_target_and_finish(self):
        if self._place_target_point is None:
            self.get_logger().warn("PLACE: no aligned vision point; ending test.")
            self._set_state(TestState.DONE)
            return

        self._set_state(TestState.PLACE)
        self._arm_cmd_sent = False

        target_pt = self._place_target_point
        self.get_logger().info(
            "PLACE: publishing /arm/target_place in camera frame (meters) "
            f"({target_pt.x:.3f}, {target_pt.y:.3f}, {target_pt.z:.3f})"
        )
        self._arm_place_pub.publish(target_pt)
        self._arm_cmd_sent = True

    def _try_finish_arm_action_from_topics(self):
        if self.current_state not in (TestState.GRASP, TestState.PLACE):
            return
        if not self._arm_cmd_sent:
            return

        if self.current_state == TestState.GRASP:
            if self.arm_status == "holding" and self.gripper_status == "object_held":
                self.get_logger().info("GRASP succeeded (arm holding + object_held).")
                self._arm_cmd_sent = False
                self._enter_backup_before_place_phase()
                return

            if self.arm_status == "error":
                self.get_logger().warn("GRASP failed (arm_status=error).")
                self._arm_cmd_sent = False
                self._set_state(TestState.DONE)
                return
            return

        # PLACE state
        if self.arm_status == "idle":
            self.get_logger().info("PLACE succeeded (arm_status=idle).")
            self._arm_cmd_sent = False
            self._enter_backup_after_place_phase()
            return

        if self.arm_status == "error":
            self.get_logger().warn("PLACE failed (arm_status=error).")
            self._arm_cmd_sent = False
            self._set_state(TestState.DONE)
            return


def main(args=None):
    rclpy.init(args=args)
    startup_logger = rclpy.logging.get_logger("module_test_docking")
    startup_logger.info("Startup delay enabled: countdown 30s before node starts.")
    for remaining in range(10, 0, -1):
        startup_logger.info(f"Node starts in {remaining:02d}s...")
        time.sleep(1.0)
    node = ModuleTestDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_cmd_vel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
