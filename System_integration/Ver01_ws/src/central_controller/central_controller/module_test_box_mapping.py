#!/usr/bin/env python3
"""
Module test: 与 Task Manager V4 初始阶段一致——预探索 spin 时在 map 下缓存「盒子」位姿
（来自 /target_place/*，camera→map），随后：

1. 发送 Nav2 目标到当前朝向身后 ``post_spin_backup_distance_m``（默认 0.6 m）处；
2. 到达后按顺序对每个已记录位姿：先 Nav2 到预靠近点（与 V4 bin 预放置类似的 standoff + yaw_offset=π），
3. 再使用与 V4 相同的纯视觉伺服（camera frame 下 x、z → cmd_vel，容差与 V4 参数一致）。

不发送 DockRobot；仅用于在仿真/实机上验证地图记录 + 回程 + 多目标视觉对位。
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
from std_srvs.srv import Trigger

from central_controller.task_manager_utils import compute_pregrasp_pose, quat_yaw, quaternion_from_yaw


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class TestState(Enum):
    INIT = "init"
    PRE_EXPLORE_SPIN = "pre_explore_spin"
    RETURN_REAR = "return_rear"
    BOX_APPROACH = "box_approach"
    VISUAL_ALIGN = "visual_align"
    DONE = "done"


class NavPurpose(Enum):
    NONE = "none"
    PRE_EXPLORE_NAV = "pre_explore_nav"
    RETURN_REAR_NAV = "return_rear_nav"
    BOX_APPROACH_NAV = "box_approach_nav"


class ModuleTestBoxMappingNode(Node):
    def __init__(self):
        super().__init__("module_test_box_mapping")

        self.current_state = TestState.INIT
        self.current_nav_purpose = NavPurpose.NONE
        self.home_pose: Optional[geometry_msgs.PoseStamped] = None
        self.cached_bin_poses: dict[str, geometry_msgs.PoseStamped] = {}
        self.nav2_goal_handle = None

        self._visit_queue: List[Tuple[str, geometry_msgs.PoseStamped]] = []
        self._visit_index = 0
        self._current_visit_color: str = ""

        self._visual_docking_last_point: Optional[geometry_msgs.Point] = None
        self._visual_align_deadline: Optional[float] = None

        self._reset_odom_done = False
        self._reset_odom_future = None
        self._reset_odom_started_at = None
        self._skip_spin_post_init_started = False

        self.declare_parameter("camera_frame_id", "D435i_camera_link")
        self.declare_parameter("pre_explore_spin_enable", True)
        self.declare_parameter("pre_explore_nav_offset_x_m", 0.3)
        self.declare_parameter("pre_explore_nav_offset_y_m", 0.0)
        self.declare_parameter("post_spin_backup_distance_m", 0.6)
        self.declare_parameter("box_approach_standoff_m", 0.6)
        self.declare_parameter("target_bin_color", "")
        self.declare_parameter("visual_align_timeout_sec", 45.0)

        # Match task_manager_node_v4 / alignment visual docking
        self.declare_parameter("docking_linear_speed_mps", 0.01)
        self.declare_parameter("docking_angular_speed_max_rps", 0.25)
        self.declare_parameter("visual_docking_x_kp", 1.5)
        self.declare_parameter("visual_docking_z_kp", 1.0)
        self.declare_parameter("visual_docking_x_tolerance_m", 0.05)
        self.declare_parameter("grasp_target_camera_z_m", 0.265)
        self.declare_parameter("grasp_target_camera_z_tolerance_m", 0.01)

        self.state_pub = self.create_publisher(
            std_msgs.String, "module_test_box_mapping/state", 10
        )
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.Twist, "/cmd_vel", 10)

        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, "navigate_to_pose")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self._reset_odom_client = self.create_client(Trigger, "/reset_odometry")

        self._vision_subs = []
        for color, topic in [
            ("red", "/target_place/red"),
            ("green", "/target_place/green"),
            ("blue", "/target_place/blue"),
        ]:
            sub = self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color: self._bin_point_callback(msg, c),
                qos_profile_sensor_data,
            )
            self._vision_subs.append(sub)

        self._state_timer = self.create_timer(0.1, self._state_timer_callback)

        self.get_logger().info("module_test_box_mapping initialized (V4-style spin + rear nav + visual align).")
        self._publish_state()

    def _publish_state(self):
        msg = std_msgs.String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)

    def _set_state(self, new_state: TestState):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        self._publish_state()

    def _stop_cmd_vel(self):
        self.cmd_vel_pub.publish(geometry_msgs.Twist())

    def _state_timer_callback(self):
        self._publish_state()

        if self.current_state == TestState.INIT:
            self._handle_init_state()
        elif self.current_state == TestState.VISUAL_ALIGN:
            self._visual_align_timer_step()

    def _handle_init_state(self):
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Nav2 server...")
            return

        if not self._reset_odom_done:
            self._start_reset_odometry_if_needed()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f"Waiting for map<-base_link TF: {e}")
            return

        self.home_pose = geometry_msgs.PoseStamped()
        self.home_pose.header.frame_id = "map"
        self.home_pose.header.stamp = self.get_clock().now().to_msg()
        self.home_pose.pose.position.x = transform.transform.translation.x
        self.home_pose.pose.position.y = transform.transform.translation.y
        self.home_pose.pose.position.z = transform.transform.translation.z
        self.home_pose.pose.orientation = transform.transform.rotation
        self.get_logger().info("System ready, home pose saved.")

        if not bool(self.get_parameter("pre_explore_spin_enable").value):
            if not self._skip_spin_post_init_started:
                self._skip_spin_post_init_started = True
                self.get_logger().info(
                    "pre_explore_spin_enable=false: skip spin, go to rear nav then visit cached boxes."
                )
                self._begin_return_rear_or_visit()
            return

        goal = geometry_msgs.PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = (
            self.home_pose.pose.position.x
            + float(self.get_parameter("pre_explore_nav_offset_x_m").value)
        )
        goal.pose.position.y = (
            self.home_pose.pose.position.y
            + float(self.get_parameter("pre_explore_nav_offset_y_m").value)
        )
        goal.pose.position.z = self.home_pose.pose.position.z
        goal.pose.orientation = quaternion_from_yaw(math.pi)

        self._set_state(TestState.PRE_EXPLORE_SPIN)
        self.get_logger().info(
            "PRE_EXPLORE_NAV: send map goal "
            f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}), yaw=pi (same as V4)."
        )
        self._send_nav_goal(goal, NavPurpose.PRE_EXPLORE_NAV)

    def _start_reset_odometry_if_needed(self):
        now = self.get_clock().now()
        if self._reset_odom_started_at is None:
            self._reset_odom_started_at = now

        elapsed = (now - self._reset_odom_started_at).nanoseconds / 1e9
        if elapsed > 8.0:
            self.get_logger().warn(
                "INIT: /reset_odometry not completed within 8s; continue without odom reset."
            )
            self._reset_odom_done = True
            return

        if self._reset_odom_future is None:
            if not self._reset_odom_client.wait_for_service(timeout_sec=0.2):
                return
            self.get_logger().info("INIT: calling /reset_odometry ...")
            self._reset_odom_future = self._reset_odom_client.call_async(Trigger.Request())
            return

        if not self._reset_odom_future.done():
            return

        try:
            resp = self._reset_odom_future.result()
            if resp is not None and getattr(resp, "success", False):
                self.get_logger().info(f"INIT: /reset_odometry success: {resp.message}")
            else:
                msg = "" if resp is None else getattr(resp, "message", "")
                self.get_logger().warn(f"INIT: /reset_odometry failed: {msg}")
        except Exception as e:
            self.get_logger().warn(f"INIT: /reset_odometry call error: {e}")

        self._reset_odom_done = True

    def _bin_point_callback(self, msg: geometry_msgs.Point, color: str):
        if self.current_state == TestState.PRE_EXPLORE_SPIN:
            try:
                pose_stamped = self._point_to_pose_stamped_in_frame(msg, "map")
            except Exception as e:
                self.get_logger().warn(f"Box map calibration failed for {color}: {e}")
                return

            self.cached_bin_poses[color] = pose_stamped
            self.get_logger().info(
                f"Cached box map pose for {color}: "
                f"({pose_stamped.pose.position.x:.3f}, {pose_stamped.pose.position.y:.3f})."
            )
            return

        if self.current_state == TestState.VISUAL_ALIGN and color == self._current_visit_color:
            self._visual_docking_last_point = msg

    def _point_to_pose_stamped_in_frame(self, point_msg: geometry_msgs.Point, target_frame: str):
        frame_id = str(self.get_parameter("camera_frame_id").value)

        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = rclpy.time.Time().to_msg()
        point_stamped.point = point_msg

        if not self.tf_buffer.can_transform(
            target_frame,
            frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        ):
            raise RuntimeError(f"TF not available: {target_frame}<-{frame_id}")

        transform = self.tf_buffer.lookup_transform(
            target_frame,
            frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        point_in_target = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.frame_id = target_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = point_in_target.point
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def _build_backward_nav_goal_map(self, distance_m: float) -> Optional[geometry_msgs.PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().error(f"RETURN_REAR: map<-base_link failed: {exc}")
            return None

        yaw = quat_yaw(transform.transform.rotation)
        goal_pose = geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = (
            transform.transform.translation.x - distance_m * math.cos(yaw)
        )
        goal_pose.pose.position.y = (
            transform.transform.translation.y - distance_m * math.sin(yaw)
        )
        goal_pose.pose.position.z = transform.transform.translation.z
        goal_pose.pose.orientation = transform.transform.rotation
        return goal_pose

    def _send_nav_goal(self, goal_pose: geometry_msgs.PoseStamped, purpose: NavPurpose):
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.current_nav_purpose = purpose
        self.nav2_goal_handle = None
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 goal response error: {e}")
            self._on_nav_aborted_for_current_purpose()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            self._on_nav_aborted_for_current_purpose()
            return

        self.get_logger().info("Nav2 goal accepted.")
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        try:
            goal_result = future.result()
            status = goal_result.status
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            status = GoalStatus.STATUS_UNKNOWN

        purpose = self.current_nav_purpose
        ok = status == GoalStatus.STATUS_SUCCEEDED

        if not ok:
            self.get_logger().warn(f"Nav2 finished with status={status} (purpose={purpose.value}).")

        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE

        if purpose == NavPurpose.PRE_EXPLORE_NAV:
            cached_colors = list(self.cached_bin_poses.keys())
            self.get_logger().info(
                f"Initial spin / mapping finished. Cached box colors: {cached_colors}"
            )
            self._begin_return_rear_or_visit()
            return

        if purpose == NavPurpose.RETURN_REAR_NAV:
            if ok:
                self.get_logger().info("RETURN_REAR: reached pose behind robot.")
            self._start_visit_queue_or_done()
            return

        if purpose == NavPurpose.BOX_APPROACH_NAV:
            if ok:
                self._enter_visual_align_for_current_box()
            else:
                self._advance_visit_after_box(skip_visual=True)
            return

    def _on_nav_aborted_for_current_purpose(self):
        purpose = self.current_nav_purpose
        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE

        if purpose == NavPurpose.PRE_EXPLORE_NAV:
            self._begin_return_rear_or_visit()
        elif purpose == NavPurpose.RETURN_REAR_NAV:
            self._start_visit_queue_or_done()
        elif purpose == NavPurpose.BOX_APPROACH_NAV:
            self._advance_visit_after_box(skip_visual=True)

    def _begin_return_rear_or_visit(self):
        dist = abs(float(self.get_parameter("post_spin_backup_distance_m").value))
        goal = self._build_backward_nav_goal_map(dist)
        if goal is None:
            self.get_logger().warn("RETURN_REAR: could not build goal; skip to visit queue.")
            self._start_visit_queue_or_done()
            return

        self._set_state(TestState.RETURN_REAR)
        self.get_logger().info(
            f"RETURN_REAR: Nav2 to {dist:.2f} m behind current heading "
            f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}) map."
        )
        self._send_nav_goal(goal, NavPurpose.RETURN_REAR_NAV)

    def _build_visit_queue(self) -> List[Tuple[str, geometry_msgs.PoseStamped]]:
        preferred = str(self.get_parameter("target_bin_color").value).strip().lower()
        ordered: List[Tuple[str, geometry_msgs.PoseStamped]] = []

        if preferred and preferred in self.cached_bin_poses:
            ordered.append((preferred, self.cached_bin_poses[preferred]))

        for color in ("red", "green", "blue"):
            if color == preferred:
                continue
            if color in self.cached_bin_poses:
                ordered.append((color, self.cached_bin_poses[color]))

        for color, pose in self.cached_bin_poses.items():
            if any(c == color for c, _ in ordered):
                continue
            ordered.append((color, pose))

        return ordered

    def _start_visit_queue_or_done(self):
        self._visit_queue = self._build_visit_queue()
        self._visit_index = 0
        if not self._visit_queue:
            self.get_logger().warn("No cached box poses to visit.")
            self._set_state(TestState.DONE)
            return
        self._start_nav_to_current_box()

    def _start_nav_to_current_box(self):
        if self._visit_index >= len(self._visit_queue):
            self.get_logger().info("All recorded boxes visited.")
            self._set_state(TestState.DONE)
            return

        color, box_pose = self._visit_queue[self._visit_index]
        self._current_visit_color = color

        robot_x, robot_y = self._get_robot_xy_map()
        standoff = float(self.get_parameter("box_approach_standoff_m").value)
        stamp = self.get_clock().now().to_msg()
        try:
            goal_pose = compute_pregrasp_pose(
                box_pose,
                standoff,
                robot_x,
                robot_y,
                frame_id="map",
                stamp=stamp,
                yaw_offset=math.pi,
            )
        except Exception as exc:
            self.get_logger().error(f"BOX_APPROACH: compute_pregrasp_pose failed: {exc}")
            self._advance_visit_after_box(skip_visual=True)
            return

        self._set_state(TestState.BOX_APPROACH)
        self.get_logger().info(
            f"BOX_APPROACH [{color}]: Nav2 to standoff {standoff:.2f} m "
            f"({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f}) map."
        )
        self._send_nav_goal(goal_pose, NavPurpose.BOX_APPROACH_NAV)

    def _get_robot_xy_map(self) -> Tuple[float, float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception:
            return (0.0, 0.0)

    def _enter_visual_align_for_current_box(self):
        self._stop_cmd_vel()
        self._visual_docking_last_point = None
        timeout = float(self.get_parameter("visual_align_timeout_sec").value)
        self._visual_align_deadline = time.monotonic() + max(1.0, timeout)

        self._set_state(TestState.VISUAL_ALIGN)
        self.get_logger().info(
            f"VISUAL_ALIGN [{self._current_visit_color}]: "
            "pure vision cmd_vel (same law as V4 alignment), waiting for /target_place messages."
        )

    def _visual_align_timer_step(self):
        if self._visual_align_deadline is not None and time.monotonic() >= self._visual_align_deadline:
            self.get_logger().warn(
                f"VISUAL_ALIGN [{self._current_visit_color}]: timeout; skip and continue."
            )
            self._stop_cmd_vel()
            self._advance_visit_after_box(skip_visual=False)
            return

        point = self._visual_docking_last_point
        if point is None:
            return

        target_z = float(self.get_parameter("grasp_target_camera_z_m").value)
        z_tol = abs(float(self.get_parameter("grasp_target_camera_z_tolerance_m").value))
        x_tol = abs(float(self.get_parameter("visual_docking_x_tolerance_m").value))

        max_w = abs(float(self.get_parameter("docking_angular_speed_max_rps").value))
        max_v = abs(float(self.get_parameter("docking_linear_speed_mps").value))

        kp_x = float(self.get_parameter("visual_docking_x_kp").value)
        kp_z = float(self.get_parameter("visual_docking_z_kp").value)

        # Same as task_manager_v4_refactor/alignment.py _visual_docking_control_step
        x_error = -float(point.x)
        z_error = float(point.z) - target_z

        aligned = (abs(x_error) <= x_tol) and (abs(z_error) <= z_tol)
        if aligned:
            self._stop_cmd_vel()
            self.get_logger().info(
                f"VISUAL_ALIGN [{self._current_visit_color}]: aligned "
                f"(|x_err|={abs(x_error):.4f} <= {x_tol}, |z_err|={abs(z_error):.4f} <= {z_tol})."
            )
            self._advance_visit_after_box(skip_visual=False)
            return

        twist = geometry_msgs.Twist()
        twist.angular.z = _clamp(kp_x * x_error, -max_w, max_w)
        twist.linear.x = _clamp(kp_z * z_error, -max_v, max_v)
        self.cmd_vel_pub.publish(twist)

    def _advance_visit_after_box(self, skip_visual: bool):
        self._stop_cmd_vel()
        self._visual_docking_last_point = None
        self._visual_align_deadline = None
        self._visit_index += 1
        if skip_visual:
            self.get_logger().info(
                f"Skipping remaining steps for box index {self._visit_index - 1} ({self._current_visit_color})."
            )
        self._start_nav_to_current_box()


def main(args=None):
    rclpy.init(args=args)
    node = ModuleTestBoxMappingNode()
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
