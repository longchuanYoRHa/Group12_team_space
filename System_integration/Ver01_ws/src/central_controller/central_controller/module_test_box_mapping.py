#!/usr/bin/env python3
"""
Module test: cache bin map coordinates during the initial pre-explore spin,
then send a single DockRobot goal from the cached coordinate and stop 20 cm
away from the bin. Unlike the vision-guided docking test, this node does not
continuously feed external detection updates during docking.
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
from std_srvs.srv import Trigger

try:
    from nav2_msgs.action import DockRobot  # type: ignore[attr-defined]
    _dockrobot_import_error = None
except Exception as nav2_import_error:
    try:
        from opennav_docking_msgs.action import DockRobot  # pyright: ignore[reportMissingImports]
        _dockrobot_import_error = None
    except Exception as docking_msgs_import_error:
        DockRobot = None  # type: ignore
        _dockrobot_import_error = (
            "Failed to import DockRobot from both nav2_msgs.action and "
            f"opennav_docking_msgs.action: nav2_msgs={nav2_import_error}, "
            f"opennav_docking_msgs={docking_msgs_import_error}"
        )

from central_controller.task_manager_utils import quaternion_from_yaw


class TestState(Enum):
    INIT = "init"
    PRE_EXPLORE_SPIN = "pre_explore_spin"
    DOCKING = "docking"
    DONE = "done"


class NavPurpose(Enum):
    NONE = "none"
    PRE_EXPLORE_NAV = "pre_explore_nav"


class ModuleTestBoxMappingNode(Node):
    def __init__(self):
        super().__init__("module_test_box_mapping")

        self.current_state = TestState.INIT
        self.current_nav_purpose = NavPurpose.NONE
        self.home_pose = None
        self.cached_bin_poses = {}
        self.nav2_goal_handle = None

        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None

        self._reset_odom_done = False
        self._reset_odom_future = None
        self._reset_odom_started_at = None

        self.declare_parameter("camera_frame_id", "D435i_camera_link")
        self.declare_parameter("dock_action_name", "dock_robot")
        self.declare_parameter("dock_type", "simple_non_charging_dock")
        self.declare_parameter("docking_stop_distance_m", 0.20)
        self.declare_parameter("pre_explore_spin_enable", True)
        self.declare_parameter("pre_explore_nav_offset_x_m", 0.3)
        self.declare_parameter("pre_explore_nav_offset_y_m", 0.0)
        self.declare_parameter("target_bin_color", "")

        self.state_pub = self.create_publisher(
            std_msgs.String, "module_test_box_mapping/state", 10
        )

        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, "navigate_to_pose")
        self.dock_client = None
        if DockRobot is None:
            self.get_logger().error(
                "DockRobot action import failed. "
                "This environment should provide it from `nav2_msgs.action` "
                "or older setups from `opennav_docking_msgs.action`. "
                f"Original error: {_dockrobot_import_error}."
            )
        else:
            self.dock_client = ActionClient(
                self, DockRobot, self.get_parameter("dock_action_name").value
            )

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

        self.get_logger().info("module_test_box_mapping initialized.")
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

    def _state_timer_callback(self):
        self._publish_state()

        if self.current_state == TestState.INIT:
            self._handle_init_state()
        elif self.current_state == TestState.DOCKING:
            self._handle_dock_result_if_ready()

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
            self.get_logger().info("pre_explore_spin_enable=false, docking from cached bins directly.")
            self._try_start_docking_from_cached_bins()
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
            f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}), yaw=pi."
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
        if self.current_state != TestState.PRE_EXPLORE_SPIN:
            return

        try:
            pose_stamped = self._point_to_pose_stamped_in_frame(msg, "map")
        except Exception as e:
            self.get_logger().warn(f"Bin map calibration failed for {color}: {e}")
            return

        self.cached_bin_poses[color] = pose_stamped
        self.get_logger().info(
            f"Cached bin map pose for {color}: "
            f"({pose_stamped.pose.position.x:.3f}, {pose_stamped.pose.position.y:.3f})."
        )

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
            self._finish_mapping_and_try_docking()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("PRE_EXPLORE_NAV rejected.")
            self._finish_mapping_and_try_docking()
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

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("PRE_EXPLORE_NAV succeeded.")
        else:
            self.get_logger().warn(f"PRE_EXPLORE_NAV finished with status={status}.")

        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE
        self._finish_mapping_and_try_docking()

    def _finish_mapping_and_try_docking(self):
        cached_colors = list(self.cached_bin_poses.keys())
        self.get_logger().info(f"Initial mapping finished. Cached bin colors: {cached_colors}")
        self._try_start_docking_from_cached_bins()

    def _select_cached_bin_pose(self):
        preferred = str(self.get_parameter("target_bin_color").value).strip().lower()
        if preferred:
            pose = self.cached_bin_poses.get(preferred)
            if pose is None:
                self.get_logger().warn(
                    f"Preferred color '{preferred}' not found in cached bins: "
                    f"{list(self.cached_bin_poses.keys())}"
                )
            else:
                return preferred, pose

        for color in ("red", "green", "blue"):
            pose = self.cached_bin_poses.get(color)
            if pose is not None:
                return color, pose

        for color, pose in self.cached_bin_poses.items():
            return color, pose

        return None, None

    def _try_start_docking_from_cached_bins(self):
        if self._dock_goal_sent:
            return

        color, cached_pose_map = self._select_cached_bin_pose()
        if cached_pose_map is None:
            self.get_logger().warn("No cached bin coordinate available after initial mapping.")
            self._set_state(TestState.DONE)
            return

        if self.dock_client is None:
            self._set_state(TestState.DONE)
            return
        if not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("DockRobot server not ready; skip docking test.")
            self._set_state(TestState.DONE)
            return

        try:
            target_pose = self._build_dock_target_from_cached_map_pose(cached_pose_map)
        except Exception as e:
            self.get_logger().error(f"Build docking target from cached map pose failed: {e}")
            self._set_state(TestState.DONE)
            return

        self.get_logger().info(
            f"Using cached {color} bin pose to send one DockRobot goal: "
            f"({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}) in odom."
        )

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose = target_pose
        goal.dock_type = self.get_parameter("dock_type").value
        goal.navigate_to_staging_pose = False
        goal.max_staging_time = 0.0

        send_goal_future = self.dock_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._dock_goal_response_callback)
        self._dock_goal_sent = True
        self._set_state(TestState.DOCKING)

    def _build_dock_target_from_cached_map_pose(
        self, cached_pose_map: geometry_msgs.PoseStamped
    ) -> geometry_msgs.PoseStamped:
        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = cached_pose_map.header.frame_id
        point_stamped.header.stamp = rclpy.time.Time().to_msg()
        point_stamped.point = cached_pose_map.pose.position

        transform = self.tf_buffer.lookup_transform(
            "odom",
            cached_pose_map.header.frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        point_in_odom = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

        robot_x, robot_y = self._get_robot_xy_in_frame("odom")
        bx = point_in_odom.point.x
        by = point_in_odom.point.y
        dx = bx - robot_x
        dy = by - robot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            raise RuntimeError("cached bin pose too close to robot origin")

        stop_dist = max(0.0, float(self.get_parameter("docking_stop_distance_m").value))
        ux = dx / dist
        uy = dy / dist
        tx = bx - ux * stop_dist
        ty = by - uy * stop_dist

        pose = geometry_msgs.PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tx
        pose.pose.position.y = ty
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(math.atan2(by - ty, bx - tx))
        return pose

    def _get_robot_xy_in_frame(self, target_frame: str):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            "base_link",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _dock_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"DockRobot goal response error: {e}")
            self._dock_goal_sent = False
            self._set_state(TestState.DONE)
            return

        if not goal_handle.accepted:
            self.get_logger().warn("DockRobot goal rejected.")
            self._dock_goal_sent = False
            self._set_state(TestState.DONE)
            return

        self.get_logger().info("DockRobot goal accepted.")
        self._dock_goal_handle = goal_handle
        self._dock_result_future = goal_handle.get_result_async()

    def _handle_dock_result_if_ready(self):
        if self._dock_result_future is None or not self._dock_result_future.done():
            return

        try:
            result = self._dock_result_future.result().result
        except Exception as e:
            self.get_logger().error(f"DockRobot result error: {e}")
            self._dock_goal_sent = False
            self._dock_goal_handle = None
            self._dock_result_future = None
            self._set_state(TestState.DONE)
            return

        if result.success:
            self.get_logger().info("Cached-coordinate docking test succeeded.")
        else:
            self.get_logger().warn(
                f"Cached-coordinate docking test failed, error_code={result.error_code}."
            )

        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None
        self._set_state(TestState.DONE)


def main(args=None):
    rclpy.init(args=args)
    node = ModuleTestBoxMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
