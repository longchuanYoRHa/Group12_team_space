from __future__ import annotations

import math

import rclpy
from action_msgs.msg import GoalStatus
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
import tf2_geometry_msgs

from central_controller.task_manager_utils import quat_yaw
from central_controller.task_manager_v3_refactor.models import (
    Nav2GoalResponseEvent,
    Nav2ResultEvent,
    NavPurpose,
    TaskState,
)


class TaskManagerNavigationMixin:
    def _cache_missing_bins_as_home_pose_if_empty(self) -> bool:
        if self.cached_bin_poses or self.home_pose is None:
            return False

        home_position = self.home_pose.pose.position
        for color in ("red", "green", "blue"):
            pose_stamped = geometry_msgs.PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = home_position.x
            pose_stamped.pose.position.y = home_position.y
            pose_stamped.pose.position.z = home_position.z
            pose_stamped.pose.orientation = self.home_pose.pose.orientation
            self.cached_bin_poses[color] = pose_stamped

        self.get_logger().warn(
            "No bin detected during pre-explore spin; defaulting all cached bin poses "
            "to the robot home pose."
        )
        return True

    def _build_camera_point_stamped(self, point_msg: geometry_msgs.Point):
        frame_id = str(self.get_parameter("camera_frame_id").value)
        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = rclpy.time.Time().to_msg()
        point_stamped.point = point_msg
        return point_stamped

    def _transform_camera_point(self, point_msg: geometry_msgs.Point, target_frame: str):
        frame_id = str(self.get_parameter("camera_frame_id").value)
        point_stamped = self._build_camera_point_stamped(point_msg)
        timeout = rclpy.duration.Duration(seconds=0.5)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=timeout,
            )
            return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        except Exception as exc:
            if target_frame not in ("map", "odom"):
                raise

            self.get_logger().warn(
                f"Direct TF {target_frame}<-{frame_id} failed, retry via base_link: {exc}"
            )

        cam_to_base = self.tf_buffer.lookup_transform(
            "base_link",
            frame_id,
            rclpy.time.Time(),
            timeout=timeout,
        )
        point_in_base = tf2_geometry_msgs.do_transform_point(point_stamped, cam_to_base)
        base_to_target = self.tf_buffer.lookup_transform(
            target_frame,
            "base_link",
            rclpy.time.Time(),
            timeout=timeout,
        )
        return tf2_geometry_msgs.do_transform_point(point_in_base, base_to_target)

    def _get_point_in_base_link_mm(self, pose_stamped: geometry_msgs.PoseStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            pose_in_base = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
            pt = geometry_msgs.Point()
            pt.x = pose_in_base.position.x * 1000.0
            pt.y = pose_in_base.position.y * 1000.0
            pt.z = pose_in_base.position.z * 1000.0
            return pt
        except Exception as exc:
            self.get_logger().error(f"TF transform to base_link failed: {exc}")
            return None

    def _point_camera_to_base_link_mm(self, point_msg: geometry_msgs.Point):
        try:
            point_in_base = self._transform_camera_point(point_msg, "base_link")
            pt = geometry_msgs.Point()
            pt.x = point_in_base.point.x * 1000.0
            pt.y = point_in_base.point.y * 1000.0
            pt.z = point_in_base.point.z * 1000.0
            return pt
        except Exception as exc:
            self.get_logger().error(f"TF camera->base_link failed: {exc}")
            return None

    def _publish_explore_resume_if_changed(self, resume: bool):
        if self._last_explore_resume is not None and self._last_explore_resume == resume:
            return
        self._last_explore_resume = resume
        msg = std_msgs.Bool()
        msg.data = resume
        self.explore_control_pub.publish(msg)

    def _point_to_pose_stamped_in_map(self, point_msg: geometry_msgs.Point):
        point_in_map = self._transform_camera_point(point_msg, "map")
        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = point_in_map.point
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def _point_to_pose_stamped_in_frame(
        self, point_msg: geometry_msgs.Point, target_frame: str
    ):
        point_in_target = self._transform_camera_point(point_msg, target_frame)
        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.frame_id = target_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = point_in_target.point
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def _get_robot_xy_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception:
            return (0.0, 0.0)

    def _get_robot_xy_in_frame(self, target_frame: str):
        try:
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
        except Exception:
            return (0.0, 0.0)

    def _build_backward_nav_goal_in_frame(self, distance_m: float, target_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to get robot pose in {target_frame} for backup nav goal: {exc}"
            )
            return None

        yaw = quat_yaw(transform.transform.rotation)
        goal_pose = geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = target_frame
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

    def _cancel_nav2_goal_if_any(self):
        if self.nav2_goal_handle is not None:
            self.nav2_client.cancel_goal_async(self.nav2_goal_handle)
            self.nav2_goal_handle = None
            self.current_nav_purpose = NavPurpose.NONE

    def _send_nav_goal(self, goal_pose: geometry_msgs.PoseStamped, purpose: NavPurpose):
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.current_nav_purpose = purpose
        self.nav2_goal_handle = None
        self.get_logger().info(f"Sending Nav2 goal for {purpose.value}")
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        self.dispatch(Nav2GoalResponseEvent(future))

    def _apply_nav2_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected!")
            if self.current_nav_purpose == NavPurpose.BACKUP_AFTER_ACTION:
                self.get_logger().warn(
                    "BACKUP_AFTER_ACTION nav goal rejected; falling back to cmd_vel backup."
                )
                self._start_backup_cmd_vel_fallback()
                self.nav2_goal_handle = None
                self.current_nav_purpose = NavPurpose.NONE
                return
            if (
                self.current_nav_purpose == NavPurpose.PRE_EXPLORE_NAV
                and self.state == TaskState.PRE_EXPLORE_SPIN
            ):
                self._cache_missing_bins_as_home_pose_if_empty()
                self.get_logger().warn(
                    "PRE_EXPLORE_NAV rejected; starting exploration without pre-navigation."
                )
                self.state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            self.nav2_goal_handle = None
            self.current_nav_purpose = NavPurpose.NONE
            return

        self.get_logger().info("Nav2 goal accepted")
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        self.dispatch(Nav2ResultEvent(future))

    def _apply_nav2_result(self, future):
        goal_handle = future.result()
        status = goal_handle.status
        purpose = self.current_nav_purpose

        if purpose == NavPurpose.PRE_EXPLORE_NAV:
            self._cache_missing_bins_as_home_pose_if_empty()
            keys = list(self.cached_bin_poses.keys())
            if self.state == TaskState.PRE_EXPLORE_SPIN:
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(
                        f"PRE_EXPLORE_NAV succeeded. Cached bin colors: {keys}. "
                        "Starting frontier exploration."
                    )
                else:
                    self.get_logger().warn(
                        f"PRE_EXPLORE_NAV finished with status={status}; "
                        f"cached bin colors: {keys}. Starting exploration anyway."
                    )
                self.state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            self.nav2_goal_handle = None
            self.current_nav_purpose = NavPurpose.NONE
            return

        if purpose == NavPurpose.BACKUP_AFTER_ACTION:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Nav2 goal succeeded (backup_after_action)")
                self._finish_backup_after_action()
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    "Nav2 goal aborted (backup_after_action); falling back to cmd_vel backup."
                )
                self._start_backup_cmd_vel_fallback()
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(
                    "Nav2 goal canceled (backup_after_action); falling back to cmd_vel backup."
                )
                self._start_backup_cmd_vel_fallback()
            else:
                self.get_logger().warn(
                    f"Nav2 goal finished with status={status} (backup_after_action); "
                    "falling back to cmd_vel backup."
                )
                self._start_backup_cmd_vel_fallback()
            self.nav2_goal_handle = None
            self.current_nav_purpose = NavPurpose.NONE
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Nav2 goal succeeded ({purpose.value})")
            if purpose == NavPurpose.OBJECT_PREGRASP:
                self._enter_precision_align(purpose, next_state_after_align=TaskState.GRASP)
            elif purpose == NavPurpose.BIN_PREPLACE:
                self._enter_precision_align(
                    purpose, next_state_after_align=TaskState.PLACE_IN_BIN
                )
            elif purpose == NavPurpose.INTEREST_POINT:
                self._enter_precision_align(purpose, next_state_after_align=None)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"Nav2 goal aborted ({purpose.value})")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Nav2 goal canceled ({purpose.value})")

        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE
