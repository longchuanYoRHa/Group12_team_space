from __future__ import annotations

import math
import time

import geometry_msgs.msg as geometry_msgs

from central_controller.task_manager_utils import quaternion_from_yaw
from central_controller.task_manager_v3_refactor.docking import DockRobot
from central_controller.task_manager_v3_refactor.models import CargoState, NavPurpose, TaskState


class TaskManagerAlignmentMixin:
    def _enter_precision_align(self, source_purpose: NavPurpose, next_state_after_align):
        self.state = TaskState.PRECISION_ALIGN
        self._precision_align_source_purpose = source_purpose
        self._precision_align_next_state = next_state_after_align
        self._docking_active = False
        self._docking_phase = "rotate"
        self._last_docking_target_base_m = None
        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None
        self._precision_align_waiting_for_object_z_check = False
        self.wait_at_point_start_time = time.monotonic()
        self.get_logger().info(
            f"Entered PRECISION_ALIGN (source={source_purpose.value}); waiting for vision trigger."
        )

    def _build_dock_target_pose_from_vision(
        self,
        point_msg: geometry_msgs.Point,
        *,
        stop_dist: float | None = None,
        robot_delta: float | None = None,
    ):
        cube_pose_odom = self._point_to_pose_stamped_in_frame(point_msg, "map")
        robot_x, robot_y = self._get_robot_xy_in_frame("map")
        cx = cube_pose_odom.pose.position.x
        cy = cube_pose_odom.pose.position.y
        dx = cx - robot_x
        dy = cy - robot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            raise RuntimeError("invalid cube distance")

        ux = dx / dist
        uy = dy / dist
        if robot_delta is not None:
            tx = robot_x + ux * robot_delta
            ty = robot_y + uy * robot_delta
        else:
            if stop_dist is None:
                raise RuntimeError("stop_dist or robot_delta must be provided")
            tx = cx - ux * stop_dist
            ty = cy - uy * stop_dist

        target_pose = geometry_msgs.PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = tx
        target_pose.pose.position.y = ty
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation = quaternion_from_yaw(math.atan2(cy - ty, cx - tx))
        return target_pose, dist, (tx, ty)

    def _send_dock_goal(self, target_pose: geometry_msgs.PoseStamped, log_reason: str):
        self.get_logger().info(
            "PRECISION_ALIGN: sending DockRobot goal "
            f"({log_reason}, target=({target_pose.pose.position.x:.3f},"
            f"{target_pose.pose.position.y:.3f}) map)."
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

    def _handle_precision_align_timeout_if_needed(self):
        if self._precision_align_source_purpose != NavPurpose.INTEREST_POINT:
            return
        duration = self.get_parameter("wait_at_interest_point_sec").value
        if self.wait_at_point_start_time is None:
            self.wait_at_point_start_time = time.monotonic()
            return
        if (time.monotonic() - self.wait_at_point_start_time) < duration:
            return

        if self.current_interest_point is not None:
            point = geometry_msgs.Point()
            point.x = self.current_interest_point[0]
            point.y = self.current_interest_point[1]
            point.z = 0.0
            self.object_blacklist.append(point)
            self.get_logger().info(
                "No vision trigger in PRECISION_ALIGN within timeout; mark and skip."
            )

        self._stop_cmd_vel()
        self._docking_active = False
        self._docking_phase = "rotate"
        self._last_docking_target_base_m = None

        self.interest_point_index += 1
        self.current_interest_point = None
        self.wait_at_point_start_time = None
        self.state = TaskState.NAV_TO_INTEREST_POINT
        self._nav_to_next_interest_point()

    def _handle_precision_align_vision(
        self, point_msg: geometry_msgs.Point, is_object: bool
    ):
        if is_object and self.cargo_state != CargoState.EMPTY:
            return
        if (not is_object) and self.cargo_state != CargoState.HAS_OBJECT:
            return
        if self._dock_goal_sent:
            return
        if self.dock_client is None:
            return
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("PRECISION_ALIGN: DockRobot action server not ready yet.")
            return
        if self._precision_align_source_purpose == NavPurpose.INTEREST_POINT:
            self._precision_align_next_state = (
                TaskState.GRASP if is_object else TaskState.PLACE_IN_BIN
            )

        target_z = float(self.get_parameter("grasp_target_camera_z_m").value)
        z_tol = abs(float(self.get_parameter("grasp_target_camera_z_tolerance_m").value))

        if is_object and self._precision_align_waiting_for_object_z_check:
            z_error = point_msg.z - target_z
            if abs(z_error) <= z_tol:
                self.get_logger().info(
                    "PRECISION_ALIGN: object camera z within grasp tolerance "
                    f"(z={point_msg.z:.3f} m, target={target_z:.3f} +/- {z_tol:.3f})."
                )
                self._precision_align_waiting_for_object_z_check = False
                self.state = TaskState.GRASP
                self._arm_cmd_sent = False
                return

            try:
                target_pose, _, _ = self._build_dock_target_pose_from_vision(
                    point_msg,
                    robot_delta=z_error,
                )
            except Exception as exc:
                self.get_logger().error(
                    f"PRECISION_ALIGN: refine DockRobot goal failed: {exc}"
                )
                return

            self._precision_align_waiting_for_object_z_check = False
            self._send_dock_goal(
                target_pose,
                (
                    "refine_by_camera_z "
                    f"(z={point_msg.z:.3f} m, target={target_z:.3f} m, "
                    f"error={z_error:.3f} m)"
                ),
            )
            return

        stop_dist = max(0.0, float(self.get_parameter("docking_stop_distance_m").value))
        try:
            target_pose, _, _ = self._build_dock_target_pose_from_vision(
                point_msg,
                stop_dist=stop_dist,
            )
        except Exception as exc:
            self.get_logger().error(f"PRECISION_ALIGN: build DockRobot goal failed: {exc}")
            return

        self._send_dock_goal(target_pose, f"stop_dist={stop_dist:.3f} m")

    def _dock_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"DockRobot goal response error: {exc}")
            self._dock_goal_sent = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("DockRobot goal rejected.")
            self._dock_goal_sent = False
            return

        self.get_logger().info("DockRobot goal accepted.")
        self._dock_goal_handle = goal_handle
        self._dock_result_future = goal_handle.get_result_async()

    def _precision_align_control_step(self):
        if not self._dock_goal_sent or self._dock_result_future is None:
            return
        if not self._dock_result_future.done():
            return

        try:
            result = self._dock_result_future.result().result
        except Exception as exc:
            self.get_logger().error(f"PRECISION_ALIGN: DockRobot result error: {exc}")
            self._dock_goal_sent = False
            self._dock_goal_handle = None
            self._dock_result_future = None
            return

        if result.success:
            self.get_logger().info("PRECISION_ALIGN: DockRobot succeeded.")
            if self._precision_align_next_state == TaskState.GRASP:
                self._precision_align_waiting_for_object_z_check = True
                self.get_logger().info(
                    "PRECISION_ALIGN: waiting for post-dock object camera z check."
                )
            elif self._precision_align_next_state is not None:
                self.state = self._precision_align_next_state
                self._arm_cmd_sent = False
            else:
                self.get_logger().warn(
                    "PRECISION_ALIGN: next state is None; staying in PRECISION_ALIGN."
                )
        else:
            self.get_logger().warn(
                f"PRECISION_ALIGN: DockRobot failed (error_code={result.error_code})."
            )
            self._precision_align_waiting_for_object_z_check = False

        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None

    def _start_backup_after_action(
        self, next_state: TaskState, explore_resume_after_restore
    ):
        backup_dist = abs(float(self.get_parameter("backup_distance_m").value))
        self._backup_end_time = None
        self._backup_next_state = next_state
        self._backup_after_restore_explore_resume = explore_resume_after_restore
        self.state = TaskState.BACKUP_AFTER_ACTION

        goal_pose = self._build_backward_nav_goal_in_frame(backup_dist, "map")
        if goal_pose is None:
            self.get_logger().warn(
                "BACKUP_AFTER_ACTION: failed to build backup nav goal; "
                "falling back to cmd_vel backup."
            )
            self._start_backup_cmd_vel_fallback()
            return

        self.get_logger().info(
            "BACKUP_AFTER_ACTION: sending nav goal to current-heading rear point "
            f"({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f}) in map."
        )
        self._send_nav_goal(goal_pose, NavPurpose.BACKUP_AFTER_ACTION)

    def _start_backup_cmd_vel_fallback(self):
        backup_dist = float(self.get_parameter("backup_distance_m").value)
        linear_speed = float(self.get_parameter("docking_linear_speed_mps").value)
        linear_speed = max(1e-4, abs(linear_speed))
        duration = backup_dist / linear_speed

        self._backup_end_time = time.monotonic() + duration
        self.get_logger().info(
            f"BACKUP_AFTER_ACTION: cmd_vel fallback backing up {backup_dist:.2f} m "
            f"for {duration:.1f} s."
        )

    def _finish_backup_after_action(self):
        self._stop_cmd_vel()
        self._backup_end_time = None

        next_state = self._backup_next_state
        self._backup_next_state = None
        self.state = next_state

        if next_state == TaskState.RESUME_EXPLORE_FOR_BIN:
            self._start_bin_search_or_go_to_cached()
        elif next_state == TaskState.NAV_TO_INTEREST_POINT:
            self._nav_to_next_interest_point()
        elif next_state == TaskState.POST_ACTION:
            self._handle_post_action()

        if self._backup_after_restore_explore_resume is not None:
            self._publish_explore_resume_if_changed(
                bool(self._backup_after_restore_explore_resume)
            )
        self._backup_after_restore_explore_resume = None

    def _backup_control_step(self):
        if self._backup_end_time is None:
            return

        now = time.monotonic()
        if now >= self._backup_end_time:
            self._finish_backup_after_action()
            return

        linear_speed = float(self.get_parameter("docking_linear_speed_mps").value)
        twist = geometry_msgs.Twist()
        twist.linear.x = -abs(linear_speed)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _stop_cmd_vel(self):
        twist = geometry_msgs.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
