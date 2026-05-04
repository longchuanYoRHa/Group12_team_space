from __future__ import annotations

import time

import geometry_msgs.msg as geometry_msgs

from central_controller.task_manager_v4_refactor.models import CargoState, NavPurpose, TaskState


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class TaskManagerAlignmentMixin:
    """
    V4 precision align:
    - 对 object：纯视觉伺服（camera frame 下 x 偏移 -> 角速度；z 与目标抓取 z 的误差 -> 线速度）
    - 对 bin：纯视觉伺服到放置触发 z，随后前进固定距离并发布固定放置点
    """

    def _enter_precision_align(self, source_purpose: NavPurpose, next_state_after_align):
        self.state = TaskState.PRECISION_ALIGN
        self._precision_align_source_purpose = source_purpose
        self._precision_align_next_state = next_state_after_align

        # Visual servo path (object/bin alignment)
        self._visual_docking_active = False
        self._visual_docking_last_point = None  # geometry_msgs.Point in camera frame
        self._visual_docking_start_time = None
        self._forward_stop_hold_deadline = None
        self._forward_deadline = None
        self._forward_start_time = None
        self._forward_speed_mps = 0.0
        self._forward_distance_m = 0.0

        self.wait_at_point_start_time = time.monotonic()
        self.get_logger().info(
            f"Entered PRECISION_ALIGN (source={source_purpose.value}); waiting for vision trigger."
        )

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
        self._visual_docking_active = False
        self._visual_docking_last_point = None

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

        if self._precision_align_source_purpose == NavPurpose.INTEREST_POINT:
            self._precision_align_next_state = (
                TaskState.GRASP if is_object else TaskState.PLACE_IN_BIN
            )

        if self._visual_docking_start_time is None:
            self._visual_docking_start_time = time.monotonic()
        self._visual_docking_last_point = point_msg
        self._visual_docking_active = True

    def _precision_align_control_step(self):
        if getattr(self, "_visual_docking_active", False):
            self._visual_docking_control_step()

    def _visual_docking_control_step(self) -> None:
        point = self._visual_docking_last_point
        if point is None:
            return

        x_tol = abs(float(self.get_parameter("visual_docking_x_tolerance_m").value))
        is_place_align = self._precision_align_next_state == TaskState.PLACE_IN_BIN

        if is_place_align:
            target_z = float(self.get_parameter("place_trigger_camera_z_m").value)
            z_tol = abs(float(self.get_parameter("place_trigger_camera_z_tolerance").value))
        else:
            target_z = float(self.get_parameter("grasp_target_camera_z_m").value)
            z_tol = abs(float(self.get_parameter("grasp_target_camera_z_tolerance_m").value))

        max_w = abs(float(self.get_parameter("docking_angular_speed_max_rps").value))
        max_v = abs(float(self.get_parameter("docking_linear_speed_mps").value))

        kp_x = float(self.get_parameter("visual_docking_x_kp").value)
        kp_z = float(self.get_parameter("visual_docking_z_kp").value)

        x_error = -float(point.x)
        z_error = float(point.z) - target_z

        if is_place_align and abs(z_error) <= z_tol:
            self._stop_cmd_vel()
            self._visual_docking_active = False
            self._visual_docking_last_point = None
            self._enter_forward_before_place_phase()
            return

        aligned = (abs(x_error) <= x_tol) and (abs(z_error) <= z_tol)
        if (not is_place_align) and aligned:
            self._stop_cmd_vel()
            self._visual_docking_active = False
            self._visual_docking_last_point = None

            # v2-like behavior: once within tolerance, switch to next state directly.
            if self._precision_align_next_state is not None:
                self.state = self._precision_align_next_state
                self._arm_cmd_sent = False
            return

        twist = geometry_msgs.Twist()
        twist.angular.z = _clamp(kp_x * x_error, -max_w, max_w)
        twist.linear.x = _clamp(kp_z * z_error, -max_v, max_v)
        self.cmd_vel_pub.publish(twist)

    def _enter_forward_before_place_phase(self):
        stop_hold_s = max(
            0.0, float(self.get_parameter("forward_before_place_stop_hold_sec").value)
        )
        self._forward_stop_hold_deadline = time.monotonic() + stop_hold_s
        self._forward_deadline = None
        self._forward_start_time = None
        self._forward_speed_mps = 0.0
        self._forward_distance_m = 0.0
        self.state = TaskState.FORWARD_BEFORE_PLACE
        self.get_logger().info(
            "PRECISION_ALIGN_PLACE: z trigger reached, entering FORWARD_BEFORE_PLACE."
        )

    def _forward_before_place_control_step(self):
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
            self.get_logger().info(
                f"FORWARD_BEFORE_PLACE: start cmd_vel forward {dist_m:.2f}m at {speed_mps:.2f}m/s."
            )

        if self._forward_deadline is not None and now >= self._forward_deadline:
            self._stop_cmd_vel()
            self._forward_stop_hold_deadline = None
            self._forward_deadline = None
            self._forward_start_time = None
            self._execute_place_with_fixed_target()
            return

        twist = geometry_msgs.Twist()
        twist.linear.x = self._forward_speed_mps
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _start_backup_after_action(
        self, next_state: TaskState, explore_resume_after_restore
    ):
        self._backup_end_time = None
        self._backup_next_state = next_state
        self._backup_after_restore_explore_resume = explore_resume_after_restore
        self.state = TaskState.BACKUP_AFTER_ACTION

        # 直接使用 cmd_vel 反向后退 backup_distance_m（默认 0.20 m），不再尝试 nav2 goal。
        self._start_backup_cmd_vel_fallback()

    def _start_backup_cmd_vel_fallback(self):
        backup_dist = abs(float(self.get_parameter("backup_distance_m").value))
        linear_speed = float(self.get_parameter("docking_linear_speed_mps").value)
        linear_speed = max(1e-4, abs(linear_speed))
        duration = backup_dist / linear_speed

        self._backup_end_time = time.monotonic() + duration
        self.get_logger().info(
            f"BACKUP_AFTER_ACTION: cmd_vel backing up {backup_dist:.2f} m "
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

