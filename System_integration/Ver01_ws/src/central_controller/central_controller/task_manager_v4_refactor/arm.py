from __future__ import annotations

import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs

from central_controller.task_manager_v4_refactor.models import CargoState, TaskState


class TaskManagerArmMixin:
    """Arm/gripper command publication and result handling for V4."""

    def _copy_point(self, source: geometry_msgs.Point):
        target = geometry_msgs.Point()
        target.x = source.x
        target.y = source.y
        target.z = source.z
        return target

    def _arm_status_callback(self, msg: std_msgs.String):
        self.arm_status = msg.data.lower()

    def _arm_gripper_status_callback(self, msg: std_msgs.String):
        self.gripper_status = msg.data.lower()

    def adjust_nav2_for_carry_mode(self, enable: bool):
        if enable:
            self.get_logger().info("Carry mode on: reduced speed, larger inflation radius")
        else:
            self.get_logger().info("Carry mode off: normal parameters restored")

    def _handle_grasp_arm_result(self):
        if self.arm_status == "holding" and self.gripper_status == "object_held":
            self.get_logger().info("Grasp succeeded!")
            self.cargo_state = CargoState.HAS_OBJECT
            # Persist the picked object's color so later bin detections can be filtered.
            self._carried_object_color = self._grasp_command_color
            self.grasp_retry_count = 0
            self.adjust_nav2_for_carry_mode(True)
            self._arm_cmd_sent = False
            self._start_backup_after_action(
                next_state=TaskState.RESUME_EXPLORE_FOR_BIN,
                explore_resume_after_restore=True,
            )
            return

        if self.arm_status == "error" or (self.arm_status == "idle" and self._arm_cmd_sent):
            self.grasp_retry_count += 1
            self._arm_cmd_sent = False
            self._grasp_command_color = None
            if self.grasp_retry_count >= self.max_grasp_retries:
                self.get_logger().warn("Grasp failed, max retries reached, abandoning object")
                if self.object_pose:
                    self.object_blacklist.append(self.object_pose.pose.position)
                self.grasp_retry_count = 0
                self.state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            else:
                # Do not resend a pick target here; the next vision callback will re-drive the flow.
                self.get_logger().info(
                    f"Grasp failed, retrying ({self.grasp_retry_count}/{self.max_grasp_retries})"
                )

    def _handle_place_arm_result(self):
        if (
            not self._place_status_changed_after_command
            and self.arm_status != self._place_status_at_command
        ):
            self._place_status_changed_after_command = True

        if self.arm_status == "idle":
            if (
                self._place_status_at_command == "idle"
                and not self._place_status_changed_after_command
            ):
                return
            self.get_logger().info("Place succeeded!")
            self.cargo_state = CargoState.EMPTY
            self.place_retry_count = 0
            self.adjust_nav2_for_carry_mode(False)
            self._arm_cmd_sent = False
            self._grasp_command_color = None
            self._carried_object_color = None
            self._place_status_at_command = "unknown"
            self._place_status_changed_after_command = False
            if self.bin_pose is not None:
                self.bin_blacklist.append(self.bin_pose.pose.position)
            if self.explore_done_flag:
                self._start_backup_after_action(
                    next_state=TaskState.NAV_TO_INTEREST_POINT,
                    explore_resume_after_restore=None,
                )
            else:
                self._start_backup_after_action(
                    next_state=TaskState.POST_ACTION,
                    explore_resume_after_restore=True,
                )
            return

        if self.arm_status == "error":
            self.place_retry_count += 1
            self._arm_cmd_sent = False
            self._place_status_at_command = "unknown"
            self._place_status_changed_after_command = False
            if self.place_retry_count >= self.max_place_retries:
                self.get_logger().warn(
                    "Place failed, max retries reached, resuming explore for bin"
                )
                self.place_retry_count = 0
                self.state = TaskState.RESUME_EXPLORE_FOR_BIN
                self._publish_explore_resume_if_changed(True)
            else:
                self.get_logger().info(
                    f"Place failed, retrying ({self.place_retry_count}/{self.max_place_retries})"
                )

    def _execute_grasp_with_current_object(
        self, point_msg: geometry_msgs.Point, color: str
    ):
        if self._arm_cmd_sent:
            return
        target_pt = self._copy_point(point_msg)
        if self._map_coords_csv:
            mx = my = None
            if self.object_pose is not None:
                mx = self.object_pose.pose.position.x
                my = self.object_pose.pose.position.y
            self._map_coords_csv.log_object_pick_arm(
                color, point_msg.x, point_msg.y, point_msg.z, mx, my
            )
        self.get_logger().info(
            "Sending pick target to manipulator (/arm/target_pick) "
            f"in {self.get_parameter('camera_frame_id').value} frame."
        )
        self.arm_pick_pub.publish(target_pt)
        self._grasp_command_color = color
        self._arm_cmd_sent = True

    def _execute_place_with_current_bin(self):
        if self._arm_cmd_sent:
            return
        if self._last_bin_vision_xyz is None:
            self.get_logger().warn(
                "Place: no cached camera-frame bin point, skipping this cycle"
            )
            return
        target_pt = geometry_msgs.Point()
        target_pt.x, target_pt.y, target_pt.z = self._last_bin_vision_xyz

        if self._map_coords_csv:
            position = None if self.bin_pose is None else self.bin_pose.pose.position
            vx = vy = vz = None
            if self._last_bin_vision_xyz is not None:
                vx, vy, vz = self._last_bin_vision_xyz
            self._map_coords_csv.log_bin_map_place_command(
                self._last_bin_map_color,
                vx,
                vy,
                vz,
                None if position is None else position.x,
                None if position is None else position.y,
            )

        self.get_logger().info(
            "Sending place target to manipulator (/arm/target_place) "
            f"in {self.get_parameter('camera_frame_id').value} frame."
        )
        self.arm_place_pub.publish(target_pt)
        self._place_status_at_command = self.arm_status
        self._place_status_changed_after_command = False
        self._arm_cmd_sent = True

    def _execute_place_with_fixed_target(self):
        if self._arm_cmd_sent:
            return

        target_pt = geometry_msgs.Point()
        target_pt.x = float(self.get_parameter("fixed_place_target_x").value)
        target_pt.y = float(self.get_parameter("fixed_place_target_y").value)
        target_pt.z = float(self.get_parameter("fixed_place_target_z").value)

        self.get_logger().info(
            "FORWARD_BEFORE_PLACE done: sending fixed place target "
            "to manipulator (/arm/target_place)."
        )
        self.arm_place_pub.publish(target_pt)
        self._place_status_at_command = self.arm_status
        self._place_status_changed_after_command = False
        self._arm_cmd_sent = True
        self.state = TaskState.PLACE_IN_BIN

