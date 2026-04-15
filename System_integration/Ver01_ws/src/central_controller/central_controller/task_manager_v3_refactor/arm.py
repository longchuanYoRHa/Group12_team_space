from __future__ import annotations

import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs

from central_controller.task_manager_v3_refactor.models import CargoState, TaskState


class TaskManagerArmMixin:
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
            if self.grasp_retry_count >= self.max_grasp_retries:
                self.get_logger().warn("Grasp failed, max retries reached, abandoning object")
                if self.object_pose:
                    self.object_blacklist.append(self.object_pose.pose.position)
                self.grasp_retry_count = 0
                self.state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            else:
                self.get_logger().info(
                    f"Grasp failed, retrying ({self.grasp_retry_count}/{self.max_grasp_retries})"
                )

    def _handle_place_arm_result(self):
        if self.arm_status == "idle":
            self.get_logger().info("Place succeeded!")
            self.cargo_state = CargoState.EMPTY
            self.place_retry_count = 0
            self.adjust_nav2_for_carry_mode(False)
            self._arm_cmd_sent = False
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
        target_pt = self._point_camera_to_base_link_mm(point_msg)
        if target_pt is None:
            self.get_logger().warn("Grasp: camera->base_link failed, skipping this cycle")
            return
        if self._map_coords_csv:
            mx = my = None
            if self.object_pose is not None:
                mx = self.object_pose.pose.position.x
                my = self.object_pose.pose.position.y
            self._map_coords_csv.log_object_pick_arm(
                color, point_msg.x, point_msg.y, point_msg.z, mx, my
            )
        self.get_logger().info("Sending pick target to manipulator (/arm/target_pick)...")
        self.arm_pick_pub.publish(target_pt)
        self._arm_cmd_sent = True

    def _execute_place_with_current_bin(self):
        if self.bin_pose is None:
            self.get_logger().error("PLACE_IN_BIN state but bin_pose is None!")
            return
        if self._arm_cmd_sent:
            return

        target_pt = self._get_point_in_base_link_mm(self.bin_pose)
        if target_pt is None:
            self.get_logger().warn("Place: cannot get target in base_link, skipping this cycle")
            return

        if self._map_coords_csv:
            position = self.bin_pose.pose.position
            vx = vy = vz = None
            if self._last_bin_vision_xyz is not None:
                vx, vy, vz = self._last_bin_vision_xyz
            self._map_coords_csv.log_bin_map_place_command(
                self._last_bin_map_color, vx, vy, vz, position.x, position.y
            )

        self.get_logger().info("Sending place target to manipulator (/arm/target_place)...")
        self.arm_place_pub.publish(target_pt)
        self._arm_cmd_sent = True
