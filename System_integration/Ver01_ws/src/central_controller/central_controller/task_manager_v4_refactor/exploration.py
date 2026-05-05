from __future__ import annotations

import math
import os
import subprocess

import geometry_msgs.msg as geometry_msgs
import rclpy
import std_msgs.msg as std_msgs

from central_controller.detect_objects_in_pgm_map import (
    compute_nav_goal_map_xy,
    get_interest_points_from_pgm,
    load_map_metadata_from_yaml,
    load_pgm,
)
from central_controller.task_manager_utils import (
    compute_pregrasp_pose,
    is_pose_in_blacklist as check_pose_in_blacklist,
    quaternion_from_yaw,
)
from central_controller.task_manager_v4_refactor.models import (
    BinVisionEvent,
    CargoState,
    ExploreFinishedEvent,
    NavPurpose,
    ObjectVisionEvent,
    TaskState,
)


class TaskManagerExplorationMixin:
    def _handle_init_state(self):
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Nav2 server...")
            return

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.home_pose = geometry_msgs.PoseStamped()
            self.home_pose.header.frame_id = "map"
            self.home_pose.pose.position.x = transform.transform.translation.x
            self.home_pose.pose.position.y = transform.transform.translation.y
            self.home_pose.pose.orientation = transform.transform.rotation
            self.get_logger().info("System ready, home pose saved")

            if self.get_parameter("pre_explore_spin_enable").value:
                offset_x = float(self.get_parameter("pre_explore_nav_offset_x_m").value)
                offset_y = float(self.get_parameter("pre_explore_nav_offset_y_m").value)
                goal = geometry_msgs.PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = self.home_pose.pose.position.x + offset_x
                goal.pose.position.y = self.home_pose.pose.position.y + offset_y
                goal.pose.position.z = self.home_pose.pose.position.z
                goal.pose.orientation = quaternion_from_yaw(math.pi)
                self.state = TaskState.PRE_EXPLORE_SPIN
                self.get_logger().info(
                    f"PRE_EXPLORE_NAV: map goal ({goal.pose.position.x:.3f}, "
                    f"{goal.pose.position.y:.3f}), yaw=pi (-x). "
                    f"Offsets dx={offset_x:.3f}, dy={offset_y:.3f} m from home."
                )
                self._send_nav_goal(goal, NavPurpose.PRE_EXPLORE_NAV)
                return

            self.state = TaskState.EXPLORE
            self._publish_explore_resume_if_changed(True)
        except Exception as exc:
            self.get_logger().warn(f"Waiting for TF: {exc}")

    def _handle_object_vision_by_state(self, event: ObjectVisionEvent) -> None:
        msg = event.point
        color = event.color
        self.detected_object_colors.add(color)

        if self.cargo_state != CargoState.EMPTY:
            self.object_detection_count = 0
            return

        if self.state == TaskState.PRECISION_ALIGN:
            self._handle_precision_align_vision(point_msg=msg, is_object=True)
            if self.state == TaskState.GRASP:
                self._execute_grasp_with_current_object(msg, color)
            return

        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
        except Exception as exc:
            self.get_logger().error(f"Error processing target_pick message: {exc}")
            self.object_detection_count = 0
            return

        if check_pose_in_blacklist(
            pose_stamped.pose.position, self.object_blacklist, self.blacklist_radius
        ):
            return

        self.object_pose = pose_stamped

        if self.state == TaskState.GRASP:
            self._execute_grasp_with_current_object(msg, color)
            return

        if self.state == TaskState.NAV_TO_INTEREST_POINT:
            trigger_distance_m = float(
                self.get_parameter("interest_point_vision_trigger_distance_m").value
            )
            distance_remaining_m = self._last_nav2_distance_remaining_m
            if (
                self.current_nav_purpose == NavPurpose.INTEREST_POINT
                and distance_remaining_m is not None
                and distance_remaining_m < trigger_distance_m
            ):
                if self._map_coords_csv:
                    self._map_coords_csv.log_object_map_nav(
                        color,
                        msg.x,
                        msg.y,
                        msg.z,
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        self.state.name,
                    )
                self.get_logger().info(
                    "Object seen while approaching interest point "
                    f"(Nav2 remaining={distance_remaining_m:.2f} m < "
                    f"{trigger_distance_m:.2f} m); switching to visual docking."
                )
                self._cancel_nav2_goal_if_any()
                self._enter_precision_align(
                    NavPurpose.INTEREST_POINT,
                    next_state_after_align=TaskState.GRASP,
                )
                self._handle_precision_align_vision(point_msg=msg, is_object=True)
                return

            self.object_detection_count = 0
            return

        if self.state != TaskState.EXPLORE:
            self.object_detection_count = 0
            return

        self.object_detection_count += 1
        if self.object_detection_count < self.required_detection_frames:
            return

        self.object_detection_count = 0

        if self._map_coords_csv:
            self._map_coords_csv.log_object_map_nav(
                color,
                msg.x,
                msg.y,
                msg.z,
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                self.state.name,
            )

        self.get_logger().info(
            f"Object found during EXPLORE, coords=({pose_stamped.pose.position.x:.2f}, "
            f"{pose_stamped.pose.position.y:.2f}), stopping explore and docking directly."
        )
        self._publish_explore_resume_if_changed(False)
        self._cancel_nav2_goal_if_any()
        self._enter_precision_align(
            NavPurpose.OBJECT_PREGRASP,
            next_state_after_align=TaskState.GRASP,
        )

        self._handle_precision_align_vision(
            point_msg=msg,
            is_object=True,
        )

    def _object_point_callback(self, msg: geometry_msgs.Point, color: str):
        self.dispatch(ObjectVisionEvent(color, msg))

    def _handle_bin_vision_by_state(self, event: BinVisionEvent) -> None:
        msg = event.point
        color = event.color
        self.detected_bin_colors.add(color)

        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
        except Exception as exc:
            self.get_logger().error(f"Error processing target_place message: {exc}")
            self.bin_detection_count = 0
            return

        if check_pose_in_blacklist(
            pose_stamped.pose.position, self.bin_blacklist, self.blacklist_radius
        ):
            return

        self.bin_pose = pose_stamped
        self._last_bin_map_color = color
        self._last_bin_vision_xyz = (msg.x, msg.y, msg.z)

        if (
            self.state in (TaskState.EXPLORE, TaskState.PRE_EXPLORE_SPIN)
            and self.cargo_state == CargoState.EMPTY
        ):
            self.cached_bin_poses[color] = pose_stamped
            phase = (
                "PRE_EXPLORE_SPIN"
                if self.state == TaskState.PRE_EXPLORE_SPIN
                else "EXPLORE"
            )
            self.get_logger().info(
                f"Bin detected during {phase} for color {color}, cached for later use."
            )
            if self._map_coords_csv:
                self._map_coords_csv.log_bin_map_cached(
                    color,
                    msg.x,
                    msg.y,
                    msg.z,
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    self.state.name,
                )
            return

        if self.state == TaskState.PLACE_IN_BIN and self.cargo_state == CargoState.HAS_OBJECT:
            # Place command is now issued only by FORWARD_BEFORE_PLACE fixed target logic.
            return

        if self.state == TaskState.PRECISION_ALIGN:
            self._handle_precision_align_vision(point_msg=msg, is_object=False)
            return

        if self.cargo_state != CargoState.HAS_OBJECT:
            self.bin_detection_count = 0
            return

        if self.state != TaskState.RESUME_EXPLORE_FOR_BIN:
            self.bin_detection_count = 0
            return

        self.bin_detection_count += 1
        if self.bin_detection_count < self.required_detection_frames:
            return

        self.bin_detection_count = 0

        if self._map_coords_csv:
            self._map_coords_csv.log_bin_map_nav(
                color,
                msg.x,
                msg.y,
                msg.z,
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                self.state.name,
            )

        self.get_logger().info(
            f"Bin found during RESUME_EXPLORE_FOR_BIN, "
            f"coords=({pose_stamped.pose.position.x:.2f}, "
            f"{pose_stamped.pose.position.y:.2f}), stopping explore and entering visual align."
        )
        self._publish_explore_resume_if_changed(False)
        self._cancel_nav2_goal_if_any()
        self._enter_precision_align(
            NavPurpose.BIN_PREPLACE,
            next_state_after_align=TaskState.PLACE_IN_BIN,
        )
        self._handle_precision_align_vision(point_msg=msg, is_object=False)

    def _bin_point_callback(self, msg: geometry_msgs.Point, color: str):
        self.dispatch(BinVisionEvent(color, msg))

    def _get_maps_directory(self):
        maps_dir = self.get_parameter("maps_directory").value
        if maps_dir:
            return maps_dir
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory("central_controller")
            return os.path.join(pkg_share, "maps")
        except Exception:
            return os.path.expanduser("~/maps")

    def _terminate_task_manager(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._publish_explore_resume_if_changed(False)
        self._cancel_nav2_goal_if_any()
        self._stop_cmd_vel()
        if rclpy.ok():
            rclpy.shutdown()

    def _send_bin_preplace_goal(
        self,
        bin_pose: geometry_msgs.PoseStamped,
        *,
        color: str,
        source: str,
    ) -> None:
        self.bin_pose = bin_pose
        self._last_bin_map_color = color
        robot_x, robot_y = self._get_robot_xy_in_map()
        preplace_distance = self.get_parameter("preplace_distance").value
        goal_pose = compute_pregrasp_pose(
            self.bin_pose,
            preplace_distance,
            robot_x,
            robot_y,
            frame_id="map",
            stamp=self.get_clock().now().to_msg(),
            yaw_offset=math.pi,
        )
        self.get_logger().info(
            f"Navigating to {source} bin [{color}] at "
            f"({self.bin_pose.pose.position.x:.2f}, {self.bin_pose.pose.position.y:.2f}) "
            "for place flow."
        )
        self.state = TaskState.NAV_TO_BIN_PREPLACE
        self._send_nav_goal(goal_pose, NavPurpose.BIN_PREPLACE)

    def _try_navigate_to_cached_bin(self) -> bool:
        if not self.cached_bin_poses:
            return False

        robot_x, robot_y = self._get_robot_xy_in_map()
        best_choice = None
        best_dist_sq = None

        for color, pose_stamped in self.cached_bin_poses.items():
            position = pose_stamped.pose.position
            if check_pose_in_blacklist(
                position, self.bin_blacklist, self.blacklist_radius
            ):
                continue

            dist_sq = (position.x - robot_x) ** 2 + (position.y - robot_y) ** 2
            if best_dist_sq is None or dist_sq < best_dist_sq:
                best_choice = (color, pose_stamped)
                best_dist_sq = dist_sq

        if best_choice is None:
            return False

        color, pose_stamped = best_choice
        self._send_bin_preplace_goal(
            pose_stamped,
            color=color,
            source="cached",
        )
        return True

    def _start_explore_finished_fallback(self) -> None:
        self.get_logger().info(
            "Exploration finished, starting map fallback and interest point detection."
        )
        self._publish_explore_resume_if_changed(False)

    def _explore_finished_callback(self, msg: std_msgs.Bool):
        if not msg.data:
            return
        self.explore_finished_received = True
        self.dispatch(ExploreFinishedEvent())

    def _handle_explore_finished_dispatch(self) -> None:
        if self.cargo_state == CargoState.HAS_OBJECT:
            if self.state == TaskState.RESUME_EXPLORE_FOR_BIN:
                self.get_logger().info(
                    "Exploration finished while carrying object; "
                    "using cached bin pose before fallback."
                )
                self._publish_explore_resume_if_changed(False)
                self._cancel_nav2_goal_if_any()
                self.explore_done_flag = True
                if self._try_navigate_to_cached_bin():
                    return
                self.get_logger().warn(
                    "Exploration finished while carrying object, but no cached bin pose "
                    "is available; entering fallback directly."
                )
            elif self.state != TaskState.EXPLORE:
                return

        if self.state != TaskState.EXPLORE and self.state != TaskState.RESUME_EXPLORE_FOR_BIN:
            return

        self._start_explore_finished_fallback()

        maps_dir = self._get_maps_directory()
        os.makedirs(maps_dir, exist_ok=True)
        basename = self.get_parameter("map_save_basename").value
        map_base = os.path.join(maps_dir, basename)
        try:
            proc = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_base],
                capture_output=True,
                timeout=15,
                text=True,
            )
            if proc.returncode != 0:
                self._terminate_task_manager(
                    f"Fallback aborted: map_saver_cli returncode={proc.returncode}; "
                    f"stderr: {proc.stderr.strip() or '(none)'}"
                )
                return
        except subprocess.TimeoutExpired:
            self._terminate_task_manager(
                "Fallback aborted: map_saver_cli timed out after 15s."
            )
            return
        except FileNotFoundError:
            self._terminate_task_manager(
                "Fallback aborted: ros2 or map_saver_cli not found in PATH."
            )
            return
        except Exception as exc:
            self._terminate_task_manager(f"Fallback aborted: map_saver_cli error: {exc}")
            return

        pgm_path = os.path.join(maps_dir, basename + ".pgm")
        if not os.path.isfile(pgm_path):
            self._terminate_task_manager(
                f"Fallback aborted: PGM not found at {pgm_path}; "
                "cannot run interest point detection."
            )
            return

        try:
            raw_points = get_interest_points_from_pgm(
                pgm_path,
                prefer_yaml=True,
                max_bbox_extent_m=float(
                    self.get_parameter("interest_point_max_bbox_m").value
                ),
                dedupe_min_separation_px=float(
                    self.get_parameter("interest_point_dedupe_min_separation_px").value
                ),
            )
        except Exception as exc:
            self._terminate_task_manager(f"Fallback aborted: PGM detection failed: {exc}")
            return

        if self._map_coords_csv:
            self._map_coords_csv.log_pgm_points(raw_points, "pgm_raw", pgm_path=pgm_path)

        # 缓存 PGM 栅格，用于兴趣点导航时做 8 方向近障检查并选 standoff 方向。
        self._interest_nav_map_context = None
        try:
            meta = load_map_metadata_from_yaml(pgm_path)
            if meta is None:
                self.get_logger().warn(
                    "Map YAML metadata not found; interest-point nav will use fallback pregrasp logic."
                )
            else:
                map_res, map_origin = meta
                map_w, map_h, map_pixels = load_pgm(pgm_path)
                self._interest_nav_map_context = {
                    "w": map_w,
                    "h": map_h,
                    "pixels": map_pixels,
                    "resolution": map_res,
                    "origin": map_origin,
                }
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to load map context for obstacle-aware nav; fallback to pregrasp. err={exc}"
            )

        filtered = []
        for map_x, map_y in raw_points:
            point = geometry_msgs.Point()
            point.x = map_x
            point.y = map_y
            point.z = 0.0
            if check_pose_in_blacklist(point, self.object_blacklist, self.blacklist_radius):
                continue
            if check_pose_in_blacklist(point, self.bin_blacklist, self.blacklist_radius):
                continue
            filtered.append((map_x, map_y))

        if self._map_coords_csv:
            self._map_coords_csv.log_pgm_points(
                filtered, "pgm_filtered", pgm_path=pgm_path
            )

        self.interest_points = filtered
        self.interest_point_index = 0
        self.get_logger().info(
            f"Map detection: {len(raw_points)} points, {len(filtered)} after filtering."
        )

        if not self.interest_points:
            self._terminate_task_manager(
                "Fallback finished immediately: no interest points left after filtering."
            )
            return

        self.explore_done_flag = True
        self.state = TaskState.NAV_TO_INTEREST_POINT
        self._nav_to_next_interest_point()

    def _nav_to_next_interest_point(self):
        if self.interest_point_index >= len(self.interest_points):
            self._terminate_task_manager(
                "Fallback completed: all interest points visited; no more targets."
            )
            return

        map_x, map_y = self.interest_points[self.interest_point_index]
        self.current_interest_point = (map_x, map_y)

        target_pose = geometry_msgs.PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = map_x
        target_pose.pose.position.y = map_y
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0

        robot_x, robot_y = self._get_robot_xy_in_map()
        standoff_m = float(self.get_parameter("interest_point_standoff_m").value)
        map_ctx = getattr(self, "_interest_nav_map_context", None)
        if map_ctx is not None:
            goal_x, goal_y = compute_nav_goal_map_xy(
                map_x,
                map_y,
                robot_x,
                robot_y,
                standoff_m,
                w=map_ctx["w"],
                h=map_ctx["h"],
                pixels=map_ctx["pixels"],
                resolution=map_ctx["resolution"],
                origin=map_ctx["origin"],
                obstacle_check_radius_m=1.0,
                obstacle_check_start_cardinal_m=0.28,
                obstacle_check_start_diagonal_m=0.20,
            )
            goal_pose = geometry_msgs.PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0
            yaw_to_poi = math.atan2(map_y - goal_y, map_x - goal_x)
            goal_pose.pose.orientation = quaternion_from_yaw(yaw_to_poi)
        else:
            goal_pose = compute_pregrasp_pose(
                target_pose,
                standoff_m,
                robot_x,
                robot_y,
                frame_id="map",
                stamp=self.get_clock().now().to_msg(),
            )

        self.get_logger().info(
            f"Nav to interest point {self.interest_point_index + 1}/{len(self.interest_points)} "
            f"POI=({map_x:.2f}, {map_y:.2f}) standoff={standoff_m:.2f} m (facing POI)"
        )
        self._send_nav_goal(goal_pose, NavPurpose.INTEREST_POINT)

    def _skip_current_interest_point_after_nav_failure(self, reason: str) -> None:
        """Nav2 兴趣点目标被拒或结果非成功时：跳过当前索引并尝试下一个兴趣点。"""
        if self.state != TaskState.NAV_TO_INTEREST_POINT:
            return
        self.get_logger().warn(
            f"Interest point nav failure ({reason}); skipping index "
            f"{self.interest_point_index} and advancing."
        )
        self.interest_point_index += 1
        self.current_interest_point = None
        self.state = TaskState.NAV_TO_INTEREST_POINT
        self._nav_to_next_interest_point()

    def _start_bin_search_or_go_to_cached(self):
        if self.explore_finished_received and self.cargo_state == CargoState.HAS_OBJECT:
            self.get_logger().info(
                "Exploration already finished while carrying object; "
                "switching to cached bin delivery/fallback."
            )
            self._backup_after_restore_explore_resume = None
            self.explore_done_flag = True
            if self._try_navigate_to_cached_bin():
                return
            self.get_logger().warn(
                "No cached bin pose available after exploration finished; "
                "starting fallback directly."
            )
            self._handle_explore_finished_dispatch()
            return
        self._publish_explore_resume_if_changed(True)

    def _handle_post_action(self):
        self.get_logger().info("Post action: returning to explore")
        self.state = TaskState.EXPLORE
        self._publish_explore_resume_if_changed(True)

