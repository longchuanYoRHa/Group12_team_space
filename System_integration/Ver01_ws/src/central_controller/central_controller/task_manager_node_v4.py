#!/usr/bin/env python3
"""
Task manager state machine node V4.

约束：
- 不修改 `task_manager_node_v3.py` 与 `task_manager_v3_refactor/`
- 基于 v3 refactor 结构复制出 v4 refactor

改进点（相对 V3）：
- 识别到 object 后的 PRECISION_ALIGN 对位改为纯视觉引导：
  - 使用 camera frame 下的 x 偏移量控制角速度（差速转向）
  - 使用 z 与 0.265（参数 grasp_target_camera_z_m）做差控制线速度
  - 达到容差后直接进入 GRASP（与 v2 “达标即抓取” 的切换方式一致）
"""

import os
import time

import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import std_msgs.msg as std_msgs
import tf2_ros
from std_srvs.srv import Trigger

from central_controller.detect_objects_in_pgm_map import DEFAULT_ORIGIN, DEFAULT_RESOLUTION
from central_controller.detection_map_coordinates_csv import DetectionMapCoordinatesCsvLogger
from central_controller.task_manager_v4_refactor.alignment import TaskManagerAlignmentMixin
from central_controller.task_manager_v4_refactor.arm import TaskManagerArmMixin
from central_controller.task_manager_v4_refactor.exploration import (
    TaskManagerExplorationMixin,
)
from central_controller.task_manager_v4_refactor.models import (
    BinVisionEvent,
    CargoState,
    ExploreFinishedEvent,
    Nav2GoalResponseEvent,
    Nav2ResultEvent,
    NavPurpose,
    ObjectVisionEvent,
    TaskEvent,
    TaskState,
    TickEvent,
)
from central_controller.task_manager_v4_refactor.navigation import (
    TaskManagerNavigationMixin,
)


class TaskManagerNodeV4(
    TaskManagerExplorationMixin,
    TaskManagerAlignmentMixin,
    TaskManagerArmMixin,
    TaskManagerNavigationMixin,
    Node,
):
    def __init__(self):
        super().__init__("task_manager_v4")
        self._init_runtime_state()
        self._setup_action_clients()
        self._setup_publishers()
        self._setup_subscriptions()
        self._setup_tf_and_timer()
        self._declare_parameters()
        self._setup_services()
        self._setup_csv_logger()
        self._setup_precision_align_state()
        self.get_logger().info("Task manager V4 node initialized")

    def _init_runtime_state(self) -> None:
        self.state = TaskState.INIT
        self.cargo_state = CargoState.EMPTY

        self.home_pose = None
        self.object_pose = None
        self.bin_pose = None

        self.cached_bin_poses = {}
        self._last_bin_map_color = ""
        self._last_bin_vision_xyz = None

        self.explore_done_flag = False
        self.explore_finished_received = False

        self.object_detection_count = 0
        self.bin_detection_count = 0
        self.required_detection_frames = 5

        self.grasp_retry_count = 0
        self.max_grasp_retries = 2
        self.place_retry_count = 0
        self.max_place_retries = 2

        self.object_blacklist = []
        self.bin_blacklist = []
        self.blacklist_radius = 0.3

        self.detected_object_colors = set()
        self.detected_bin_colors = set()
        self._map_fallback_round_count = 0
        self._map_fallback_max_rounds = 15
        self.interest_points = []
        self.interest_point_index = 0
        self.current_interest_point = None
        self.wait_at_point_start_time = None
        self._last_nav2_distance_remaining_m = None

        self.arm_status = "idle"
        self.gripper_status = "unknown"
        self._arm_cmd_sent = False
        self._place_status_at_command = "unknown"
        self._place_status_changed_after_command = False

        self._last_explore_resume = None
        self._map_coords_csv = None

    def _setup_action_clients(self) -> None:
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, "navigate_to_pose")
        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE

    def _setup_publishers(self) -> None:
        self.explore_control_pub = self.create_publisher(std_msgs.Bool, "explore/resume", 10)
        self.state_pub = self.create_publisher(std_msgs.String, "task_manager/state", 10)
        self.cargo_state_pub = self.create_publisher(
            std_msgs.String, "task_manager/cargo_state", 10
        )
        self.arm_pick_pub = self.create_publisher(
            geometry_msgs.Point, "/arm/target_pick", 10
        )
        self.arm_place_pub = self.create_publisher(
            geometry_msgs.Point, "/arm/target_place", 10
        )
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.Twist, "/cmd_vel", 10)

    def _setup_subscriptions(self) -> None:
        self.arm_status_sub = self.create_subscription(
            std_msgs.String, "/arm/status", self._arm_status_callback, 10
        )
        self.arm_gripper_status_sub = self.create_subscription(
            std_msgs.String, "/arm/gripper_status", self._arm_gripper_status_callback, 10
        )

        for color, topic in [
            ("red", "/target_pick/red"),
            ("green", "/target_pick/green"),
            ("blue", "/target_pick/blue"),
        ]:
            self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color: self._object_point_callback(msg, c),
                qos_profile_sensor_data,
            )

        for color, topic in [
            ("red", "/target_place/red"),
            ("green", "/target_place/green"),
            ("blue", "/target_place/blue"),
        ]:
            self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color: self._bin_point_callback(msg, c),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            std_msgs.Bool, "explore/finished", self._explore_finished_callback, 10
        )

    def _setup_tf_and_timer(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.state_timer = self.create_timer(0.1, self._state_timer_callback)

    def _declare_parameters(self) -> None:
        self.declare_parameter("pregrasp_distance", 0.5)
        self.declare_parameter("interest_point_standoff_m", 0.30)
        self.declare_parameter("interest_point_max_bbox_m", 0.40)
        self.declare_parameter("interest_point_dedupe_min_separation_px", 6.0)
        self.declare_parameter("interest_point_vision_trigger_distance_m", 1.0)
        self.declare_parameter("preplace_distance", 0.6)
        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("maps_directory", "")
        self.declare_parameter("map_save_basename", "explore_complete")
        self.declare_parameter("map_resolution", DEFAULT_RESOLUTION)
        self.declare_parameter("map_origin_x", DEFAULT_ORIGIN[0])
        self.declare_parameter("map_origin_y", DEFAULT_ORIGIN[1])
        self.declare_parameter("wait_at_interest_point_sec", 15.0)

        self.declare_parameter("docking_linear_speed_mps", 0.08)
        self.declare_parameter("docking_angular_speed_max_rps", 0.25)

        # Visual docking (object)
        self.declare_parameter("visual_docking_x_kp", 1.5)
        self.declare_parameter("visual_docking_z_kp", 1.0)
        self.declare_parameter("visual_docking_x_tolerance_m", 0.05)

        # Grasp decision threshold (distance in camera z)
        self.declare_parameter("grasp_target_camera_z_m", 0.265)
        self.declare_parameter("grasp_target_camera_z_tolerance_m", 0.01)
        self.declare_parameter("place_trigger_camera_z_m", 0.36)
        self.declare_parameter("place_trigger_camera_z_tolerance", 0.005)
        self.declare_parameter("forward_before_place_distance_m", 0.10)
        self.declare_parameter("forward_before_place_speed_mps", 0.10)
        self.declare_parameter("forward_before_place_stop_hold_sec", 0.2)
        self.declare_parameter("fixed_place_target_x", 0.0)
        self.declare_parameter("fixed_place_target_y", 0.0)
        self.declare_parameter("fixed_place_target_z", 0.275)

        self.declare_parameter("backup_distance_m", 0.20)
        self.declare_parameter("pre_explore_spin_enable", True)
        self.declare_parameter("pre_explore_nav_offset_x_m", 0.3)
        self.declare_parameter("pre_explore_nav_offset_y_m", 0.0)
        self.declare_parameter("detection_map_coordinates_csv_enable", True)
        self.declare_parameter("detection_map_coordinates_csv_path", "")

    def _setup_services(self) -> None:
        self.state_service = self.create_service(
            Trigger,
            "task_manager/get_state",
            self._handle_get_state_service,
        )

    def _setup_csv_logger(self) -> None:
        if not self.get_parameter("detection_map_coordinates_csv_enable").value:
            return

        csv_path = self.get_parameter("detection_map_coordinates_csv_path").value
        if not csv_path:
            csv_path = os.path.join(self._get_maps_directory(), "detection_map_coordinates.csv")

        try:
            self._map_coords_csv = DetectionMapCoordinatesCsvLogger(csv_path)
            self.get_logger().info(f"Detection map coordinates CSV: {csv_path}")
        except OSError as exc:
            self.get_logger().warn(f"Could not open detection map CSV ({csv_path}): {exc}")

    def _setup_precision_align_state(self) -> None:
        self._precision_align_source_purpose = NavPurpose.NONE
        self._precision_align_next_state = None

        self._visual_docking_active = False
        self._visual_docking_last_point = None
        self._visual_docking_start_time = None
        self._forward_stop_hold_deadline = None
        self._forward_deadline = None
        self._forward_start_time = None
        self._forward_speed_mps = 0.0
        self._forward_distance_m = 0.0

        self._backup_end_time = None
        self._backup_next_state = None
        self._backup_after_restore_explore_resume = None

    def _handle_get_state_service(self, request, response):
        response.success = True
        response.message = f"state={self.state.value}, cargo={self.cargo_state.value}"
        return response

    def dispatch(self, event: TaskEvent) -> None:
        if isinstance(event, Nav2GoalResponseEvent):
            self._apply_nav2_goal_response(event.future)
            return
        if isinstance(event, Nav2ResultEvent):
            self._apply_nav2_result(event.future)
            return
        if isinstance(event, TickEvent):
            self._publish_state_topics()
            self._handle_tick_by_state()
            return
        if isinstance(event, ExploreFinishedEvent):
            self._handle_explore_finished_dispatch()
            return
        if isinstance(event, ObjectVisionEvent):
            self._handle_object_vision_by_state(event)
            return
        if isinstance(event, BinVisionEvent):
            self._handle_bin_vision_by_state(event)
            return

    def _publish_state_topics(self) -> None:
        state_msg = std_msgs.String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        cargo_msg = std_msgs.String()
        cargo_msg.data = self.cargo_state.value
        self.cargo_state_pub.publish(cargo_msg)

    def _handle_tick_by_state(self) -> None:
        if self.state == TaskState.INIT:
            self._handle_init_state()
        elif self.state == TaskState.GRASP and self._arm_cmd_sent:
            self._handle_grasp_arm_result()
        elif self.state == TaskState.PLACE_IN_BIN and self._arm_cmd_sent:
            self._handle_place_arm_result()
        elif self.state == TaskState.PRECISION_ALIGN:
            self._precision_align_control_step()
            self._handle_precision_align_timeout_if_needed()
        elif self.state == TaskState.FORWARD_BEFORE_PLACE:
            self._forward_before_place_control_step()
        elif self.state == TaskState.BACKUP_AFTER_ACTION:
            self._backup_control_step()

    def _state_timer_callback(self) -> None:
        self.dispatch(TickEvent())


def main(args=None):
    rclpy.init(args=args)
    startup_logger = rclpy.logging.get_logger("task_manager_v4")
    startup_logger.info("Startup delay enabled: countdown 30s before node starts.")
    for remaining in range(10, 0, -1):
        startup_logger.info(f"Node starts in {remaining:02d}s...")
        time.sleep(1.0)
    node = TaskManagerNodeV4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

