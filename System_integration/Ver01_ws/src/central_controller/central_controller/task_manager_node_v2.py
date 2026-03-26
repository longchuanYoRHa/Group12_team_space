#!/usr/bin/env python3
"""
Task manager state machine node V2 (topic-driven).

基于 task_manager_node.py 的 V1 版本重构：
- 使用视觉话题回调作为主要驱动（完全 topic 驱动）
- 在 object/bin 话题回调中直接完成：暂停/恢复探索、发送 Nav2 目标、设置 done callback 完成状态转换
- 增加 PRECISION_ALIGN：在 Nav2 到达预位后，视觉触发关闭 local inflation 并用 /cmd_vel 做慢速对位
- 探索结束后的地图检测与兴趣点导航逻辑保持不变，但移动到 explore_finished 回调里

探索节点：与 `custom_explore` 包的 `custom_explore_node` 配合（starter_launch 中启动）。
- 发布 `explore/resume`（Bool）控制开始/暂停；默认节点侧等待 resume=true 后才开始 frontier 探索。
- 订阅 `explore/finished`（Bool）获知探索结束。

启动流程：INIT 就绪后进入 `PRE_EXPLORE_SPIN`：发送 Nav2 目标——在 **map** 系下相对起点沿 **+x 偏移 0.3 m、y=0**，姿态为**朝向 −x**
（yaw=π）。期间视觉检测到箱子仍只缓存到 `cached_bin_poses`（逻辑同前）；导航结束后 `explore/resume=True` 进入 EXPLORE。
"""

import math
import os
import sys
import subprocess
import time
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
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter
try:
    from opennav_docking_msgs.action import DockRobot  # pyright: ignore[reportMissingImports]
except Exception:  # 运行环境有该包；静态检查环境可能没有
    DockRobot = None  # type: ignore

from central_controller.task_manager_utils import (
    is_pose_in_blacklist as check_pose_in_blacklist,
    compute_pregrasp_pose,
    quat_yaw,
    normalize_angle,
    quaternion_from_yaw,
)
from central_controller.detect_objects_in_pgm_map import (
    get_interest_points_from_pgm,
    DEFAULT_RESOLUTION,
    DEFAULT_ORIGIN,
)
from central_controller.detection_map_coordinates_csv import DetectionMapCoordinatesCsvLogger


class CargoState(Enum):
    EMPTY = "empty"
    HAS_OBJECT = "has_object"


class TaskState(Enum):
    INIT = "init"
    # 启动探索前：Nav2 预导航位（map +x 偏移、朝 −x），再进入 EXPLORE
    PRE_EXPLORE_SPIN = "pre_explore_spin"
    EXPLORE = "explore"
    NAV_TO_OBJECT_PREGRASP = "nav_to_object_pregrasp"
    PRECISION_ALIGN = "precision_align"
    GRASP = "grasp"
    RESUME_EXPLORE_FOR_BIN = "resume_explore_for_bin"
    NAV_TO_BIN_PREPLACE = "nav_to_bin_preplace"
    PLACE_IN_BIN = "place_in_bin"
    BACKUP_AFTER_ACTION = "backup_after_action"
    POST_ACTION = "post_action"
    EXPLORE_FINISHED_FALLBACK = "explore_finished_fallback"
    RUN_MAP_DETECTION = "run_map_detection"
    NAV_TO_INTEREST_POINT = "nav_to_interest_point"
    WAIT_AT_INTEREST_POINT = "wait_at_interest_point"


class NavPurpose(Enum):
    NONE = "none"
    PRE_EXPLORE_NAV = "pre_explore_nav"
    OBJECT_PREGRASP = "object_pregrasp"
    BIN_PREPLACE = "bin_preplace"
    INTEREST_POINT = "interest_point"


class TaskManagerNodeV2(Node):
    """
    V2 任务管理器：核心控制节点，使用话题驱动状态机。

    探索由 `custom_explore/custom_explore_node` 执行，本节点仅通过 `explore/resume` 与
    `explore/finished` 与之交互（与原先 explore_lite 话题接口一致）。
    """

    def __init__(self):
        super().__init__('task_manager_v2')

        # ========== 状态变量 ==========
        self.current_state = TaskState.INIT
        self.cargo_state = CargoState.EMPTY
        self.home_pose = None
        self.object_pose = None
        self.bin_pose = None

        # 在 EXPLORE / 启动前旋转扫描阶段优先发现 bin 时缓存，按颜色存储
        self.cached_bin_poses = {}  # color -> PoseStamped
        self._last_bin_map_color = ''
        self._last_bin_vision_xyz = None  # (x,y,z) camera frame，供 CSV 与 place 对齐

        # 探索结束标志（完成地图存储与兴趣点检测后置 True）
        self.explore_done_flag = False

        # ========== 检测稳定性计数 ==========
        self.object_detection_count = 0
        self.bin_detection_count = 0
        self.required_detection_frames = 5

        # ========== 重试计数 ==========
        self.grasp_retry_count = 0
        self.max_grasp_retries = 2
        self.place_retry_count = 0
        self.max_place_retries = 2

        # ========== 黑名单 ==========
        self.object_blacklist = []
        self.bin_blacklist = []
        self.blacklist_radius = 0.3

        # ========== 探索结束回退：地图兴趣点 ==========
        self.detected_object_colors = set()
        self.detected_bin_colors = set()
        self.explore_finished_received = False
        self._map_fallback_round_count = 0
        self._map_fallback_max_rounds = 15
        self.interest_points = []
        self.interest_point_index = 0
        self.current_interest_point = None
        self.wait_at_point_start_time = None

        # ========== 动作客户端 ==========
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE

        # Docking server action client（Nav2 Docking Server / opennav_docking）
        self.declare_parameter('dock_action_name', 'dock_robot')
        self.declare_parameter('dock_type', 'simple_charging_dock')
        self.dock_client = None
        if DockRobot is None:
            self.get_logger().warn(
                'DockRobot action type not importable in this environment. '
                'Docking via DockRobot will be unavailable.'
            )
        else:
            self.dock_client = ActionClient(
                self, DockRobot, self.get_parameter('dock_action_name').value
            )
        self._dock_goal_handle = None
        self._dock_result_future = None

        # ========== 发布者 ==========
        # 与 custom_explore_node 约定：True 开始/恢复探索，False 暂停（节点会 cancel nav2 goals）
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, 'explore/resume', 10
        )
        self._last_explore_resume = None
        self.state_pub = self.create_publisher(std_msgs.String, 'task_manager/state', 10)
        self.cargo_state_pub = self.create_publisher(std_msgs.String, 'task_manager/cargo_state', 10)

        # ========== 机械臂：与 central_controller/task_manager 一致 ==========
        # 状态变量
        self.arm_status = "idle"
        self.gripper_status = "unknown"
        self._arm_cmd_sent = False
        # 发布抓取/放置目标点（base_link 下毫米，供 mycobot 等使用）
        self.arm_pick_pub = self.create_publisher(
            geometry_msgs.Point, '/arm/target_pick', 10
        )
        self.arm_place_pub = self.create_publisher(
            geometry_msgs.Point, '/arm/target_place', 10
        )
        # 订阅机械臂状态（来自 manipulator 节点）
        self.arm_status_sub = self.create_subscription(
            std_msgs.String, '/arm/status', self._arm_status_callback, 10
        )
        self.arm_gripper_status_sub = self.create_subscription(
            std_msgs.String, '/arm/gripper_status', self._arm_gripper_status_callback, 10
        )

        # ========== 订阅 vision 话题（6 个） ==========
        for color, topic in [('red', '/target_pick/red'), ('green', '/target_pick/green'), ('blue', '/target_pick/blue')]:
            self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color: self._object_point_callback(msg, c),
                qos_profile_sensor_data,
            )
        for color, topic in [('red', '/target_place/red'), ('green', '/target_place/green'), ('blue', '/target_place/blue')]:
            self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color: self._bin_point_callback(msg, c),
                qos_profile_sensor_data,
            )

        # ========== 探索结束（custom_explore_node） ==========
        self.create_subscription(
            std_msgs.Bool, 'explore/finished', self._explore_finished_callback, 10
        )

        # ========== TF ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ========== 定时器：发布状态、处理异步状态（对位/后退/超时） ==========
        self.state_timer = self.create_timer(0.1, self._state_timer_callback)

        # ========== 参数 ==========
        self.declare_parameter('pregrasp_distance', 0.5)
        self.declare_parameter('preplace_distance', 0.6)
        self.declare_parameter('camera_frame_id', 'D435i_camera_link') # change to 'D435i_camera_link' for simulation
        self.declare_parameter('maps_directory', '')
        self.declare_parameter('map_save_basename', 'explore_complete')
        self.declare_parameter('map_resolution', DEFAULT_RESOLUTION)
        self.declare_parameter('map_origin_x', DEFAULT_ORIGIN[0])
        self.declare_parameter('map_origin_y', DEFAULT_ORIGIN[1])
        self.declare_parameter('wait_at_interest_point_sec', 15.0)
        # 精确对位（diffdrive docking）参数
        self.declare_parameter('docking_linear_speed_mps', 0.005)  # 0.5 cm/s
        self.declare_parameter('docking_angular_speed_max_rps', 0.25)
        self.declare_parameter('docking_yaw_kp', 1.5)
        self.declare_parameter('docking_y_tolerance_m', 0.01)
        self.declare_parameter('docking_stop_distance_m', 0.20)
        self.declare_parameter('backup_distance_m', 0.20)
        # local costmap inflation 临时关闭（通过参数写入）
        self.declare_parameter('local_costmap_node_fqn', '/local_costmap/local_costmap')
        self.declare_parameter('local_inflation_radius_off', 0.0)
        # 进入 frontier 探索前：Nav2 到 map 下 (home+Δx, home.y)、朝 −x（yaw=π）
        self.declare_parameter('pre_explore_spin_enable', True)
        self.declare_parameter('pre_explore_nav_offset_x_m', 0.3)
        self.declare_parameter('pre_explore_nav_offset_y_m', 0.0)
        # 将 object/bin map 坐标与 PGM 兴趣点写入 CSV（路径空则使用 maps_directory 下默认文件名）
        self.declare_parameter('detection_map_coordinates_csv_enable', True)
        self.declare_parameter('detection_map_coordinates_csv_path', '')

        # ========== Service：查询当前状态 ==========
        self.state_service = self.create_service(
            Trigger,
            'task_manager/get_state',
            self._handle_get_state_service,
        )

        # ========== Service client：reset odometry（Leo Rover firmware） ==========
        self._reset_odom_client = self.create_client(Trigger, '/reset_odometry')
        self._reset_odom_after_nav2_done = False
        self._reset_odom_after_nav2_started_at = None  # rclpy.time.Time
        self._reset_odom_after_nav2_future = None
        self._reset_odom_after_nav2_last_warn_sec = 0.0

        self.get_logger().info('Task manager V2 node initialized')

        # ========== map 坐标 CSV（object / bin / PGM）==========
        self._map_coords_csv = None
        if self.get_parameter('detection_map_coordinates_csv_enable').value:
            csv_path = self.get_parameter('detection_map_coordinates_csv_path').value
            if not csv_path:
                csv_path = os.path.join(
                    self._get_maps_directory(), 'detection_map_coordinates.csv'
                )
            try:
                self._map_coords_csv = DetectionMapCoordinatesCsvLogger(csv_path)
                self.get_logger().info(f'Detection map coordinates CSV: {csv_path}')
            except OSError as e:
                self.get_logger().warn(f'Could not open detection map CSV ({csv_path}): {e}')

        # ========== cmd_vel & costmap 参数客户端 ==========
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.Twist, '/cmd_vel', 10)
        self._param_client_local_costmap = AsyncParameterClient(
            self, self.get_parameter('local_costmap_node_fqn').value
        )
        self._local_inflation_radius_saved = None
        self._local_inflation_temporarily_off = False

        # ========== 精确对位状态变量 ==========
        self._precision_align_source_purpose = NavPurpose.NONE
        self._precision_align_next_state = None  # TaskState
        self._docking_active = False
        self._docking_phase = 'rotate'  # 'rotate' -> 'drive'
        self._last_docking_target_base_m = None  # (x, y) in meters (base_link)
        self._dock_goal_sent = False
        self._backup_end_time = None
        self._backup_next_state = None  # TaskState
        self._backup_after_restore_explore_resume = None  # bool | None

    def _handle_get_state_service(self, request, response):
        """
        service: task_manager/get_state (std_srvs/Trigger)
        - success: True
        - message: 当前 TaskState.value 和 CargoState.value
        """
        response.success = True
        response.message = f'state={self.current_state.value}, cargo={self.cargo_state.value}'
        return response

    def _arm_status_callback(self, msg: std_msgs.String):
        self.arm_status = msg.data.lower()

    def _arm_gripper_status_callback(self, msg: std_msgs.String):
        self.gripper_status = msg.data.lower()

    def _get_point_in_base_link_mm(self, pose_stamped: geometry_msgs.PoseStamped):
        """将位姿变换到 base_link 并转为毫米，供机械臂控制器使用。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
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
        except Exception as e:
            self.get_logger().error(f'TF transform to base_link failed: {e}')
            return None

    def _point_camera_to_base_link_mm(self, point_msg: geometry_msgs.Point):
        """
        将相机坐标系下的点变换到 base_link 并转为毫米。
        抓取时使用：仅依赖 camera→base_link 的 static TF，不经过 map。
        """
        frame_id = self.get_parameter('camera_frame_id').value
        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = point_msg
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            point_in_base = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            pt = geometry_msgs.Point()
            pt.x = point_in_base.point.x * 1000.0
            pt.y = point_in_base.point.y * 1000.0
            pt.z = point_in_base.point.z * 1000.0
            return pt
        except Exception as e:
            self.get_logger().error(f'TF camera->base_link failed: {e}')
            return None

    def _publish_explore_resume_if_changed(self, resume: bool):
        if self._last_explore_resume is not None and self._last_explore_resume == resume:
            return
        self._last_explore_resume = resume
        msg = std_msgs.Bool()
        msg.data = resume
        self.explore_control_pub.publish(msg)

    def _point_to_pose_stamped_in_map(self, point_msg: geometry_msgs.Point):
        frame_id = self.get_parameter('camera_frame_id').value
        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = point_msg
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            point_in_map = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            pose_stamped = geometry_msgs.PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position = point_in_map.point
            pose_stamped.pose.orientation.w = 1.0
            return pose_stamped
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}, using raw coords (assumed map)')
            pose_stamped = geometry_msgs.PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position = point_msg
            pose_stamped.pose.orientation.w = 1.0
            return pose_stamped

    def _point_to_pose_stamped_in_frame(self, point_msg: geometry_msgs.Point, target_frame: str):
        """
        将相机坐标系下的点变换到 target_frame，并返回 PoseStamped（orientation 置为单位四元数）。
        """
        frame_id = self.get_parameter('camera_frame_id').value
        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = point_msg
        transform = self.tf_buffer.lookup_transform(
            target_frame, frame_id, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        point_in_target = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.frame_id = target_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = point_in_target.point
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def _get_robot_xy_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception:
            return (0.0, 0.0)

    def _get_robot_xy_in_frame(self, target_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception:
            return (0.0, 0.0)

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
        self.get_logger().info(f'Sending Nav2 goal for {purpose.value}')
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            if (
                self.current_nav_purpose == NavPurpose.PRE_EXPLORE_NAV
                and self.current_state == TaskState.PRE_EXPLORE_SPIN
            ):
                self.get_logger().warn(
                    'PRE_EXPLORE_NAV rejected; starting exploration without pre-navigation.'
                )
                self.current_state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            self.nav2_goal_handle = None
            self.current_nav_purpose = NavPurpose.NONE
            return

        self.get_logger().info('Nav2 goal accepted')
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        goal_handle = future.result()
        status = goal_handle.status
        purpose = self.current_nav_purpose

        if purpose == NavPurpose.PRE_EXPLORE_NAV:
            keys = list(self.cached_bin_poses.keys())
            if self.current_state == TaskState.PRE_EXPLORE_SPIN:
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(
                        f'PRE_EXPLORE_NAV succeeded. Cached bin colors: {keys}. '
                        'Starting frontier exploration.'
                    )
                else:
                    self.get_logger().warn(
                        f'PRE_EXPLORE_NAV finished with status={status}; '
                        f'cached bin colors: {keys}. Starting exploration anyway.'
                    )
                self.current_state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            self.nav2_goal_handle = None
            self.current_nav_purpose = NavPurpose.NONE
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Nav2 goal succeeded ({purpose.value})')
            if purpose == NavPurpose.OBJECT_PREGRASP:
                # 到达物体预抓取位 -> 进入精确对位，视觉触发对位后再进入 GRASP
                self._enter_precision_align(purpose, next_state_after_align=TaskState.GRASP)
            elif purpose == NavPurpose.BIN_PREPLACE:
                # 到达 bin 预放置位 -> 进入精确对位，视觉触发对位后再进入 PLACE_IN_BIN
                self._enter_precision_align(purpose, next_state_after_align=TaskState.PLACE_IN_BIN)
            elif purpose == NavPurpose.INTEREST_POINT:
                # 到达兴趣点 -> 进入精确对位状态（替代 WAIT_AT_INTEREST_POINT 等待）
                self._enter_precision_align(purpose, next_state_after_align=None)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'Nav2 goal aborted ({purpose.value})')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Nav2 goal canceled ({purpose.value})')

        # 导航结束后清空 handle，但保留状态机状态
        self.nav2_goal_handle = None
        self.current_nav_purpose = NavPurpose.NONE

    def adjust_nav2_for_carry_mode(self, enable: bool):
        if enable:
            self.get_logger().info('Carry mode on: reduced speed, larger inflation radius')
        else:
            self.get_logger().info('Carry mode off: normal parameters restored')

    def _handle_grasp_arm_result(self):
        """
        在 GRASP 状态且已发送抓取命令后，根据 /arm/status、/arm/gripper_status 做状态转换。
        与 central_controller task_manager 的 handle_grasp_state 结果判断一致。
        """
        if self.arm_status == 'holding' and self.gripper_status == 'object_held':
            self.get_logger().info('Grasp succeeded!')
            self.cargo_state = CargoState.HAS_OBJECT
            self.grasp_retry_count = 0
            self.adjust_nav2_for_carry_mode(True)
            self._arm_cmd_sent = False
            self._start_backup_after_action(
                next_state=TaskState.RESUME_EXPLORE_FOR_BIN,
                explore_resume_after_restore=True,
            )
            return

        if self.arm_status == 'error' or (self.arm_status == 'idle' and self._arm_cmd_sent):
            self.grasp_retry_count += 1
            self._arm_cmd_sent = False
            if self.grasp_retry_count >= self.max_grasp_retries:
                self.get_logger().warn('Grasp failed, max retries reached, abandoning object')
                if self.object_pose:
                    self.object_blacklist.append(self.object_pose.pose.position)
                self.grasp_retry_count = 0
                self.current_state = TaskState.EXPLORE
                self._publish_explore_resume_if_changed(True)
            else:
                self.get_logger().info(
                    f'Grasp failed, retrying ({self.grasp_retry_count}/{self.max_grasp_retries})'
                )

    def _handle_place_arm_result(self):
        """
        在 PLACE_IN_BIN 状态且已发送放置命令后，根据 /arm/status 做状态转换。
        与 central_controller task_manager 的 handle_place_in_bin_state 结果判断一致。
        """
        if self.arm_status == 'idle':
            self.get_logger().info('Place succeeded!')
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

        if self.arm_status == 'error':
            self.place_retry_count += 1
            self._arm_cmd_sent = False
            if self.place_retry_count >= self.max_place_retries:
                self.get_logger().warn('Place failed, max retries reached, resuming explore for bin')
                self.place_retry_count = 0
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
                self._publish_explore_resume_if_changed(True)
            else:
                self.get_logger().info(
                    f'Place failed, retrying ({self.place_retry_count}/{self.max_place_retries})'
                )

    # ========================= INIT & 状态定时器 =========================

    def _handle_init_state(self):
        # 等待 Nav2 server 就绪并保存 home；可选先发 PRE_EXPLORE Nav2 再 explore
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Nav2 server...')
            return

        # 在 INIT：等待 Nav2 后重置一次 odom（/reset_odometry），再继续保存 home / 进入探索
        if not self._reset_odom_after_nav2_done:
            now = self.get_clock().now()
            if self._reset_odom_after_nav2_started_at is None:
                self._reset_odom_after_nav2_started_at = now

            elapsed = (now - self._reset_odom_after_nav2_started_at).nanoseconds / 1e9
            if elapsed > 8.0:
                self.get_logger().warn(
                    'INIT: /reset_odometry not completed within 8s; continue without odom reset.'
                )
                self._reset_odom_after_nav2_done = True
            elif self._reset_odom_after_nav2_future is not None:
                if self._reset_odom_after_nav2_future.done():
                    try:
                        resp = self._reset_odom_after_nav2_future.result()
                        if resp is not None and getattr(resp, 'success', False):
                            self.get_logger().info(
                                f'INIT: /reset_odometry success: {resp.message}'
                            )
                        else:
                            msg = '' if resp is None else getattr(resp, 'message', '')
                            self.get_logger().warn(
                                f'INIT: /reset_odometry returned failure: {msg}'
                            )
                    except Exception as e:
                        self.get_logger().warn(f'INIT: /reset_odometry call failed: {e}')
                    self._reset_odom_after_nav2_done = True
                else:
                    return
            else:
                if not self._reset_odom_client.service_is_ready():
                    sec = now.nanoseconds / 1e9
                    if sec - self._reset_odom_after_nav2_last_warn_sec >= 1.0:
                        self.get_logger().warn('INIT: waiting for /reset_odometry service...')
                        self._reset_odom_after_nav2_last_warn_sec = sec
                    return

                self.get_logger().info('INIT: calling /reset_odometry ...')
                self._reset_odom_after_nav2_future = self._reset_odom_client.call_async(
                    Trigger.Request()
                )
                return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            self.home_pose = geometry_msgs.PoseStamped()
            self.home_pose.header.frame_id = 'map'
            self.home_pose.pose.position.x = transform.transform.translation.x
            self.home_pose.pose.position.y = transform.transform.translation.y
            self.home_pose.pose.orientation = transform.transform.rotation
            self.get_logger().info('System ready, home pose saved')

            if self.get_parameter('pre_explore_spin_enable').value:
                ox = float(self.get_parameter('pre_explore_nav_offset_x_m').value)
                oy = float(self.get_parameter('pre_explore_nav_offset_y_m').value)
                stamp = self.get_clock().now().to_msg()
                goal = geometry_msgs.PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = stamp
                goal.pose.position.x = self.home_pose.pose.position.x + ox
                goal.pose.position.y = self.home_pose.pose.position.y + oy
                goal.pose.position.z = self.home_pose.pose.position.z
                # 朝向 map −x：yaw = π
                goal.pose.orientation = quaternion_from_yaw(math.pi)
                self.current_state = TaskState.PRE_EXPLORE_SPIN
                self.get_logger().info(
                    f'PRE_EXPLORE_NAV: map goal ({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}), '
                    f'yaw=π (−x). Offsets Δx={ox:.3f}, Δy={oy:.3f} m from home.'
                )
                self._send_nav_goal(goal, NavPurpose.PRE_EXPLORE_NAV)
                return

            self.current_state = TaskState.EXPLORE
            self._publish_explore_resume_if_changed(True)
        except Exception as e:
            self.get_logger().warn(f'Waiting for TF: {e}')

    def _state_timer_callback(self):
        # 发布当前状态和载荷状态
        state_msg = std_msgs.String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)
        cargo_msg = std_msgs.String()
        cargo_msg.data = self.cargo_state.value
        self.cargo_state_pub.publish(cargo_msg)

        # 在 INIT 阶段驱动一次性初始化
        if self.current_state == TaskState.INIT:
            self._handle_init_state()

        # GRASP：已发送抓取命令后，根据 /arm/status、/arm/gripper_status 判断成功/失败
        if self.current_state == TaskState.GRASP and self._arm_cmd_sent:
            self._handle_grasp_arm_result()

        # PLACE_IN_BIN：已发送放置命令后，根据 /arm/status 判断成功/失败
        if self.current_state == TaskState.PLACE_IN_BIN and self._arm_cmd_sent:
            self._handle_place_arm_result()

        # PRECISION_ALIGN：对位控制循环 & 超时等待（兴趣点）
        if self.current_state == TaskState.PRECISION_ALIGN:
            self._precision_align_control_step()
            self._handle_precision_align_timeout_if_needed()

        # BACKUP_AFTER_ACTION：后退 20cm，结束后恢复 costmap 并进入下一状态
        if self.current_state == TaskState.BACKUP_AFTER_ACTION:
            self._backup_control_step()

        # WAIT_AT_INTEREST_POINT 的超时逻辑
        if self.current_state == TaskState.WAIT_AT_INTEREST_POINT:
            self._handle_wait_at_interest_point_timeout()

    # ========================= vision 话题回调：object （pick） =========================

    def _object_point_callback(self, msg: geometry_msgs.Point, color: str):
        """
        统一处理三个 /target_pick/* 话题。视觉点为 camera frame。
        根据当前状态决定行为：
        - NAV_TO_OBJECT_PREGRASP：不更新、不重发 goal，防止重复发送
        - EXPLORE 或 WAIT_AT_INTEREST_POINT：稳定检测 -> 物体转 map、算进近点并导航（camera→map）
        - GRASP：用当前视觉点直接 camera→base_link 发抓取目标，不经过 map
        """
        self.detected_object_colors.add(color)

        # 只有空载时才考虑新的抓取目标
        if self.cargo_state != CargoState.EMPTY:
            self.object_detection_count = 0
            return

        # 精确对位状态：由视觉触发 docking（不再做地图导航）
        if self.current_state == TaskState.PRECISION_ALIGN:
            self._handle_precision_align_vision(point_msg=msg, is_object=True)
            return

        # 已在前往预抓取位时不再更新目标，防止重复发送 goal
        if self.current_state == TaskState.NAV_TO_OBJECT_PREGRASP:
            return

        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing target_pick message: {e}')
            self.object_detection_count = 0
            return

        # 黑名单过滤
        if check_pose_in_blacklist(pose_stamped.pose.position, self.object_blacklist, self.blacklist_radius):
            return

        self.object_pose = pose_stamped

        # 如果当前处于 GRASP 状态：用当前视觉点 camera→base_link 直接算抓取目标（不经过 map）
        if self.current_state == TaskState.GRASP:
            self._execute_grasp_with_current_object(msg, color)
            return

        # 只在探索或兴趣点等待时用检测计数来触发导航
        if self.current_state not in (TaskState.EXPLORE, TaskState.WAIT_AT_INTEREST_POINT):
            self.object_detection_count = 0
            return

        self.object_detection_count += 1
        if self.object_detection_count < self.required_detection_frames:
            return

        # 检测稳定：重置计数并开始导航到预抓取位
        self.object_detection_count = 0

        if self._map_coords_csv:
            self._map_coords_csv.log_object_map_nav(
                color,
                msg.x,
                msg.y,
                msg.z,
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                self.current_state.name,
            )

        # EXPLORE 阶段：在回调中直接完成 PAUSE_EXPLORE 的功能（停止探索 & 取消导航）
        if self.current_state == TaskState.EXPLORE:
            self.get_logger().info(
                f'Object found during EXPLORE, coords=({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f}), stopping explore and nav to pregrasp.'
            )
            self._publish_explore_resume_if_changed(False)
            self._cancel_nav2_goal_if_any()
        else:
            self.get_logger().info(
                f'Object found during WAIT_AT_INTEREST_POINT, coords=({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f}), nav to pregrasp.'
            )

        # 计算并发送预抓取导航目标（在“指向目标”基础上转 180°，与底盘/相机实际朝前一致）
        robot_x, robot_y = self._get_robot_xy_in_map()
        pregrasp_distance = self.get_parameter('pregrasp_distance').value
        goal_pose = compute_pregrasp_pose(
            self.object_pose, pregrasp_distance, robot_x, robot_y,
            frame_id='map', stamp=self.get_clock().now().to_msg(),
            yaw_offset=math.pi,
        )
        self.current_state = TaskState.NAV_TO_OBJECT_PREGRASP
        self._send_nav_goal(goal_pose, NavPurpose.OBJECT_PREGRASP)

    def _execute_grasp_with_current_object(self, point_msg: geometry_msgs.Point, color: str):
        """
        在 GRASP 状态下，由 object 话题回调触发。
        使用当前视觉点（camera frame）直接变换到 base_link（毫米），不经过 map；
        camera→base_link 与机械臂基座→base_link 均为 static TF。
        仅发送一次抓取目标到 /arm/target_pick；成功/失败由定时器根据 /arm/status、/arm/gripper_status 判断。
        """
        if self._arm_cmd_sent:
            return
        target_pt = self._point_camera_to_base_link_mm(point_msg)
        if target_pt is None:
            self.get_logger().warn('Grasp: camera->base_link failed, skipping this cycle')
            return
        if self._map_coords_csv:
            mx = my = None
            if self.object_pose is not None:
                mx = self.object_pose.pose.position.x
                my = self.object_pose.pose.position.y
            self._map_coords_csv.log_object_pick_arm(
                color, point_msg.x, point_msg.y, point_msg.z, mx, my
            )
        self.get_logger().info('Sending pick target to manipulator (/arm/target_pick)...')
        self.arm_pick_pub.publish(target_pt)
        self._arm_cmd_sent = True

    # ========================= vision 话题回调：bin （place） =========================

    def _bin_point_callback(self, msg: geometry_msgs.Point, color: str):
        """
        统一处理三个 /target_place/* 话题。
        行为：
        - EXPLORE 阶段：先发现 bin 则仅缓存位姿（按颜色），以备后续抓取对应颜色物体后直接导航
        - RESUME_EXPLORE_FOR_BIN 或 WAIT_AT_INTEREST_POINT：稳定检测到 bin，直接导航到预放置位
        - PLACE_IN_BIN：使用最新视觉信息执行放置动作
        """
        self.detected_bin_colors.add(color)

        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing target_place message: {e}')
            self.bin_detection_count = 0
            return

        # black list 过滤
        if check_pose_in_blacklist(pose_stamped.pose.position, self.bin_blacklist, self.blacklist_radius):
            return

        self.bin_pose = pose_stamped
        self._last_bin_map_color = color
        self._last_bin_vision_xyz = (msg.x, msg.y, msg.z)

        # EXPLORE / 启动前旋转扫描：先发现 bin 则只缓存，不导航
        if (
            self.current_state in (TaskState.EXPLORE, TaskState.PRE_EXPLORE_SPIN)
            and self.cargo_state == CargoState.EMPTY
        ):
            self.cached_bin_poses[color] = pose_stamped
            phase = 'PRE_EXPLORE_SPIN' if self.current_state == TaskState.PRE_EXPLORE_SPIN else 'EXPLORE'
            self.get_logger().info(
                f'Bin detected during {phase} for color {color}, cached for later use.'
            )
            if self._map_coords_csv:
                self._map_coords_csv.log_bin_map_cached(
                    color,
                    msg.x,
                    msg.y,
                    msg.z,
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    self.current_state.name,
                )
            return

        # PLACE_IN_BIN 状态：使用视觉信息执行放置（不再导航）
        if self.current_state == TaskState.PLACE_IN_BIN and self.cargo_state == CargoState.HAS_OBJECT:
            self._execute_place_with_current_bin()
            return

        # 精确对位状态：由视觉触发 docking（不再做地图导航）
        if self.current_state == TaskState.PRECISION_ALIGN:
            self._handle_precision_align_vision(point_msg=msg, is_object=False)
            return

        # 只有载有物体时才会去 bin
        if self.cargo_state != CargoState.HAS_OBJECT:
            self.bin_detection_count = 0
            return

        # 仅在找 bin 的阶段或兴趣点等待时触发导航
        if self.current_state not in (TaskState.RESUME_EXPLORE_FOR_BIN, TaskState.WAIT_AT_INTEREST_POINT):
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
                self.current_state.name,
            )

        if self.current_state == TaskState.RESUME_EXPLORE_FOR_BIN:
            self.get_logger().info(
                f'Bin found during RESUME_EXPLORE_FOR_BIN, coords=({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f}), stopping explore and nav to bin preplace.'
            )
            self._publish_explore_resume_if_changed(False)
            self._cancel_nav2_goal_if_any()
        else:
            self.get_logger().info(
                f'Bin found during WAIT_AT_INTEREST_POINT, coords=({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f}), nav to bin preplace.'
            )

        robot_x, robot_y = self._get_robot_xy_in_map()
        preplace_distance = self.get_parameter('preplace_distance').value
        goal_pose = compute_pregrasp_pose(
            self.bin_pose, preplace_distance, robot_x, robot_y,
            frame_id='map', stamp=self.get_clock().now().to_msg(),
            yaw_offset=math.pi,
        )
        self.current_state = TaskState.NAV_TO_BIN_PREPLACE
        self._send_nav_goal(goal_pose, NavPurpose.BIN_PREPLACE)

    def _execute_place_with_current_bin(self):
        """
        在 PLACE_IN_BIN 状态下，由 bin 话题回调触发。
        仅发送一次放置目标到 /arm/target_place（base_link 下毫米）；
        成功/失败由定时器根据 /arm/status 异步判断并做状态转换。
        """
        if self.bin_pose is None:
            self.get_logger().error('PLACE_IN_BIN state but bin_pose is None!')
            return
        if self._arm_cmd_sent:
            return

        target_pt = self._get_point_in_base_link_mm(self.bin_pose)
        if target_pt is None:
            self.get_logger().warn('Place: cannot get target in base_link, skipping this cycle')
            return

        if self._map_coords_csv:
            p = self.bin_pose.pose.position
            vx = vy = vz = None
            if self._last_bin_vision_xyz is not None:
                vx, vy, vz = self._last_bin_vision_xyz
            self._map_coords_csv.log_bin_map_place_command(
                self._last_bin_map_color, vx, vy, vz, p.x, p.y
            )

        self.get_logger().info('Sending place target to manipulator (/arm/target_place)...')
        self.arm_place_pub.publish(target_pt)
        self._arm_cmd_sent = True

    # ========================= 探索结束 & 地图兴趣点逻辑 =========================

    def _get_maps_directory(self):
        maps_dir = self.get_parameter('maps_directory').value
        if maps_dir:
            return maps_dir
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('central_controller')
            return os.path.join(pkg_share, 'maps')
        except Exception:
            return os.path.expanduser('~/maps')

    def _explore_finished_callback(self, msg: std_msgs.Bool):
        """
        探索结束回调：
        - 逻辑与 V1 基本一致：保存地图、PGM 检测、过滤兴趣点
        - 成功后设置 explore_done_flag，并立即开始 NAV_TO_INTEREST_POINT
        """
        if not msg.data:
            return

        self.explore_finished_received = True

        # 仅在 EXPLORE 状态下触发地图保存和检测
        if self.current_state != TaskState.EXPLORE:
            return

        self.get_logger().info('Exploration finished, starting map fallback and interest point detection.')
        self._publish_explore_resume_if_changed(False)

        # 保存地图
        maps_dir = self._get_maps_directory()
        os.makedirs(maps_dir, exist_ok=True)
        basename = self.get_parameter('map_save_basename').value
        map_base = os.path.join(maps_dir, basename)
        try:
            proc = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_base],
                capture_output=True,
                timeout=15,
                text=True,
            )
            if proc.returncode != 0:
                self.get_logger().warn(
                    f'map_saver_cli returncode={proc.returncode}; stderr: {proc.stderr.strip() or "(none)"}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn('map_saver_cli timed out after 15s.')
        except FileNotFoundError:
            self.get_logger().error('ros2 or map_saver_cli not found in PATH.')
        except Exception as e:
            self.get_logger().warn(f'map_saver_cli error: {e}')

        # PGM 检测 + 兴趣点过滤
        pgm_path = os.path.join(maps_dir, basename + '.pgm')
        if not os.path.isfile(pgm_path):
            self.get_logger().error(f'PGM not found at {pgm_path}; cannot run interest point detection.')
            return

        try:
            raw_points = get_interest_points_from_pgm(
                pgm_path,
                # 自动读取同名 yaml 的 origin/resolution（PGM 不包含原点信息）
                prefer_yaml=True,
            )
        except Exception as e:
            self.get_logger().error(f'PGM detection failed: {e}; returning to EXPLORE.')
            self.current_state = TaskState.EXPLORE
            self._publish_explore_resume_if_changed(True)
            return

        if self._map_coords_csv:
            self._map_coords_csv.log_pgm_points(
                raw_points, 'pgm_raw', pgm_path=pgm_path
            )

        filtered = []
        for (mx, my) in raw_points:
            p = geometry_msgs.Point()
            p.x = mx
            p.y = my
            p.z = 0.0
            if check_pose_in_blacklist(p, self.object_blacklist, self.blacklist_radius):
                continue
            if check_pose_in_blacklist(p, self.bin_blacklist, self.blacklist_radius):
                continue
            filtered.append((mx, my))

        if self._map_coords_csv:
            self._map_coords_csv.log_pgm_points(
                filtered, 'pgm_filtered', pgm_path=pgm_path
            )

        self.interest_points = filtered
        self.interest_point_index = 0
        self.get_logger().info(f'Map detection: {len(raw_points)} points, {len(filtered)} after filtering.')

        if not self.interest_points:
            self.get_logger().info('No interest points left; exploration finished with no fallback targets.')
            return

        # 标记探索结束，并启动兴趣点导航
        self.explore_done_flag = True
        self.current_state = TaskState.NAV_TO_INTEREST_POINT
        self._nav_to_next_interest_point()

    def _nav_to_next_interest_point(self):
        if self.interest_point_index >= len(self.interest_points):
            self.get_logger().info('All interest points visited; no more fallback targets.')
            return

        mx, my = self.interest_points[self.interest_point_index]
        self.current_interest_point = (mx, my)

        target_pose = geometry_msgs.PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = mx
        target_pose.pose.position.y = my
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0

        robot_x, robot_y = self._get_robot_xy_in_map()
        pregrasp_distance = self.get_parameter('pregrasp_distance').value
        goal_pose = compute_pregrasp_pose(
            target_pose, pregrasp_distance, robot_x, robot_y,
            frame_id='map', stamp=self.get_clock().now().to_msg(),
        )

        self.get_logger().info(
            f'Nav to interest point {self.interest_point_index + 1}/{len(self.interest_points)} '
            f'at ({mx:.2f}, {my:.2f})'
        )
        self._send_nav_goal(goal_pose, NavPurpose.INTEREST_POINT)

    def _handle_wait_at_interest_point_timeout(self):
        duration = self.get_parameter('wait_at_interest_point_sec').value
        if self.wait_at_point_start_time is None:
            self.wait_at_point_start_time = time.monotonic()
            return

        elapsed = time.monotonic() - self.wait_at_point_start_time
        if elapsed < duration:
            return

        # 超时：当前兴趣点视为失败，加入黑名单，跳到下一个兴趣点
        if self.current_interest_point is not None:
            p = geometry_msgs.Point()
            p.x = self.current_interest_point[0]
            p.y = self.current_interest_point[1]
            p.z = 0.0
            self.object_blacklist.append(p)
            self.get_logger().info('No vision detection at interest point within timeout; marked as failed target.')

        self.interest_point_index += 1
        self.current_interest_point = None
        self.wait_at_point_start_time = None
        self.current_state = TaskState.NAV_TO_INTEREST_POINT
        self._nav_to_next_interest_point()

    # ========================= 精确对位（diffdrive docking） =========================

    def _enter_precision_align(self, source_purpose: NavPurpose, next_state_after_align):
        """
        在 Nav2 done callback 中调用：
        - OBJECT_PREGRASP / BIN_PREPLACE：进入精确对位，完成后进入 GRASP / PLACE_IN_BIN
        - INTEREST_POINT：进入精确对位（替代 WAIT），完成后根据检测到的目标进入 GRASP / PLACE_IN_BIN
        """
        self.current_state = TaskState.PRECISION_ALIGN
        self._precision_align_source_purpose = source_purpose
        self._precision_align_next_state = next_state_after_align
        self._docking_active = False
        self._docking_phase = 'rotate'
        self._last_docking_target_base_m = None
        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None
        self.wait_at_point_start_time = time.monotonic()
        self.get_logger().info(
            f'Entered PRECISION_ALIGN (source={source_purpose.value}); waiting for vision trigger.'
        )

    def _handle_precision_align_timeout_if_needed(self):
        """
        仅当精确对位接在 NAV_TO_INTEREST_POINT 后（作为 WAIT 替代）时启用超时跳点逻辑。
        """
        if self._precision_align_source_purpose != NavPurpose.INTEREST_POINT:
            return
        duration = self.get_parameter('wait_at_interest_point_sec').value
        if self.wait_at_point_start_time is None:
            self.wait_at_point_start_time = time.monotonic()
            return
        if (time.monotonic() - self.wait_at_point_start_time) < duration:
            return

        # 超时：与 WAIT_AT_INTEREST_POINT 一致，标记该兴趣点失败并跳下一个
        if self.current_interest_point is not None:
            p = geometry_msgs.Point()
            p.x = self.current_interest_point[0]
            p.y = self.current_interest_point[1]
            p.z = 0.0
            self.object_blacklist.append(p)
            self.get_logger().info('No vision trigger in PRECISION_ALIGN within timeout; mark and skip.')

        self._stop_cmd_vel()
        self._docking_active = False
        self._docking_phase = 'rotate'
        self._last_docking_target_base_m = None

        self.interest_point_index += 1
        self.current_interest_point = None
        self.wait_at_point_start_time = None
        self.current_state = TaskState.NAV_TO_INTEREST_POINT
        self._nav_to_next_interest_point()

    def _handle_precision_align_vision(self, point_msg: geometry_msgs.Point, is_object: bool):
        """
        在 PRECISION_ALIGN 状态下由 vision 回调触发：
        - 临时关闭 local costmap inflation（等效于“关掉 costmap 膨胀影响”）
        - 记录目标在 base_link 下的位置并启动 diffdrive docking 控制
        - 对位达标后切入下一状态（GRASP / PLACE_IN_BIN）
        """
        # 根据载荷状态决定当前应该响应 object 还是 bin
        if is_object and self.cargo_state != CargoState.EMPTY:
            return
        if (not is_object) and self.cargo_state != CargoState.HAS_OBJECT:
            return

        # 仅发送一次 DockRobot goal；后续由 action result 驱动状态切换
        if self._dock_goal_sent:
            return

        if self.dock_client is None:
            return
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('PRECISION_ALIGN: DockRobot action server not ready yet.')
            return

        # 在兴趣点触发时，下一状态由目标类型决定（替代 WAIT_AT_INTEREST_POINT 的功能）
        if self._precision_align_source_purpose == NavPurpose.INTEREST_POINT:
            self._precision_align_next_state = TaskState.GRASP if is_object else TaskState.PLACE_IN_BIN

        stop_dist = float(self.get_parameter('docking_stop_distance_m').value)
        stop_dist = max(0.0, stop_dist)

        try:
            # 视觉点 -> odom（与 docking_server.fixed_frame 对齐）
            cube_pose_odom = self._point_to_pose_stamped_in_frame(point_msg, 'odom')
            robot_x, robot_y = self._get_robot_xy_in_frame('odom')
            cx = cube_pose_odom.pose.position.x
            cy = cube_pose_odom.pose.position.y
            dx = cx - robot_x
            dy = cy - robot_y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < 1e-6:
                self.get_logger().warn('PRECISION_ALIGN: invalid cube distance; skip DockRobot goal.')
                return

            # 目标停靠点：沿 robot->cube 方向回退 stop_dist
            ux = dx / dist
            uy = dy / dist
            tx = cx - ux * stop_dist
            ty = cy - uy * stop_dist

            target_pose = geometry_msgs.PoseStamped()
            target_pose.header.frame_id = 'odom'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = tx
            target_pose.pose.position.y = ty
            target_pose.pose.position.z = 0.0
            target_pose.pose.orientation = quaternion_from_yaw(math.atan2(cy - ty, cx - tx))
        except Exception as e:
            self.get_logger().error(f'PRECISION_ALIGN: build DockRobot goal failed: {e}')
            return

        self.get_logger().info(
            f'PRECISION_ALIGN: vision trigger received, sending DockRobot goal '
            f'(stop_dist={stop_dist:.2f} m, target=({tx:.3f},{ty:.3f}) odom).'
        )

        self._disable_local_inflation_if_needed()

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose = target_pose
        goal.dock_type = self.get_parameter('dock_type').value
        goal.navigate_to_staging_pose = False
        goal.max_staging_time = 0.0

        send_goal_future = self.dock_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._dock_goal_response_callback)
        self._dock_goal_sent = True

    def _dock_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'DockRobot goal response error: {e}')
            self._dock_goal_sent = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn('DockRobot goal rejected.')
            self._dock_goal_sent = False
            return

        self.get_logger().info('DockRobot goal accepted.')
        self._dock_goal_handle = goal_handle
        self._dock_result_future = goal_handle.get_result_async()

    def _precision_align_control_step(self):
        if not self._dock_goal_sent or self._dock_result_future is None:
            return

        if not self._dock_result_future.done():
            return

        try:
            result = self._dock_result_future.result().result
        except Exception as e:
            self.get_logger().error(f'PRECISION_ALIGN: DockRobot result error: {e}')
            self._dock_goal_sent = False
            self._dock_goal_handle = None
            self._dock_result_future = None
            return

        if result.success:
            self.get_logger().info('PRECISION_ALIGN: DockRobot succeeded.')
            if self._precision_align_next_state is not None:
                self.current_state = self._precision_align_next_state
                self._arm_cmd_sent = False
            else:
                self.get_logger().warn('PRECISION_ALIGN: next state is None; staying in PRECISION_ALIGN.')
        else:
            self.get_logger().warn(f'PRECISION_ALIGN: DockRobot failed (error_code={result.error_code}).')

        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None

    def _start_backup_after_action(self, next_state: TaskState, explore_resume_after_restore):
        """
        机械臂完成动作后：
        - 先后退固定距离（默认 20cm）
        - 再恢复 costmap inflation
        - 进入下一状态（并按需恢复探索）
        """
        backup_dist = float(self.get_parameter('backup_distance_m').value)
        v_lin = float(self.get_parameter('docking_linear_speed_mps').value)
        v_lin = max(1e-4, abs(v_lin))
        duration = backup_dist / v_lin

        self._backup_end_time = time.monotonic() + duration
        self._backup_next_state = next_state
        self._backup_after_restore_explore_resume = explore_resume_after_restore
        self.current_state = TaskState.BACKUP_AFTER_ACTION
        self.get_logger().info(f'BACKUP_AFTER_ACTION: backing up {backup_dist:.2f} m for {duration:.1f} s.')

    def _backup_control_step(self):
        if self._backup_end_time is None:
            return

        now = time.monotonic()
        if now >= self._backup_end_time:
            self._stop_cmd_vel()
            self._backup_end_time = None

            # 恢复 costmap inflation（若曾关闭）
            self._restore_local_inflation_if_needed()

            # 进入下一状态
            next_state = self._backup_next_state
            self._backup_next_state = None
            self.current_state = next_state

            # 进入下一状态后额外动作
            if next_state == TaskState.RESUME_EXPLORE_FOR_BIN:
                self._start_bin_search_or_go_to_cached()
            elif next_state == TaskState.NAV_TO_INTEREST_POINT:
                self._nav_to_next_interest_point()
            elif next_state == TaskState.POST_ACTION:
                self._handle_post_action()

            # 恢复探索控制（如需要）
            if self._backup_after_restore_explore_resume is not None:
                self._publish_explore_resume_if_changed(bool(self._backup_after_restore_explore_resume))
            self._backup_after_restore_explore_resume = None
            return

        # 持续后退
        v_lin = float(self.get_parameter('docking_linear_speed_mps').value)
        twist = geometry_msgs.Twist()
        twist.linear.x = -abs(v_lin)
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

    def _disable_local_inflation_if_needed(self):
        if self._local_inflation_temporarily_off:
            return

        # 若尚未拿到原始值，先尝试读取
        if self._local_inflation_radius_saved is None:
            try:
                fut = self._param_client_local_costmap.get_parameters(['inflation_layer.inflation_radius'])
                rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
                params = fut.result()
                if params and params[0].type_ != Parameter.Type.NOT_SET:
                    self._local_inflation_radius_saved = float(params[0].value)
            except Exception:
                pass

        off_val = float(self.get_parameter('local_inflation_radius_off').value)
        try:
            fut = self._param_client_local_costmap.set_parameters(
                [Parameter('inflation_layer.inflation_radius', Parameter.Type.DOUBLE, off_val)]
            )
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
            self._local_inflation_temporarily_off = True
        except Exception as e:
            self.get_logger().warn(f'Failed to disable local inflation via params: {e}')

    def _restore_local_inflation_if_needed(self):
        if not self._local_inflation_temporarily_off:
            return
        if self._local_inflation_radius_saved is None:
            # 没有保存值时，与 nav2_params.yaml 中 local_costmap inflation_radius 一致
            self._local_inflation_radius_saved = 0.15

        try:
            fut = self._param_client_local_costmap.set_parameters(
                [Parameter('inflation_layer.inflation_radius', Parameter.Type.DOUBLE, float(self._local_inflation_radius_saved))]
            )
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
        except Exception as e:
            self.get_logger().warn(f'Failed to restore local inflation via params: {e}')
        finally:
            self._local_inflation_temporarily_off = False

    # ========================= bin 搜索辅助逻辑 =========================

    def _start_bin_search_or_go_to_cached(self):
        """
        抓取成功后调用：
        - 如果有缓存的 bin 位姿，可以直接根据颜色选择最近或固定策略导航；
          当前简化为如果有缓存则恢复探索，由 vision 回调触发去 bin。
        """
        # 这里保持简单：默认还是通过 vision 回调触发去 bin，已缓存位姿仅用于后续规划扩展
        self._publish_explore_resume_if_changed(True)

    # ========================= Post action =========================

    def _handle_post_action(self):
        """
        放置完成后的行为：
        - 当前版本保持与 V1 一致：回到 EXPLORE（如果 explore_done_flag 为 False）
        """
        self.get_logger().info('Post action: returning to explore')
        self.current_state = TaskState.EXPLORE
        self._publish_explore_resume_if_changed(True)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

