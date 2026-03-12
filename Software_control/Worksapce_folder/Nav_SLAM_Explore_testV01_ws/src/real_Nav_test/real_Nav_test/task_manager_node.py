#!/usr/bin/env python3
"""
Task manager state machine node.
Implements the full Explore-Pick-SearchBin-Place workflow (grasp then go directly to bin, no stow).

This node is the core scheduler of the system, responsible for:
1. Controlling the start and stop of exploration behavior (explore_lite)
2. Subscribing to rover_vision_node topics (/target_pick/*, /target_place/*)
3. Coordinating Nav2 navigation for goal execution
4. Invoking arm manipulation actions (grasp, place)
5. Managing state transitions and error recovery
"""

import os
import sys
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
from enum import Enum
import time

from real_Nav_test.task_manager_utils import (
    is_pose_in_blacklist as check_pose_in_blacklist,
    compute_pregrasp_pose,
)
from real_Nav_test.detect_objects_in_pgm_map import (
    get_interest_points_from_pgm,
    DEFAULT_RESOLUTION,
    DEFAULT_ORIGIN,
)


class CargoState(Enum):
    """
    Cargo state enum.
    Tracks whether the robot is currently carrying an object.
    """
    EMPTY = "empty"          # Empty: robot carries nothing, can grasp new object
    HAS_OBJECT = "has_object"  # Loaded: robot carries an object, needs to find bin


class TaskState(Enum):
    """
    Task state enum.
    Defines all states in the full workflow.
    """
    INIT = "init"                          # Init: wait for system ready
    EXPLORE = "explore"                    # Explore: start exploration, listen for object detection
    OBJECT_FOUND = "object_found"          # Object found: recyclable object detected
    PAUSE_EXPLORE = "pause_explore"        # Pause explore: stop exploration, prepare for navigation
    NAV_TO_OBJECT_PREGRASP = "nav_to_object_pregrasp"  # Navigate to object pregrasp pose
    PRECISION_ALIGN_OBJECT = "precision_align_object"  # Precision align to object (optional)
    GRASP = "grasp"                        # Grasp: execute grasp action
    RESUME_EXPLORE_FOR_BIN = "resume_explore_for_bin"  # Resume exploration to find bin
    BIN_FOUND = "bin_found"                # Bin found: bin detected
    NAV_TO_BIN_PREPLACE = "nav_to_bin_preplace"  # Navigate to bin preplace pose
    PRECISION_ALIGN_BIN = "precision_align_bin"  # Precision align to bin (optional)
    PLACE_IN_BIN = "place_in_bin"          # Place: put object into bin
    POST_ACTION = "post_action"            # Post action: handling after task completion
    # Fallback when exploration finished but vision missed targets or not all 3 groups
    EXPLORE_FINISHED_FALLBACK = "explore_finished_fallback"
    RUN_MAP_DETECTION = "run_map_detection"  # Run PGM detection, filter, queue interest points
    NAV_TO_INTEREST_POINT = "nav_to_interest_point"  # Navigate to next interest point (approach)
    WAIT_AT_INTEREST_POINT = "wait_at_interest_point"  # 15s timer; if no vision detection -> fail


class TaskManagerNode(Node):
    """
    Task manager state machine node.

    Core control node: exploration (explore_lite), Nav2, rover_vision_node topics,
    arm manipulation (grasp, place). No stow: grasp then go directly to bin.
    """

    def __init__(self):
        super().__init__('task_manager')

        # ========== State variables ==========
        self.current_state = TaskState.INIT
        self.cargo_state = CargoState.EMPTY
        self.home_pose = None
        self.object_pose = None
        self.bin_pose = None

        # ========== Detection stability counters ==========
        self.object_detection_count = 0
        self.bin_detection_count = 0
        self.required_detection_frames = 5

        # ========== Retry counters ==========
        self.grasp_retry_count = 0
        self.max_grasp_retries = 2
        self.place_retry_count = 0
        self.max_place_retries = 2

        # ========== Failed object blacklist ==========
        self.object_blacklist = []
        self.bin_blacklist = []  # Bins already placed at (to filter in fallback)
        self.blacklist_radius = 0.3

        # ========== Explore-finished fallback: vision tracking ==========
        self.detected_object_colors = set()   # {'red','green','blue'} ever seen for pick
        self.detected_bin_colors = set()      # for place
        self.explore_finished_received = False

        # ========== Fallback: save map + PGM detection ==========
        self._map_fallback_round_count = 0   # Count: run save -> next state -> file not found -> back to fallback
        self._map_fallback_max_rounds = 15    # After this many rounds, terminate
        self.interest_points = []             # List of (x, y) in map frame
        self.interest_point_index = 0
        self.current_interest_point = None   # (x, y) for blacklist on timeout
        self.wait_at_point_start_time = None
        self.wait_at_point_duration_sec = 15.0

        # ========== Action clients ==========
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None

        # ========== Publishers ==========
        # Control explore_lite start/stop (publish only on change)
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, 'explore/resume', 10
        )
        self._last_explore_resume = None
        self.state_pub = self.create_publisher(std_msgs.String, 'task_manager/state', 10)
        self.cargo_state_pub = self.create_publisher(std_msgs.String, 'task_manager/cargo_state', 10)

        # ========== Subscribers (aligned with rover_vision_node) ==========
        for color, topic in [('red', '/target_pick/red'), ('green', '/target_pick/green'), ('blue', '/target_pick/blue')]:
            self.create_subscription(
                geometry_msgs.Point, topic,
                lambda msg, c=color: self._object_point_callback(msg, c),
                qos_profile_sensor_data,
            )
        for color, topic in [('red', '/target_place/red'), ('green', '/target_place/green'), ('blue', '/target_place/blue')]:
            self.create_subscription(
                geometry_msgs.Point, topic,
                lambda msg, c=color: self._bin_point_callback(msg, c),
                qos_profile_sensor_data,
            )

        # ========== Explore finished (from explore node) ==========
        self.create_subscription(
            std_msgs.Bool, 'explore/finished', self._explore_finished_callback, 10
        )

        # ========== TF ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ========== State machine timer ==========
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)

        # ========== Node parameters ==========
        self.declare_parameter('pregrasp_distance', 0.5)
        self.declare_parameter('preplace_distance', 0.6)
        self.declare_parameter('camera_frame_id', 'camera_depth_optical_frame')
        self.declare_parameter('maps_directory', '')
        self.declare_parameter('map_save_basename', 'explore_complete')  # e.g. "my_map" for -f my_map
        self.declare_parameter('map_resolution', DEFAULT_RESOLUTION)
        self.declare_parameter('map_origin_x', DEFAULT_ORIGIN[0])
        self.declare_parameter('map_origin_y', DEFAULT_ORIGIN[1])
        self.declare_parameter('wait_at_interest_point_sec', 15.0)

        self.get_logger().info('Task manager node initialized')
    def _point_to_pose_stamped_in_map(self, point_msg):
        """Transform geometry_msgs.Point (vision, camera frame) to PoseStamped in map frame."""
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

    def _object_point_callback(self, msg, color: str):
        """Object (pick) detection; records color and forwards to common logic."""
        self.detected_object_colors.add(color)
        if self.cargo_state != CargoState.EMPTY:
            self.object_detection_count = 0
            return
        if self.current_state not in (TaskState.EXPLORE, TaskState.WAIT_AT_INTEREST_POINT):
            self.object_detection_count = 0
            return
        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
            if check_pose_in_blacklist(pose_stamped.pose.position, self.object_blacklist, self.blacklist_radius):
                return
            self.object_pose = pose_stamped
            self.object_detection_count += 1
            if self.object_detection_count >= self.required_detection_frames:
                self.get_logger().info('Object found and confirmed!')
                self.object_detection_count = 0
                self.current_state = TaskState.OBJECT_FOUND
        except Exception as e:
            self.get_logger().error(f'Error processing target_pick message: {e}')
            self.object_detection_count = 0

    def _bin_point_callback(self, msg, color: str):
        """Bin (place) detection; records color, filters bin_blacklist."""
        self.detected_bin_colors.add(color)
        if self.cargo_state != CargoState.HAS_OBJECT or self.current_state != TaskState.RESUME_EXPLORE_FOR_BIN:
            self.bin_detection_count = 0
            return
        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
            if check_pose_in_blacklist(pose_stamped.pose.position, self.bin_blacklist, self.blacklist_radius):
                return
            self.bin_pose = pose_stamped
            self.bin_detection_count += 1
            if self.bin_detection_count >= self.required_detection_frames:
                self.get_logger().info('Bin found and confirmed!')
                self.current_state = TaskState.BIN_FOUND
                self.bin_detection_count = 0
        except Exception as e:
            self.get_logger().error(f'Error processing target_place message: {e}')
            self.bin_detection_count = 0

    def _explore_finished_callback(self, msg):
        """When exploration finishes: if vision missed targets or not all 3 groups -> start fallback."""
        if not msg.data:
            return
        self.explore_finished_received = True
        if self.current_state != TaskState.EXPLORE:
            return
        need_fallback = (
            len(self.detected_object_colors) == 0
            or len(self.detected_object_colors) < 3
        )
        if need_fallback:
            self.get_logger().info(
                'Exploration finished but vision missed targets or not all 3 groups; starting map fallback.'
            )
            self._publish_explore_resume_if_changed(False)
            self.current_state = TaskState.EXPLORE_FINISHED_FALLBACK

    def state_machine_callback(self):
        """
        State machine main callback.

        Core execution; called every 0.1 s. Responsibilities:
        1. Publish current and cargo state (for monitoring)
        2. Dispatch to the handler for the current state
        3. Execute state transition logic
        """
        # Publish state (for monitoring and debugging)
        state_msg = std_msgs.String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)

        cargo_msg = std_msgs.String()
        cargo_msg.data = self.cargo_state.value
        self.cargo_state_pub.publish(cargo_msg)

        # State machine dispatch
        if self.current_state == TaskState.INIT:
            self.handle_init_state()
        elif self.current_state == TaskState.EXPLORE:
            self.handle_explore_state()
        elif self.current_state == TaskState.OBJECT_FOUND:
            self.handle_object_found_state()
        elif self.current_state == TaskState.PAUSE_EXPLORE:
            self.handle_pause_explore_state()
        elif self.current_state == TaskState.NAV_TO_OBJECT_PREGRASP:
            self.handle_nav_to_object_pregrasp_state()
        elif self.current_state == TaskState.PRECISION_ALIGN_OBJECT:
            self.handle_precision_align_object_state()
        elif self.current_state == TaskState.GRASP:
            self.handle_grasp_state()
        elif self.current_state == TaskState.RESUME_EXPLORE_FOR_BIN:
            self.handle_resume_explore_for_bin_state()
        elif self.current_state == TaskState.BIN_FOUND:
            self.handle_bin_found_state()
        elif self.current_state == TaskState.NAV_TO_BIN_PREPLACE:
            self.handle_nav_to_bin_preplace_state()
        elif self.current_state == TaskState.PRECISION_ALIGN_BIN:
            self.handle_precision_align_bin_state()
        elif self.current_state == TaskState.PLACE_IN_BIN:
            self.handle_place_in_bin_state()
        elif self.current_state == TaskState.POST_ACTION:
            self.handle_post_action_state()
        elif self.current_state == TaskState.EXPLORE_FINISHED_FALLBACK:
            self.handle_explore_finished_fallback_state()
        elif self.current_state == TaskState.RUN_MAP_DETECTION:
            self.handle_run_map_detection_state()
        elif self.current_state == TaskState.NAV_TO_INTEREST_POINT:
            self.handle_nav_to_interest_point_state()
        elif self.current_state == TaskState.WAIT_AT_INTEREST_POINT:
            self.handle_wait_at_interest_point_state()
        #TODO: 
    
    def handle_init_state(self):
        """
        Handle init state.

        Wait for system ready (TF, SLAM, Nav2), save home pose, then enter EXPLORE.
        """
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Nav2 server...')
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
            self.current_state = TaskState.EXPLORE
        except Exception as e:
            self.get_logger().warn(f'Waiting for TF: {e}')
    
    def handle_explore_state(self):
        """Start or resume explore_lite. Object detection in object_point_callback (/target_pick/*)."""
        self._publish_explore_resume_if_changed(True)

    def _publish_explore_resume_if_changed(self, resume: bool):
        """Publish to explore/resume only when the value changes."""
        if self._last_explore_resume is not None and self._last_explore_resume == resume:
            return
        self._last_explore_resume = resume
        msg = std_msgs.Bool()
        msg.data = resume
        self.explore_control_pub.publish(msg)

    def handle_object_found_state(self):
        """When object is stably detected, go to PAUSE_EXPLORE if coords available."""
        if self.object_pose is not None:
            self.get_logger().info(
                f'Object found, coords received: ({self.object_pose.pose.position.x:.2f}, '
                f'{self.object_pose.pose.position.y:.2f}, {self.object_pose.pose.position.z:.2f}), '
                'transitioning to pause explore'
            )
            self.current_state = TaskState.PAUSE_EXPLORE
        else:
            self.get_logger().debug('Object found, waiting for target_pick coords...')

    def handle_pause_explore_state(self):
        """Cancel Nav2 goal, stop explore_lite, prepare for grasp or place."""
        if self.nav2_goal_handle is not None:
            self.nav2_client.cancel_goal_async(self.nav2_goal_handle)
            self.nav2_goal_handle = None
        self._publish_explore_resume_if_changed(False)
        if self.cargo_state == CargoState.EMPTY:
            self.current_state = TaskState.NAV_TO_OBJECT_PREGRASP
        else:
            self.current_state = TaskState.NAV_TO_BIN_PREPLACE
    
    def handle_nav_to_object_pregrasp_state(self):
        """
        Handle navigate to object pregrasp state.

        Compute pregrasp pose in front of object (safe distance, facing object), then navigate with Nav2.
        """
        if self.object_pose is None:
            self.get_logger().error('Object pose not available!')
            self.current_state = TaskState.EXPLORE
            return
        robot_x, robot_y = self._get_robot_xy_in_map()
        pregrasp_distance = self.get_parameter('pregrasp_distance').value
        goal_pose = compute_pregrasp_pose(
            self.object_pose, pregrasp_distance, robot_x, robot_y,
            frame_id='map', stamp=self.get_clock().now().to_msg()
        )
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        if self.nav2_goal_handle is None:
            self.get_logger().info('Sending Nav2 goal to object pregrasp')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Reached pregrasp pose')
                self.nav2_goal_handle = None
                self.current_state = TaskState.PRECISION_ALIGN_OBJECT
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.get_logger().warn('Navigation failed, returning to explore')
                self.nav2_goal_handle = None
                self.current_state = TaskState.EXPLORE
    
    def handle_precision_align_object_state(self):
        """
        Handle precision align to object state (optional but recommended).

        Use D435i for cm-level alignment. Currently placeholder: jump to GRASP.
        """
        # TODO: Implement D435i precision alignment
        self.get_logger().info('Precision align (placeholder - jumping to grasp)')
        self.current_state = TaskState.GRASP
    
    def handle_grasp_state(self):
        """
        Handle grasp state.

        Call arm grasp action (grasp_server). Includes retry and blacklist on failure.
        """
        # TODO: Call actual grasp action
        self.get_logger().info('Calling grasp action (placeholder)')

        grasp_success = True  # Placeholder

        if grasp_success:
            self.get_logger().info('Grasp succeeded!')
            self.cargo_state = CargoState.HAS_OBJECT
            self.grasp_retry_count = 0
            self.adjust_nav2_for_carry_mode(True)
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
        else:
            self.grasp_retry_count += 1
            if self.grasp_retry_count >= self.max_grasp_retries:
                self.get_logger().warn('Grasp failed, max retries reached, abandoning object')
                if self.object_pose:
                    self.object_blacklist.append(self.object_pose.pose.position)
                self.grasp_retry_count = 0
                self.current_state = TaskState.EXPLORE
            else:
                self.get_logger().info(f'Grasp failed, retrying ({self.grasp_retry_count}/{self.max_grasp_retries})')
                self.current_state = TaskState.PRECISION_ALIGN_OBJECT
    
    def handle_resume_explore_for_bin_state(self):
        """Resume explore_lite with bin as target. Bin detection in bin_point_callback (/target_place/*)."""
        self._publish_explore_resume_if_changed(True)

    def handle_bin_found_state(self):
        """
        Handle bin found state.

        When bin is stably detected, transition to pause explore.
        """
        self.get_logger().info('Bin found, transitioning to pause explore')
        self.current_state = TaskState.PAUSE_EXPLORE
    
    def handle_nav_to_bin_preplace_state(self):
        """
        Handle navigate to bin preplace state.

        Compute preplace pose in front of bin (safe distance, facing bin), then navigate with Nav2.
        """
        if self.bin_pose is None:
            self.get_logger().error('Bin pose not available!')
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            return
        robot_x, robot_y = self._get_robot_xy_in_map()
        preplace_distance = self.get_parameter('preplace_distance').value
        goal_pose = compute_pregrasp_pose(
            self.bin_pose, preplace_distance, robot_x, robot_y,
            frame_id='map', stamp=self.get_clock().now().to_msg()
        )
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        if self.nav2_goal_handle is None:
            self.get_logger().info('Sending Nav2 goal to bin preplace')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Reached preplace pose')
                self.nav2_goal_handle = None
                self.current_state = TaskState.PRECISION_ALIGN_BIN
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.get_logger().warn('Navigation failed, returning to resume explore for bin')
                self.nav2_goal_handle = None
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
    
    def handle_precision_align_bin_state(self):
        """
        Handle precision align to bin state (recommended).

        Use D435i for alignment. Currently placeholder: jump to PLACE_IN_BIN.
        """
        # TODO: Implement D435i precision alignment for bin
        self.get_logger().info('Precision align to bin (placeholder - jumping to place)')
        self.current_state = TaskState.PLACE_IN_BIN
    
    def handle_place_in_bin_state(self):
        """
        Handle place in bin state.

        Execute place action (gripper -> bin). Includes retry on failure.
        """
        # TODO: Call actual place action
        self.get_logger().info('Calling place action (placeholder)')

        place_success = True  # Placeholder

        if place_success:
            self.get_logger().info('Place succeeded!')
            if self.bin_pose is not None:
                self.bin_blacklist.append(self.bin_pose.pose.position)
            self.cargo_state = CargoState.EMPTY
            self.place_retry_count = 0
            self.adjust_nav2_for_carry_mode(False)
            self.current_state = TaskState.POST_ACTION
        else:
            self.place_retry_count += 1
            if self.place_retry_count >= self.max_place_retries:
                self.get_logger().warn('Place failed, max retries reached, resuming explore for bin')
                self.place_retry_count = 0
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            else:
                self.get_logger().info(f'Place failed, retrying ({self.place_retry_count}/{self.max_place_retries})')
                self.current_state = TaskState.PRECISION_ALIGN_BIN
    
    def handle_post_action_state(self):
        """
        Handle post action state.

        Options: return to explore, return to home, or end task. Current: return to explore.
        """
        self.get_logger().info('Post action: returning to explore')
        self.current_state = TaskState.EXPLORE

    def _get_maps_directory(self):
        """Resolve writable maps directory for saving map."""
        maps_dir = self.get_parameter('maps_directory').value
        if maps_dir:
            return maps_dir
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('real_Nav_test')
            return os.path.join(pkg_share, 'maps')
        except Exception:
            return os.path.expanduser('~/maps')

    def handle_explore_finished_fallback_state(self):
        """
        Run map_saver_cli once, then go directly to RUN_MAP_DETECTION.
        If RUN_MAP_DETECTION finds no file at the expected path, it will return here;
        after 15 such rounds, the process terminates.
        """
        maps_dir = self._get_maps_directory()
        os.makedirs(maps_dir, exist_ok=True)
        basename = self.get_parameter('map_save_basename').value
        map_base = os.path.join(maps_dir, basename)
        self.get_logger().info(
            f'Map save round {self._map_fallback_round_count + 1}/{self._map_fallback_max_rounds}: '
            f'ros2 run nav2_map_server map_saver_cli -f {map_base}'
        )
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
        # Always go to next state; RUN_MAP_DETECTION will check file and possibly return here
        self.current_state = TaskState.RUN_MAP_DETECTION

    def handle_run_map_detection_state(self):
        """Run PGM detection on saved map; if file not found, go back to fallback; after 15 rounds, terminate."""
        maps_dir = self._get_maps_directory()
        basename = self.get_parameter('map_save_basename').value
        pgm_path = os.path.join(maps_dir, basename + '.pgm')
        if not os.path.isfile(pgm_path):
            self._map_fallback_round_count += 1
            self.get_logger().warn(
                f'PGM not found at {pgm_path}; round {self._map_fallback_round_count}/{self._map_fallback_max_rounds}, '
                'returning to fallback to retry save.'
            )
            if self._map_fallback_round_count >= self._map_fallback_max_rounds:
                self.get_logger().error(
                    f'Map file still not found after {self._map_fallback_max_rounds} rounds. Exiting process with failure.'
                )
                rclpy.shutdown()
                sys.exit(1)
            self.current_state = TaskState.EXPLORE_FINISHED_FALLBACK
            return
        self._map_fallback_round_count = 0  # Reset on success
        resolution = self.get_parameter('map_resolution').value
        origin = (
            self.get_parameter('map_origin_x').value,
            self.get_parameter('map_origin_y').value,
        )
        try:
            raw_points = get_interest_points_from_pgm(
                pgm_path,
                resolution=resolution,
                origin=origin,
            )
        except Exception as e:
            self.get_logger().error(f'PGM detection failed: {e}; returning to EXPLORE.')
            self.current_state = TaskState.EXPLORE
            return
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
        self.interest_points = filtered
        self.interest_point_index = 0
        self.get_logger().info(f'Map detection: {len(raw_points)} points, {len(filtered)} after filtering.')
        if not self.interest_points:
            self.get_logger().info('No interest points left; returning to EXPLORE.')
            self.current_state = TaskState.EXPLORE
            return
        self.current_state = TaskState.NAV_TO_INTEREST_POINT

    def handle_nav_to_interest_point_state(self):
        """Navigate to next interest point (approach pose); on arrival start 15s wait."""
        if self.interest_point_index >= len(self.interest_points):
            self.get_logger().info('All interest points visited; returning to EXPLORE.')
            self.current_state = TaskState.EXPLORE
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
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        if self.nav2_goal_handle is None:
            self.get_logger().info(
                f'Nav to interest point {self.interest_point_index + 1}/{len(self.interest_points)} at ({mx:.2f}, {my:.2f})'
            )
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Reached interest point; starting 15s wait for vision.')
                self.nav2_goal_handle = None
                self.wait_at_point_start_time = time.monotonic()
                self.current_state = TaskState.WAIT_AT_INTEREST_POINT
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.get_logger().warn('Nav to interest point failed; skipping to next.')
                self.nav2_goal_handle = None
                self.interest_point_index += 1
                self.current_interest_point = None

    def handle_wait_at_interest_point_state(self):
        """15s at current interest point; if no vision detection by then, mark as failed and next."""
        duration = self.get_parameter('wait_at_interest_point_sec').value
        if self.wait_at_point_start_time is None:
            self.wait_at_point_start_time = time.monotonic()
        elapsed = time.monotonic() - self.wait_at_point_start_time
        if elapsed >= duration:
            if self.current_interest_point is not None:
                p = geometry_msgs.Point()
                p.x = self.current_interest_point[0]
                p.y = self.current_interest_point[1]
                p.z = 0.0
                self.object_blacklist.append(p)
                self.get_logger().info('No vision detection at interest point within 15s; marked as failed target.')
            self.interest_point_index += 1
            self.current_interest_point = None
            self.wait_at_point_start_time = None
            self.current_state = TaskState.NAV_TO_INTEREST_POINT

    def _get_robot_xy_in_map(self):
        """Get base_link (x, y) in map from TF; return (0.0, 0.0) on failure."""
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

    def nav2_goal_response_callback(self, future):
        """
        Nav2 goal response callback.

        Called when Nav2 accepts or rejects the navigation goal.

        Args:
            future: Future containing the goal handle
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self.nav2_goal_handle = None
            return

        self.get_logger().info('Nav2 goal accepted')
        self.nav2_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        """
        Nav2 goal result callback.

        Called when Nav2 navigation completes (success/fail/cancel). Mainly for logging; state checks are in state handlers.

        Args:
            future: Future containing the goal handle
        """
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded')
        elif goal_handle.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 goal aborted')
        elif goal_handle.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal canceled')
    
    def adjust_nav2_for_carry_mode(self, enable):
        """
        Adjust Nav2 parameters for carry mode.

        When carrying an object: reduce max linear/angular velocity, increase inflation radius, optionally increase footprint.

        Args:
            enable: bool, True to enable carry mode, False to restore normal
        """
        # TODO: Implement Nav2 parameter adjustment (e.g. dynamic reconfigure, Nav2 services, or costmap config switch)
        if enable:
            self.get_logger().info('Carry mode on: reduced speed, larger inflation radius')
        else:
            self.get_logger().info('Carry mode off: normal parameters restored')


def main(args=None):
    """
    Main entry point.

    Initialize ROS2 and run the task manager state machine.

    Args:
        args: Optional command-line arguments
    """
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

