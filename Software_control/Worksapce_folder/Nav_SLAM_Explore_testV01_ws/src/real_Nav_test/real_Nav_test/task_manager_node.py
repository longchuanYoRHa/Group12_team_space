#!/usr/bin/env python3
"""
Task manager state machine node.
Implements the full Explore-Pick-Stow-SearchBin-Place workflow.

This node is the core scheduler of the system, responsible for:
1. Controlling the start and stop of exploration behavior (explore_lite)
2. Subscribing to object detector (object_detector) and bin detector (bin_detector)
3. Coordinating Nav2 navigation for goal execution
4. Invoking arm manipulation actions (grasp, stow, place)
5. Managing state transitions and error recovery
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
from enum import Enum
import math
import time


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
    STOW_ON_ROBOT = "stow_on_robot"        # Stow: move object to onboard stow pose
    RESUME_EXPLORE_FOR_BIN = "resume_explore_for_bin"  # Resume exploration to find bin
    BIN_FOUND = "bin_found"                # Bin found: bin detected
    NAV_TO_BIN_PREPLACE = "nav_to_bin_preplace"  # Navigate to bin preplace pose
    PRECISION_ALIGN_BIN = "precision_align_bin"  # Precision align to bin (optional)
    PLACE_IN_BIN = "place_in_bin"          # Place: put object into bin
    POST_ACTION = "post_action"            # Post action: handling after task completion


class TaskManagerNode(Node):
    """
    Task manager state machine node.

    Core control node that implements a state machine to coordinate:
    - Exploration (explore_lite)
    - Navigation (Nav2)
    - Object detection (object_detector)
    - Bin detection (bin_detector)
    - Arm manipulation (grasp/stow/place actions)
    """

    def __init__(self):
        super().__init__('task_manager')

        # ========== State variables ==========
        self.current_state = TaskState.INIT  # Current task state
        self.cargo_state = CargoState.EMPTY  # Cargo state (empty/loaded)
        self.home_pose = None  # Start pose for return after task
        self.object_pose = None  # Detected object pose (map frame)
        self.bin_pose = None  # Detected bin pose (map frame)
        self.stow_pose = None  # Onboard stow pose (arm_base frame, fixed)

        # ========== Detection stability counters ==========
        # Multi-frame confirmation to reduce false positives
        self.object_detection_count = 0  # Consecutive object detection frames
        self.bin_detection_count = 0  # Consecutive bin detection frames
        self.required_detection_frames = 5  # Frames required for stable detection (N-frame confirm)

        # ========== Retry counters ==========
        # For failure recovery
        self.grasp_retry_count = 0  # Grasp retry count
        self.max_grasp_retries = 2  # Max grasp retries
        self.stow_retry_count = 0  # Stow retry count
        self.max_stow_retries = 2  # Max stow retries
        self.place_retry_count = 0  # Place retry count
        self.max_place_retries = 2  # Max place retries

        # ========== Failed object blacklist ==========
        # Record failed grasp positions to avoid repeated attempts
        self.object_blacklist = []  # Blacklist position list
        self.blacklist_radius = 0.3  # Blacklist radius (m); objects within are ignored

        # ========== Action clients ==========
        # Nav2 navigation action client
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None  # Current navigation goal handle

        # ========== Publishers ==========
        # Control explore_lite start/stop via /explore/resume topic
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, 'explore/resume', 10
        )
        # Publish current state (for monitoring and debugging)
        self.state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/state', 10
        )
        # Publish cargo state (for monitoring and debugging)
        self.cargo_state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/cargo_state', 10
        )

        # ========== Subscribers ==========
        # TODO: Replace with actual object_detector topic
        # Subscribe to object pose from object detector
        self.object_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'object_detector/object_pose',  # Placeholder
            self.object_pose_callback,
            qos_profile_sensor_data
        )

        # Subscribe to camera-detected object coordinates (obj_xy topic)
        # Assumes geometry_msgs.PointStamped (x, y, z); adjust if message type differs
        self.obj_xy_sub = self.create_subscription(
            geometry_msgs.PointStamped,
            'obj_xy',
            self.obj_xy_callback,
            qos_profile_sensor_data
        )

        # TODO: Replace with actual bin_detector topic
        # Subscribe to bin pose from bin detector
        self.bin_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'bin_detector/bin_pose',  # Placeholder
            self.bin_pose_callback,
            qos_profile_sensor_data
        )

        # ========== TF ==========
        # For frame transforms (map <-> base_link <-> arm_base, etc.)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ========== State machine timer ==========
        # Run state machine logic every 0.1 s (10 Hz)
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)

        # ========== Node parameters ==========
        self.declare_parameter('pregrasp_distance', 0.5)  # Pregrasp distance (m)
        self.declare_parameter('preplace_distance', 0.6)  # Preplace distance (m)
        self.declare_parameter('stow_pose_x', 0.3)  # Stow pose X
        self.declare_parameter('stow_pose_y', 0.0)  # Stow pose Y
        self.declare_parameter('stow_pose_z', 0.2)  # Stow pose Z

        self.get_logger().info('Task manager node initialized')
        
    def object_pose_callback(self, msg):
        """
        Object pose detection callback.

        Called when the object detector publishes a new object pose.
        Only processes object detection when empty and in EXPLORE state.
        Uses multi-frame confirmation to reduce false positives.

        Args:
            msg: geometry_msgs.msg.PoseStamped, object pose (map frame)
        """
        # Only process object detection when empty and exploring
        if self.cargo_state == CargoState.EMPTY and self.current_state == TaskState.EXPLORE:
            # Skip if position is in blacklist (previous failed grasp)
            if self.is_pose_in_blacklist(msg.pose.position):
                return

            self.object_pose = msg
            self.object_detection_count += 1

            if self.object_detection_count >= self.required_detection_frames:
                self.get_logger().info('Object found and confirmed!')
                self.current_state = TaskState.OBJECT_FOUND
                self.object_detection_count = 0
        else:
            self.object_detection_count = 0
    
    def obj_xy_callback(self, msg):
        """
        Camera object coordinates callback.

        Called when the camera detects an object and publishes to obj_xy.
        Converts coordinates to PoseStamped and stores for navigation.

        Args:
            msg: geometry_msgs.msg.PointStamped, object coordinates (may be in different frame)
        """
        try:
            # Convert PointStamped to PoseStamped; transform to map if needed
            if msg.header.frame_id != 'map':
                try:
                    msg_time = rclpy.time.Time.from_msg(msg.header.stamp)

                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        msg.header.frame_id,
                        msg_time,
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )

                    point_stamped_in_map = tf2_geometry_msgs.do_transform_point(msg, transform)

                    pose_stamped = geometry_msgs.PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    pose_stamped.pose.position = point_stamped_in_map.point
                    pose_stamped.pose.orientation.w = 1.0

                    self.object_pose = pose_stamped
                    self.get_logger().info(
                        f'Object coords ({msg.header.frame_id}): ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}), '
                        f'map: ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f}, {pose_stamped.pose.position.z:.2f})'
                    )
                except Exception as e:
                    self.get_logger().warn(f'TF transform failed: {e}, using raw coords (assumed map)')
                    pose_stamped = geometry_msgs.PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    pose_stamped.pose.position = msg.point
                    pose_stamped.pose.orientation.w = 1.0
                    self.object_pose = pose_stamped
            else:
                pose_stamped = geometry_msgs.PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position = msg.point
                pose_stamped.pose.orientation.w = 1.0
                self.object_pose = pose_stamped
                self.get_logger().info(
                    f'Object coords (map): ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})'
                )
        except Exception as e:
            self.get_logger().error(f'Error processing obj_xy message: {e}')
    
    def bin_pose_callback(self, msg):
        """
        Bin pose detection callback.

        Called when the bin detector publishes a new bin pose.
        Only processes when loaded and in RESUME_EXPLORE_FOR_BIN state.
        Uses multi-frame confirmation to reduce false positives.

        Args:
            msg: geometry_msgs.msg.PoseStamped, bin pose (map frame)
        """
        if self.cargo_state == CargoState.HAS_OBJECT and self.current_state == TaskState.RESUME_EXPLORE_FOR_BIN:
            self.bin_pose = msg
            self.bin_detection_count += 1

            if self.bin_detection_count >= self.required_detection_frames:
                self.get_logger().info('Bin found and confirmed!')
                self.current_state = TaskState.BIN_FOUND
                self.bin_detection_count = 0
        else:
            self.bin_detection_count = 0
    
    def is_pose_in_blacklist(self, position):
        """
        Check if a position is in the blacklist.

        Used to avoid repeated attempts at previously failed grasp positions.

        Args:
            position: geometry_msgs.msg.Point, position to check

        Returns:
            bool: True if within blacklist radius, else False
        """
        for blacklist_pos in self.object_blacklist:
            distance = math.sqrt(
                (position.x - blacklist_pos.x)**2 +
                (position.y - blacklist_pos.y)**2
            )
            if distance < self.blacklist_radius:
                return True
        return False
    
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
        elif self.current_state == TaskState.STOW_ON_ROBOT:
            self.handle_stow_on_robot_state()
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
        """
        Handle explore state.

        Start or resume explore_lite. Object detection is handled asynchronously in object_pose_callback.
        """
        explore_msg = std_msgs.Bool()
        explore_msg.data = True
        self.explore_control_pub.publish(explore_msg)
    
    def handle_object_found_state(self):
        """
        Handle object found state.

        When object is stably detected, check if coords were received from obj_xy.
        If yes, go to PAUSE_EXPLORE; otherwise keep waiting.
        """
        if self.object_pose is not None:
            self.get_logger().info(
                f'Object found, coords received: ({self.object_pose.pose.position.x:.2f}, '
                f'{self.object_pose.pose.position.y:.2f}, {self.object_pose.pose.position.z:.2f}), '
                'transitioning to pause explore'
            )
            self.current_state = TaskState.PAUSE_EXPLORE
        else:
            self.get_logger().debug('Object found, waiting for obj_xy coords...')
    
    def handle_pause_explore_state(self):
        """
        Handle pause explore state.

        Cancel current Nav2 goal, stop explore_lite, prepare for grasp or place.
        Used for both object grasp and bin place scenarios.
        """
        if self.nav2_goal_handle is not None:
            self.nav2_client.cancel_goal_async(self.nav2_goal_handle)
            self.nav2_goal_handle = None

        explore_msg = std_msgs.Bool()
        explore_msg.data = False
        self.explore_control_pub.publish(explore_msg)

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

        pregrasp_distance = self.get_parameter('pregrasp_distance').value
        goal_pose = self.calculate_pregrasp_pose(self.object_pose, pregrasp_distance)

        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        if self.nav2_goal_handle is None:
            self.get_logger().info('Sending Nav2 goal to object pregrasp')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            from rclpy.action import GoalStatus
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
            self.current_state = TaskState.STOW_ON_ROBOT
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
    
    def handle_stow_on_robot_state(self):
        """
        Handle stow on robot state.

        Move grasped object to onboard stow pose. On success, enable carry mode (adjust Nav2) and go to RESUME_EXPLORE_FOR_BIN.
        """
        # TODO: Call actual stow action
        self.get_logger().info('Calling stow action (placeholder)')

        stow_success = True  # Placeholder

        if stow_success:
            self.get_logger().info('Stow succeeded!')
            self.stow_retry_count = 0
            self.adjust_nav2_for_carry_mode(True)
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
        else:
            self.stow_retry_count += 1
            if self.stow_retry_count >= self.max_stow_retries:
                self.get_logger().warn('Stow failed, max retries reached')
                self.stow_retry_count = 0
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            else:
                self.get_logger().info(f'Stow failed, retrying ({self.stow_retry_count}/{self.max_stow_retries})')
                self.current_state = TaskState.GRASP
    
    def handle_resume_explore_for_bin_state(self):
        """
        Handle resume explore for bin state.

        Resume explore_lite with bin as target. Bin detection is async in bin_pose_callback.
        """
        explore_msg = std_msgs.Bool()
        explore_msg.data = True
        self.explore_control_pub.publish(explore_msg)

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

        preplace_distance = self.get_parameter('preplace_distance').value
        goal_pose = self.calculate_pregrasp_pose(self.bin_pose, preplace_distance)

        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        if self.nav2_goal_handle is None:
            self.get_logger().info('Sending Nav2 goal to bin preplace')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            from rclpy.action import GoalStatus
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

        Execute place action (stow -> bin). Includes retry on failure.
        """
        # TODO: Call actual place action
        self.get_logger().info('Calling place action (placeholder)')

        place_success = True  # Placeholder

        if place_success:
            self.get_logger().info('Place succeeded!')
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
    
    def calculate_pregrasp_pose(self, target_pose, distance):
        """
        Compute pregrasp/preplace pose.

        Compute a pose in front of target at given distance, facing target.
        Used for object grasp and bin place navigation.

        Args:
            target_pose: geometry_msgs.msg.PoseStamped, target pose (map frame)
            distance: float, standoff distance (m)

        Returns:
            geometry_msgs.msg.PoseStamped: pregrasp/preplace pose (map frame)
        """
        goal_pose = geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception:
            robot_x = 0.0
            robot_y = 0.0

        dx = robot_x - target_pose.pose.position.x
        dy = robot_y - target_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist > 0:
            dx /= dist
            dy /= dist
        else:
            dx = 1.0
            dy = 0.0

        goal_pose.pose.position.x = target_pose.pose.position.x + distance * dx
        goal_pose.pose.position.y = target_pose.pose.position.y + distance * dy
        goal_pose.pose.position.z = 0.0

        yaw = math.atan2(
            target_pose.pose.position.y - goal_pose.pose.position.y,
            target_pose.pose.position.x - goal_pose.pose.position.x
        )

        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        return goal_pose
    
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
        from rclpy.action import GoalStatus
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

