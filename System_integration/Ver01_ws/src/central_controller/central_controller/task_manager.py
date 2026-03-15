#!/usr/bin/env python3
"""
Task manager state machine node.
Implements the full Explore-Pick-Stow-SearchBin-Place workflow.

This node is the core scheduler of the system, responsible for:
1. Controlling the start and stop of exploration behavior (explore_lite)
2. Subscribing to rover_vision_node topics (/target_pick/*, /target_place/*)
3. Coordinating Nav2 navigation for goal execution
4. Invoking arm manipulation actions (grasp, place)
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
import time

from central_controller.task_manager_utils import (
    is_pose_in_blacklist as check_pose_in_blacklist,
    compute_pregrasp_pose,
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


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')

        # ========== State variables ==========
        self.current_state = TaskState.INIT  # Current task state
        self.cargo_state = CargoState.EMPTY  # Cargo state (empty/loaded)
        self.home_pose = None  # Start pose for return after task
        self.object_pose = None  # Detected object pose (map frame)
        self.bin_pose = None  # Detected bin pose (map frame)

        # ========== Arm state variables ==========
        self.arm_status = "idle"
        self.gripper_status = "unknown"
        self._arm_cmd_sent = False

        # ========== Detection stability counters ==========
        # Multi-frame confirmation to reduce false positives
        self.object_detection_count = 0  # Consecutive object detection frames
        self.bin_detection_count = 0  # Consecutive bin detection frames
        self.required_detection_frames = 5  # Frames required for stable detection (N-frame confirm)

        # ========== Retry counters ==========
        # For failure recovery
        self.grasp_retry_count = 0  # Grasp retry count
        self.max_grasp_retries = 2  # Max grasp retries
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
        # Control explore_lite start/stop via /explore/resume topic (publish only on change)
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, 'explore/resume', 10
        )
        self._last_explore_resume = None  # Track last published value to avoid redundant publishes
        # Publish current state (for monitoring and debugging)
        self.state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/state', 10
        )
        # Publish cargo state (for monitoring and debugging)
        self.cargo_state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/cargo_state', 10
        )
        
        # Arm command publishers for pick and place coordinates (Point in base_link frame, in mm for mycobot)
        self.arm_pick_pub = self.create_publisher(
            geometry_msgs.Point, '/arm/target_pick', 10
        )
        self.arm_place_pub = self.create_publisher(
            geometry_msgs.Point, '/arm/target_place', 10
        )
        
        # ========== Subscribers (consistent with rover_vision_node topics)==========
        # Grasp target: /target_pick/{red,green,blue}, message type geometry_msgs.Point
        for topic in ['/target_pick/red', '/target_pick/green', '/target_pick/blue']:
            self.create_subscription(
                geometry_msgs.Point,
                topic,
                self.object_point_callback,
                qos_profile_sensor_data
            )
        # Place target: /target_place/{red,green,blue}, message type geometry_msgs.Point
        for topic in ['/target_place/red', '/target_place/green', '/target_place/blue']:
            self.create_subscription(
                geometry_msgs.Point,
                topic,
                self.bin_point_callback,
                qos_profile_sensor_data
            )
        
        # Arm status subscribers (from manipulator node)
        # Arm status: /arm/status (e.g. 'idle', 'holding', 'error')
        self.arm_status_sub = self.create_subscription(
            std_msgs.String, '/arm/status', self.arm_status_callback, 10
        )
        # Gripper status: /arm/gripper_status (e.g. 'object_held', 'unknown')
        self.arm_gripper_status_sub = self.create_subscription(
            std_msgs.String, '/arm/gripper_status', self.arm_gripper_status_callback, 10
        )

        #TODO: create service for other node to check task state

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
        self.declare_parameter('camera_frame_id', 'camera_depth_optical_frame')  # Vision point frame

        self.get_logger().info('Task manager node initialized')

    def arm_status_callback(self, msg):
        self.arm_status = msg.data.lower()

    def arm_gripper_status_callback(self, msg):
        self.gripper_status = msg.data.lower()

    def _get_point_in_base_link_mm(self, pose_stamped):
        """Transform a pose to base_link and convert to mm for the arm controller."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                pose_stamped.header.frame_id, 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            pose_in_base = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
            pt = geometry_msgs.Point()
            # mycobot requires mm
            pt.x = pose_in_base.position.x * 1000.0
            pt.y = pose_in_base.position.y * 1000.0
            pt.z = pose_in_base.position.z * 1000.0
            return pt
        except Exception as e:
            self.get_logger().error(f'TF transform to base_link failed: {e}')
            return None

    def _point_to_pose_stamped_in_map(self, point_msg):
        """
        Transform geometry_msgs.Point (from vision, in camera frame) to PoseStamped in map frame.
        Use parameter camera_frame_id as source frame.
        """
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

    def object_point_callback(self, msg):
        """
        Object (graspable cube) detection callback.
        Corresponds to rover_vision_node's /target_pick/red, /target_pick/green, /target_pick/blue,
        Message type is geometry_msgs.Point (x,y,z in camera frame).
        """
        if self.cargo_state != CargoState.EMPTY or self.current_state != TaskState.EXPLORE:
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
                self.current_state = TaskState.OBJECT_FOUND
                self.object_detection_count = 0
        except Exception as e:
            self.get_logger().error(f'Error processing target_pick message: {e}')
            self.object_detection_count = 0

    def bin_point_callback(self, msg):
        """
        Bin (place target) detection callback.
        Corresponds to rover_vision_node's /target_place/red, /target_place/green, /target_place/blue,
        Message type is geometry_msgs.Point (x,y,z in camera frame).
        """
        if self.cargo_state != CargoState.HAS_OBJECT or self.current_state != TaskState.RESUME_EXPLORE_FOR_BIN:
            self.bin_detection_count = 0
            return
        try:
            pose_stamped = self._point_to_pose_stamped_in_map(msg)
            self.bin_pose = pose_stamped
            self.bin_detection_count += 1
            if self.bin_detection_count >= self.required_detection_frames:
                self.get_logger().info('Bin found and confirmed!')
                self.current_state = TaskState.BIN_FOUND
                self.bin_detection_count = 0
        except Exception as e:
            self.get_logger().error(f'Error processing target_place message: {e}')
            self.bin_detection_count = 0
    
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

        Start or resume explore_lite. Object detection is handled asynchronously in object_point_callback (/target_pick/*).
        """
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
            self.get_logger().debug('Object found, waiting for target_pick coords...')
    
    def handle_pause_explore_state(self):
        """
        Handle pause explore state.

        Cancel current Nav2 goal, stop explore_lite, prepare for grasp or place.
        Used for both object grasp and bin place scenarios.
        """
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
        if not self._arm_cmd_sent:
            if self.object_pose is None:
                self.current_state = TaskState.EXPLORE
                return
            
            target_pt = self._get_point_in_base_link_mm(self.object_pose)
            if target_pt is None:
                self.current_state = TaskState.EXPLORE
                return

            self.get_logger().info('Sending pick target to manipulator...')
            self.arm_pick_pub.publish(target_pt)
            self._arm_cmd_sent = True
            return

        # Check async status
        if self.arm_status == 'holding' and self.gripper_status == 'object_held':
            self.get_logger().info('Grasp succeeded!')
            self.cargo_state = CargoState.HAS_OBJECT
            self.grasp_retry_count = 0
            self.adjust_nav2_for_carry_mode(True)
            self._arm_cmd_sent = False
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN

        elif self.arm_status == 'error' or (self.arm_status == 'idle' and self._arm_cmd_sent):
            self.grasp_retry_count += 1
            self._arm_cmd_sent = False
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
        """
        Handle resume explore for bin state.

        Resume explore_lite with bin as target. Bin detection is async in bin_point_callback (/target_place/*).
        """
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

        Execute place action (gripper -> bin). Includes retry on failure.
        """
        if not self._arm_cmd_sent:
            if self.bin_pose is None:
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
                return
                
            target_pt = self._get_point_in_base_link_mm(self.bin_pose)
            if target_pt is None:
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
                return
            
            self.get_logger().info('Sending place target to manipulator...')
            self.arm_place_pub.publish(target_pt)
            self._arm_cmd_sent = True
            return

        if self.arm_status == 'idle':
            self.get_logger().info('Place succeeded!')
            self.cargo_state = CargoState.EMPTY
            self.place_retry_count = 0
            self.adjust_nav2_for_carry_mode(False)
            self._arm_cmd_sent = False
            self.current_state = TaskState.POST_ACTION

        elif self.arm_status == 'error':
            self.place_retry_count += 1
            self._arm_cmd_sent = False
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
    
    def _get_robot_xy_in_map(self):
        """from TF get base_link in map (x, y), return (0.0, 0.0) if failed."""
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
    except Exception as e:
        print(f'Error: {e}')


if __name__ == '__main__': 
    main()

