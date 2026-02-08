#!/usr/bin/env python3
"""
Task Manager State Machine Node
Implements Explore-Pick-Stow-SearchBin-Place workflow
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
import tf2_ros
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
from enum import Enum
import math
import time


class CargoState(Enum):
    """Cargo state enumeration"""
    EMPTY = "empty"
    HAS_OBJECT = "has_object"


class TaskState(Enum):
    """Task state enumeration"""
    INIT = "init"
    EXPLORE = "explore"
    OBJECT_FOUND = "object_found"
    PAUSE_EXPLORE = "pause_explore"
    NAV_TO_OBJECT_PREGRASP = "nav_to_object_pregrasp"
    PRECISION_ALIGN_OBJECT = "precision_align_object"
    GRASP = "grasp"
    STOW_ON_ROBOT = "stow_on_robot"
    RESUME_EXPLORE_FOR_BIN = "resume_explore_for_bin"
    BIN_FOUND = "bin_found"
    NAV_TO_BIN_PREPLACE = "nav_to_bin_preplace"
    PRECISION_ALIGN_BIN = "precision_align_bin"
    PLACE_IN_BIN = "place_in_bin"
    POST_ACTION = "post_action"


class TaskManagerNode(Node):
    """Task Manager State Machine Node"""
    
    def __init__(self):
        super().__init__('task_manager')
        
        # State variables
        self.current_state = TaskState.INIT
        self.cargo_state = CargoState.EMPTY
        self.home_pose = None
        self.object_pose = None
        self.bin_pose = None
        self.stow_pose = None  # Fixed stow pose in arm_base frame
        
        # Detection stability counters
        self.object_detection_count = 0
        self.bin_detection_count = 0
        self.required_detection_frames = 5  # N frames for stable detection
        
        # Retry counters
        self.grasp_retry_count = 0
        self.max_grasp_retries = 2
        self.stow_retry_count = 0
        self.max_stow_retries = 2
        self.place_retry_count = 0
        self.max_place_retries = 2
        
        # Blacklist for failed objects
        self.object_blacklist = []
        self.blacklist_radius = 0.3  # meters
        
        # Action clients
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None
        
        # Publishers
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, 'explore/resume', 10
        )
        self.state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/state', 10
        )
        self.cargo_state_pub = self.create_publisher(
            std_msgs.String, 'task_manager/cargo_state', 10
        )
        
        # Subscribers
        # TODO: Replace with actual object_detector topic
        self.object_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'object_detector/object_pose',  # Pseudo topic
            self.object_pose_callback,
            qos_profile_sensor_data
        )
        
        # TODO: Replace with actual bin_detector topic
        self.bin_pose_sub = self.create_subscription(
            geometry_msgs.PoseStamped,
            'bin_detector/bin_pose',  # Pseudo topic
            self.bin_pose_callback,
            qos_profile_sensor_data
        )
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for state machine execution
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)
        
        # Parameters
        self.declare_parameter('pregrasp_distance', 0.5)  # meters
        self.declare_parameter('preplace_distance', 0.6)  # meters
        self.declare_parameter('stow_pose_x', 0.3)
        self.declare_parameter('stow_pose_y', 0.0)
        self.declare_parameter('stow_pose_z', 0.2)
        
        self.get_logger().info('Task Manager Node initialized')
        
    def object_pose_callback(self, msg):
        """Callback for object pose detection"""
        if self.cargo_state == CargoState.EMPTY and self.current_state == TaskState.EXPLORE:
            # Check if object is in blacklist
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
    
    def bin_pose_callback(self, msg):
        """Callback for bin pose detection"""
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
        """Check if position is in blacklist"""
        for blacklist_pos in self.object_blacklist:
            distance = math.sqrt(
                (position.x - blacklist_pos.x)**2 +
                (position.y - blacklist_pos.y)**2
            )
            if distance < self.blacklist_radius:
                return True
        return False
    
    def state_machine_callback(self):
        """Main state machine execution"""
        # Publish current state
        state_msg = std_msgs.String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)
        
        cargo_msg = std_msgs.String()
        cargo_msg.data = self.cargo_state.value
        self.cargo_state_pub.publish(cargo_msg)
        
        # State machine logic
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
        """Handle INIT state"""
        # Wait for system ready (TF/SLAM/Nav2)
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Nav2 server...')
            return
        
        # Save home pose
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
        """Handle EXPLORE state"""
        # Start/resume explore_lite
        explore_msg = std_msgs.Bool()
        explore_msg.data = True
        self.explore_control_pub.publish(explore_msg)
        
        # Object detection is handled in callback
        # State transition happens in object_pose_callback
    
    def handle_object_found_state(self):
        """Handle OBJECT_FOUND state"""
        self.get_logger().info('Object found, transitioning to PAUSE_EXPLORE')
        self.current_state = TaskState.PAUSE_EXPLORE
    
    def handle_pause_explore_state(self):
        """Handle PAUSE_EXPLORE state"""
        # Cancel Nav2 current goal
        if self.nav2_goal_handle is not None:
            self.nav2_client.cancel_goal_async(self.nav2_goal_handle)
            self.nav2_goal_handle = None
        
        # Stop explore_lite
        explore_msg = std_msgs.Bool()
        explore_msg.data = False
        self.explore_control_pub.publish(explore_msg)
        
        # Transition to navigation
        self.current_state = TaskState.NAV_TO_OBJECT_PREGRASP
    
    def handle_nav_to_object_pregrasp_state(self):
        """Handle NAV_TO_OBJECT_PREGRASP state"""
        if self.object_pose is None:
            self.get_logger().error('Object pose not available!')
            self.current_state = TaskState.EXPLORE
            return
        
        # Generate pregrasp navigation point
        pregrasp_distance = self.get_parameter('pregrasp_distance').value
        
        # Calculate position at distance from object, facing object
        goal_pose = self.calculate_pregrasp_pose(self.object_pose, pregrasp_distance)
        
        # Send Nav2 goal
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        if self.nav2_goal_handle is None:
            self.get_logger().info('Sending Nav2 goal to object pregrasp position')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            # Check goal status (status values: 1=ACCEPTED, 2=EXECUTING, 3=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED)
            from rclpy.action import GoalStatus
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Reached pregrasp position')
                self.nav2_goal_handle = None
                self.current_state = TaskState.PRECISION_ALIGN_OBJECT
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.get_logger().warn('Navigation failed, returning to EXPLORE')
                self.nav2_goal_handle = None
                self.current_state = TaskState.EXPLORE
    
    def handle_precision_align_object_state(self):
        """Handle PRECISION_ALIGN_OBJECT state (Optional)"""
        # TODO: Implement precision alignment using D435i
        # For now, skip to GRASP
        self.get_logger().info('Precision alignment (pseudo code - skip to GRASP)')
        self.current_state = TaskState.GRASP
    
    def handle_grasp_state(self):
        """Handle GRASP state"""
        # TODO: Call grasp action (pseudo code)
        self.get_logger().info('Calling grasp action (pseudo code)')
        
        # Pseudo code: Call grasp_server
        # grasp_success = self.call_grasp_action(self.object_pose)
        grasp_success = True  # Placeholder
        
        if grasp_success:
            self.get_logger().info('Grasp successful!')
            self.cargo_state = CargoState.HAS_OBJECT
            self.grasp_retry_count = 0
            self.current_state = TaskState.STOW_ON_ROBOT
        else:
            self.grasp_retry_count += 1
            if self.grasp_retry_count >= self.max_grasp_retries:
                self.get_logger().warn('Grasp failed after retries, abandoning object')
                # Add to blacklist
                if self.object_pose:
                    self.object_blacklist.append(self.object_pose.pose.position)
                self.grasp_retry_count = 0
                self.current_state = TaskState.EXPLORE
            else:
                self.get_logger().info(f'Grasp failed, retrying ({self.grasp_retry_count}/{self.max_grasp_retries})')
                # Retry precision alignment
                self.current_state = TaskState.PRECISION_ALIGN_OBJECT
    
    def handle_stow_on_robot_state(self):
        """Handle STOW_ON_ROBOT state"""
        # TODO: Call stow action (pseudo code)
        self.get_logger().info('Calling stow action (pseudo code)')
        
        # Pseudo code: Call stow_server
        # stow_success = self.call_stow_action()
        stow_success = True  # Placeholder
        
        if stow_success:
            self.get_logger().info('Stow successful!')
            self.stow_retry_count = 0
            # Adjust Nav2 parameters for carry mode
            self.adjust_nav2_for_carry_mode(True)
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
        else:
            self.stow_retry_count += 1
            if self.stow_retry_count >= self.max_stow_retries:
                self.get_logger().warn('Stow failed after retries')
                # Option: place back or continue holding
                self.stow_retry_count = 0
                # For now, continue holding and proceed
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            else:
                self.get_logger().info(f'Stow failed, retrying ({self.stow_retry_count}/{self.max_stow_retries})')
                # Retry from grasp state
                self.current_state = TaskState.GRASP
    
    def handle_resume_explore_for_bin_state(self):
        """Handle RESUME_EXPLORE_FOR_BIN state"""
        # Resume explore_lite
        explore_msg = std_msgs.Bool()
        explore_msg.data = True
        self.explore_control_pub.publish(explore_msg)
        
        # Bin detection is handled in callback
        # State transition happens in bin_pose_callback
    
    def handle_bin_found_state(self):
        """Handle BIN_FOUND state"""
        self.get_logger().info('Bin found, transitioning to PAUSE_EXPLORE')
        self.current_state = TaskState.PAUSE_EXPLORE
    
    def handle_nav_to_bin_preplace_state(self):
        """Handle NAV_TO_BIN_PREPLACE state"""
        if self.bin_pose is None:
            self.get_logger().error('Bin pose not available!')
            self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            return
        
        # Generate preplace navigation point
        preplace_distance = self.get_parameter('preplace_distance').value
        
        # Calculate position at distance from bin, facing bin
        goal_pose = self.calculate_pregrasp_pose(self.bin_pose, preplace_distance)
        
        # Send Nav2 goal
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        if self.nav2_goal_handle is None:
            self.get_logger().info('Sending Nav2 goal to bin preplace position')
            send_goal_future = self.nav2_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav2_goal_response_callback)
        else:
            # Check goal status
            from rclpy.action import GoalStatus
            status = self.nav2_goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Reached preplace position')
                self.nav2_goal_handle = None
                self.current_state = TaskState.PRECISION_ALIGN_BIN
            elif status in [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED]:
                self.get_logger().warn('Navigation failed, returning to EXPLORE')
                self.nav2_goal_handle = None
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
    
    def handle_precision_align_bin_state(self):
        """Handle PRECISION_ALIGN_BIN state"""
        # TODO: Implement precision alignment using D435i
        # For now, skip to PLACE
        self.get_logger().info('Precision alignment bin (pseudo code - skip to PLACE)')
        self.current_state = TaskState.PLACE_IN_BIN
    
    def handle_place_in_bin_state(self):
        """Handle PLACE_IN_BIN state"""
        # TODO: Call place action (pseudo code)
        self.get_logger().info('Calling place action (pseudo code)')
        
        # Pseudo code: Call place_server
        # place_success = self.call_place_action(self.bin_pose)
        place_success = True  # Placeholder
        
        if place_success:
            self.get_logger().info('Place successful!')
            self.cargo_state = CargoState.EMPTY
            self.place_retry_count = 0
            # Restore Nav2 parameters
            self.adjust_nav2_for_carry_mode(False)
            self.current_state = TaskState.POST_ACTION
        else:
            self.place_retry_count += 1
            if self.place_retry_count >= self.max_place_retries:
                self.get_logger().warn('Place failed after retries, resuming exploration')
                self.place_retry_count = 0
                self.current_state = TaskState.RESUME_EXPLORE_FOR_BIN
            else:
                self.get_logger().info(f'Place failed, retrying ({self.place_retry_count}/{self.max_place_retries})')
                # Retry precision alignment
                self.current_state = TaskState.PRECISION_ALIGN_BIN
    
    def handle_post_action_state(self):
        """Handle POST_ACTION state"""
        # Option: Return to exploration or return home
        self.get_logger().info('Post action: returning to exploration')
        self.current_state = TaskState.EXPLORE
    
    def calculate_pregrasp_pose(self, target_pose, distance):
        """Calculate pregrasp/preplace pose at distance from target, facing target"""
        goal_pose = geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Calculate direction vector from target to robot (simplified: assume robot at origin)
        # In practice, get current robot pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except:
            robot_x = 0.0
            robot_y = 0.0
        
        # Vector from target to robot
        dx = robot_x - target_pose.pose.position.x
        dy = robot_y - target_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist > 0:
            # Normalize
            dx /= dist
            dy /= dist
        else:
            dx = 1.0
            dy = 0.0
        
        # Goal position: target position + distance * direction (toward robot)
        goal_pose.pose.position.x = target_pose.pose.position.x + distance * dx
        goal_pose.pose.position.y = target_pose.pose.position.y + distance * dy
        goal_pose.pose.position.z = 0.0
        
        # Orientation: face target
        yaw = math.atan2(
            target_pose.pose.position.y - goal_pose.pose.position.y,
            target_pose.pose.position.x - goal_pose.pose.position.x
        )
        
        # Convert yaw to quaternion
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal_pose
    
    def nav2_goal_response_callback(self, future):
        """Callback for Nav2 goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self.nav2_goal_handle = None
            return
        
        self.get_logger().info('Nav2 goal accepted')
        self.nav2_goal_handle = goal_handle
        
        # Get result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)
    
    def nav2_result_callback(self, future):
        """Callback for Nav2 goal result"""
        from rclpy.action import GoalStatus
        goal_handle = future.result()
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded')
        elif goal_handle.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 goal aborted')
        elif goal_handle.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal canceled')
        # Status check is also done in state handlers for state transitions
    
    def adjust_nav2_for_carry_mode(self, enable):
        """Adjust Nav2 parameters for carry mode"""
        # TODO: Implement Nav2 parameter adjustment
        # This could be done via:
        # 1. Dynamic reconfigure
        # 2. Service calls to Nav2
        # 3. Switching costmap parameters
        if enable:
            self.get_logger().info('Enabling carry mode: reduce speed, increase inflation')
            # Pseudo code:
            # nav2_params.max_vel_x = 0.3  # Reduced from default
            # nav2_params.inflation_radius = 0.5  # Increased
        else:
            self.get_logger().info('Disabling carry mode: restore normal parameters')
            # Pseudo code:
            # nav2_params.max_vel_x = 0.5  # Restore default
            # nav2_params.inflation_radius = 0.3  # Restore default


def main(args=None):
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

