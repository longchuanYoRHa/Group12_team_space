#!/usr/bin/env python3

"""
arm_client.py — Arm Communication Client (Runs on NUC)

Acts as a bridge node between the NUC and the robotic arm's Raspberry Pi:
- Encapsulates all communication with the arm control node (mycobot_controller_tf2)
- Provides a ROS 2 Action interface for mission_controller to call
- Aggregates arm status and publishes ArmStatus messages
- Supports emergency stop

The arm control node (mycobot_controller_tf2) runs on the arm's built-in Raspberry Pi,
communicating over the network via ROS 2 DDS (NUC and Pi are in the same subnet/ROS_DOMAIN_ID).

Communication Interfaces (corresponding to mycobot_controller_tf2):
  Publishers:
    /arm/target_pick   (geometry_msgs/Point) -> Triggers arm pick-up
    /arm/target_place  (geometry_msgs/Point) -> Triggers arm placement
  Subscriptions:
    /arm/status         (std_msgs/String)     <- Arm state machine status
    /arm/gripper_status (std_msgs/String)     <- Gripper status
    /arm/joint_states   (sensor_msgs/JointState) <- Joint angles
  Publishers (Output of this node):
    /arm_client/arm_status (maze_core_interfaces/ArmStatus) — Aggregated status
  Action Server:
    /arm_client/pick_and_place (maze_core_interfaces/PickAndPlace)
"""

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_group import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from maze_core_interfaces.msg import ArmStatus
from maze_core_interfaces.action import PickAndPlace


class ArmClient(Node):
    """
    Robotic Arm Client Node.

    Encapsulates communication with mycobot_controller_tf2 on the arm's Pi,
    providing a high-level Action interface for the mission_controller.
    """

    def __init__(self):
        super().__init__('arm_client')

        # ---- Parameters -----------------------------------------------------------
        self.declare_parameter('arm_namespace', '/arm')
        self.declare_parameter('pick_timeout', 60.0)
        self.declare_parameter('place_timeout', 60.0)
        self.declare_parameter('status_publish_hz', 5.0)

        self._arm_ns = self.get_parameter('arm_namespace').value
        self._pick_timeout = self.get_parameter('pick_timeout').value
        self._place_timeout = self.get_parameter('place_timeout').value
        self._status_hz = self.get_parameter('status_publish_hz').value

        # ---- Internal States -------------------------------------------------------
        self._arm_state = 'unknown'
        self._gripper_status = 'unknown'
        self._joint_angles = [0.0] * 6
        self._arm_connected = False
        self._last_status_time = 0.0
        self._lock = threading.Lock()

        cb_group = ReentrantCallbackGroup()

        # ---- Publishers for sending commands to the arm ---------------------------
        self.pick_pub = self.create_publisher(
            Point, f'{self._arm_ns}/target_pick', 10)
        self.place_pub = self.create_publisher(
            Point, f'{self._arm_ns}/target_place', 10)

        # ---- Subscribers for receiving status from the arm ------------------------
        self.create_subscription(
            String, f'{self._arm_ns}/status',
            self._arm_status_cb, 10, callback_group=cb_group)
        self.create_subscription(
            String, f'{self._arm_ns}/gripper_status',
            self._gripper_cb, 10, callback_group=cb_group)
        self.create_subscription(
            JointState, f'{self._arm_ns}/joint_states',
            self._joint_state_cb, 10, callback_group=cb_group)

        # ---- Aggregated Status Publisher ------------------------------------------
        self.arm_status_pub = self.create_publisher(
            ArmStatus, 'arm_status', 10)

        # ---- Action Server --------------------------------------------------------
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            execute_callback=self._execute_pick_and_place,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group
        )

        # ---- Timers ---------------------------------------------------------------
        self.create_timer(1.0 / self._status_hz, self._publish_arm_status)
        self.create_timer(2.0, self._check_connection)

        self.get_logger().info('ArmClient started')
        self.get_logger().info(f'  Arm Namespace: {self._arm_ns}')
        self.get_logger().info(f'  Waiting for arm to come online...')

    # ======================================================================
    # Arm Status Callbacks
    # ======================================================================
    def _arm_status_cb(self, msg: String):
        with self._lock:
            self._arm_state = msg.data
            self._last_status_time = time.time()
            self._arm_connected = True

    def _gripper_cb(self, msg: String):
        with self._lock:
            self._gripper_status = msg.data

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            if len(msg.position) == 6:
                self._joint_angles = list(msg.position)

    def _check_connection(self):
        """Check if the connection with the arm is healthy."""
        with self._lock:
            if self._arm_connected and \
               (time.time() - self._last_status_time > 5.0):
                self._arm_connected = False
                self.get_logger().warn('Arm connection lost!')

    # ======================================================================
    # Status Publishing
    # ======================================================================
    def _publish_arm_status(self):
        with self._lock:
            msg = ArmStatus()
            msg.state = self._arm_state
            msg.is_holding = (self._arm_state == 'holding')
            msg.is_idle = (self._arm_state == 'idle')
            msg.has_error = (self._arm_state == 'error')
            msg.gripper_status = self._gripper_status
            msg.joint_angles = self._joint_angles
            self.arm_status_pub.publish(msg)

    # ======================================================================
    # Public Methods (for mission_controller or action calls)
    # ======================================================================
    def send_pick(self, x: float, y: float, z: float):
        """Send pick coordinates to the arm (base_link frame, m)."""
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = z
        self.pick_pub.publish(pt)
        self.get_logger().info(f'Sent target_pick: ({x:.1f}, {y:.1f}, {z:.1f})')

    def send_place(self, x: float, y: float, z: float):
        """Send place coordinates to the arm (base_link frame, m)."""
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = z
        self.place_pub.publish(pt)
        self.get_logger().info(f'Sent target_place: ({x:.1f}, {y:.1f}, {z:.1f})')

    def wait_for_state(self, target_state: str, timeout: float = 30.0) -> bool:
        """Block until the arm enters the target state."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._arm_state == target_state:
                    return True
                if self._arm_state == 'error':
                    return False
            time.sleep(0.1)
        return False

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._arm_connected

    @property
    def state(self) -> str:
        with self._lock:
            return self._arm_state

    @property
    def is_idle(self) -> bool:
        with self._lock:
            return self._arm_state == 'idle'

    @property
    def is_holding(self) -> bool:
        with self._lock:
            return self._arm_state == 'holding'

    # ======================================================================
    # Action Server: PickAndPlace
    # ======================================================================
    def _goal_callback(self, goal_request):
        """Determine whether to accept the pick_and_place goal."""
        self.get_logger().info(
            f'Received PickAndPlace request: color={goal_request.color}')

        with self._lock:
            if not self._arm_connected:
                self.get_logger().warn('Rejected: Arm not connected')
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('PickAndPlace cancel request received')
        return CancelResponse.ACCEPT

    def _execute_pick_and_place(self, goal_handle):
        """
        Execute the complete pick-and-place workflow:
        1. Send target_pick -> Wait for 'holding' state
        2. Send target_place -> Wait for 'idle' state
        """
        request = goal_handle.request
        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        start_time = time.time()

        self.get_logger().info(
            f'Starting PickAndPlace: color={request.color}')

        # ---- Phase 1: Pick --------------------------------------------------
        feedback.phase = 'waiting_arm_idle'
        feedback.progress = 0.0
        feedback.message = 'Waiting for arm to be idle'
        goal_handle.publish_feedback(feedback)

        if not self.wait_for_state('idle', timeout=10.0):
            result.success = False
            result.color = request.color
            result.message = f'Arm not ready (Current state: {self.state})'
            result.duration = time.time() - start_time
            goal_handle.abort()
            return result

        feedback.phase = 'picking'
        feedback.progress = 0.1
        feedback.message = 'Sending pick command'
        goal_handle.publish_feedback(feedback)

        self.send_pick(
            request.pick_position.x,
            request.pick_position.y,
            request.pick_position.z)

        # Wait for arm movement -> holding
        poll_start = time.time()
        while time.time() - poll_start < self._pick_timeout:
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'Canceled'
                result.duration = time.time() - start_time
                goal_handle.canceled()
                return result

            with self._lock:
                current = self._arm_state

            if current == 'holding':
                break
            if current == 'error':
                result.success = False
                result.color = request.color
                result.message = 'Arm pick error'
                result.duration = time.time() - start_time
                goal_handle.abort()
                return result

            # Update progress
            elapsed = time.time() - poll_start
            feedback.progress = min(0.1 + 0.4 * (elapsed / self._pick_timeout), 0.49)
            feedback.phase = f'picking ({current})'
            feedback.message = f'Arm status: {current}'
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)
        else:
            result.success = False
            result.color = request.color
            result.message = 'Pick timeout'
            result.duration = time.time() - start_time
            goal_handle.abort()
            return result

        feedback.phase = 'pick_complete'
        feedback.progress = 0.5
        feedback.message = 'Object picked, preparing to place'
        goal_handle.publish_feedback(feedback)

        # ---- Phase 2: Place -------------------------------------------------
        feedback.phase = 'placing'
        feedback.progress = 0.55
        feedback.message = 'Sending place command'
        goal_handle.publish_feedback(feedback)

        self.send_place(
            request.place_position.x,
            request.place_position.y,
            request.place_position.z)

        poll_start = time.time()
        while time.time() - poll_start < self._place_timeout:
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'Canceled'
                result.duration = time.time() - start_time
                goal_handle.canceled()
                return result

            with self._lock:
                current = self._arm_state

            if current == 'idle':
                break
            if current == 'error':
                result.success = False
                result.color = request.color
                result.message = 'Arm place error'
                result.duration = time.time() - start_time
                goal_handle.abort()
                return result

            elapsed = time.time() - poll_start
            feedback.progress = min(0.55 + 0.4 * (elapsed / self._place_timeout), 0.95)
            feedback.phase = f'placing ({current})'
            feedback.message = f'Arm status: {current}'
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)
        else:
            result.success = False
            result.color = request.color
            result.message = 'Place timeout'
            result.duration = time.time() - start_time
            goal_handle.abort()
            return result

        # ---- Completion -----------------------------------------------------
        result.success = True
        result.color = request.color
        result.message = 'Pick and Place completed'
        result.duration = time.time() - start_time
        goal_handle.succeed()

        self.get_logger().info(
            f'PickAndPlace finished: {request.color}, '
            f'duration {result.duration:.1f}s')
        return result


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ArmClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
