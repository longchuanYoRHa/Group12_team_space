#!/usr/bin/env python3

"""
ROS2 control node for MyCobot 280 Pi with TF2 coordinate transforms.

This version uses TF2 to handle all coordinate transformations:
- Camera to arm base
- Gripper offset compensation
- Support for moving rover

Subscribes:
  - target_pick  (geometry_msgs/Point)  -- 3D pick coords in camera frame
  - target_place (geometry_msgs/Point)  -- 3D place coords in camera frame

Publishes:
  - status         (std_msgs/String)
  - joint_states   (sensor_msgs/JointState)
  - gripper_status (std_msgs/String)

TF2 Frames Expected:
  camera_link -> arm_base_link (or via rover_base)
  joint6_flange -> gripper_base -> gripper_tip
"""

import os
import json
import math
import time
import threading
from enum import Enum, auto
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# TF2 imports
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


# ---------------------------------------------------------------------------
# State machine (same as before)
# ---------------------------------------------------------------------------
class ArmState(Enum):
    IDLE = auto()
    MOVING_TO_PICK = auto()
    DESCENDING_PICK = auto()
    GRIPPING = auto()
    GRIP_CHECK = auto()
    LIFTING = auto()
    RETURNING_HOME = auto()
    HOLDING = auto()
    MOVING_TO_PLACE = auto()
    RELEASING = auto()
    ERROR = auto()


# ---------------------------------------------------------------------------
# Mock for development
# ---------------------------------------------------------------------------
class MockMyCobot280:
    """Drop-in mock for testing without hardware."""

    def __init__(self, port, baud):
        self._angles = [0.0] * 6
        self._gripper_val = 0
        print(f"[MOCK] MyCobot280 on {port} @ {baud}")

    def is_controller_connected(self):
        return 1

    def power_on(self):
        print("[MOCK] power_on")

    def send_angles(self, angles, speed):
        print(f"[MOCK] send_angles({angles}, {speed})")
        self._angles = list(angles)

    def send_coords(self, coords, speed, mode):
        label = "LINEAR" if mode == 1 else "JOINT"
        print(f"[MOCK] send_coords({coords}, {speed}, {label})")

    def send_coord(self, axis, value, speed):
        print(f"[MOCK] send_coord(axis={axis}, val={value}, spd={speed})")

    def get_angles(self):
        return self._angles

    def get_coords(self):
        return [0.0, 0.0, 200.0, -180.0, 0.0, 0.0]

    def is_moving(self):
        return 0

    def stop(self):
        print("[MOCK] stop")

    def set_gripper_state(self, state, speed):
        self._gripper_val = 15 if state == 1 else 100
        print(f"[MOCK] gripper {'CLOSE' if state == 1 else 'OPEN'} spd={speed}")

    def get_gripper_value(self, gripper_type=1):
        return self._gripper_val

    def set_HTS_gripper_torque(self, torque):
        print(f"[MOCK] set_HTS_gripper_torque({torque})")

    def get_error_information(self):
        return 0

    def clear_error_information(self):
        print("[MOCK] clear_error_information")

    def release_all_servos(self):
        print("[MOCK] release_all_servos")

    def focus_all_servos(self):
        print("[MOCK] focus_all_servos")


# ---------------------------------------------------------------------------
# Main ROS 2 node with TF2
# ---------------------------------------------------------------------------
class MyCobotControllerTF2(Node):

    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    COORD_LIMITS = {
        'x': (-281.45, 281.45),
        'y': (-281.45, 281.45),
        'z': (-70.0, 450.0),
    }

    def __init__(self):
        super().__init__('mycobot_controller_tf2')

        # ---- ROS parameters ------------------------------------------------
        self.declare_parameter('safe_z', 250.0)
        self.declare_parameter('move_speed', 40)
        self.declare_parameter('gripper_speed', 80)
        self.declare_parameter('end_rx', -178.63)
        self.declare_parameter('end_ry', 4.73)
        self.declare_parameter('end_rz', -85.94)
        self.declare_parameter('gripper_torque', 300)
        self.declare_parameter('grip_threshold', 25)
        self.declare_parameter('grip_check_retries', 2)
        self.declare_parameter('home_angles', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('calibration_file', '')
        
        # TF2 related parameters
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('arm_base_frame', 'arm_base_link')
        self.declare_parameter('flange_frame', 'joint6_flange')
        self.declare_parameter('gripper_tip_frame', 'gripper_tip')
        self.declare_parameter('tf_timeout', 1.0)  # seconds
        self.declare_parameter('compensate_gripper_offset', True)
        self.declare_parameter('gripper_offset_z', 80.0)  # mm, fallback if TF2 fails

        self._safe_z = self.get_parameter('safe_z').value
        self._move_speed = self.get_parameter('move_speed').value
        self._gripper_speed = self.get_parameter('gripper_speed').value
        self._end_rpy = [
            self.get_parameter('end_rx').value,
            self.get_parameter('end_ry').value,
            self.get_parameter('end_rz').value,
        ]
        self._gripper_torque = self.get_parameter('gripper_torque').value
        self._grip_threshold = self.get_parameter('grip_threshold').value
        self._grip_retries = self.get_parameter('grip_check_retries').value
        self._home_angles = list(self.get_parameter('home_angles').value)

        # TF2 frames
        self._camera_frame = self.get_parameter('camera_frame').value
        self._arm_base_frame = self.get_parameter('arm_base_frame').value
        self._flange_frame = self.get_parameter('flange_frame').value
        self._gripper_tip_frame = self.get_parameter('gripper_tip_frame').value
        self._tf_timeout = self.get_parameter('tf_timeout').value
        self._compensate_gripper = self.get_parameter('compensate_gripper_offset').value
        self._gripper_offset_z = self.get_parameter('gripper_offset_z').value

        # ---- Load calibration -----------------------------------------------
        self._calibration = {}
        cal_file = self.get_parameter('calibration_file').value
        if cal_file and os.path.exists(cal_file):
            self._load_calibration(cal_file)
        else:
            self._auto_load_calibration()

        if 'home' in self._calibration:
            cal_home = self._calibration['home'].get('angles', [])
            if len(cal_home) == 6:
                self._home_angles = cal_home
        if 'pickup' in self._calibration:
            cal_coords = self._calibration['pickup'].get('coords', [])
            if len(cal_coords) == 6:
                self._end_rpy = cal_coords[3:6]

        # ---- TF2 setup ------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Give TF2 time to receive initial transforms
        self.get_logger().info('Waiting for TF2 transforms to become available...')
        time.sleep(0.5)

        # ---- Hardware -------------------------------------------------------
        hw_port = "/dev/ttyAMA0"
        hw_baud = 1000000
        use_sim = not os.path.exists(hw_port)
        if use_sim:
            self.get_logger().warn('No hardware -- running in MOCK mode')
            self.mc = MockMyCobot280(hw_port, hw_baud)
        else:
            self.get_logger().info(f'Connecting to real arm on {hw_port}')
            from pymycobot import MyCobot280
            self.mc = MyCobot280(hw_port, hw_baud)

        self.mc.power_on()
        time.sleep(0.5)
        self._hw_lock = threading.Lock()

        try:
            with self._hw_lock:
                self.mc.set_HTS_gripper_torque(self._gripper_torque)
        except Exception:
            pass

        # ---- State machine --------------------------------------------------
        self._state = ArmState.IDLE
        self._task_thread = None

        # ---- Publishers -----------------------------------------------------
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.gripper_pub = self.create_publisher(String, 'gripper_status', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)

        # ---- Subscribers ----------------------------------------------------
        self.create_subscription(Point, 'target_pick', self._pick_cb, 10)
        self.create_subscription(Point, 'target_place', self._place_cb, 10)

        # ---- Timers ---------------------------------------------------------
        self.create_timer(0.1, self._publish_joint_states)

        self._publish_status()
        self.get_logger().info('MyCobot TF2 controller ready')
        self.get_logger().info(f'  Camera frame    : {self._camera_frame}')
        self.get_logger().info(f'  Arm base frame  : {self._arm_base_frame}')
        self.get_logger().info(f'  Flange frame    : {self._flange_frame}')
        self.get_logger().info(f'  Gripper tip frame: {self._gripper_tip_frame}')
        self.get_logger().info(f'  TF timeout      : {self._tf_timeout} s')
        self.get_logger().info(f'  Gripper offset  : {"TF2" if self._compensate_gripper else "manual"} ({self._gripper_offset_z} mm)' )
        self.get_logger().info('Waiting for target_pick ...')

    # ======================================================================
    # Calibration
    # ======================================================================
    def _load_calibration(self, filepath):
        try:
            with open(filepath, 'r') as f:
                self._calibration = json.load(f)
            self.get_logger().info(f'Loaded calibration: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    def _auto_load_calibration(self):
        cal_dir = Path(__file__).parent / 'calibration_data'
        if not cal_dir.exists():
            return
        cal_files = sorted(cal_dir.glob('calibration_*.json'))
        if cal_files:
            self._load_calibration(str(cal_files[-1]))

    # ======================================================================
    # TF2 Transform Methods
    # ======================================================================
    def _transform_point_tf2(self, x_src, y_src, z_src, target_frame, source_frame):
        """
        Transform a point from source_frame to target_frame using TF2.
        
        Args:
            x_src, y_src, z_src: Point in source frame (mm)
            target_frame: Target frame name
            source_frame: Source frame name
            
        Returns:
            (x_target, y_target, z_target): Point in target frame (mm)
            Returns None if transform fails
        """
        try:
            # Create PointStamped in source frame (convert mm to m)
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = x_src / 1000.0
            point_stamped.point.y = y_src / 1000.0
            point_stamped.point.z = z_src / 1000.0

            # Lookup transform with timeout
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout)
            )

            # Apply transform
            point_transformed = tf2_geometry_msgs.do_transform_point(
                point_stamped, transform)

            # Convert back to mm
            return (
                point_transformed.point.x * 1000.0,
                point_transformed.point.y * 1000.0,
                point_transformed.point.z * 1000.0
            )

        except Exception as e:
            self.get_logger().error(
                f'TF2 transform failed ({source_frame} -> {target_frame}): {e}')
            return None

    def _transform_camera_to_base(self, x_cam, y_cam, z_cam):
        """
        Transform from camera frame to arm base frame.
        
        Returns coordinates suitable for send_coords (flange position).
        If gripper compensation is enabled, subtracts gripper offset.
        """
        # First, transform to arm base
        result = self._transform_point_tf2(
            x_cam, y_cam, z_cam,
            target_frame=self._arm_base_frame,
            source_frame=self._camera_frame
        )

        if result is None:
            self.get_logger().error('Using camera coords directly (TF2 failed)')
            return x_cam, y_cam, z_cam

        x_base, y_base, z_base = result

        # Compensate for gripper offset
        if self._compensate_gripper:
            # Try to get gripper offset from TF2
            gripper_offset = self._get_gripper_offset_from_tf2()
            if gripper_offset is not None:
                z_flange = z_base - gripper_offset
                self.get_logger().info(
                    f'  Gripper offset from TF2: {gripper_offset:.1f} mm')
            else:
                # Fallback to configured offset
                z_flange = z_base - self._gripper_offset_z
                self.get_logger().info(
                    f'  Gripper offset (config): {self._gripper_offset_z:.1f} mm')
        else:
            z_flange = z_base

        return x_base, y_base, z_flange

    def _get_gripper_offset_from_tf2(self):
        """
        Calculate gripper offset by querying TF2.
        Returns Z offset from flange to gripper tip (in mm).
        """
        try:
            # Get transform from flange to gripper tip
            transform = self.tf_buffer.lookup_transform(
                target_frame=self._gripper_tip_frame,
                source_frame=self._flange_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            # Extract Z offset (in mm)
            z_offset = transform.transform.translation.z * 1000.0
            return z_offset

        except Exception as e:
            self.get_logger().debug(f'Gripper offset TF2 lookup failed: {e}')
            return None

    # ======================================================================
    # Callbacks
    # ======================================================================
    def _pick_cb(self, msg: Point):
        """Receive pick target in camera frame -- only accepted when IDLE."""
        if self._state != ArmState.IDLE:
            self.get_logger().warn(
                f'Ignoring target_pick (state={self._state.name})')
            return

        # Transform from camera to base using TF2
        result = self._transform_camera_to_base(msg.x, msg.y, msg.z)
        if result is None:
            self.get_logger().error('TF2 transform failed, aborting pick')
            return

        x_base, y_base, z_base = result

        # Create transformed point
        target = Point()
        target.x = x_base
        target.y = y_base
        target.z = z_base

        if not self._validate_coords(target):
            return

        self.get_logger().info(
            f'[PICK] Camera coords: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) mm')
        self.get_logger().info(
            f'[PICK] Base/Flange:   ({x_base:.1f}, {y_base:.1f}, {z_base:.1f}) mm')

        self._task_thread = threading.Thread(
            target=self._run_pick, args=(target,), daemon=True)
        self._task_thread.start()

    def _place_cb(self, msg: Point):
        """Receive place target in camera frame -- only accepted when HOLDING."""
        if self._state != ArmState.HOLDING:
            self.get_logger().warn(
                f'Ignoring target_place (state={self._state.name}, need HOLDING)')
            return

        # Transform using TF2
        result = self._transform_camera_to_base(msg.x, msg.y, msg.z)
        if result is None:
            self.get_logger().error('TF2 transform failed, aborting place')
            return

        x_base, y_base, z_base = result

        target = Point()
        target.x = x_base
        target.y = y_base
        target.z = z_base

        if not self._validate_coords(target):
            return

        self.get_logger().info(
            f'[PLACE] Camera coords: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) mm')
        self.get_logger().info(
            f'[PLACE] Base/Flange:   ({x_base:.1f}, {y_base:.1f}, {z_base:.1f}) mm')

        self._task_thread = threading.Thread(
            target=self._run_place, args=(target,), daemon=True)
        self._task_thread.start()

    # ======================================================================
    # Validation
    # ======================================================================
    def _validate_coords(self, pt: Point) -> bool:
        for axis, val in [('x', pt.x), ('y', pt.y), ('z', pt.z)]:
            lo, hi = self.COORD_LIMITS[axis]
            if val < lo or val > hi:
                self.get_logger().error(
                    f'Coordinate {axis}={val:.1f} out of range [{lo}, {hi}]')
                return False
        return True

    # ======================================================================
    # Hardware helpers (same as before)
    # ======================================================================
    def _hw_send_coords(self, coords, speed, mode):
        with self._hw_lock:
            self.mc.send_coords(coords, speed, mode)

    def _hw_send_coord(self, axis, value, speed):
        with self._hw_lock:
            self.mc.send_coord(axis, value, speed)

    def _hw_send_angles(self, angles, speed):
        with self._hw_lock:
            self.mc.send_angles(angles, speed)

    def _hw_set_gripper(self, state, speed):
        with self._hw_lock:
            self.mc.set_gripper_state(state, speed)

    def _hw_get_gripper_value(self):
        with self._hw_lock:
            return self.mc.get_gripper_value(gripper_type=1)

    def _hw_is_moving(self):
        with self._hw_lock:
            return self.mc.is_moving()

    def _hw_stop(self):
        with self._hw_lock:
            self.mc.stop()

    def _hw_check_error(self):
        with self._hw_lock:
            return self.mc.get_error_information()

    def _hw_clear_error(self):
        with self._hw_lock:
            self.mc.clear_error_information()

    def _hw_focus_all(self):
        with self._hw_lock:
            self.mc.focus_all_servos()

    def _wait_until_done(self, timeout=15.0):
        time.sleep(0.5)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self._hw_is_moving() == 0:
                    return True
            except Exception:
                pass
            time.sleep(0.2)
        self.get_logger().warn('Motion timeout reached')
        return False

    def _verify_grip(self):
        time.sleep(1.5)
        val = self._hw_get_gripper_value()
        self.get_logger().info(f'Gripper value after close: {val}')
        if val is None:
            self._publish_gripper('unknown')
            return False
        if val > self._grip_threshold:
            self._publish_gripper('object_held')
            return True
        else:
            self._publish_gripper('no_object')
            return False

    def _set_state(self, state):
        self._state = state
        self._publish_status()
        self.get_logger().info(f'State -> {state.name}')

    def _publish_status(self):
        msg = String()
        msg.data = self._state.name.lower()
        self.status_pub.publish(msg)

    def _publish_gripper(self, text):
        msg = String()
        msg.data = text
        self.gripper_pub.publish(msg)

    def _publish_joint_states(self):
        try:
            with self._hw_lock:
                angles_deg = self.mc.get_angles()
            if not angles_deg or len(angles_deg) != 6:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.JOINT_NAMES
            msg.position = [math.radians(a) for a in angles_deg]
            self.js_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f'Joint state read failed: {e}')

    def _make_coords(self, x, y, z):
        return [x, y, z] + list(self._end_rpy)

    # ======================================================================
    # PICK and PLACE phases (same as before, target is already transformed)
    # ======================================================================
    def _run_pick(self, target: Point):
        spd = self._move_speed
        g_spd = self._gripper_speed

        try:
            err = self._hw_check_error()
            if err and err != 0:
                self.get_logger().warn(f'Clearing error {err}')
                self._hw_clear_error()

            self._set_state(ArmState.MOVING_TO_PICK)
            self.get_logger().info('[PICK 1/6] Open gripper')
            self._hw_set_gripper(0, g_spd)
            time.sleep(1.0)

            above_pick = self._make_coords(target.x, target.y, self._safe_z)
            self.get_logger().info(f'[PICK 2/6] Move above pick: {above_pick}')
            self._hw_send_coords(above_pick, spd, 0)
            self._wait_until_done()
            time.sleep(0.3)

            self._set_state(ArmState.DESCENDING_PICK)
            pick_coords = self._make_coords(target.x, target.y, target.z)
            self.get_logger().info(f'[PICK 3/6] Descend to: {pick_coords}')
            self._hw_send_coords(pick_coords, spd, 1)
            self._wait_until_done()
            time.sleep(0.3)

            self._set_state(ArmState.GRIPPING)
            self.get_logger().info('[PICK 4/6] Closing gripper')
            self._hw_set_gripper(1, g_spd)

            self._set_state(ArmState.GRIP_CHECK)
            grip_ok = False
            for attempt in range(1, self._grip_retries + 1):
                self.get_logger().info(
                    f'[PICK 5/6] Grip check {attempt}/{self._grip_retries}')
                if self._verify_grip():
                    grip_ok = True
                    break
                self.get_logger().warn('  Grip failed, retrying...')
                self._hw_set_gripper(0, g_spd)
                time.sleep(1.0)
                self._hw_set_gripper(1, g_spd)

            if not grip_ok:
                self.get_logger().error('GRIP FAILED -- aborting pick')
                self._set_state(ArmState.ERROR)
                self._cleanup_pick()
                return

            self.get_logger().info('  Grip confirmed!')

            self._set_state(ArmState.LIFTING)
            self.get_logger().info(f'[PICK 6/6] Lifting to z={self._safe_z}')
            self._hw_send_coords(above_pick, spd, 1)
            self._wait_until_done()

            val = self._hw_get_gripper_value()
            self.get_logger().info(f'  Grip after lift: {val}')
            if val is not None and val <= self._grip_threshold:
                self.get_logger().error('Object lost during lift!')
                self._publish_gripper('object_dropped')
                self._set_state(ArmState.ERROR)
                self._cleanup_pick()
                return

            self._set_state(ArmState.RETURNING_HOME)
            self.get_logger().info(f'Returning to home: {self._home_angles}')
            self._hw_send_angles(self._home_angles, spd)
            self._wait_until_done()
            time.sleep(0.5)

            self._hw_focus_all()
            self._set_state(ArmState.HOLDING)
            self._publish_gripper('object_held')
            self.get_logger().info('='*50)
            self.get_logger().info('HOLDING -- block secured')
            self.get_logger().info('='*50)

        except Exception as e:
            self.get_logger().error(f'Pick exception: {e}')
            self._set_state(ArmState.ERROR)
            self._cleanup_pick()

    def _cleanup_pick(self):
        try:
            self._hw_set_gripper(0, self._gripper_speed)
            time.sleep(1.0)
            self._hw_send_coord(3, self._safe_z, self._move_speed)
            self._wait_until_done(timeout=10.0)
            self._hw_send_angles(self._home_angles, self._move_speed)
            self._wait_until_done(timeout=10.0)
        except Exception:
            pass
        time.sleep(1.0)
        self._set_state(ArmState.IDLE)

    def _run_place(self, target: Point):
        spd = self._move_speed
        g_spd = self._gripper_speed

        try:
            err = self._hw_check_error()
            if err and err != 0:
                self.get_logger().warn(f'Clearing error {err}')
                self._hw_clear_error()

            self._set_state(ArmState.MOVING_TO_PLACE)
            above_bin = self._make_coords(target.x, target.y, self._safe_z)
            self.get_logger().info(f'[PLACE 1/3] Moving above bin: {above_bin}')
            self._hw_send_coords(above_bin, spd, 0)
            self._wait_until_done()
            time.sleep(0.3)

            val = self._hw_get_gripper_value()
            self.get_logger().info(f'  Grip before release: {val}')
            if val is not None and val <= self._grip_threshold:
                self.get_logger().error('Object lost during move to bin!')
                self._publish_gripper('object_dropped')
                self._set_state(ArmState.ERROR)
                self._cleanup_place()
                return

            self._set_state(ArmState.RELEASING)
            self.get_logger().info('[PLACE 2/3] Releasing block into bin')
            self._hw_set_gripper(0, g_spd)
            time.sleep(1.5)
            self._publish_gripper('released')

            self._set_state(ArmState.RETURNING_HOME)
            self.get_logger().info('[PLACE 3/3] Returning home')
            self._hw_send_angles(self._home_angles, spd)
            self._wait_until_done()

            self._set_state(ArmState.IDLE)
            self.get_logger().info('='*50)
            self.get_logger().info('PLACE complete -- ready for next pick')
            self.get_logger().info('='*50)

        except Exception as e:
            self.get_logger().error(f'Place exception: {e}')
            self._set_state(ArmState.ERROR)
            self._cleanup_place()

    def _cleanup_place(self):
        try:
            self._hw_set_gripper(0, self._gripper_speed)
            time.sleep(1.0)
            self._hw_send_angles(self._home_angles, self._move_speed)
            self._wait_until_done(timeout=10.0)
        except Exception:
            pass
        time.sleep(1.0)
        self._set_state(ArmState.IDLE)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MyCobotControllerTF2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
