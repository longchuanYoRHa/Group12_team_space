#!/usr/bin/env python3

"""
ROS2 control node for MyCobot 280 Pi + adaptive gripper.

Split pick/place workflow:
  1. PICK phase:  target_pick  -> grab block -> verify -> return home -> HOLDING
  2. HOLDING:     arm locked at home with block, rover can move freely
  3. PLACE phase: target_place -> move above bin -> release -> return home -> IDLE

Subscribes:
  - target_pick  (geometry_msgs/Point)  -- 3D pick coords (triggers pick phase)
  - target_place (geometry_msgs/Point)  -- 3D place coords (triggers place phase, only in HOLDING)

Publishes:
  - status         (std_msgs/String)       -- state machine status
  - joint_states   (sensor_msgs/JointState)-- real-time joint feedback (10 Hz)
  - gripper_status (std_msgs/String)       -- gripper grip confirmation

Status flow:
  idle -> moving_to_pick -> descending_pick -> gripping -> grip_check
       -> lifting -> returning_home -> holding
                                         (rover moves to find bin)
  holding -> moving_to_place -> releasing -> returning_home -> idle

Designed to run under namespace 'arm' (set via launch file).
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
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import JointState


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
class ArmState(Enum):
    IDLE = auto()             # no block, waiting for pick command
    MOVING_TO_PICK = auto()   # moving above pick point
    DESCENDING_PICK = auto()  # descending to grab
    GRIPPING = auto()         # closing gripper
    GRIP_CHECK = auto()       # verifying grip
    LIFTING = auto()          # lifting block after grab
    RETURNING_HOME = auto()   # returning to home position
    HOLDING = auto()          # at home, block held, arm locked -- rover can move
    MOVING_TO_PLACE = auto()  # moving above bin
    RELEASING = auto()        # opening gripper to drop block
    ERROR = auto()            # something went wrong


# ---------------------------------------------------------------------------
# Mock for development on NUC (no hardware)
# ---------------------------------------------------------------------------
class MockMyCobot280:
    """Drop-in mock so the node can be tested without real hardware."""

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
        self._gripper_val = 15 if state == 1 else 100  # simulate object
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
# Main ROS 2 node
# ---------------------------------------------------------------------------
class MyCobotController(Node):

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
        super().__init__('mycobot_controller')

        # ---- ROS parameters ------------------------------------------------
        self.declare_parameter('safe_z', 250.0)           # mm, safe travel height
        self.declare_parameter('move_speed', 40)          # 1-100
        self.declare_parameter('gripper_speed', 80)       # 1-100
        self.declare_parameter('end_rx', -178.63)         # from calibration
        self.declare_parameter('end_ry', 4.73)
        self.declare_parameter('end_rz', -85.94)
        self.declare_parameter('gripper_torque', 300)     # 150-980
        self.declare_parameter('grip_threshold', 5)       # gripper val > this = held
        self.declare_parameter('grip_check_retries', 2)
        self.declare_parameter('home_angles',
                               [5.18, 4.92, -10.45, 5.62, 2.1, -9.4])
        self.declare_parameter('calibration_file', '')

        self._safe_z         = self.get_parameter('safe_z').value
        self._move_speed     = self.get_parameter('move_speed').value
        self._gripper_speed  = self.get_parameter('gripper_speed').value
        self._end_rpy = [
            self.get_parameter('end_rx').value,
            self.get_parameter('end_ry').value,
            self.get_parameter('end_rz').value,
        ]
        self._gripper_torque = self.get_parameter('gripper_torque').value
        self._grip_threshold = self.get_parameter('grip_threshold').value
        self._grip_retries   = self.get_parameter('grip_check_retries').value
        self._home_angles    = list(self.get_parameter('home_angles').value)

        # ---- Load calibration (optional override) ---------------------------
        self._calibration = {}
        cal_file = self.get_parameter('calibration_file').value
        if cal_file and os.path.exists(cal_file):
            self._load_calibration(cal_file)
        else:
            self._auto_load_calibration()

        # Override home_angles / end_rpy from calibration if available
        if 'home' in self._calibration:
            cal_home = self._calibration['home'].get('angles', [])
            if len(cal_home) == 6:
                self._home_angles = cal_home
                self.get_logger().info(f'Home angles from calibration: {cal_home}')
        if 'pickup' in self._calibration:
            cal_coords = self._calibration['pickup'].get('coords', [])
            if len(cal_coords) == 6:
                self._end_rpy = cal_coords[3:6]
                self.get_logger().info(f'End orientation from calibration: {self._end_rpy}')

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
        self.status_pub  = self.create_publisher(String, 'status', 10)
        self.gripper_pub = self.create_publisher(String, 'gripper_status', 10)
        self.js_pub      = self.create_publisher(JointState, 'joint_states', 10)

        # ---- Subscribers ----------------------------------------------------
        self.create_subscription(Point, 'target_pick',  self._pick_cb,  10)
        self.create_subscription(Point, 'target_place', self._place_cb, 10)

        # ---- Timers ---------------------------------------------------------
        self.create_timer(0.1, self._publish_joint_states)   # 10 Hz

        self._publish_status()
        self.get_logger().info('MyCobot controller ready')
        self.get_logger().info(f'  Home angles : {self._home_angles}')
        self.get_logger().info(f'  End orient. : {self._end_rpy}')
        self.get_logger().info(f'  Safe Z      : {self._safe_z} mm')
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
    # Callbacks -- PICK and PLACE are independent triggers
    # ======================================================================
    def _pick_cb(self, msg: Point):
        """Receive pick target -- only accepted when IDLE."""
        if self._state != ArmState.IDLE:
            self.get_logger().warn(
                f'Ignoring target_pick (state={self._state.name})')
            return
        if not self._validate_coords(msg):
            return

        self.get_logger().info(
            f'[PICK] Target received: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})')
        self._task_thread = threading.Thread(
            target=self._run_pick, args=(msg,), daemon=True)
        self._task_thread.start()

    def _place_cb(self, msg: Point):
        """Receive place target -- only accepted when HOLDING."""
        if self._state != ArmState.HOLDING:
            self.get_logger().warn(
                f'Ignoring target_place (state={self._state.name}, '
                f'need HOLDING)')
            return
        if not self._validate_coords(msg):
            return

        self.get_logger().info(
            f'[PLACE] Target received: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})')
        self._task_thread = threading.Thread(
            target=self._run_place, args=(msg,), daemon=True)
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
    # Hardware helpers (thread-safe wrappers)
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

    # ======================================================================
    # Wait for motion
    # ======================================================================
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

    # ======================================================================
    # Gripper verification
    # ======================================================================
    def _verify_grip(self):
        """
        Check if object is held after closing gripper.
        Adaptive gripper: value > threshold means not fully closed = object held.
        value ~ 0 means fully closed = nothing grasped.
        """
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

    # ======================================================================
    # Publishing helpers
    # ======================================================================
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

    # ======================================================================
    # Coord helper
    # ======================================================================
    def _make_coords(self, x, y, z):
        """Build [x, y, z, rx, ry, rz] using calibrated end orientation."""
        return [x, y, z] + list(self._end_rpy)

    # ======================================================================
    # PICK phase  (IDLE -> ... -> HOLDING)
    # ======================================================================
    def _run_pick(self, target: Point):
        """
        Pick sequence:
          1. Open gripper
          2. Move above pick at safe_z
          3. Descend to pick (linear)
          4. Close gripper
          5. Verify grip (retry if needed)
          6. Lift to safe_z
          7. Return to home angles (block held)
          8. Lock servos -> HOLDING
        """
        rx, ry, rz = self._end_rpy
        spd   = self._move_speed
        g_spd = self._gripper_speed

        try:
            # Clear any pre-existing error
            err = self._hw_check_error()
            if err and err != 0:
                self.get_logger().warn(f'Clearing error {err}')
                self._hw_clear_error()

            # Step 1: Open gripper
            self._set_state(ArmState.MOVING_TO_PICK)
            self.get_logger().info('[PICK 1/6] Open gripper')
            self._hw_set_gripper(0, g_spd)
            time.sleep(1.0)

            # Step 2: Move above pick point at safe height
            above_pick = self._make_coords(target.x, target.y, self._safe_z)
            self.get_logger().info(f'[PICK 2/6] Move above pick: {above_pick}')
            self._hw_send_coords(above_pick, spd, 0)  # joint-space (fast)
            self._wait_until_done()
            time.sleep(0.3)

            # Step 3: Descend to pick (linear for precision)
            self._set_state(ArmState.DESCENDING_PICK)
            pick_coords = self._make_coords(target.x, target.y, target.z)
            self.get_logger().info(f'[PICK 3/6] Descend to: {pick_coords}')
            self._hw_send_coords(pick_coords, spd, 1)  # linear
            self._wait_until_done()
            time.sleep(0.3)

            # Step 4: Close gripper
            self._set_state(ArmState.GRIPPING)
            self.get_logger().info('[PICK 4/6] Closing gripper')
            self._hw_set_gripper(1, g_spd)

            # Step 5: Verify grip with retries
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

            # Step 6: Lift
            self._set_state(ArmState.LIFTING)
            self.get_logger().info(f'[PICK 6/6] Lifting to z={self._safe_z}')
            self._hw_send_coords(above_pick, spd, 1)  # linear lift
            self._wait_until_done()

            # Check grip after lift
            val = self._hw_get_gripper_value()
            self.get_logger().info(f'  Grip after lift: {val}')
            if val is not None and val <= self._grip_threshold:
                self.get_logger().error('Object lost during lift!')
                self._publish_gripper('object_dropped')
                self._set_state(ArmState.ERROR)
                self._cleanup_pick()
                return

            # Step 7: Return to home angles (arm tucked, block held)
            self._set_state(ArmState.RETURNING_HOME)
            self.get_logger().info(
                f'Returning to home: {self._home_angles}')
            self._hw_send_angles(self._home_angles, spd)
            self._wait_until_done()
            time.sleep(0.5)

            # Step 8: Lock servos and enter HOLDING
            self._hw_focus_all()
            self._set_state(ArmState.HOLDING)
            self._publish_gripper('object_held')
            self.get_logger().info('='*50)
            self.get_logger().info('HOLDING -- block secured at home position')
            self.get_logger().info('Rover can move now. Waiting for target_place ...')
            self.get_logger().info('='*50)

        except Exception as e:
            self.get_logger().error(f'Pick exception: {e}')
            self._set_state(ArmState.ERROR)
            self._cleanup_pick()

    def _cleanup_pick(self):
        """Recovery after failed pick: open gripper, go home, IDLE."""
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

    # ======================================================================
    # PLACE phase  (HOLDING -> ... -> IDLE)
    # ======================================================================
    def _run_place(self, target: Point):
        """
        Place sequence (simple drop into bin):
          1. Move above bin at safe_z
          2. Open gripper to release block
          3. Return to home
          4. -> IDLE
        """
        spd   = self._move_speed
        g_spd = self._gripper_speed

        try:
            err = self._hw_check_error()
            if err and err != 0:
                self.get_logger().warn(f'Clearing error {err}')
                self._hw_clear_error()

            # Step 1: Move above bin
            self._set_state(ArmState.MOVING_TO_PLACE)
            above_bin = self._make_coords(target.x, target.y, self._safe_z)
            self.get_logger().info(
                f'[PLACE 1/3] Moving above bin: {above_bin}')
            self._hw_send_coords(above_bin, spd, 0)  # joint-space
            self._wait_until_done()
            time.sleep(0.3)

            # Optional: check grip still held after moving
            val = self._hw_get_gripper_value()
            self.get_logger().info(f'  Grip before release: {val}')
            if val is not None and val <= self._grip_threshold:
                self.get_logger().error('Object lost during move to bin!')
                self._publish_gripper('object_dropped')
                self._set_state(ArmState.ERROR)
                self._cleanup_place()
                return

            # Step 2: Open gripper -- block drops into bin
            self._set_state(ArmState.RELEASING)
            self.get_logger().info('[PLACE 2/3] Releasing block into bin')
            self._hw_set_gripper(0, g_spd)
            time.sleep(1.5)
            self._publish_gripper('released')

            # Step 3: Return home
            self._set_state(ArmState.RETURNING_HOME)
            self.get_logger().info('[PLACE 3/3] Returning home')
            self._hw_send_angles(self._home_angles, spd)
            self._wait_until_done()

            # Done -- ready for next block
            self._set_state(ArmState.IDLE)
            self.get_logger().info('='*50)
            self.get_logger().info('PLACE complete -- ready for next pick')
            self.get_logger().info('='*50)

        except Exception as e:
            self.get_logger().error(f'Place exception: {e}')
            self._set_state(ArmState.ERROR)
            self._cleanup_place()

    def _cleanup_place(self):
        """Recovery after failed place: open gripper, go home, IDLE."""
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
    node = MyCobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
