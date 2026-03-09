#!/usr/bin/env python3
"""
MyCobot 280 Pi + Adaptive Gripper - Interactive Control Script

Modes:
  1. Interactive CLI  -- manually input target coordinates for Gazebo testing
  2. Auto task        -- run full pick-and-place sequence

Usage:
  ros2 run mycobot280_pi move_mycobot_gripper.py             # interactive mode (default)
  ros2 run mycobot280_pi move_mycobot_gripper.py --auto      # auto task mode

  Auto Task sequence:
  1. Move to home position
  2. Move to pre-grasp position (above target block)
  3. Descend to grasp position
  4. Close gripper (grasp)
  5. Retreat (lift up)
  6. Move to pre-place position (above placement target)
  7. Descend to place position
  8. Open gripper (release)
  9. Retreat (lift up)
  10. Return to home position
"""

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from time import sleep
import threading

# --------------------------------------------------------------------------
# ikpy inverse kinematics
# --------------------------------------------------------------------------
try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False
    print("[WARN] ikpy not installed. Install: pip install ikpy")


# --------------------------------------------------------------------------
# MyCobot 280 Pi kinematic chain (unit: meters, matched to URDF)
# --------------------------------------------------------------------------
def build_mycobot_chain():
    chain = Chain(name="mycobot_280", links=[
        OriginLink(),
        URDFLink(
            name="joint1",
            origin_translation=[0, 0, 0.13156],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-math.pi, math.pi),
        ),
        URDFLink(
            name="joint2",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, math.pi/2, 0],
            rotation=[0, 1, 0],
            bounds=(-math.radians(165), math.radians(165)),
        ),
        URDFLink(
            name="joint3",
            origin_translation=[0, -0.1104, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=(-math.radians(165), math.radians(165)),
        ),
        URDFLink(
            name="joint4",
            origin_translation=[0, -0.096, 0],
            origin_orientation=[0, math.pi/2, 0],
            rotation=[0, 1, 0],
            bounds=(-math.radians(165), math.radians(165)),
        ),
        URDFLink(
            name="joint5",
            origin_translation=[0, 0, 0.06639],
            origin_orientation=[0, -math.pi/2, 0],
            rotation=[0, 0, 1],
            bounds=(-math.pi, math.pi),
        ),
        URDFLink(
            name="joint6",
            origin_translation=[0, 0, 0.04761 + 0.0456],  # includes gripper base offset
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-math.pi, math.pi),
        ),
        URDFLink(
            name="tcp",
            origin_translation=[0, 0, 0.0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0],
        ),
    ])
    return chain


def coords_to_target_matrix(x_mm, y_mm, z_mm, rx_deg=0.0, ry_deg=0.0, rz_deg=0.0):
    """Convert MyCobot API coords (mm, degrees) to 4x4 homogeneous transform (meters)"""
    x, y, z = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
    rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)

    Rz = np.array([[math.cos(rz), -math.sin(rz), 0],
                   [math.sin(rz),  math.cos(rz), 0],
                   [0,             0,             1]])
    Ry = np.array([[ math.cos(ry), 0, math.sin(ry)],
                   [0,             1, 0            ],
                   [-math.sin(ry), 0, math.cos(ry)]])
    Rx = np.array([[1, 0,              0            ],
                   [0, math.cos(rx), -math.sin(rx)  ],
                   [0, math.sin(rx),  math.cos(rx)  ]])

    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3]  = [x, y, z]
    return T


def solve_ik(chain, target_matrix, initial_angles=None):
    """Solve IK, returns 6 joint angles (radians), or None on failure"""
    if not IKPY_AVAILABLE:
        return None

    full_initial = [0.0] + (list(initial_angles) if initial_angles else [0.0]*6) + [0.0]

    result = chain.inverse_kinematics(
        target_position=target_matrix[:3, 3],
        target_orientation=target_matrix[:3, :3],
        orientation_mode="all",
        initial_position=full_initial,
    )
    return list(result[1:7])


# --------------------------------------------------------------------------
# Task presets (real-world recorded coordinates for reference)
# NOTE: These real-world coords may NOT match Gazebo directly.
#       Use interactive mode to find correct Gazebo coordinates first.
# --------------------------------------------------------------------------
HOME_ANGLES   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
GRIPPER_OPEN  = [0.15]    # upper limit per URDF
GRIPPER_CLOSE = [-0.5]    # soft close (~70%), use -0.74 for full close

# Real-world recorded coords (mm, degrees) — for reference only
REAL_TARGET_COORDS = [36.2, -61.3, 421.6, -88.77, 1.87, -87.41]
REAL_PLACE_COORDS  = [150.0, 100.0, 421.6, -88.77, 1.87, -87.41]

# Gazebo-calibrated coords — fill in after interactive testing
GAZEBO_TARGET_COORDS = REAL_TARGET_COORDS   # TODO: calibrate with interactive mode
GAZEBO_PLACE_COORDS  = REAL_PLACE_COORDS    # TODO: calibrate with interactive mode


# --------------------------------------------------------------------------
# ROS2 Node
# --------------------------------------------------------------------------
class MoveMyCobotGripper(Node):

    def __init__(self, interactive=True):
        super().__init__("move_mycobot_gripper")
        self.interactive = interactive

        self.arm_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10)

        self.arm_joints = [
            "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
            "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
        ]

        if IKPY_AVAILABLE:
            self.chain = build_mycobot_chain()
            self.get_logger().info("IK chain built successfully.")
        else:
            self.chain = None
            self.get_logger().warn("ikpy unavailable — only joint-angle mode supported.")

        self.current_angles = HOME_ANGLES[:]

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def send_arm(self, joint_angles_rad, duration_sec=3):
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        pt = JointTrajectoryPoint()
        pt.positions = list(joint_angles_rad)
        pt.time_from_start.sec = int(duration_sec)
        traj.points.append(pt)
        self.arm_pub.publish(traj)
        sleep(0.1)
        self.arm_pub.publish(traj)
        self.get_logger().info(
            f"  -> Joints (rad): {[round(a, 4) for a in joint_angles_rad]}"
        )

    def send_gripper(self, position):
        msg = Float64MultiArray()
        msg.data = list(position)
        self.gripper_pub.publish(msg)
        sleep(0.1)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"  -> Gripper: {position}")

    def move_to_coords(self, coords, duration_sec=3, label="Move"):
        """Solve IK for given coords and send to arm"""
        x, y, z = coords[0], coords[1], coords[2]
        rx = coords[3] if len(coords) > 3 else 0.0
        ry = coords[4] if len(coords) > 4 else 0.0
        rz = coords[5] if len(coords) > 5 else 0.0

        self.get_logger().info(
            f"[{label}] x={x} y={y} z={z} mm | rx={rx} ry={ry} rz={rz} deg"
        )

        if not IKPY_AVAILABLE or self.chain is None:
            self.get_logger().error("ikpy unavailable — cannot solve IK.")
            return False

        T = coords_to_target_matrix(x, y, z, rx, ry, rz)
        angles = solve_ik(self.chain, T, self.current_angles)

        if angles is None:
            self.get_logger().error(f"[{label}] IK solving failed!")
            return False

        self.current_angles = angles[:]
        self.send_arm(angles, duration_sec)
        return True

    def wait(self, seconds, msg=""):
        if msg:
            self.get_logger().info(f"  Waiting {seconds}s — {msg}")
        sleep(seconds)

    # ------------------------------------------------------------------
    # Interactive CLI mode
    # ------------------------------------------------------------------

    def run_interactive(self):
        """
        Interactive command loop.

        Supported commands:
          xyz <x> <y> <z>                     move to position only (mm), orientation = current
          xyzrpy <x> <y> <z> <rx> <ry> <rz>  move to full pose (mm + degrees)
          joints <j1> <j2> <j3> <j4> <j5> <j6>  send joint angles directly (degrees)
          gripper <value>                      set gripper position (-0.74 ~ 0.15)
          open                                 open gripper
          close                                close gripper (soft)
          home                                 return to home position
          fk                                   print current joint angles
          help                                 show this help
          quit / exit                          exit program

        Recommended inputs for Gazebo coordinate calibration:
          1. Start with: xyz 0 0 300        (straight up, safe position)
          2. Try:        xyz 100 0 200      (test X axis direction)
          3. Try:        xyz 0 100 200      (test Y axis direction)
          4. Record which Gazebo coords match your real-world target block.
        """
        print("\n" + "="*60)
        print("  MyCobot 280 Pi — Interactive Gazebo Control")
        print("="*60)
        print(self.run_interactive.__doc__)
        print("="*60)

        # Wait for ROS to be ready
        sleep(2.0)
        self.send_arm(HOME_ANGLES, duration_sec=3)
        self.send_gripper(GRIPPER_OPEN)
        sleep(3.0)

        while rclpy.ok():
            try:
                raw = input("\n> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting.")
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            # ---- xyz ----
            if cmd == "xyz" and len(parts) == 4:
                try:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    # Keep current orientation (use 0,0,0 as default for Gazebo testing)
                    self.move_to_coords([x, y, z, 0.0, 0.0, 0.0], duration_sec=3, label="XYZ")
                except ValueError:
                    print("[ERROR] Invalid numbers. Usage: xyz <x> <y> <z>")

            # ---- xyzrpy ----
            elif cmd == "xyzrpy" and len(parts) == 7:
                try:
                    vals = [float(p) for p in parts[1:]]
                    self.move_to_coords(vals, duration_sec=3, label="XYZRPY")
                except ValueError:
                    print("[ERROR] Usage: xyzrpy <x> <y> <z> <rx> <ry> <rz>")

            # ---- joints (degrees input, convert to radians) ----
            elif cmd == "joints" and len(parts) == 7:
                try:
                    angles_deg = [float(p) for p in parts[1:]]
                    angles_rad = [math.radians(a) for a in angles_deg]
                    print(f"  Sending joints (deg): {angles_deg}")
                    self.current_angles = angles_rad[:]
                    self.send_arm(angles_rad, duration_sec=3)
                except ValueError:
                    print("[ERROR] Usage: joints <j1> <j2> <j3> <j4> <j5> <j6>  (degrees)")

            # ---- gripper ----
            elif cmd == "gripper" and len(parts) == 2:
                try:
                    val = float(parts[1])
                    if not (-0.74 <= val <= 0.15):
                        print("[WARN] Value out of range [-0.74, 0.15], clamping.")
                        val = max(-0.74, min(0.15, val))
                    self.send_gripper([val])
                except ValueError:
                    print("[ERROR] Usage: gripper <value>  (range: -0.74 ~ 0.15)")

            # ---- open / close ----
            elif cmd == "open":
                self.send_gripper(GRIPPER_OPEN)

            elif cmd == "close":
                self.send_gripper(GRIPPER_CLOSE)

            # ---- home ----
            elif cmd == "home":
                print("  Returning to home position...")
                self.current_angles = HOME_ANGLES[:]
                self.send_arm(HOME_ANGLES, duration_sec=3)
                self.send_gripper(GRIPPER_OPEN)

            # ---- fk: print current joint angles ----
            elif cmd == "fk":
                deg = [round(math.degrees(a), 2) for a in self.current_angles]
                rad = [round(a, 4) for a in self.current_angles]
                print(f"  Current joints (deg): {deg}")
                print(f"  Current joints (rad): {rad}")

            elif cmd in ("help", "?"):
                print(self.run_interactive.__doc__)

            elif cmd in ("quit", "exit", "q"):
                print("Exiting interactive mode.")
                break

            else:
                print(f"[ERROR] Unknown command: '{raw}'. Type 'help' for usage.")

        self.destroy_node()

    # ------------------------------------------------------------------
    # Auto task mode (pick-and-place sequence)
    # ------------------------------------------------------------------

    def run_auto_task(self):
        self.get_logger().info("="*50)
        self.get_logger().info("Starting auto pick-and-place task")
        self.get_logger().info("="*50)

        target = GAZEBO_TARGET_COORDS
        target_above = target[:3] + [target[2] + 80.0] + target[3:]  # z+80mm
        target_above = [target[0], target[1], target[2] + 80.0] + list(target[3:])
        place  = GAZEBO_PLACE_COORDS
        place_above = [place[0], place[1], place[2] + 80.0] + list(place[3:])

        steps = [
            ("Step 0 — Home",              lambda: (self.send_arm(HOME_ANGLES, 3), self.send_gripper(GRIPPER_OPEN), self.wait(4))),
            ("Step 1 — Pre-Grasp",         lambda: (self.move_to_coords(target_above, 4, "Pre-Grasp"), self.wait(5))),
            ("Step 2 — Grasp Position",    lambda: (self.move_to_coords(target, 3, "Grasp"), self.wait(4))),
            ("Step 3 — Close Gripper",     lambda: (self.send_gripper(GRIPPER_CLOSE), self.wait(2))),
            ("Step 4 — Retreat",           lambda: (self.move_to_coords(target_above, 3, "Retreat-Grasp"), self.wait(4))),
            ("Step 5 — Pre-Place",         lambda: (self.move_to_coords(place_above, 4, "Pre-Place"), self.wait(5))),
            ("Step 6 — Place Position",    lambda: (self.move_to_coords(place, 3, "Place"), self.wait(4))),
            ("Step 7 — Open Gripper",      lambda: (self.send_gripper(GRIPPER_OPEN), self.wait(2))),
            ("Step 8 — Retreat",           lambda: (self.move_to_coords(place_above, 3, "Retreat-Place"), self.wait(4))),
            ("Step 9 — Home",              lambda: (self.send_arm(HOME_ANGLES, 3), self.wait(4))),
        ]

        for name, action in steps:
            self.get_logger().info(f"\n[{name}]")
            action()

        self.get_logger().info("Task completed!")
        self.destroy_node()


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main(args=None):
    try:
        rclpy.init(args=args)
        # Check launch argument: --auto triggers auto task mode
        interactive = "--auto" not in sys.argv
        node = MoveMyCobotGripper(interactive=interactive)
        
        if interactive:
            # Run CLI in main thread; spin ROS in background thread
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()
            node.run_interactive()
        else:
            node.run_auto_task()
            rclpy.spin(node)
    except Exception as e:
        print(f"[ERROR] Exception in main: {e}")
        pass


if __name__ == "__main__":
    main()