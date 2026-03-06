#!/usr/bin/env python3
"""
Simple ROS2 node: send a time-parameterized sinusoidal joint trajectory to
/<<controller>>/follow_joint_trajectory, record /joint_states while motion runs,
interpolate recorded data to desired times, compute per-joint + overall RMS,
optionally save CSV and plot.

Assumptions:
 - simulation publishes /joint_states
 - action server at "/joint_trajectory_controller/follow_joint_trajectory"
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient

import numpy as np
import csv
import matplotlib.pyplot as plt
import time
from typing import List

# ----- simple parameters -----
CONTROLLER = 'joint_trajectory_controller'               # controller name (no leading slash)
ACTION_NAME = f'/{CONTROLLER}/follow_joint_trajectory'
DURATION = 80.0       # seconds
FREQ = 0.25          # Hz (sinusoidal frequency)
AMP = 0.1            # radians
PTS_PER_SEC = 20     # discrete points/sec in goal
SAVE_CSV = True
CSV_FILE = 'traj_log.csv'
PLOT = True
# -----------------------------

class SimpleTrajExec(Node):
    def __init__(self):
        super().__init__('simple_traj_exec')
        # Declare parameter (default False if not provided via launch)
        
        # Action client
        self._ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        # subscriber to joint states
        self._js_sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self._last_js = None       # latest JointState message
        self._recording = False
        self._rec_times = []       # times relative to start
        self._rec_pos = []         # list of lists matching joint order self.joints

        self.joints: List[str] = []

    def _js_cb(self, msg: JointState):
        """Cache latest joint_states; if recording, snapshot joint positions for our joint list."""
        self._last_js = msg
        if not self._recording or not self.joints:
            return
        # compute timestamp (use header if available else node time)
        if msg.header and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            now = self.get_clock().now()
            t = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        if (not hasattr(self, '_start_time')) or (self._start_time is None):
    	    self._start_time = t
        t_rel = t - self._start_time


        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        row = []
        for j in self.joints:
            row.append(float(name_to_pos.get(j, float('nan'))))
        self._rec_times.append(t_rel)
        self._rec_pos.append(row)

    def wait_for_action(self, timeout=5.0):
        self.get_logger().info(f'Waiting for action server {ACTION_NAME}...')
        ok = self._ac.wait_for_server(timeout_sec=timeout)
        if ok:
            self.get_logger().info('Action server available.')
        else:
            self.get_logger().warn('Action server not available (timeout).')
        return ok

    def determine_joints_from_joint_states(self, wait_s=3.0):
        """Simplest approach: use names from the latest /joint_states message."""
        self.get_logger().info('Waiting for /joint_states to determine joint list...')
        deadline = time.time() + wait_s
        while rclpy.ok() and time.time() < deadline:
            if self._last_js is not None and len(self._last_js.name) > 0:
                self.joints = list(self._last_js.name)
                self.get_logger().info(f'Using joints: {self.joints}')
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error('No /joint_states received; cannot determine joints.')
        return False

    def build_traj(self, current: List[float]):
        """Return list of times and desired positions (list of lists)."""
        n_pts = max(2, int(DURATION * PTS_PER_SEC) + 1)
        times = np.linspace(0.0, DURATION, n_pts)
        n_j = len(self.joints)
        phases = np.linspace(0.0, 2*np.pi, n_j, endpoint=False)
        pos = []
        for t in times:
            row = [float(cur + AMP * np.sin(2*np.pi*FREQ*t + phases[i])) for i, cur in enumerate(current)]
            pos.append(row)
        return times.tolist(), pos

    def send_goal_and_record(self, times: List[float], positions: List[List[float]]):
        """Send FollowJointTrajectory goal (time_from_start) and record /joint_states while it runs."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        pts = []
        for t, p in zip(times, positions):
            pt = JointTrajectoryPoint()
            pt.positions = p
            pt.time_from_start = Duration(seconds=float(t)).to_msg()
            pts.append(pt)
        goal.trajectory.points = pts

        # prepare to record
        self._rec_times = []
        self._rec_pos = []
        self._start_time = None
        self._recording = True

        send_fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected.')
            self._recording = False
            return None
        self.get_logger().info('Goal accepted; executing...')

        # wait for result (allow generous timeout)
        res_fut = gh.get_result_async()
        # spin until done
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=DURATION + 30.0)
        res = res_fut.result()
        self.get_logger().info(f'Action finished with status: {res.status}')
        # stop recording (but keep collected data)
        self._recording = False
        return True

    @staticmethod
    def interpolate(record_times: List[float], record_pos: List[List[float]], query_times: List[float]):
        """Linear interpolation per joint; clamp outside range."""
        if len(record_times) == 0:
            return None
        rt = np.array(record_times)
        rp = np.array(record_pos)   # shape (M, N)
        n_j = rp.shape[1]
        out = []
        for qt in query_times:
            if qt <= rt[0]:
                vals = rp[0, :]
            elif qt >= rt[-1]:
                vals = rp[-1, :]
            else:
                vals = np.array([np.interp(qt, rt, rp[:, j]) for j in range(n_j)])
            out.append(vals.tolist())
        return out

    @staticmethod
    def compute_rms(desired: List[List[float]], actual: List[List[float]]):
        D = np.array(desired)
        A = np.array(actual)
        if D.shape != A.shape:
            raise RuntimeError('Shape mismatch for RMS calculation')
        err = D - A
        per_joint = np.sqrt(np.mean(err**2, axis=0)).tolist()
        overall = float(np.sqrt(np.mean(err**2)))
        return per_joint, overall

    def save_csv(self, filename, joints, times, desired, actual):
        header = ['time'] + [f'des_{j}' for j in joints] + [f'act_{j}' for j in joints]
        with open(filename, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            for t, d, a in zip(times, desired, actual):
                row = [f'{t:.6f}'] + [f'{v:.6f}' for v in d] + [f'{v:.6f}' for v in a]
                w.writerow(row)
        self.get_logger().info(f'CSV saved: {filename}')

    def plot(self, joints, times, desired, actual):
        n = len(joints)
        fig, axs = plt.subplots(n, 1, figsize=(8, max(3, n*1.2)))
        if n == 1:
            axs = [axs]
        for i, j in enumerate(joints):
            des = [row[i] for row in desired]
            act = [row[i] for row in actual]
            axs[i].plot(times, des, label='desired')
            axs[i].plot(times, act, label='actual', alpha=0.8)
            axs[i].set_ylabel(j)
            axs[i].legend()
            axs[i].grid(True)
        axs[-1].set_xlabel('time (s)')
        fig.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = SimpleTrajExec()
    try:
        # 1) wait for action
        if not node.wait_for_action(timeout=5.0):
            node.get_logger().error('Action server unavailable; exiting.')
            return

        # 2) determine joints from /joint_states
        if not node.determine_joints_from_joint_states(wait_s=5.0):
            return

        # 3) get current positions (use latest joint_states; default 0.0 if missing)
        if node._last_js is None:
            node.get_logger().error('No joint_states yet; exiting.')
            return
        current = []
        for j in node.joints:
            if j in node._last_js.name:
                idx = node._last_js.name.index(j)
                current.append(float(node._last_js.position[idx]))
            else:
                current.append(0.0)

        # 4) build trajectory
        times, desired = node.build_traj(current)

        # 5) send goal and record
        ok = node.send_goal_and_record(times, desired)
        if not ok:
            return

        # 6) interpolate recorded data to desired times
        actual = node.interpolate(node._rec_times, node._rec_pos, times)
        if actual is None:
            node.get_logger().error('No recorded joint_states; cannot compute RMS.')
            return

        # 7) compute RMS
        per_joint_rms, overall_rms = node.compute_rms(desired, actual)
        node.get_logger().info(f'Per-joint RMS (rad): {per_joint_rms}')
        node.get_logger().info(f'Overall RMS (rad): {overall_rms:.6f}')

        # 8) save and plot if requested
        if SAVE_CSV:
            node.save_csv(CSV_FILE, node.joints, times, desired, actual)
        if PLOT:
            node.plot(node.joints, times, desired, actual)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

