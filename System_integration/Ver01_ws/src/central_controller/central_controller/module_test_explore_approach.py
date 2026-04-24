#!/usr/bin/env python3
"""
Exploration + map-interest-point approach test node.

Flow:
1. Wait for Nav2 `navigate_to_pose` action server.
2. Publish `explore/resume=True` to start frontier exploration.
3. When `explore/finished=True` arrives, save the map.
4. Detect interest points from the saved PGM map.
5. Navigate to a standoff pose for each interest point.

Compared with `task_manager_node_v2.py`, this test module intentionally:
- does not do the pre-explore rotation/navigation step
- does not process object/bin detections during exploration
- only verifies exploration -> map detection -> 40 cm approach behavior
"""

import os
import subprocess
import time
from enum import Enum

import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
import std_msgs.msg as std_msgs

from central_controller.detect_objects_in_pgm_map import get_interest_points_from_pgm
from central_controller.task_manager_utils import compute_pregrasp_pose


class TestState(Enum):
    INIT = "init"
    EXPLORE = "explore"
    PROCESS_MAP = "process_map"
    NAV_TO_INTEREST_POINT = "nav_to_interest_point"
    DONE = "done"


class ModuleTestExploreApproachNode(Node):
    def __init__(self):
        super().__init__("module_test_explore_approach")

        self.current_state = TestState.INIT
        self.explore_finished_received = False
        self.interest_points = []
        self.interest_point_index = 0
        self.current_interest_point = None
        self.nav2_goal_handle = None
        self._last_explore_resume = None
        self._last_explore_resume_publish_time = 0.0

        self.declare_parameter("maps_directory", "")
        self.declare_parameter("map_save_basename", "explore_complete")
        self.declare_parameter("approach_distance_m", 0.30)

        self.state_pub = self.create_publisher(
            std_msgs.String, "module_test_explore_approach/state", 10
        )
        self.explore_control_pub = self.create_publisher(
            std_msgs.Bool, "explore/resume", 10
        )
        self.create_subscription(
            std_msgs.Bool, "explore/finished", self._explore_finished_callback, 10
        )

        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, "navigate_to_pose")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.state_timer = self.create_timer(0.5, self._state_timer_callback)
        self._publish_state()
        self.get_logger().info("module_test_explore_approach initialized.")

    def _publish_state(self):
        msg = std_msgs.String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)

    def _set_state(self, new_state: TestState):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        self._publish_state()

    def _publish_explore_resume(self, resume: bool, force: bool = False):
        if not force and self._last_explore_resume is not None:
            if self._last_explore_resume == resume:
                return
        msg = std_msgs.Bool()
        msg.data = resume
        self.explore_control_pub.publish(msg)
        self._last_explore_resume = resume
        self._last_explore_resume_publish_time = time.monotonic()

    def _state_timer_callback(self):
        if self.current_state == TestState.INIT:
            if not self.nav2_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info("INIT: waiting for navigate_to_pose action server...")
                return
            self.get_logger().info("INIT: Nav2 ready, start frontier exploration.")
            self._set_state(TestState.EXPLORE)
            self._publish_explore_resume(True, force=True)
            return

        if self.current_state == TestState.EXPLORE and not self.explore_finished_received:
            now = time.monotonic()
            if now - self._last_explore_resume_publish_time >= 2.0:
                self._publish_explore_resume(True, force=True)

    def _get_maps_directory(self):
        maps_dir = self.get_parameter("maps_directory").value
        if maps_dir:
            return maps_dir
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_share = get_package_share_directory("central_controller")
            return os.path.join(pkg_share, "maps")
        except Exception:
            return os.path.expanduser("~/maps")

    def _get_robot_xy_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot pose in map: {e}")
            return (0.0, 0.0)

    def _explore_finished_callback(self, msg: std_msgs.Bool):
        if not msg.data:
            return
        if self.explore_finished_received:
            return

        self.explore_finished_received = True

        if self.current_state != TestState.EXPLORE:
            self.get_logger().info(
                f"Ignore explore/finished because current state is {self.current_state.value}."
            )
            return

        self.get_logger().info(
            "Exploration finished, start map save and interest point detection."
        )
        self._publish_explore_resume(False, force=True)
        self._set_state(TestState.PROCESS_MAP)
        self._run_map_detection_and_navigation()

    def _run_map_detection_and_navigation(self):
        maps_dir = self._get_maps_directory()
        os.makedirs(maps_dir, exist_ok=True)
        basename = self.get_parameter("map_save_basename").value
        map_base = os.path.join(maps_dir, basename)
        use_sim_time = False
        try:
            use_sim_time = bool(self.get_parameter("use_sim_time").value)
        except Exception:
            pass

        map_saver_cmd = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            map_base,
            "--ros-args",
            "-p",
            "save_map_timeout:=15.0",
            "-p",
            "map_subscribe_transient_local:=true",
        ]
        if use_sim_time:
            map_saver_cmd.extend(["-p", "use_sim_time:=true"])

        try:
            proc = subprocess.run(
                map_saver_cmd,
                capture_output=True,
                timeout=25,
                text=True,
            )
            if proc.returncode != 0:
                self.get_logger().warn(
                    f"map_saver_cli returncode={proc.returncode}; "
                    f"stderr: {proc.stderr.strip() or '(none)'}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn("map_saver_cli timed out after 15s.")
        except FileNotFoundError:
            self.get_logger().error("ros2 or map_saver_cli not found in PATH.")
        except Exception as e:
            self.get_logger().warn(f"map_saver_cli error: {e}")

        pgm_path = map_base + ".pgm"
        if not os.path.isfile(pgm_path):
            self.get_logger().error(f"PGM not found at {pgm_path}.")
            self._set_state(TestState.DONE)
            return

        try:
            raw_points = get_interest_points_from_pgm(
                pgm_path,
                prefer_yaml=True,
            )
        except Exception as e:
            self.get_logger().error(f"PGM detection failed: {e}")
            self._set_state(TestState.DONE)
            return

        self.interest_points = raw_points
        self.interest_point_index = 0
        self.current_interest_point = None

        self.get_logger().info(
            f"Map detection complete: found {len(self.interest_points)} interest points."
        )

        if not self.interest_points:
            self.get_logger().info("No interest points found, test completed.")
            self._set_state(TestState.DONE)
            return

        self._set_state(TestState.NAV_TO_INTEREST_POINT)
        self._nav_to_next_interest_point()

    def _nav_to_next_interest_point(self):
        if self.interest_point_index >= len(self.interest_points):
            self.get_logger().info("All interest points have been approached.")
            self.current_interest_point = None
            self._set_state(TestState.DONE)
            return

        mx, my = self.interest_points[self.interest_point_index]
        self.current_interest_point = (mx, my)

        target_pose = geometry_msgs.PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = mx
        target_pose.pose.position.y = my
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0

        robot_x, robot_y = self._get_robot_xy_in_map()
        approach_distance = float(self.get_parameter("approach_distance_m").value)
        goal_pose = compute_pregrasp_pose(
            target_pose,
            approach_distance,
            robot_x,
            robot_y,
            frame_id="map",
            stamp=self.get_clock().now().to_msg(),
        )

        self.get_logger().info(
            f"Nav to interest point {self.interest_point_index + 1}/{len(self.interest_points)} "
            f"at ({mx:.2f}, {my:.2f}), standoff={approach_distance:.2f} m."
        )
        self._send_nav_goal(goal_pose)

    def _send_nav_goal(self, goal_pose: geometry_msgs.PoseStamped):
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.nav2_goal_handle = None
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._nav2_goal_response_callback)

    def _nav2_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 goal response failed: {e}")
            self._advance_to_next_interest_point()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected, skip this interest point.")
            self._advance_to_next_interest_point()
            return

        self.get_logger().info("Nav2 goal accepted.")
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_callback)

    def _nav2_result_callback(self, future):
        try:
            result_wrapper = future.result()
            status = result_wrapper.status
        except Exception as e:
            self.get_logger().error(f"Nav2 result callback failed: {e}")
            self._advance_to_next_interest_point()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 goal succeeded.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Nav2 goal aborted.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Nav2 goal canceled.")
        else:
            self.get_logger().warn(f"Nav2 goal finished with status={status}.")

        self._advance_to_next_interest_point()

    def _advance_to_next_interest_point(self):
        self.nav2_goal_handle = None
        self.interest_point_index += 1
        self.current_interest_point = None
        if self.current_state != TestState.NAV_TO_INTEREST_POINT:
            return
        self._nav_to_next_interest_point()


def main(args=None):
    rclpy.init(args=args)
    node = ModuleTestExploreApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
