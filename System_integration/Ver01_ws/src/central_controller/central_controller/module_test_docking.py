#!/usr/bin/env python3
"""
Simple docking module test:
- Startup reset odom once
- Enter precision align state directly
- On vision detection, send DockRobot goal using visual coordinates
  with use_dock_id=False and SimpleNonChargingDock type.
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
from std_srvs.srv import Trigger

try:
    from nav2_msgs.action import DockRobot  # type: ignore[attr-defined]
    _dockrobot_import_error = None
except Exception as nav2_import_error:
    try:
        from opennav_docking_msgs.action import DockRobot  # pyright: ignore[reportMissingImports]
        _dockrobot_import_error = None
    except Exception as docking_msgs_import_error:
        DockRobot = None  # type: ignore
        _dockrobot_import_error = (
            "Failed to import DockRobot from both nav2_msgs.action and "
            f"opennav_docking_msgs.action: nav2_msgs={nav2_import_error}, "
            f"opennav_docking_msgs={docking_msgs_import_error}"
        )

from central_controller.task_manager_utils import quaternion_from_yaw, quat_yaw


class TestState(Enum):
    INIT = "init"
    PRECISION_ALIGN = "precision_align"
    DOCKING = "docking"
    GRASP = "grasp"
    DONE = "done"


class ModuleTestDockingNode(Node):
    def __init__(self):
        super().__init__("module_test_docking")

        self.current_state = TestState.INIT
        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None
        self._vision_target_pose_odom = None
        self._vision_trigger_point_camera = None  # geometry_msgs.Point in camera_frame
        self._vision_trigger_is_pick = True

        # Arm state for GRASP
        self.arm_status = "idle"
        self.gripper_status = "unknown"
        self._arm_cmd_sent = False

        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("dock_action_name", "dock_robot")
        self.declare_parameter("dock_type", "simple_non_charging_dock")
        self.declare_parameter("docking_stop_distance_m", 0.20)
        # If enabled, during DOCKING we continuously publish a PoseStamped to the
        # docking plugin's "external detection pose" input so DockingServer can
        # call getRefinedPose() using the latest vision result.
        self.declare_parameter("use_external_detection_pose", False)
        self.declare_parameter("external_detection_pose_topic", "detected_dock_pose")
        self.declare_parameter("external_detection_pose_frame", "odom")

        self._use_external_detection_pose = (
            self.get_parameter("use_external_detection_pose").value
        )
        external_detection_pose_topic = (
            self.get_parameter("external_detection_pose_topic").value
        )
        # Keep frame configurable for easy debugging/remapping.
        self._external_detection_pose_frame = (
            self.get_parameter("external_detection_pose_frame").value
        )
        self._external_detection_pose_pub = self.create_publisher(
            geometry_msgs.PoseStamped, external_detection_pose_topic, 10
        )
        self._external_detection_pose_pub_alt = None
        if isinstance(external_detection_pose_topic, str):
            if external_detection_pose_topic.startswith("/docking_server/"):
                alt_topic = external_detection_pose_topic[len("/docking_server/"):]
            elif external_detection_pose_topic.startswith("/"):
                alt_topic = f"/docking_server{external_detection_pose_topic}"
            else:
                alt_topic = f"/docking_server/{external_detection_pose_topic}"

            if alt_topic != external_detection_pose_topic:
                self._external_detection_pose_pub_alt = self.create_publisher(
                    geometry_msgs.PoseStamped, alt_topic, 10
                )

        self.state_pub = self.create_publisher(std_msgs.String, "module_test_docking/state", 10)

        # TF:
        # - Longer cache reduces intermittent "extrapolation" failures.
        # - Dedicated spinning thread avoids TF reception being starved.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self._reset_odom_client = self.create_client(Trigger, "/reset_odometry")
        self._reset_odom_done = False
        self._reset_odom_future = None
        self._reset_odom_started_at = None

        self.dock_client = None
        if DockRobot is None:
            self.get_logger().error(
                "DockRobot action import failed. "
                "This environment should provide it from `nav2_msgs.action` "
                "or older setups from `opennav_docking_msgs.action`. "
                f"Original error: {_dockrobot_import_error}."
            )
        else:
            self.dock_client = ActionClient(
                self, DockRobot, self.get_parameter("dock_action_name").value
            )

        # Keep strong references to subscriptions; otherwise Python GC may drop them
        # and callbacks become unreliable even when `ros2 topic echo` shows data.
        self._vision_subs = []
        for color, topic in [
            ("red", "/target_pick/red"),
            ("green", "/target_pick/green"),
            ("blue", "/target_pick/blue"),
            ("red", "/target_place/red"),
            ("green", "/target_place/green"),
            ("blue", "/target_place/blue"),
        ]:
            sub = self.create_subscription(
                geometry_msgs.Point,
                topic,
                lambda msg, c=color, t=topic: self._vision_point_callback(msg, c, t),
                qos_profile_sensor_data,
            )
            self._vision_subs.append(sub)

        # Arm feedback subscriptions (used for GRASP success/failure)
        self.arm_status_sub = self.create_subscription(
            std_msgs.String,
            "/arm/status",
            self._arm_status_callback,
            10,
        )
        self.arm_gripper_status_sub = self.create_subscription(
            std_msgs.String,
            "/arm/gripper_status",
            self._arm_gripper_status_callback,
            10,
        )

        self.get_logger().info("module_test_docking initialized.")

        # Use pure callbacks (no polling timer) for state transitions.
        self._publish_state()
        self._last_vision_msg_time = None  # rclpy.time.Time
        self._last_tf_ok_time = None  # rclpy.time.Time
        self._last_external_pose_pub_time = None  # rclpy.time.Time
        self._last_external_pose = None  # geometry_msgs.PoseStamped

        # Periodic watchdog + external detection pose keep-alive.
        self._watchdog_timer = self.create_timer(0.5, self._watchdog_timer_cb)
        use_sim_time = False
        if self.has_parameter("use_sim_time"):
            use_sim_time = bool(self.get_parameter("use_sim_time").value)

        if use_sim_time:
            self.get_logger().info(
                "INIT: use_sim_time=true, skip /reset_odometry and enter PRECISION_ALIGN."
            )
            self._reset_odom_done = True
            self._set_state(TestState.PRECISION_ALIGN)
        else:
            self._start_reset_odometry()

    def _publish_state(self):
        state_msg = std_msgs.String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)

    def _set_state(self, new_state: TestState):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        self._publish_state()

    def _start_reset_odometry(self):
        # Block briefly until service becomes available; then use a done callback.
        self.get_logger().info("INIT: waiting for /reset_odometry service ...")
        if not self._reset_odom_client.wait_for_service(timeout_sec=8.0):
            self.get_logger().warn("INIT: /reset_odometry service not available, continue test.")
            self._reset_odom_done = True
            self._set_state(TestState.PRECISION_ALIGN)
            return

        self.get_logger().info("INIT: calling /reset_odometry ...")
        self._reset_odom_future = self._reset_odom_client.call_async(Trigger.Request())
        self._reset_odom_future.add_done_callback(self._reset_odometry_done_callback)

    def _reset_odometry_done_callback(self, future):
        try:
            resp = future.result()
            if resp is not None and getattr(resp, "success", False):
                self.get_logger().info(f"INIT: /reset_odometry success: {resp.message}")
            else:
                msg = "" if resp is None else getattr(resp, "message", "")
                self.get_logger().warn(f"INIT: /reset_odometry failed: {msg}")
        except Exception as e:
            self.get_logger().warn(f"INIT: /reset_odometry call error: {e}")

        self._reset_odom_done = True
        self._set_state(TestState.PRECISION_ALIGN)

    def _vision_point_callback(self, msg: geometry_msgs.Point, color: str, topic: str):
        self._last_vision_msg_time = self.get_clock().now()
        is_pick_topic = ("/target_pick/" in topic)
        if self.current_state not in (TestState.PRECISION_ALIGN, TestState.DOCKING):
            return

        try:
            target_pose = self._build_dock_target_from_vision(msg)
            # self.get_logger().info(f"Vision->dock target: {target_pose}")
        except Exception as e:
            self.get_logger().warn(f"Vision->dock target transform failed: {e}")
            return

        # During DOCKING, we only update external detection pose (if enabled).
        if self.current_state == TestState.DOCKING:
            if self._use_external_detection_pose:
                self._publish_external_detection_pose(target_pose)
            return

        # PRECISION_ALIGN
        # Always keep external detection pose updated when enabled, even before sending the goal.
        if self._use_external_detection_pose:
            self._publish_external_detection_pose(target_pose)
            # self.get_logger().info(f"External detection pose published: {target_pose}")

        # Prevent multiple DockRobot goal sends.
        if self._dock_goal_sent:
            return
        if self.dock_client is None:
            return
        if not self.dock_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("DockRobot server not ready.")
            return

        self._vision_target_pose_odom = target_pose
        self._vision_trigger_point_camera = msg
        self._vision_trigger_is_pick = is_pick_topic
        self.get_logger().info(
            f"Vision trigger from {topic} ({color}), send DockRobot to "
            f"({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}) in odom."
        )

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose = target_pose
        goal.dock_type = self.get_parameter("dock_type").value
        goal.navigate_to_staging_pose = False
        goal.max_staging_time = 0.0

        send_goal_future = self.dock_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._dock_goal_response_callback)
        self._dock_goal_sent = True
        self._set_state(TestState.DOCKING)

    def _publish_external_detection_pose(self, dock_pose_stamped: geometry_msgs.PoseStamped):
        pose = geometry_msgs.PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self._external_detection_pose_frame)
        pose.pose = dock_pose_stamped.pose
        self._external_detection_pose_pub.publish(pose)
        if self._external_detection_pose_pub_alt is not None:
            self._external_detection_pose_pub_alt.publish(pose)
        self._last_external_pose = pose
        self._last_external_pose_pub_time = self.get_clock().now()

    def _watchdog_timer_cb(self):
        now = self.get_clock().now()

        # Keep feeding DockingServer if external detection is enabled.
        if self.current_state == TestState.DOCKING and self._use_external_detection_pose:
            if self._last_external_pose is not None:
                should_pub = True
                if self._last_external_pose_pub_time is not None:
                    should_pub = (now - self._last_external_pose_pub_time).nanoseconds > int(0.2 * 1e9)
                if should_pub:
                    self._external_detection_pose_pub.publish(self._last_external_pose)
                    if self._external_detection_pose_pub_alt is not None:
                        self._external_detection_pose_pub_alt.publish(self._last_external_pose)
                    self._last_external_pose_pub_time = now

        # Warn if vision seems to have stopped (or is being state-filtered).
        if self._last_vision_msg_time is not None:
            if (now - self._last_vision_msg_time).nanoseconds > int(3.0 * 1e9):
                self.get_logger().warn(
                    "No vision messages processed for >3s. "
                    "If `ros2 topic echo` still shows data, check state filtering "
                    f"(current_state={self.current_state.value}) and QoS."
                )
                self._last_vision_msg_time = now

        # Quick TF readiness check (diagnostic only).
        cam_frame = str(self.get_parameter("camera_frame_id").value)
        if self.tf_buffer.can_transform(
            "odom", cam_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
        ):
            self._last_tf_ok_time = now
        else:
            if self._last_tf_ok_time is None or (now - self._last_tf_ok_time).nanoseconds > int(3.0 * 1e9):
                self.get_logger().warn(f"TF not ready: odom<-{cam_frame} (yet).")

    def _point_to_pose_stamped_in_frame(self, point_msg: geometry_msgs.Point, target_frame: str):
        """
        Align with task_manager_node_v2: transform camera-frame point into target_frame
        and return PoseStamped with identity orientation.
        """
        frame_id = self.get_parameter("camera_frame_id").value
        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        # Use latest available TF (time=0) to reduce intermittent lookup failures.
        point_stamped.header.stamp = rclpy.time.Time().to_msg()
        point_stamped.point = point_msg

        try:
            if not self.tf_buffer.can_transform(
                target_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            ):
                raise RuntimeError(f"TF not available: {target_frame}<-{frame_id}")

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            point_in_target = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

            pose_stamped = geometry_msgs.PoseStamped()
            pose_stamped.header.frame_id = target_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position = point_in_target.point
            pose_stamped.pose.orientation.w = 1.0
            self._last_tf_ok_time = self.get_clock().now()
            return pose_stamped
        except Exception as e:
            if target_frame != "odom":
                raise

            # Fallback: camera -> base_link, then compose with odom -> base_link/base_footprint.
            # This is more tolerant when the node can't directly resolve odom->camera_link
            # even though the global TF tree can.
            self.get_logger().warn(
                f"Direct TF {target_frame}<-{frame_id} failed, try fallback via base frame: {e}"
            )

            cam_to_base = self.tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            point_in_base = tf2_geometry_msgs.do_transform_point(point_stamped, cam_to_base)

            base_parent_frame = "base_link"
            try:
                odom_to_base = self.tf_buffer.lookup_transform(
                    "odom",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except Exception:
                base_parent_frame = "base_footprint"
                odom_to_base = self.tf_buffer.lookup_transform(
                    "odom",
                    "base_footprint",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )

            yaw = quat_yaw(odom_to_base.transform.rotation)
            bx = point_in_base.point.x
            by = point_in_base.point.y

            pose_stamped = geometry_msgs.PoseStamped()
            pose_stamped.header.frame_id = "odom"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = (
                odom_to_base.transform.translation.x + math.cos(yaw) * bx - math.sin(yaw) * by
            )
            pose_stamped.pose.position.y = (
                odom_to_base.transform.translation.y + math.sin(yaw) * bx + math.cos(yaw) * by
            )
            pose_stamped.pose.position.z = (
                odom_to_base.transform.translation.z + point_in_base.point.z
            )
            pose_stamped.pose.orientation.w = 1.0
            self._last_tf_ok_time = self.get_clock().now()
            self.get_logger().info(
                f"Fallback TF used via {base_parent_frame}: "
                f"odom target=({pose_stamped.pose.position.x:.3f}, "
                f"{pose_stamped.pose.position.y:.3f}, "
                f"{pose_stamped.pose.position.z:.3f})"
            )
            return pose_stamped

    def _get_robot_xy_in_frame(self, target_frame: str):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            "base_link",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _build_dock_target_from_vision(self, point_msg: geometry_msgs.Point):
        stop_dist = max(0.0, float(self.get_parameter("docking_stop_distance_m").value))

        target_pose_odom = self._point_to_pose_stamped_in_frame(point_msg, "odom")
        robot_x, robot_y = self._get_robot_xy_in_frame("odom")
        vx = target_pose_odom.pose.position.x
        vy = target_pose_odom.pose.position.y

        dx = vx - robot_x
        dy = vy - robot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            raise RuntimeError("vision point too close to robot origin")

        ux = dx / dist
        uy = dy / dist
        tx = vx - ux * stop_dist
        ty = vy - uy * stop_dist

        pose = geometry_msgs.PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tx
        pose.pose.position.y = ty
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(math.atan2(vy - ty, vx - tx))
        return pose

    def _dock_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"DockRobot goal response error: {e}")
            self._dock_goal_sent = False
            self.current_state = TestState.PRECISION_ALIGN
            return

        if not goal_handle.accepted:
            self.get_logger().warn("DockRobot goal rejected.")
            self._dock_goal_sent = False
            self.current_state = TestState.PRECISION_ALIGN
            return

        self.get_logger().info("DockRobot goal accepted.")
        self._dock_goal_handle = goal_handle
        self._dock_result_future = goal_handle.get_result_async()
        self._dock_result_future.add_done_callback(self._dock_result_done_callback)

    def _dock_result_done_callback(self, future):
        try:
            result = self._dock_result_future.result().result
        except Exception as e:
            self.get_logger().error(f"DockRobot result error: {e}")
            self._set_state(TestState.PRECISION_ALIGN)
            self._dock_goal_sent = False
            self._dock_goal_handle = None
            self._dock_result_future = None
            return

        if result.success:
            self.get_logger().info("Docking test succeeded.")
            if self._vision_trigger_is_pick:
                self._enter_grasp_state()
            else:
                self._set_state(TestState.DONE)
        else:
            self.get_logger().warn(f"Docking test failed, error_code={result.error_code}.")
            self._set_state(TestState.PRECISION_ALIGN)

        self._dock_goal_sent = False
        self._dock_goal_handle = None
        self._dock_result_future = None

    def _arm_status_callback(self, msg: std_msgs.String):
        self.arm_status = msg.data.lower()
        self._try_finish_grasp_from_topics()

    def _arm_gripper_status_callback(self, msg: std_msgs.String):
        self.gripper_status = msg.data.lower()
        self._try_finish_grasp_from_topics()

    def _enter_grasp_state(self):
        self.get_logger().info("Entering GRASP after successful docking.")
        self._set_state(TestState.GRASP)
        self._arm_cmd_sent = False

        # If we didn't store a triggering vision point, we cannot compute /arm/target_pick.
        if self._vision_trigger_point_camera is None:
            self.get_logger().warn("GRASP: no stored vision point; skipping arm command.")
            self._set_state(TestState.DONE)
            return

        # Send pick command once immediately on state entry.
        target_pt = self._point_camera_to_base_link_m(self._vision_trigger_point_camera)
        self.get_logger().info(
            "GRASP: publishing /arm/target_pick in base_link (meters) "
            f"({target_pt.x:.3f}, {target_pt.y:.3f}, {target_pt.z:.3f})"
        )
        if not hasattr(self, "_arm_pick_pub"):
            self._arm_pick_pub = self.create_publisher(geometry_msgs.Point, "/arm/target_pick", 10)
        self._arm_pick_pub.publish(target_pt)
        self._arm_cmd_sent = True

    def _try_finish_grasp_from_topics(self):
        if self.current_state != TestState.GRASP:
            return
        if not self._arm_cmd_sent:
            return

        if self.arm_status == "holding" and self.gripper_status == "object_held":
            self.get_logger().info("GRASP succeeded (arm holding + object_held).")
            self._set_state(TestState.DONE)
            return

        if self.arm_status == "error":
            self.get_logger().warn("GRASP failed (arm_status=error).")
            self._set_state(TestState.DONE)
            return

    def _point_camera_to_base_link_m(self, point_msg: geometry_msgs.Point):
        """
        Convert camera-frame point to base_link frame (meters).
        arm_interfaces.md specifies /arm/target_pick uses geometry_msgs/Point in base_link, in meters.
        """
        frame_id = self.get_parameter("camera_frame_id").value

        point_stamped = geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = point_msg

        transform = self.tf_buffer.lookup_transform(
            "base_link",
            frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5),
        )
        point_in_base = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        return geometry_msgs.Point(
            x=point_in_base.point.x,
            y=point_in_base.point.y,
            z=point_in_base.point.z,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ModuleTestDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
