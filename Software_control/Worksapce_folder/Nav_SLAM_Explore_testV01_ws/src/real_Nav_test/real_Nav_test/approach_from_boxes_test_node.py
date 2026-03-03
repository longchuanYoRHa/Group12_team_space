#!/usr/bin/env python3
"""
进近点测试节点（仅测试用）

基于 task_manager_node 的逻辑，用仿真中相机发布的 /boxes 作为“探索→进近”的触发信号，
收到 /boxes 后使用图中假设的世界坐标，模拟计算进近点流程。

- 订阅: /boxes（相机识别到目标物体时发布，作为状态切换触发）
- 假设物体世界坐标（来自界面）: X=2.2766 m, Y=4.8066 m, Z=0.15 m
- 流程: 触发 → 暂停探索 → 用假设坐标计算预抓取位姿 → 可选发送 Nav2 目标
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
import geometry_msgs.msg as geometry_msgs
import nav2_msgs.action as nav2_msgs
import std_msgs.msg as std_msgs
import math

# /boxes 类型：优先 vision_msgs/Detection2DArray；若仿真用 ros_gz_bridge 的 sensor_msgs/BoundingBoxes 则取消下面注释并注释掉 vision_msgs 分支
try:
    from vision_msgs.msg import Detection2DArray
    BOXES_MSG_TYPE = Detection2DArray
    BOXES_HAS_DETECTIONS = True  # 用 msg.detections 判断是否有目标
except ImportError:
    try:
        from sensor_msgs.msg import BoundingBoxes
        BOXES_MSG_TYPE = BoundingBoxes
        BOXES_HAS_DETECTIONS = True  # 用 msg.bounding_boxes 判断
    except ImportError:
        BOXES_MSG_TYPE = None
        BOXES_HAS_DETECTIONS = False


# 图中给出的假设物体世界坐标 (m)
DEFAULT_OBJECT_WORLD_X = 2.2766
DEFAULT_OBJECT_WORLD_Y = 4.8066
DEFAULT_OBJECT_WORLD_Z = 0.1500


class ApproachFromBoxesTestNode(Node):
    def __init__(self):
        super().__init__('approach_from_boxes_test')

        # 假设物体位姿（map 系），使用图中坐标
        self.object_pose = geometry_msgs.PoseStamped()
        self.object_pose.header.frame_id = 'map'
        self.object_pose.pose.position.x = DEFAULT_OBJECT_WORLD_X
        self.object_pose.pose.position.y = DEFAULT_OBJECT_WORLD_Y
        self.object_pose.pose.position.z = DEFAULT_OBJECT_WORLD_Z
        self.object_pose.pose.orientation.w = 1.0

        self.triggered = False  # 是否已由 /boxes 触发过一次（测试可重复触发则改为 False）
        self.pregrasp_distance = 0.5

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 探索控制（与 task_manager 一致）
        self.explore_control_pub = self.create_publisher(std_msgs.Bool, 'explore/resume', 10)

        # Nav2 客户端（可选，用于真实发目标）
        self.nav2_client = ActionClient(self, nav2_msgs.NavigateToPose, 'navigate_to_pose')
        self.nav2_goal_handle = None

        # 订阅 /boxes 作为触发
        if BOXES_MSG_TYPE is not None:
            self.boxes_sub = self.create_subscription(
                BOXES_MSG_TYPE,
                'boxes',
                self.boxes_callback,
                qos_profile_sensor_data
            )
            self.get_logger().info('订阅 /boxes (Detection2DArray)，作为探索→进近的触发')
        else:
            self.get_logger().warn('vision_msgs 未安装，请安装 vision_msgs 或将 /boxes 类型改为 sensor_msgs/BoundingBoxes 并修改本文件 import')
            return

        self.get_logger().info(
            f'进近点测试节点已启动，假设物体世界坐标: '
            f'x={DEFAULT_OBJECT_WORLD_X}, y={DEFAULT_OBJECT_WORLD_Y}, z={DEFAULT_OBJECT_WORLD_Z}'
        )

    def boxes_callback(self, msg):
        """收到 /boxes 即视为相机识别到目标，触发进近流程（仅做一次完整流程演示）。"""
        # 有检测结果时才触发（若希望“任意一条 /boxes 消息都触发”可注释掉下面一段）
        if BOXES_HAS_DETECTIONS:
            if hasattr(msg, 'detections') and len(msg.detections) == 0:
                return
            if hasattr(msg, 'bounding_boxes') and len(msg.bounding_boxes) == 0:
                return
        if self.triggered:
            return
        self.triggered = True

        self.get_logger().info('收到 /boxes，触发由探索切换为进近流程（模拟）')

        # 1) 模拟暂停探索
        explore_msg = std_msgs.Bool()
        explore_msg.data = False
        self.explore_control_pub.publish(explore_msg)
        self.get_logger().info('已发布 explore/resume=False，暂停探索')

        # 2) 使用假设世界坐标计算进近点（预抓取位姿）
        self.object_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose = self.calculate_pregrasp_pose(self.object_pose, self.pregrasp_distance)

        self.get_logger().info(
            f'进近点（预抓取）计算完成: '
            f'x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}, '
            f'z={goal_pose.pose.position.z:.3f}'
        )

        # 3) 可选：发送 Nav2 目标（测试时若不需要导航可注释掉）
        self.send_nav2_goal(goal_pose)

    def calculate_pregrasp_pose(self, target_pose, distance):
        """与 task_manager 一致：在目标前方计算预抓取位姿，面向目标。"""
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
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0:
            dx /= dist
            dy /= dist
        else:
            dx, dy = 1.0, 0.0

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

    def send_nav2_goal(self, goal_pose):
        """发送 Nav2 目标（仅测试用，可选）。"""
        if self.nav2_goal_handle is not None:
            return
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 不可用，仅打印进近点，不发送导航目标')
            return
        goal_msg = nav2_msgs.NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.get_logger().info('发送 Nav2 目标到进近点（测试）')
        send_future = self.nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._nav2_goal_done)

    def _nav2_goal_done(self, future):
        from rclpy.action import GoalStatus
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 目标被拒绝')
            return
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_done)

    def _nav2_result_done(self, future):
        from rclpy.action import GoalStatus
        result = future.result()
        self.nav2_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 已到达进近点')
        else:
            self.get_logger().warn('Nav2 未成功到达进近点')


def main(args=None):
    rclpy.init(args=args)
    node = ApproachFromBoxesTestNode()
    if BOXES_MSG_TYPE is None:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
