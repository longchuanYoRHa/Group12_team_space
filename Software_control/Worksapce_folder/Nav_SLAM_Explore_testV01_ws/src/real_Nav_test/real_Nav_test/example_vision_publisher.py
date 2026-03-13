#!/usr/bin/env python3
"""
视觉算法发布物体坐标示例代码

此文件展示如何从视觉算法中获取物体在相机坐标系下的坐标，
并发布到 ROS2 topic 供控制节点使用。

使用方法：
1. 将你的视觉算法检测到的物体坐标（相对于相机坐标系）填入此代码
2. 确保设置正确的相机坐标系 frame_id
3. 发布到 'obj_xy' topic
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Trigger
from gazebo_msgs.msg import ModelStates  # type: ignore[import]
import time


class VisionObjectPublisher(Node):
    """
    视觉物体坐标发布节点
    
    从视觉算法获取物体在相机坐标系下的坐标，并发布到 obj_xy topic。
    """
    
    def __init__(self):
        super().__init__('vision_object_publisher')

        # ========== 1. 与旧示例兼容的发布器 ==========
        # 旧版本：发布 PointStamped 到 obj_xy（仅示例用途）
        self.obj_pub = self.create_publisher(PointStamped, 'obj_xy', 10)

        # ========== 2. 模拟 rover_vision_node 的发布格式 ==========
        # 真实节点 rover_vision_node 使用 geometry_msgs/Point，
        # 并发布到 /target_pick/* 与 /target_place/* 话题。
        # 这里做一个最小子集，用 red 这一组来测试 task_manager_v2。
        self.vision_pubs = {
            "red_cube": self.create_publisher(Point, '/target_pick/red', 10),
            "red_bin": self.create_publisher(Point, '/target_place/red', 10),
        }
        
        # 相机坐标系名称
        # 根据你的实际相机配置选择：
        # - D435i_camera_color_optical_frame (RGB相机光学坐标系，推荐用于2D检测)
        # - D435i_camera_depth_optical_frame (深度相机光学坐标系，推荐用于3D检测)
        # - camera_color_optical_frame (通用RGB相机)
        # - camera_depth_optical_frame (通用深度相机)
        self.camera_frame_id = 'D435i_camera_color_optical_frame'
        
        # 创建定时器，模拟视觉算法检测频率（例如 10Hz），
        # 依然发布到示例话题 obj_xy（PointStamped）。
        self.timer = self.create_timer(0.1, self.publish_object_coordinates)

        # ========== 3. Gazebo pose & 主节点状态查询 ==========
        # 从 Gazebo 的 /gazebo/model_states 获取机器人在仿真中的位姿。
        # 默认模型名与 spawn_with_lidar.launch 中一致：leo_rover
        self.robot_model_name = 'leo_rover'
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self._gazebo_states_callback,
            10,
        )

        # 创建 task_manager_v2 提供的状态查询服务客户端
        self.state_client = self.create_client(Trigger, 'task_manager/get_state')
        self._state_call_in_flight = False

        # 每 10 秒查询一次主节点状态，并根据状态 + Gazebo 位姿
        # 模拟发布一次 vision 触发消息。
        self.query_timer = self.create_timer(10.0, self._query_and_trigger)

        # 标记：避免在一次流程中重复触发
        self._pick_triggered = False
        self._place_triggered = False
        
        self.get_logger().info(f'视觉物体坐标发布节点已启动，使用坐标系: {self.camera_frame_id}')

    # ==========================================================
    # Gazebo 回调与主节点状态查询
    # ==========================================================

    def _gazebo_states_callback(self, msg: ModelStates):
        """从 /gazebo/model_states 中提取当前机器人在 map 中的大致位置。"""
        try:
            if self.robot_model_name in msg.name:
                idx = msg.name.index(self.robot_model_name)
                pose = msg.pose[idx]
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y
        except Exception as e:
            self.get_logger().warn(f'处理 Gazebo model_states 失败: {e}')

    def _query_and_trigger(self):
        """
        每 10 秒调用一次 task_manager/get_state (std_srvs/Trigger)，
        根据返回的 state/cargo 字符串以及 Gazebo 中的机器人位姿，
        向与 rover_vision_node 一致的 /target_pick/* 或 /target_place/* 话题
        发布一次假的视觉检测坐标，用于测试整个任务流程。
        """
        if self._state_call_in_flight:
            return

        if not self.state_client.wait_for_service(timeout_sec=0.1):
            # 服务尚未就绪，静默返回即可
            return

        request = Trigger.Request()
        future = self.state_client.call_async(request)
        self._state_call_in_flight = True
        future.add_done_callback(self._handle_state_response)

    def _handle_state_response(self, future):
        self._state_call_in_flight = False
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(f'调用 task_manager/get_state 失败: {e}')
            return

        if not response.success or not response.message:
            return

        # 解析 message，例如： "state=explore, cargo=empty"
        state = None
        cargo = None
        try:
            parts = [p.strip() for p in response.message.split(',')]
            for p in parts:
                if p.startswith('state='):
                    state = p.split('=', 1)[1].strip()
                elif p.startswith('cargo='):
                    cargo = p.split('=', 1)[1].strip()
        except Exception:
            # 格式异常直接忽略
            return

        if state is None or cargo is None:
            return

        # 基于当前状态 + Gazebo 中的大致位置，伪造一次视觉触发：
        # - state=explore 且 cargo=empty：模拟发现可抓取物体 -> /target_pick/red
        # - state=resume_explore_for_bin 且 cargo=has_object：模拟发现 bin -> /target_place/red
        #
        # 为简单起见，这里仅使用机器人当前在 map 中的位置 (x, y) 作为“检测点”，
        # z 设为 0.0。task_manager_v2 在转换失败时会直接把这些数当作 map 坐标使用。
        if state == 'explore' and cargo == 'empty' and not self._pick_triggered:
            self._publish_fake_pick()
            self._pick_triggered = True
            # 一旦进入抓取流程，放置前不再重复触发 pick
        elif state == 'resume_explore_for_bin' and cargo == 'has_object' and self._pick_triggered and not self._place_triggered:
            self._publish_fake_place()
            self._place_triggered = True

        # 当任务流程完成后，task_manager_v2 会回到 EXPLORE 且 cargo=empty，
        # 你可以根据需要在这里重置触发标志，形成循环测试。
        if state == 'explore' and cargo == 'empty' and self._pick_triggered and self._place_triggered:
            # 简单策略：一轮 Pick+Place 结束后重置
            self._pick_triggered = False
            self._place_triggered = False
    
    def get_object_coordinates_from_vision(self):
        """
        从视觉算法获取物体在相机坐标系下的坐标
        
        这是你需要实现的函数，替换为你的实际视觉算法代码。
        
        Returns:
            tuple: (x, y, z) 物体在相机坐标系下的坐标（米）
                   如果未检测到物体，返回 None
        """
        # ========== 示例代码 ==========
        # 这里应该调用你的视觉算法，例如：
        # - YOLO/目标检测算法
        # - 深度估计
        # - 3D重建等
        
        # 示例：假设检测到物体，坐标为 (0.5, 0.1, 0.8) 米
        # 在相机坐标系中：
        # - x: 物体在相机右侧为正（通常）
        # - y: 物体在相机下方为正（通常）
        # - z: 物体在相机前方为正（深度）
        
        # TODO: 替换为你的实际视觉算法
        # 示例返回值：
        detected = True  # 是否检测到物体
        if detected:
            # 示例坐标（相对于相机坐标系）
            obj_x = 0.5   # 米
            obj_y = 0.1   # 米
            obj_z = 0.8   # 米（深度）
            return (obj_x, obj_y, obj_z)
        else:
            return None
        # ========== 示例代码结束 ==========
    
    def publish_object_coordinates(self):
        """
        发布物体坐标到 obj_xy topic
        
        从视觉算法获取坐标，创建 PointStamped 消息并发布。
        """
        # 从视觉算法获取物体坐标
        coords = self.get_object_coordinates_from_vision()
        
        if coords is None:
            # 未检测到物体，不发布
            return
        
        x, y, z = coords

        # ========== 旧示例：发布到 obj_xy（PointStamped） ==========
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.camera_frame_id  # 重要：设置为相机坐标系
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z
        self.obj_pub.publish(point_msg)

        self.get_logger().info(
            f'发布物体坐标到 obj_xy ({self.camera_frame_id}): x={x:.3f}m, y={y:.3f}m, z={z:.3f}m'
        )

    def _publish_fake_pick(self):
        """向 /target_pick/red 发布一次假的 pick 目标（geometry_msgs/Point）。"""
        msg = Point()
        msg.x = float(self.robot_x)
        msg.y = float(self.robot_y)
        msg.z = 0.0
        pub = self.vision_pubs.get("red_cube")
        if pub is not None:
            pub.publish(msg)
            self.get_logger().info(
                f'[TEST] 依据 Gazebo pose 伪造 PICK 触发: '
                f'/target_pick/red -> ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}), '
                f'state=explore, cargo=empty'
            )

    def _publish_fake_place(self):
        """向 /target_place/red 发布一次假的 place 目标（geometry_msgs/Point）。"""
        msg = Point()
        msg.x = float(self.robot_x)
        msg.y = float(self.robot_y)
        msg.z = 0.0
        pub = self.vision_pubs.get("red_bin")
        if pub is not None:
            pub.publish(msg)
            self.get_logger().info(
                f'[TEST] 依据 Gazebo pose 伪造 PLACE 触发: '
                f'/target_place/red -> ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}), '
                f'state=resume_explore_for_bin, cargo=has_object'
            )


def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    node = VisionObjectPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

