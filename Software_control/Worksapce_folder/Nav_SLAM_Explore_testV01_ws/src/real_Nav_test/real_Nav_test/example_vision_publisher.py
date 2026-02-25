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
from geometry_msgs.msg import PointStamped
import time


class VisionObjectPublisher(Node):
    """
    视觉物体坐标发布节点
    
    从视觉算法获取物体在相机坐标系下的坐标，并发布到 obj_xy topic。
    """
    
    def __init__(self):
        super().__init__('vision_object_publisher')
        
        # 创建发布器，发布到 obj_xy topic
        self.obj_pub = self.create_publisher(
            PointStamped,
            'obj_xy',
            10
        )
        
        # 相机坐标系名称
        # 根据你的实际相机配置选择：
        # - D435i_camera_color_optical_frame (RGB相机光学坐标系，推荐用于2D检测)
        # - D435i_camera_depth_optical_frame (深度相机光学坐标系，推荐用于3D检测)
        # - camera_color_optical_frame (通用RGB相机)
        # - camera_depth_optical_frame (通用深度相机)
        self.camera_frame_id = 'D435i_camera_color_optical_frame'
        
        # 创建定时器，模拟视觉算法检测频率（例如 10Hz）
        self.timer = self.create_timer(0.1, self.publish_object_coordinates)
        
        self.get_logger().info(f'视觉物体坐标发布节点已启动，使用坐标系: {self.camera_frame_id}')
    
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
        
        # 创建 PointStamped 消息
        point_msg = PointStamped()
        
        # 设置消息头
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.camera_frame_id  # 重要：设置为相机坐标系
        
        # 设置物体坐标（相对于相机坐标系）
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z
        
        # 发布消息
        self.obj_pub.publish(point_msg)
        
        self.get_logger().info(
            f'发布物体坐标 ({self.camera_frame_id}): x={x:.3f}m, y={y:.3f}m, z={z:.3f}m'
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

