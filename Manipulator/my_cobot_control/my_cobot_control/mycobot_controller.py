# !/usr/bin/env python3

###############################################################
# This is the ROS2 control node for myCobot 280, 
# using the pymycobot library to directly control the arm.
###############################################################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymycobot import MyCobot280, PI_PORT, PI_BAUD
import math
import time

class MyCobotController(Node):
    def __init__(self):
        super().__init__('mycobot_controller')
        
        # 初始化硬件
        self.mc = MyCobot280(PI_PORT, PI_BAUD)
        self.mc.power_on()
        
        # 订阅目标物体位置（来自小车/笔记本）
        self.target_sub = self.create_subscription(
            Pose,
            '/target_object',
            self.target_callback,
            10
        )
        
        # 发布机械臂状态
        self.status_pub = self.create_publisher(String, '/arm_status', 10)
        
        self.get_logger().info('MyCobot controller ready, waiting for targets...')
    
    def target_callback(self, msg):
        """接收目标位置并执行抓取"""
        self.get_logger().info(f'Received target: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')
        
        # 转换为机械臂坐标并执行
        self.execute_pick_and_place(msg)
    
    def execute_pick_and_place(self, target_pose):
        """使用 pymycobot API 执行抓取放置"""
        try:
            # 1. 移动到目标上方
            pick_coords = [
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z + 50,  # 安全高度
                -180, 0, 0  # 姿态
            ]
            self.mc.send_coords(pick_coords, 50, 1)
            time.sleep(3)
            
            # 2. 下降抓取
            self.mc.send_coord(3, target_pose.position.z, 30)  # Z 轴
            time.sleep(2)
            
            # 3. 关闭夹爪
            self.mc.set_gripper_state(1, 80)
            time.sleep(2)
            
            # 4. 抬起
            self.mc.send_coord(3, target_pose.position.z + 100, 50)
            time.sleep(2)
            
            # 5. 放置到指定位置
            place_coords = [200, 0, 150, -180, 0, 0]
            self.mc.send_coords(place_coords, 50, 1)
            time.sleep(3)
            
            # 6. 松开夹爪
            self.mc.set_gripper_state(0, 80)
            time.sleep(2)
            
            self.status_pub.publish(String(data='completed'))
            self.get_logger().info('Pick and place completed!')
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.status_pub.publish(String(data='failed'))

def main():
    rclpy.init()
    node = MyCobotController()
    rclpy.spin(node)