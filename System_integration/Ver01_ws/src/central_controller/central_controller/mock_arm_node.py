#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import threading
import time
import math

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class MockArmNode(Node):
    def __init__(self):
        # Using a name similar to the real one, but distinct to know it's mock
        super().__init__('mock_arm_node')
        
        self.status_pub = self.create_publisher(String, '/arm/status', 10)
        self.gripper_pub = self.create_publisher(String, '/arm/gripper_status', 10)
        
        self.create_subscription(Point, '/arm/target_pick', self.pick_cb, 10)
        self.create_subscription(Point, '/arm/target_place', self.place_cb, 10)
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()
        
        self.state = 'idle'
        
        # Publish state periodically to mimic some state reporting
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("Mock Arm Node started. Current state: idle")

    def publish_static_transforms(self):
        transforms = []
        now = self.get_clock().now().to_msg()
        
        # base_link to camera_link
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = 0.158
        t_cam.transform.translation.y = 0.007
        t_cam.transform.translation.z = 0.081
        q_cam = euler_to_quaternion(-1.9199, 0.0, -1.5708)
        t_cam.transform.rotation.x = q_cam[0]
        t_cam.transform.rotation.y = q_cam[1]
        t_cam.transform.rotation.z = q_cam[2]
        t_cam.transform.rotation.w = q_cam[3]
        transforms.append(t_cam)

        # base_link to g_base
        t_arm = TransformStamped()
        t_arm.header.stamp = now
        t_arm.header.frame_id = 'base_link'
        t_arm.child_frame_id = 'g_base'
        t_arm.transform.translation.x = 0.05500
        t_arm.transform.translation.y = 0.03594
        t_arm.transform.translation.z = 0.01097
        q_arm = euler_to_quaternion(0.0, 0.0, 0.0)
        t_arm.transform.rotation.x = q_arm[0]
        t_arm.transform.rotation.y = q_arm[1]
        t_arm.transform.rotation.z = q_arm[2]
        t_arm.transform.rotation.w = q_arm[3]
        transforms.append(t_arm)
        
        # joint6_flange to gripper_tip (optional since mock arm might not have robot_state_publisher)
        t_grip = TransformStamped()
        t_grip.header.stamp = now
        t_grip.header.frame_id = 'joint6_flange'
        t_grip.child_frame_id = 'gripper_tip'
        t_grip.transform.translation.x = 0.0
        t_grip.transform.translation.y = 0.010
        t_grip.transform.translation.z = 0.095
        q_grip = euler_to_quaternion(0.0, 0.0, 0.0)
        t_grip.transform.rotation.x = q_grip[0]
        t_grip.transform.rotation.y = q_grip[1]
        t_grip.transform.rotation.z = q_grip[2]
        t_grip.transform.rotation.w = q_grip[3]
        transforms.append(t_grip)
        
        self.tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published mock TF2 static transforms for camera and arm base.")

    def timer_cb(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def pick_cb(self, msg):
        if self.state == 'idle':
            self.get_logger().info(f"Received target_pick command: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}")
            # Start simulated pick thread so we don't block the spinning
            th = threading.Thread(target=self.simulate_pick)
            th.start()
        else:
            self.get_logger().warning(f"Ignored pick command. Arm state is '{self.state}', not 'idle'")

    def place_cb(self, msg):
        if self.state == 'holding':
            self.get_logger().info(f"Received target_place command: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}")
            # Start simulated place thread
            th = threading.Thread(target=self.simulate_place)
            th.start()
        else:
            self.get_logger().warning(f"Ignored place command. Arm state is '{self.state}', not 'holding'")

    def simulate_pick(self):
        self.set_state('busy')
        self.get_logger().info("Moving to pick location...")
        time.sleep(2.0)
        
        self.get_logger().info("Gripping object...")
        time.sleep(1.0)
        
        gripper_msg = String()
        gripper_msg.data = 'object_held'
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper check passed: 'object_held'")
        
        self.get_logger().info("Lifting and returning to home...")
        time.sleep(1.5)
        
        self.set_state('holding')
        self.get_logger().info("Mock pick complete. State upgraded to: holding")

    def simulate_place(self):
        self.set_state('busy')
        self.get_logger().info("Moving to place location...")
        time.sleep(2.0)
        
        self.get_logger().info("Releasing object...")
        time.sleep(1.0)
        
        gripper_msg = String()
        gripper_msg.data = 'no_object'
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper release passed: 'no_object'")
        
        self.get_logger().info("Returning to home...")
        time.sleep(1.5)
        
        self.set_state('idle')
        self.get_logger().info("Mock place complete. State reset to: idle")
        
    def set_state(self, new_state):
        self.state = new_state
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
