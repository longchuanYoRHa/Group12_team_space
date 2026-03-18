#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import threading
import time

class MockArmNode(Node):
    def __init__(self):
        # Using a name similar to the real one, but distinct to know it's mock
        super().__init__('mock_arm_node')
        
        self.status_pub = self.create_publisher(String, '/arm/status', 10)
        self.gripper_pub = self.create_publisher(String, '/arm/gripper_status', 10)
        
        self.create_subscription(Point, '/arm/target_pick', self.pick_cb, 10)
        self.create_subscription(Point, '/arm/target_place', self.place_cb, 10)
        
        self.state = 'idle'
        
        # Publish state periodically to mimic some state reporting
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("Mock Arm Node started. Current state: idle")

    def timer_cb(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def pick_cb(self, msg):
        if self.state == 'idle':
            self.get_logger().info(f"Received target_pick command: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}")
            # Start simulated pick thread so we don't block the spinning
            th = threading.Thread(target=self.simulate_pick)
            th.start()
        else:
            self.get_logger().warning(f"Ignored pick command. Arm state is '{self.state}', not 'idle'")

    def place_cb(self, msg):
        if self.state == 'holding':
            self.get_logger().info(f"Received target_place command: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}")
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
