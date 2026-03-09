#!/usr/bin/env python3

# This is a simplified example showing how to simulate the 
# pick and place actions of myCobot 280 in RViz.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class PickAndPlaceVisualizer(Node):
    def __init__(self):
        super().__init__('pick_and_place_visualizer')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Joint names for myCobot 280
        self.joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 
                           'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
        
        self.sequence_step = 0
        self.step_timer = 0.0
        self.step_duration = 2.0  # Each step lasts 2 seconds
        
        # Define motion sequence (joint angles in radians)
        self.motion_sequence = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # Initial position
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],          # Move above pick point
            [0.0, -0.5, 0.8, 0.0, 0.0, 0.0],          # Descend to pick
            [0.0, -0.5, 0.8, 0.0, 0.0, 0.785],        # Close gripper
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.785],        # Lift object
            [1.57, -0.5, 0.5, 0.0, 0.0, 0.785],       # Move to place area
            [1.57, -0.5, 0.8, 0.0, 0.0, 0.785],       # Descend to place
            [1.57, -0.5, 0.8, 0.0, 0.0, 0.0],         # Open gripper
            [1.57, -0.5, 0.5, 0.0, 0.0, 0.0],         # Lift object
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # Return to initial position
        ]
        
        self.get_logger().info('Pick and Place Visualizer started')
        self.get_logger().info('Launch RViz with: ros2 launch mycobot_280pi test.launch.py')
    
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Get current step's joint angles
        if self.sequence_step < len(self.motion_sequence):
            msg.position = self.motion_sequence[self.sequence_step]
            
            # Switch to next step every step_duration
            self.step_timer += 0.1
            if self.step_timer >= self.step_duration:
                self.get_logger().info(f'Step {self.sequence_step + 1}/{len(self.motion_sequence)}')
                self.sequence_step += 1
                self.step_timer = 0.0
        else:
            # Sequence completed, restarting...
            self.get_logger().info('Sequence completed, restarting...')
            self.sequence_step = 0
            self.step_timer = 0.0
            msg.position = self.motion_sequence[0]
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
