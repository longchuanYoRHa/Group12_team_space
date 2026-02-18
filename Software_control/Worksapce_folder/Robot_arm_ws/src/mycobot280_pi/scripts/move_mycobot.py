#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep

class MoveMyCobot(Node):
    def __init__(self):
        super().__init__('move_mycobot')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Define joint names
        self.joints = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]

        # Step 1: Move to home
        self.move_home()

        # Step 2: wait 3 seconds, then move to next pose
        sleep(3)
        self.move_target()
        


    def move_home(self):
        self.get_logger().info('Moving to home position...')
        traj = JointTrajectory()
        traj.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # all joints zero
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.pub.publish(traj)
        sleep(0.2)
        self.pub.publish(traj)
        self.get_logger().info('Home position command sent.')

    def move_target(self):
        self.get_logger().info('Moving to target position...')
        traj = JointTrajectory()
        traj.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.9, -0.8, 0.7, -0.3, 0.0]
        point.time_from_start.sec = 3

        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info('Target position command sent.')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MoveMyCobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
