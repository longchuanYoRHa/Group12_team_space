#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv, os
from datetime import datetime

class JointLogger(Node):
    def __init__(self):
        super().__init__('joint_logger')

        # Make folder for logs
        log_dir = '/home/tejas/mycobot/logs'
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = os.path.join(log_dir, f'joint_log_{timestamp}.csv')

        self.get_logger().info(f'Logging joint data to: {self.file_path}')
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_sec', 'joint', 'position', 'velocity', 'effort'])

        self.create_subscription(JointState, '/joint_states', self.callback, 20)

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for j, p, v, e in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.writer.writerow([t, j, p, v, e])

    def destroy_node(self):
        self.file.close()
        self.get_logger().info('Log file saved.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logging stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
