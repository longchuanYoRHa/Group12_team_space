#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanMaskNode(Node):
    """
    Node that filters laser scan data to only forward 90 degrees (-45 to +45 degrees)
    and publishes the masked scan as /scan_mask
    """
    
    def __init__(self):
        super().__init__('scan_mask_node')
        
        # Subscribe to the original scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for the masked scan
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan_mask',
            10
        )
        
        self.get_logger().info('Scan mask node started: filtering scan to forward 90 degrees (-45 to +45)')
    
    def scan_callback(self, msg):
        """
        Filter the laser scan to only include forward 90 degrees
        """
        # Create a new LaserScan message
        masked_scan = LaserScan()
        masked_scan.header = msg.header
        masked_scan.angle_min = -math.pi / 4.0  # -45 degrees
        masked_scan.angle_max = math.pi / 4.0    # +45 degrees
        masked_scan.angle_increment = msg.angle_increment
        masked_scan.time_increment = msg.time_increment
        masked_scan.scan_time = msg.scan_time
        masked_scan.range_min = msg.range_min
        masked_scan.range_max = msg.range_max
        
        # Calculate the number of points in the masked range
        num_points = int((masked_scan.angle_max - masked_scan.angle_min) / masked_scan.angle_increment) + 1
        
        # Find the start and end indices in the original scan
        start_angle = masked_scan.angle_min
        end_angle = masked_scan.angle_max
        
        # Calculate indices in the original scan array
        # Assuming angle_min is the starting angle of the original scan
        start_idx = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_idx = int((end_angle - msg.angle_min) / msg.angle_increment) + 1
        
        # Ensure indices are within bounds
        start_idx = max(0, min(start_idx, len(msg.ranges) - 1))
        end_idx = max(0, min(end_idx, len(msg.ranges)))
        
        # Extract the masked ranges and intensities
        masked_scan.ranges = list(msg.ranges[start_idx:end_idx])
        masked_scan.intensities = list(msg.intensities[start_idx:end_idx]) if msg.intensities else []
        
        # Adjust the actual angle_min and angle_max based on the extracted data
        if len(masked_scan.ranges) > 0:
            masked_scan.angle_min = msg.angle_min + start_idx * msg.angle_increment
            masked_scan.angle_max = msg.angle_min + (end_idx - 1) * msg.angle_increment
        
        # Publish the masked scan
        self.scan_publisher.publish(masked_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

