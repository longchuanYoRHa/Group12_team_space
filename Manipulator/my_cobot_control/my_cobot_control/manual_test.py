#!/usr/bin/env python3

"""
Manual test script for MyCobot controller.
Sends target_pick and target_place commands via command line.

Usage:
  python3 manual_test.py pick    # Send pickup command
  python3 manual_test.py place   # Send place command
  python3 manual_test.py custom <x> <y> <z> <pick|place>  # Send custom coordinates
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


# Predefined test coordinates from calibration
TEST_COORDS = {
    'pickup': {
        'x': 202.2,
        'y': -129.3,
        'z': 237.9,
    },
    'place': {
        'x': 66.7,
        'y': -218.2,
        'z': 123.7,
    }
}


class ManualTestPublisher(Node):
    def __init__(self):
        super().__init__('manual_test_publisher')
        self.pick_pub = self.create_publisher(Point, '/arm/target_pick', 10)
        self.place_pub = self.create_publisher(Point, '/arm/target_place', 10)

    def send_pick(self, x, y, z):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        
        self.get_logger().info(f'Sending PICK target: ({x}, {y}, {z})')
        self.pick_pub.publish(msg)
        
    def send_place(self, x, y, z):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        
        self.get_logger().info(f'Sending PLACE target: ({x}, {y}, {z})')
        self.place_pub.publish(msg)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    rclpy.init()
    node = ManualTestPublisher()
    
    try:
        if command == 'pick':
            coords = TEST_COORDS['pickup']
            print(f"\n{'='*50}")
            print(f"Sending PICKUP command")
            print(f"Coordinates: x={coords['x']}, y={coords['y']}, z={coords['z']}")
            print(f"{'='*50}\n")
            node.send_pick(coords['x'], coords['y'], coords['z'])
            
        elif command == 'place':
            coords = TEST_COORDS['place']
            print(f"\n{'='*50}")
            print(f"Sending PLACE command")
            print(f"Coordinates: x={coords['x']}, y={coords['y']}, z={coords['z']}")
            print(f"{'='*50}\n")
            node.send_place(coords['x'], coords['y'], coords['z'])
            
        elif command == 'custom':
            if len(sys.argv) < 6:
                print("Usage: python3 manual_test.py custom <x> <y> <z> <pick|place>")
                sys.exit(1)
            
            x = float(sys.argv[2])
            y = float(sys.argv[3])
            z = float(sys.argv[4])
            target_type = sys.argv[5].lower()
            
            print(f"\n{'='*50}")
            print(f"Sending custom {target_type.upper()} command")
            print(f"Coordinates: x={x}, y={y}, z={z}")
            print(f"{'='*50}\n")
            
            if target_type == 'pick':
                node.send_pick(x, y, z)
            elif target_type == 'place':
                node.send_place(x, y, z)
            else:
                print(f"Error: target type must be 'pick' or 'place', got '{target_type}'")
                sys.exit(1)
        else:
            print(f"Unknown command: {command}")
            print(__doc__)
            sys.exit(1)
        
        # Give time for message to be sent
        rclpy.spin_once(node, timeout_sec=1.0)
        print("Command sent successfully!\n")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
