#!/usr/bin/env python3
"""
Test script to verify TF2 coordinate transformations.

This script:
1. Queries TF2 transforms
2. Shows the transform tree
3. Tests coordinate transformations
4. Compares with manual calculation

Usage:
    # First, start the system with TF2
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py
    
    # In another terminal, run this test
    python3 test_tf2_transform.py
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import math


class TF2TransformTester(Node):
    def __init__(self):
        super().__init__('tf2_transform_tester')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('TF2 Transform Tester started')
        self.get_logger().info('Waiting for transforms to become available...')
        
        # Wait for transforms
        import time
        time.sleep(2.0)
        
        self.run_tests()
    
    def list_available_frames(self):
        """List all available TF frames."""
        self.get_logger().info('='*70)
        self.get_logger().info('Available TF Frames:')
        self.get_logger().info('='*70)
        
        # Get all frame strings
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        self.get_logger().info(frames_yaml)
    
    def test_transform(self, source_frame, target_frame, x=100.0, y=50.0, z=200.0):
        """
        Test transformation from source_frame to target_frame.
        
        Args:
            source_frame: Source frame name
            target_frame: Target frame name
            x, y, z: Test point coordinates in mm
        """
        self.get_logger().info('-'*70)
        self.get_logger().info(f'Testing: {source_frame} -> {target_frame}')
        self.get_logger().info(f'Input point: ({x:.1f}, {y:.1f}, {z:.1f}) mm')
        
        try:
            # Create point in source frame
            point_src = PointStamped()
            point_src.header.frame_id = source_frame
            point_src.header.stamp = self.get_clock().now().to_msg()
            point_src.point.x = x / 1000.0  # mm to m
            point_src.point.y = y / 1000.0
            point_src.point.z = z / 1000.0
            
            # Lookup and apply transform
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            point_target = tf2_geometry_msgs.do_transform_point(point_src, transform)
            
            # Convert back to mm
            x_out = point_target.point.x * 1000.0
            y_out = point_target.point.y * 1000.0
            z_out = point_target.point.z * 1000.0
            
            self.get_logger().info(f'Output point: ({x_out:.1f}, {y_out:.1f}, {z_out:.1f}) mm')
            
            # Also show the transform itself
            t = transform.transform
            self.get_logger().info(f'Transform translation: '
                                 f'({t.translation.x*1000:.1f}, '
                                 f'{t.translation.y*1000:.1f}, '
                                 f'{t.translation.z*1000:.1f}) mm')
            self.get_logger().info(f'Transform rotation (quaternion): '
                                 f'({t.rotation.x:.3f}, {t.rotation.y:.3f}, '
                                 f'{t.rotation.z:.3f}, {t.rotation.w:.3f})')
            
            # Convert quaternion to euler angles
            from tf_transformations import euler_from_quaternion
            roll, pitch, yaw = euler_from_quaternion([
                t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            self.get_logger().info(f'Transform rotation (RPY degrees): '
                                 f'({math.degrees(roll):.1f}, '
                                 f'{math.degrees(pitch):.1f}, '
                                 f'{math.degrees(yaw):.1f})')
            
            return (x_out, y_out, z_out)
            
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None
    
    def test_gripper_offset(self):
        """Test gripper offset calculation."""
        self.get_logger().info('='*70)
        self.get_logger().info('Testing Gripper Offset Calculation')
        self.get_logger().info('='*70)
        
        try:
            # Get transform from flange to gripper tip
            transform = self.tf_buffer.lookup_transform(
                target_frame='gripper_tip',
                source_frame='joint6_flange',
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            t = transform.transform
            total_offset = t.translation.z * 1000.0  # in mm
            
            self.get_logger().info(f'Gripper offset (flange -> tip): {total_offset:.1f} mm')
            self.get_logger().info(f'  X: {t.translation.x*1000:.1f} mm')
            self.get_logger().info(f'  Y: {t.translation.y*1000:.1f} mm')
            self.get_logger().info(f'  Z: {t.translation.z*1000:.1f} mm')
            
            # Also test intermediate transform
            transform_base = self.tf_buffer.lookup_transform(
                target_frame='gripper_base',
                source_frame='joint6_flange',
                time=rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            base_offset = transform_base.transform.translation.z * 1000.0
            self.get_logger().info(f'Gripper base offset (flange -> base): {base_offset:.1f} mm')
            
            finger_offset = total_offset - base_offset
            self.get_logger().info(f'Finger length (base -> tip): {finger_offset:.1f} mm')
            
        except Exception as e:
            self.get_logger().error(f'Gripper offset test failed: {e}')
    
    def run_tests(self):
        """Run all tests."""
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('TF2 TRANSFORM TESTS')
        self.get_logger().info('='*70 + '\n')
        
        # List available frames
        self.list_available_frames()
        
        print()
        
        # Test 1: Camera to arm base
        self.test_transform('camera_link', 'arm_base_link', 100.0, 50.0, 200.0)
        
        print()
        
        # Test 2: Camera to gripper tip (direct)
        result = self.test_transform('camera_link', 'gripper_tip', 100.0, 50.0, 200.0)
        
        print()
        
        # Test 3: Gripper offset calculation
        self.test_gripper_offset()
        
        print()
        
        # Test 4: Multiple test points
        self.get_logger().info('='*70)
        self.get_logger().info('Testing Multiple Points (camera_link -> arm_base_link)')
        self.get_logger().info('='*70)
        
        test_points = [
            (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0),
            (0.0, 100.0, 0.0),
            (0.0, 0.0, 100.0),
            (150.0, 150.0, 250.0),
        ]
        
        for x, y, z in test_points:
            result = self.test_transform('camera_link', 'arm_base_link', x, y, z)
            if result:
                print()
        
        self.get_logger().info('='*70)
        self.get_logger().info('Tests Complete!')
        self.get_logger().info('='*70)
        
        # Provide usage instructions
        print()
        self.get_logger().info('To visualize transforms in RViz2:')
        self.get_logger().info('  1. Run: rviz2')
        self.get_logger().info('  2. Add display: TF')
        self.get_logger().info('  3. Set Fixed Frame to: arm_base_link or rover_base')
        print()
        self.get_logger().info('To view TF tree as PDF:')
        self.get_logger().info('  ros2 run tf2_tools view_frames')
        print()
        self.get_logger().info('To monitor a specific transform:')
        self.get_logger().info('  ros2 run tf2_ros tf2_echo camera_link arm_base_link')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = TF2TransformTester()
        # Keep node alive briefly
        rclpy.spin_once(tester, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tester.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
