#!/usr/bin/env python3
"""
Test script for camera to arm base coordinate transformation.
测试摄像头到机械臂基座的坐标转换。

Usage / 用法:
    python3 test_camera_transform.py
"""

import math
import numpy as np


def transform_camera_to_base(x_cam, y_cam, z_cam, tx, ty, tz, rx, ry, rz):
    """
    Transform coordinates from camera frame to robot base frame.
    
    Args:
        x_cam, y_cam, z_cam: Coordinates in camera frame (mm)
        tx, ty, tz: Translation from camera to base (mm)
        rx, ry, rz: Rotation from camera to base (degrees)
        
    Returns:
        (x_base, y_base, z_base): Coordinates in robot base frame (mm)
    """
    # Convert angles to radians
    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry)
    rz_rad = math.radians(rz)
    
    # Build rotation matrices (ZYX Euler angles)
    cos_rx, sin_rx = math.cos(rx_rad), math.sin(rx_rad)
    cos_ry, sin_ry = math.cos(ry_rad), math.sin(ry_rad)
    cos_rz, sin_rz = math.cos(rz_rad), math.sin(rz_rad)
    
    # Rotation matrix around X axis
    Rx = np.array([
        [1,      0,       0],
        [0, cos_rx, -sin_rx],
        [0, sin_rx,  cos_rx]
    ])
    
    # Rotation matrix around Y axis
    Ry = np.array([
        [ cos_ry, 0, sin_ry],
        [      0, 1,      0],
        [-sin_ry, 0, cos_ry]
    ])
    
    # Rotation matrix around Z axis
    Rz = np.array([
        [cos_rz, -sin_rz, 0],
        [sin_rz,  cos_rz, 0],
        [     0,       0, 1]
    ])
    
    # Combined rotation matrix (rotation order: Z-Y-X)
    R = Rz @ Ry @ Rx
    
    # Point in camera frame
    p_cam = np.array([x_cam, y_cam, z_cam])
    
    # Transform: p_base = R * p_cam + t
    p_base = R @ p_cam + np.array([tx, ty, tz])
    
    return float(p_base[0]), float(p_base[1]), float(p_base[2])


def main():
    print("="*70)
    print("摄像头到机械臂坐标转换测试工具")
    print("Camera to Arm Base Coordinate Transform Test Tool")
    print("="*70)
    print()
    
    # Example 1: No transform (identity)
    print("示例 1: 无转换 (恒等变换)")
    print("Example 1: No transform (identity)")
    print("-"*70)
    x_cam, y_cam, z_cam = 100.0, 50.0, 200.0
    tx, ty, tz = 0.0, 0.0, 0.0
    rx, ry, rz = 0.0, 0.0, 0.0
    
    x_base, y_base, z_base = transform_camera_to_base(
        x_cam, y_cam, z_cam, tx, ty, tz, rx, ry, rz)
    
    print(f"摄像头坐标 Camera coords: ({x_cam:.1f}, {y_cam:.1f}, {z_cam:.1f}) mm")
    print(f"平移 Translation: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm")
    print(f"旋转 Rotation: ({rx:.1f}, {ry:.1f}, {rz:.1f}) deg")
    print(f"→ 基座坐标 Base coords: ({x_base:.1f}, {y_base:.1f}, {z_base:.1f}) mm")
    print()
    
    # Example 2: Translation only
    print("示例 2: 仅平移")
    print("Example 2: Translation only")
    print("-"*70)
    x_cam, y_cam, z_cam = 100.0, 50.0, 200.0
    tx, ty, tz = 150.0, -200.0, 300.0
    rx, ry, rz = 0.0, 0.0, 0.0
    
    x_base, y_base, z_base = transform_camera_to_base(
        x_cam, y_cam, z_cam, tx, ty, tz, rx, ry, rz)
    
    print(f"摄像头坐标 Camera coords: ({x_cam:.1f}, {y_cam:.1f}, {z_cam:.1f}) mm")
    print(f"平移 Translation: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm")
    print(f"旋转 Rotation: ({rx:.1f}, {ry:.1f}, {rz:.1f}) deg")
    print(f"→ 基座坐标 Base coords: ({x_base:.1f}, {y_base:.1f}, {z_base:.1f}) mm")
    print()
    
    # Example 3: 180° rotation (camera upside down)
    print("示例 3: 摄像头倒置安装 (180° 旋转)")
    print("Example 3: Camera upside down (180° rotation)")
    print("-"*70)
    x_cam, y_cam, z_cam = 100.0, 50.0, 200.0
    tx, ty, tz = 0.0, 0.0, 500.0  # Camera 500mm above base
    rx, ry, rz = 180.0, 0.0, 0.0
    
    x_base, y_base, z_base = transform_camera_to_base(
        x_cam, y_cam, z_cam, tx, ty, tz, rx, ry, rz)
    
    print(f"摄像头坐标 Camera coords: ({x_cam:.1f}, {y_cam:.1f}, {z_cam:.1f}) mm")
    print(f"平移 Translation: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm")
    print(f"旋转 Rotation: ({rx:.1f}, {ry:.1f}, {rz:.1f}) deg")
    print(f"→ 基座坐标 Base coords: ({x_base:.1f}, {y_base:.1f}, {z_base:.1f}) mm")
    print()
    
    # Example 4: Camera tilted down 30°
    print("示例 4: 摄像头向下倾斜 30°")
    print("Example 4: Camera tilted down 30°")
    print("-"*70)
    x_cam, y_cam, z_cam = 100.0, 50.0, 200.0
    tx, ty, tz = 200.0, 0.0, 150.0
    rx, ry, rz = 0.0, -30.0, 0.0
    
    x_base, y_base, z_base = transform_camera_to_base(
        x_cam, y_cam, z_cam, tx, ty, tz, rx, ry, rz)
    
    print(f"摄像头坐标 Camera coords: ({x_cam:.1f}, {y_cam:.1f}, {z_cam:.1f}) mm")
    print(f"平移 Translation: ({tx:.1f}, {ty:.1f}, {tz:.1f}) mm")
    print(f"旋转 Rotation: ({rx:.1f}, {ry:.1f}, {rz:.1f}) deg")
    print(f"→ 基座坐标 Base coords: ({x_base:.1f}, {y_base:.1f}, {z_base:.1f}) mm")
    print()
    
    # Interactive mode
    print("="*70)
    print("交互模式 / Interactive Mode")
    print("="*70)
    print()
    print("输入您的参数进行测试 (按 Ctrl+C 退出)")
    print("Enter your parameters to test (Ctrl+C to exit)")
    print()
    
    try:
        while True:
            print("-"*70)
            print("摄像头坐标 / Camera coordinates (mm):")
            x_cam = float(input("  X: "))
            y_cam = float(input("  Y: "))
            z_cam = float(input("  Z: "))
            
            print("\n摄像头到基座的平移 / Translation camera->base (mm):")
            tx = float(input("  TX: "))
            ty = float(input("  TY: "))
            tz = float(input("  TZ: "))
            
            print("\n摄像头到基座的旋转 / Rotation camera->base (degrees):")
            rx = float(input("  RX: "))
            ry = float(input("  RY: "))
            rz = float(input("  RZ: "))
            
            x_base, y_base, z_base = transform_camera_to_base(
                x_cam, y_cam, z_cam, tx, ty, tz, rx, ry, rz)
            
            print("\n结果 / Result:")
            print(f"  摄像头坐标 Camera: ({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f}) mm")
            print(f"  →")
            print(f"  基座坐标 Base:   ({x_base:.2f}, {y_base:.2f}, {z_base:.2f}) mm")
            print()
            
            # Generate ROS parameter config
            print("ROS 参数配置 / ROS parameter config:")
            print(f"""
camera_to_base_x: {tx}
camera_to_base_y: {ty}
camera_to_base_z: {tz}
camera_to_base_rx: {rx}
camera_to_base_ry: {ry}
camera_to_base_rz: {rz}
use_camera_transform: true
            """)
            
    except KeyboardInterrupt:
        print("\n\n退出 / Exiting...")
    except Exception as e:
        print(f"\n错误 / Error: {e}")


if __name__ == '__main__':
    main()
