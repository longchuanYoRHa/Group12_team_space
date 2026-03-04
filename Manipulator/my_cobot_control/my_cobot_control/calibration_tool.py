#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration tool for MyCobot280Pi.
Records joint angles and Cartesian coordinates at key positions,
and saves them as JSON files for subsequent use.
"""

import time
import os
import sys
import termios
import tty
import json
from datetime import datetime
import serial
import serial.tools.list_ports

from pymycobot.mycobot280 import MyCobot280


class CalibrationTool:
    """Calibration tool - record and manage robot arm calibration points."""
    
    def __init__(self, mycobot):
        self.mc = mycobot
        self.calibration_points = {}
        self.current_point_name = None
        
        # Create calibration output directory if it doesn't exist
        self.calib_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 
            "calibration_data"
        )
        if not os.path.exists(self.calib_dir):
            os.makedirs(self.calib_dir)
            
    def release_servos(self):
        """Release all servos, allowing manual dragging."""
        self.mc.release_all_servos()
        print("OK - Servos released, you can now drag the arm manually")
        
    def lock_servos(self):
        """Lock all servos."""
        self.mc.focus_all_servos()
        print("OK - Servos locked")
        
    def get_current_pose(self):
        """
        Get the current arm pose.
        Returns: dict containing joint angles and Cartesian coordinates.
        """
        # Get joint angles and encoder values
        angles = self.mc.get_angles()
        encoders = self.mc.get_encoders()
        
        # Get Cartesian coordinates [x, y, z, rx, ry, rz]
        coords = self.mc.get_coords()
        
        pose_data = {
            "timestamp": datetime.now().isoformat(),
            "angles": angles,          # Joint angles (degrees)
            "encoders": encoders,      # Encoder values
            "coords": coords,          # Cartesian coordinates [x, y, z, rx, ry, rz]
        }
        
        return pose_data
    
    def save_calibration_point(self, point_name, description=""):
        """
        Save the current position as a calibration point.
        
        Args:
            point_name: Name of the point, e.g. "home", "pickup", "place_1"
            description: Optional description of the point
        """
        pose_data = self.get_current_pose()
        pose_data["description"] = description
        pose_data["name"] = point_name
        
        self.calibration_points[point_name] = pose_data
        
        print(f"\nOK - Saved calibration point: {point_name}")
        if description:
            print(f"  Description: {description}")
        print(f"  Joint angles: {pose_data['angles']}")
        print(f"  Cartesian coords: {pose_data['coords']}")
        
    def display_current_pose(self):
        """Display the current arm pose."""
        pose = self.get_current_pose()
        
        print("\n" + "="*60)
        print("Current arm pose:")
        print("-"*60)
        print(f"Joint angles (deg): {pose['angles']}")
        print(f"Encoder values:     {pose['encoders']}")
        print(f"Cartesian (mm):     {pose['coords']}")
        print(f"  X: {pose['coords'][0]:.2f} mm")
        print(f"  Y: {pose['coords'][1]:.2f} mm")
        print(f"  Z: {pose['coords'][2]:.2f} mm")
        print(f"  RX: {pose['coords'][3]:.2f} deg")
        print(f"  RY: {pose['coords'][4]:.2f} deg")
        print(f"  RZ: {pose['coords'][5]:.2f} deg")
        print("="*60 + "\n")
        
    def save_to_file(self, filename=None):
        """
        Save all calibration points to a JSON file.
        
        Args:
            filename: File name (optional), defaults to timestamp-based name.
        """
        if not self.calibration_points:
            print("WARNING: No calibration points to save")
            return
            
        if filename is None:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"calibration_{timestamp}.json"
            
        filepath = os.path.join(self.calib_dir, filename)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.calibration_points, f, indent=2, ensure_ascii=False)
            
        print(f"\nOK - Calibration data saved to: {filepath}")
        print(f"  Total points saved: {len(self.calibration_points)}")
        
    def load_from_file(self, filename):
        """
        Load calibration points from a JSON file.
        
        Args:
            filename: File name to load.
        """
        filepath = os.path.join(self.calib_dir, filename)
        
        if not os.path.exists(filepath):
            print(f"WARNING: File not found: {filepath}")
            return
            
        with open(filepath, 'r', encoding='utf-8') as f:
            self.calibration_points = json.load(f)
            
        print(f"OK - Loaded calibration data: {filepath}")
        print(f"  Total points loaded: {len(self.calibration_points)}")
        self.list_calibration_points()
        
    def list_calibration_points(self):
        """List all saved calibration points."""
        if not self.calibration_points:
            print("\nNo calibration points saved yet")
            return
            
        print("\n" + "="*60)
        print(f"Saved calibration points ({len(self.calibration_points)} total):")
        print("-"*60)
        for name, data in self.calibration_points.items():
            print(f"\nPoint: {name}")
            if data.get('description'):
                print(f"  Description: {data['description']}")
            print(f"  Joint angles: {data['angles']}")
            print(f"  Coords: {data['coords']}")
        print("="*60 + "\n")
        
    def move_to_point(self, point_name, speed=50):
        """
        Move to a saved calibration point.
        
        Args:
            point_name: Name of the calibration point.
            speed: Movement speed (1-100).
        """
        if point_name not in self.calibration_points:
            print(f"WARNING: Point not found: {point_name}")
            return
            
        point_data = self.calibration_points[point_name]
        angles = point_data['angles']
        
        print(f"Moving to point: {point_name}")
        self.mc.send_angles(angles, speed)
        
    def delete_point(self, point_name):
        """Delete a calibration point."""
        if point_name in self.calibration_points:
            del self.calibration_points[point_name]
            print(f"OK - Deleted point: {point_name}")
        else:
            print(f"WARNING: Point not found: {point_name}")
            
    def print_menu(self):
        """Print the interactive menu."""
        print("\n" + "="*60)
        print("Arm Calibration Tool - Menu")
        print("="*60)
        print("  q: Quit")
        print("  r: Release servos (allow dragging)")
        print("  l: Lock servos")
        print("  d: Display current pose")
        print("  s: Save current pose as calibration point")
        print("  v: View all saved calibration points")
        print("  m: Move to a calibration point")
        print("  w: Write calibration data to file")
        print("  o: Open (load) calibration data from file")
        print("  x: Delete a calibration point")
        print("  h: Show this menu")
        print("-"*60 + "\n")
        
    def run_interactive(self):
        """Run the interactive calibration program."""
        self.print_menu()
        
        while True:
            print("\nEnter command ('h' for menu): ", end='')
            command = input().strip().lower()
            
            if command == 'q':
                print("Exiting...")
                break
                
            elif command == 'r':
                self.release_servos()
                
            elif command == 'l':
                self.lock_servos()
                
            elif command == 'd':
                self.display_current_pose()
                
            elif command == 's':
                point_name = input("Enter point name (e.g. home, pickup, place_1): ").strip()
                if not point_name:
                    print("WARNING: Point name cannot be empty")
                    continue
                description = input("Enter description (optional): ").strip()
                self.save_calibration_point(point_name, description)
                
            elif command == 'v':
                self.list_calibration_points()
                
            elif command == 'm':
                self.list_calibration_points()
                point_name = input("Enter name of point to move to: ").strip()
                speed = input("Enter speed (1-100, default 50): ").strip()
                speed = int(speed) if speed else 50
                self.move_to_point(point_name, speed)
                
            elif command == 'w':
                filename = input("Enter filename (press Enter for default timestamp): ").strip()
                self.save_to_file(filename if filename else None)
                
            elif command == 'o':
                # List available calibration files
                files = [f for f in os.listdir(self.calib_dir) if f.endswith('.json')]
                if files:
                    print("\nAvailable calibration files:")
                    for i, f in enumerate(files, 1):
                        print(f"  {i}. {f}")
                    filename = input("Enter filename: ").strip()
                    self.load_from_file(filename)
                else:
                    print("WARNING: No calibration files found")
                    
            elif command == 'x':
                self.list_calibration_points()
                point_name = input("Enter name of point to delete: ").strip()
                self.delete_point(point_name)
                
            elif command == 'h':
                self.print_menu()
                
            else:
                print(f"WARNING: Unknown command: {command}")


def setup_mycobot():
    """Set up and connect to the arm."""
    print("\n" + "="*60)
    print("MyCobot280Pi Calibration Tool - Init")
    print("="*60)
    
    # List available serial ports
    plist = list(serial.tools.list_ports.comports())
    if not plist:
        print("WARNING: No serial ports found")
        sys.exit(1)
        
    print("\nAvailable serial ports:")
    for idx, port in enumerate(plist, 1):
        print(f"  {idx}. {port}")
        
    port_idx = input(f"\nSelect port (1-{len(plist)}): ").strip()
    try:
        port = str(plist[int(port_idx) - 1]).split(" - ")[0].strip()
    except:
        print("WARNING: Invalid selection, using first port")
        port = str(plist[0]).split(" - ")[0].strip()
        
    print(f"Using port: {port}")
    
    # Baud rate
    baud = input("Enter baud rate (default: 1000000): ").strip()
    baud = int(baud) if baud else 1000000
    
    print(f"\nConnecting to arm...")
    mc = MyCobot280(port, baud)
    time.sleep(0.5)
    
    # Power on
    mc.power_on()
    time.sleep(0.5)
    
    print("OK - Arm connected!\n")
    
    return mc


def main():
    """Main entry point."""
    try:
        # Initialise arm
        mc = setup_mycobot()
        
        # Create calibration tool
        calibration = CalibrationTool(mc)
        
        # Run interactive calibration
        calibration.run_interactive()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nGoodbye!")


if __name__ == "__main__":
    main()
