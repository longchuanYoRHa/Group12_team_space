#!/usr/bin/env python3
"""
Gripper threshold testing tool for MyCobot 280 Pi.

This script helps you determine the optimal grip_threshold value by:
1. Testing gripper closure with and without objects
2. Recording gripper values in different scenarios
3. Suggesting appropriate threshold based on measurements

Usage:
    python3 test_gripper_threshold.py

Requirements:
    - MyCobot hardware connected
    - pymycobot library installed
    - Block(s) to test with
"""

import os
import sys
import time
from pathlib import Path

# Check for hardware
if not os.path.exists("/dev/ttyAMA0"):
    print("ERROR: No hardware detected (/dev/ttyAMA0 not found)")
    print("This test requires real MyCobot hardware.")
    sys.exit(1)

try:
    from pymycobot import MyCobot280
except ImportError:
    print("ERROR: pymycobot not installed")
    print("Install with: pip install pymycobot")
    sys.exit(1)


class GripperThresholdTester:
    """Interactive tool to test gripper and determine threshold."""

    def __init__(self, port="/dev/ttyAMA0", baud=1000000):
        print("=" * 60)
        print("MyCobot Gripper Threshold Testing Tool")
        print("=" * 60)
        print(f"\nConnecting to {port} @ {baud}...")
        
        self.mc = MyCobot280(port, baud)
        self.mc.power_on()
        time.sleep(1.0)
        
        # Default gripper settings
        self.gripper_speed = 80
        self.gripper_torque = 300
        
        try:
            self.mc.set_HTS_gripper_torque(self.gripper_torque)
            print(f"✓ Gripper torque set to {self.gripper_torque}")
        except Exception as e:
            print(f"⚠ Could not set torque: {e}")
        
        # Storage for measurements
        self.empty_readings = []
        self.object_readings = []
        
        print("✓ Connection established\n")

    def get_gripper_value(self):
        """Read gripper value with error handling."""
        try:
            val = self.mc.get_gripper_value(gripper_type=1)
            return val if val is not None else -1
        except Exception as e:
            print(f"  Error reading gripper: {e}")
            return -1

    def test_empty_grip(self, num_samples=3):
        """Test gripper closing without any object."""
        print("\n" + "=" * 60)
        print("TEST 1: Empty Gripper (No Object)")
        print("=" * 60)
        print("\nMake sure there is NO object in the gripper.")
        input("Press ENTER when ready...")
        
        self.empty_readings.clear()
        
        for i in range(num_samples):
            print(f"\n--- Sample {i+1}/{num_samples} ---")
            
            # Open gripper
            print("  Opening gripper...")
            self.mc.set_gripper_state(0, self.gripper_speed)
            time.sleep(2.0)
            
            # Close gripper
            print("  Closing gripper (empty)...")
            self.mc.set_gripper_state(1, self.gripper_speed)
            time.sleep(2.5)
            
            # Read value
            val = self.get_gripper_value()
            self.empty_readings.append(val)
            print(f"  ✓ Gripper value (empty): {val}")
            
            time.sleep(1.0)
        
        # Statistics
        if self.empty_readings:
            avg = sum(self.empty_readings) / len(self.empty_readings)
            max_val = max(self.empty_readings)
            min_val = min(self.empty_readings)
            print(f"\n📊 Empty grip statistics:")
            print(f"   Average: {avg:.1f}")
            print(f"   Range:   {min_val} - {max_val}")

    def test_object_grip(self, num_samples=3):
        """Test gripper closing with an object."""
        print("\n" + "=" * 60)
        print("TEST 2: Gripper with Object")
        print("=" * 60)
        print("\nPlace a block in the gripper when prompted.")
        
        self.object_readings.clear()
        
        for i in range(num_samples):
            print(f"\n--- Sample {i+1}/{num_samples} ---")
            
            # Open gripper
            print("  Opening gripper...")
            self.mc.set_gripper_state(0, self.gripper_speed)
            time.sleep(2.0)
            
            input("  Place block in gripper, then press ENTER...")
            
            # Close gripper on object
            print("  Closing gripper on object...")
            self.mc.set_gripper_state(1, self.gripper_speed)
            time.sleep(2.5)
            
            # Read value
            val = self.get_gripper_value()
            self.object_readings.append(val)
            print(f"  ✓ Gripper value (with object): {val}")
            
            print("  Waiting for you to remove block...")
            time.sleep(2.0)
        
        # Statistics
        if self.object_readings:
            avg = sum(self.object_readings) / len(self.object_readings)
            max_val = max(self.object_readings)
            min_val = min(self.object_readings)
            print(f"\n📊 Object grip statistics:")
            print(f"   Average: {avg:.1f}")
            print(f"   Range:   {min_val} - {max_val}")

    def test_different_objects(self):
        """Test with different sized objects."""
        print("\n" + "=" * 60)
        print("TEST 3: Different Object Sizes (Optional)")
        print("=" * 60)
        
        response = input("\nTest with different sized objects? (y/n): ")
        if response.lower() != 'y':
            return
        
        object_types = []
        
        while True:
            obj_name = input("\nEnter object description (or 'done' to finish): ")
            if obj_name.lower() == 'done':
                break
            
            # Open gripper
            print("  Opening gripper...")
            self.mc.set_gripper_state(0, self.gripper_speed)
            time.sleep(2.0)
            
            input(f"  Place '{obj_name}' in gripper, then press ENTER...")
            
            # Close gripper
            print("  Closing gripper...")
            self.mc.set_gripper_state(1, self.gripper_speed)
            time.sleep(2.5)
            
            # Read value
            val = self.get_gripper_value()
            object_types.append((obj_name, val))
            print(f"  ✓ Gripper value for '{obj_name}': {val}")
            
            time.sleep(1.0)
        
        if object_types:
            print("\n📊 Different object measurements:")
            for name, val in object_types:
                print(f"   {name:20s}: {val}")

    def analyze_and_recommend(self):
        """Analyze all measurements and recommend threshold."""
        print("\n" + "=" * 60)
        print("ANALYSIS & RECOMMENDATIONS")
        print("=" * 60)
        
        if not self.empty_readings or not self.object_readings:
            print("\n⚠ Insufficient data for analysis.")
            print("  Please complete both empty and object tests.")
            return
        
        # Calculate statistics
        empty_max = max(self.empty_readings)
        empty_avg = sum(self.empty_readings) / len(self.empty_readings)
        
        object_min = min(self.object_readings)
        object_avg = sum(self.object_readings) / len(self.object_readings)
        
        print(f"\n📊 Summary:")
        print(f"   Empty grip:  avg={empty_avg:.1f}, max={empty_max}")
        print(f"   With object: avg={object_avg:.1f}, min={object_min}")
        
        # Check for overlap
        if empty_max >= object_min:
            print(f"\n⚠ WARNING: Overlap detected!")
            print(f"   Empty max ({empty_max}) >= Object min ({object_min})")
            print(f"   This may cause unreliable detection.")
        
        # Calculate recommended threshold
        # Safe threshold: halfway between empty_max and object_min, with margin
        margin = 2  # Safety margin
        recommended = empty_max + margin
        
        # Alternative: midpoint
        midpoint = (empty_max + object_min) / 2
        
        print(f"\n💡 Recommended Thresholds:")
        print(f"   Conservative (safer):  {recommended}")
        print(f"   Midpoint:             {midpoint:.1f}")
        
        # Validation check
        if recommended < object_min:
            print(f"\n✓ Threshold {recommended} should work well:")
            print(f"   - Above empty grip max ({empty_max})")
            print(f"   - Below object grip min ({object_min})")
        else:
            print(f"\n⚠ Threshold selection difficult due to overlap.")
            print(f"   Try different objects or adjust gripper torque.")
        
        # Generate config
        print(f"\n📝 Configuration for mycobot_controller:")
        print(f"   grip_threshold: {recommended}")
        print(f"\n   Add to your launch file or as ROS parameter:")
        print(f"   <param name=\"grip_threshold\" value=\"{recommended}\"/>")

    def interactive_test(self):
        """Interactive mode to test current threshold."""
        print("\n" + "=" * 60)
        print("INTERACTIVE TESTING")
        print("=" * 60)
        
        while True:
            print("\nCommands:")
            print("  o - Open gripper")
            print("  c - Close gripper")
            print("  r - Read gripper value")
            print("  q - Quit")
            
            cmd = input("\nEnter command: ").lower()
            
            if cmd == 'o':
                print("Opening gripper...")
                self.mc.set_gripper_state(0, self.gripper_speed)
                time.sleep(2.0)
                print("✓ Open")
                
            elif cmd == 'c':
                print("Closing gripper...")
                self.mc.set_gripper_state(1, self.gripper_speed)
                time.sleep(2.5)
                val = self.get_gripper_value()
                print(f"✓ Closed, value: {val}")
                
            elif cmd == 'r':
                val = self.get_gripper_value()
                print(f"Current gripper value: {val}")
                
            elif cmd == 'q':
                break
            else:
                print("Unknown command")

    def cleanup(self):
        """Safe cleanup - open gripper."""
        print("\nCleaning up...")
        try:
            self.mc.set_gripper_state(0, self.gripper_speed)
            time.sleep(2.0)
            print("✓ Gripper opened")
        except Exception as e:
            print(f"Cleanup error: {e}")

    def run_full_test(self):
        """Run complete testing workflow."""
        try:
            # Test 1: Empty grip
            self.test_empty_grip(num_samples=3)
            
            # Test 2: Object grip
            self.test_object_grip(num_samples=3)
            
            # Test 3: Different objects (optional)
            self.test_different_objects()
            
            # Analysis
            self.analyze_and_recommend()
            
            # Interactive mode
            response = input("\nEnter interactive testing mode? (y/n): ")
            if response.lower() == 'y':
                self.interactive_test()
            
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        finally:
            self.cleanup()


def main():
    """Main entry point."""
    print("\nThis tool will help you determine the optimal gripper threshold.")
    print("You will need:")
    print("  - Your robot arm powered and connected")
    print("  - One or more test blocks")
    print("  - About 5-10 minutes\n")
    
    response = input("Ready to begin? (y/n): ")
    if response.lower() != 'y':
        print("Aborted.")
        return
    
    tester = GripperThresholdTester()
    tester.run_full_test()
    
    print("\n" + "=" * 60)
    print("Testing complete!")
    print("=" * 60)
    print("\nNext steps:")
    print("  1. Update your launch file with recommended grip_threshold")
    print("  2. Test pick-and-place with real blocks")
    print("  3. Adjust if needed based on success rate")
    print()


if __name__ == '__main__':
    main()
