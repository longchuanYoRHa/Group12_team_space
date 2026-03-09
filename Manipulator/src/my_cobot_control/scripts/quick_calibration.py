#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quick calibration example - simplified calibration workflow
for recording a few key positions quickly.
"""

import time
from pymycobot.mycobot280 import MyCobot280


def quick_calibration():
    """Quick calibration workflow."""
    
    # 1. Connect to arm
    print("Connecting to arm...")
    port = "/dev/ttyAMA0"  # Default Pi serial port, adjust as needed
    baud = 1000000
    mc = MyCobot280(port, baud)
    mc.power_on()
    time.sleep(1)
    
    print("OK - Arm connected\n")
    
    # 2. Calibration points dictionary
    calibration_points = {}
    
    def save_point(name, description):
        """Save the current position."""
        angles = mc.get_angles()
        coords = mc.get_coords()
        
        calibration_points[name] = {
            "angles": angles,
            "coords": coords,
            "description": description
        }
        
        print(f"OK - Saved: {name}")
        print(f"  Joint angles: {angles}")
        print(f"  Coords: {coords}")
        print()
    
    print("="*60)
    print("Quick Calibration Workflow")
    print("="*60)
    print("\nPlease follow the steps to calibrate key positions:\n")
    
    # 3. Home position
    input("[Step 1] Drag the arm to the HOME position, then press Enter...")
    mc.release_all_servos()  # Release for manual dragging
    input("Press Enter when done dragging...")
    mc.focus_all_servos()    # Lock
    save_point("home", "Home position")
    
    # 4. Pickup ready position
    input("[Step 2] Drag the arm ABOVE the pickup point, then press Enter...")
    mc.release_all_servos()
    input("Press Enter when done dragging...")
    mc.focus_all_servos()
    save_point("pickup_ready", "Pickup ready position (above block)")
    
    # 5. Pickup grasp position
    input("[Step 3] Drag the arm to the GRASP position (gripper touching block), then press Enter...")
    mc.release_all_servos()
    input("Press Enter when done dragging...")
    mc.focus_all_servos()
    save_point("pickup_grasp", "Pickup grasp position (gripper at block)")
    
    # 6. Place ready position
    input("[Step 4] Drag the arm ABOVE the place target, then press Enter...")
    mc.release_all_servos()
    input("Press Enter when done dragging...")
    mc.focus_all_servos()
    save_point("place_ready", "Place ready position (above target)")
    
    # 7. Place down position
    input("[Step 5] Drag the arm to the PLACE position, then press Enter...")
    mc.release_all_servos()
    input("Press Enter when done dragging...")
    mc.focus_all_servos()
    save_point("place_down", "Place position (release block)")
    
    # 8. Save results
    print("\n" + "="*60)
    print("Calibration complete! Saved positions:")
    print("-"*60)
    for name, data in calibration_points.items():
        print(f"\n{name}: {data['description']}")
        print(f"  Angles: {data['angles']}")
        print(f"  Coords: {data['coords']}")
    print("="*60)
    
    # 9. Save to file
    import json
    filename = "quick_calibration.json"
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(calibration_points, f, indent=2, ensure_ascii=False)
    print(f"\nOK - Calibration data saved to: {filename}")
    
    # 10. Test movement (optional)
    test = input("\nTest moving to each calibration point? (y/n): ").strip().lower()
    if test == 'y':
        print("\nStarting movement test...")
        for name in calibration_points.keys():
            print(f"Moving to: {name}")
            mc.send_angles(calibration_points[name]['angles'], 50)
            time.sleep(3)
        print("OK - Test complete")
    
    print("\nDone!")


if __name__ == "__main__":
    try:
        quick_calibration()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
