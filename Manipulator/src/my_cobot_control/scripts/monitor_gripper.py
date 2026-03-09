#!/usr/bin/env python3
"""
Real-time gripper value monitor for MyCobot 280 Pi.

Simple tool to continuously display gripper state.
Useful for quick threshold testing.

Usage:
    python3 monitor_gripper.py
    
    Then manually open/close the gripper to see values change.
"""

import os
import sys
import time

if not os.path.exists("/dev/ttyAMA0"):
    print("ERROR: No hardware detected")
    sys.exit(1)

try:
    from pymycobot import MyCobot280
except ImportError:
    print("ERROR: pymycobot not installed")
    sys.exit(1)


def main():
    print("Connecting to MyCobot...")
    mc = MyCobot280("/dev/ttyAMA0", 1000000)
    mc.power_on()
    time.sleep(1.0)
    
    # Set torque
    try:
        mc.set_HTS_gripper_torque(300)
    except Exception:
        pass
    
    print("\n" + "=" * 60)
    print("Real-time Gripper Monitor")
    print("=" * 60)
    print("\nCommands (type and press ENTER):")
    print("  o  - Open gripper")
    print("  c  - Close gripper")
    print("  q  - Quit")
    print("\nGripper values will be displayed continuously...")
    print("(Lower values = more closed, Higher = object blocking)")
    print("=" * 60 + "\n")
    
    import select
    import sys
    import tty
    import termios
    
    # Non-blocking input setup (Unix only)
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        
        print("Starting monitor (Ctrl+C to stop)...\n")
        
        while True:
            # Read gripper value
            try:
                val = mc.get_gripper_value(gripper_type=1)
                val_str = f"{val:3d}" if val is not None else "ERR"
            except Exception as e:
                val_str = "ERR"
            
            # Display with visual bar
            bar_len = min(50, max(0, val if val else 0))
            bar = "█" * bar_len
            
            # Print on same line
            print(f"\rGripper: {val_str} [{bar:<50}]", end="", flush=True)
            
            # Check for keyboard input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.read(1).lower()
                
                if cmd == 'o':
                    print("\n\n→ Opening gripper...")
                    mc.set_gripper_state(0, 80)
                    time.sleep(2.0)
                    print("✓ Gripper opened\n")
                    
                elif cmd == 'c':
                    print("\n\n→ Closing gripper...")
                    mc.set_gripper_state(1, 80)
                    time.sleep(2.5)
                    val = mc.get_gripper_value(gripper_type=1)
                    print(f"✓ Gripper closed, value: {val}\n")
                    
                elif cmd == 'q':
                    print("\n\nQuitting...")
                    break
            
            time.sleep(0.2)  # Update rate: 5 Hz
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Open gripper before exit
        print("Opening gripper before exit...")
        try:
            mc.set_gripper_state(0, 80)
            time.sleep(2.0)
        except Exception:
            pass
        
        print("Done.")


if __name__ == '__main__':
    main()
