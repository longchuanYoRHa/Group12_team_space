from pymycobot import MyCobot280
from pymycobot import PI_PORT, PI_BAUD
import time

# 1. Initialize the MyCobot object
mc = MyCobot280(PI_PORT, PI_BAUD)

def verify_with_free_move():
    print("INFO: Releasing all servos. You can now move the arm manually.")
    
    # 2. Unlock the joints so they can be moved by hand
    # This relaxes the motor tension
    # mc.set_free_mode(1)
    mc.release_all_servos() 
    
    # Alternatively, you could use: mc.set_free_mode(1)
    
    print("Verification Mode Active: Move the arm and watch the coordinates.")
    print("Press Ctrl+C to stop and re-engage the motors.")

    try:
        while True:
            # 3. Get current coordinates [x, y, z, rx, ry, rz]
            coords = mc.get_coords()
            
            if coords and len(coords) == 6:
                # Format the output for clarity
                print(f"X: {coords[0]:>7.2f} | Y: {coords[1]:>7.2f} | Z: {coords[2]:>7.2f} | "
                      f"Rx: {coords[3]:>7.2f} | Ry: {coords[4]:>7.2f} | Rz: {coords[5]:>7.2f}")
            else:
                print("Error: Could not read coordinates.")
                
            time.sleep(0.2) # Faster update rate for smoother tracking
            
    except KeyboardInterrupt:
        print("\nINFO: Stopping verification.")
    finally:
        # 4. Re-engage the motors to lock the position before exiting
        print("INFO: Re-engaging servos (locking joints).")
        mc.focus_all_servos() # Powers the servos back on to the current position

if __name__ == "__main__":
    # Check if Atom is connected
    if mc.is_controller_connected() == 1:
        verify_with_free_move()
    else:
        print("ERROR: Connection failed. Check your hardware.")