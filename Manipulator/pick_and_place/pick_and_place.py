from pymycobot import MyCobot280
from pymycobot import PI_PORT, PI_BAUD
import time

# 1. Initialize the MyCobot object
# For Raspberry Pi version, PI_PORT and PI_BAUD are pre-defined
mc = MyCobot280(PI_PORT, PI_BAUD)

def run_pick_and_place(pick_point: list, place_point: list):
    # --- Configuration ---
    move_speed = 50 
    gripper_speed = 80
    
    # Define a safe Z-axis height to avoid obstacles
    safe_z = 220.0 

    print("INFO: Starting Pick and Place sequence...")

    # --- Phase 1: Preparation ---
    # Power on the servos and open the gripper
    mc.power_on()
    print("INFO: Powering on and opening gripper.")
    mc.set_gripper_state(0, gripper_speed) # 0 is Open
    time.sleep(2)

    # --- Phase 2: Pick Sequence ---
    # Move to the safe height above the pick point
    print(f"STEP 1: Moving to safe height above pick point: {safe_z}mm")
    mc.send_coord(3, safe_z, move_speed) # 3 corresponds to Z-axis
    time.sleep(2)

    # Descend to the pick coordinate
    print("STEP 2: Descending to pick object.")
    mc.send_coords(pick_point, move_speed, 1) # 1 is Linear move for accuracy
    time.sleep(3)

    # Close gripper to grab object
    print("STEP 3: Closing gripper.")
    mc.set_gripper_state(1, gripper_speed) # 1 is Closed
    time.sleep(2)

    # Lift the object back to safe height
    print("STEP 4: Lifting object.")
    mc.send_coord(3, safe_z, move_speed)
    time.sleep(2)

    # --- Phase 3: Place Sequence ---
    # Move horizontally to the safe height above the collection coordinate
    print("STEP 5: Moving to collection area.")
    mc.send_coords([place_point[0], place_point[1], safe_z, 
                    place_point[3], place_point[4], place_point[5]], move_speed, 0)
    time.sleep(3)

    # Lower the object to the release coordinate
    print("STEP 6: Lowering object to release point.")
    mc.send_coords(place_point, move_speed, 1)
    time.sleep(2)

    # Open gripper to release
    print("STEP 7: Releasing object.")
    mc.set_gripper_state(0, gripper_speed)
    time.sleep(2)

    # Return to safe height
    print("STEP 8: Returning to safety.")
    mc.send_coord(3, safe_z, move_speed)
    time.sleep(2)

    print("SUCCESS: Task completed.")

if __name__ == "__main__":

    # Define Target Coordinates [x, y, z, rx, ry, rz]
    # Note: Ensure rx, ry, rz are set so the gripper points downward
    pick_point = [155.0, -40.0, 100.0, -180.0, 0.0, 0.0]
    place_point = [155.0, 150.0, 100.0, -180.0, 0.0, 0.0]

    # Check if the controller (Atom) is connected before starting
    if mc.is_controller_connected() == 1:

        run_pick_and_place(pick_point, place_point)
    else:
        print("ERROR: Cannot connect to MyCobot. Check cables and power.")