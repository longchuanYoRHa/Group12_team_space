from pymycobot import MyCobot280
from pymycobot import PI_PORT, PI_BAUD
import time
import os

# Mock class for simulation
class MockMyCobot280:
    def __init__(self, port, baud):
        print(f"[MOCK] Initializing MyCobot280 on {port} @ {baud}")
        self.connected = True
    
    def is_controller_connected(self):
        print("[MOCK] Checking connection...")
        return 1
    
    def power_on(self):
        print("[MOCK] Power ON")
    
    def set_gripper_state(self, state, speed):
        state_str = "OPEN" if state == 0 else "CLOSED"
        print(f"[MOCK] Gripper {state_str} at speed {speed}")
    
    def send_coord(self, axis, value, speed):
        print(f"[MOCK] Moving axis {axis} to {value}mm at speed {speed}")
    
    def send_coords(self, coords, speed, mode):
        mode_str = "LINEAR" if mode == 1 else "JOINT"
        print(f"[MOCK] Moving to {coords} at speed {speed} ({mode_str} mode)")

def run_pick_and_place(mc, pick_point: list, place_point: list):
    # --- Configuration ---
    move_speed = 50 
    gripper_speed = 80
    
    # Define a safe Z-axis height to avoid obstacles
    safe_z = 220.0 

    print("INFO: Starting Pick and Place sequence...")

    # --- Phase 1: Preparation ---
    mc.power_on()
    print("INFO: Powering on and opening gripper.")
    mc.set_gripper_state(0, gripper_speed)
    time.sleep(0.5)  # Shorten waiting time for testing

    # --- Phase 2: Pick Sequence ---
    print(f"STEP 1: Moving to safe height above pick point: {safe_z}mm")
    mc.send_coord(3, safe_z, move_speed)
    time.sleep(0.5)

    print("STEP 2: Descending to pick object.")
    mc.send_coords(pick_point, move_speed, 1)
    time.sleep(0.5)

    print("STEP 3: Closing gripper.")
    mc.set_gripper_state(1, gripper_speed)
    time.sleep(0.5)

    print("STEP 4: Lifting object.")
    mc.send_coord(3, safe_z, move_speed)
    time.sleep(0.5)

    # --- Phase 3: Place Sequence ---
    print("STEP 5: Moving to collection area.")
    mc.send_coords([place_point[0], place_point[1], safe_z, 
                    place_point[3], place_point[4], place_point[5]], move_speed, 0)
    time.sleep(0.5)

    print("STEP 6: Lowering object to release point.")
    mc.send_coords(place_point, move_speed, 1)
    time.sleep(0.5)

    print("STEP 7: Releasing object.")
    mc.set_gripper_state(0, gripper_speed)
    time.sleep(0.5)

    print("STEP 8: Returning to safety.")
    mc.send_coord(3, safe_z, move_speed)
    time.sleep(0.5)

    print("SUCCESS: Task completed.")

def main():
    # 检测是否有真实硬件
    use_simulation = not os.path.exists(PI_PORT)
    
    if use_simulation:
        print("=" * 60)
        print("WARNING: No hardware detected. Running in SIMULATION mode.")
        print("=" * 60)
        mc = MockMyCobot280(PI_PORT, PI_BAUD)
    else:
        print("INFO: Hardware detected. Running in REAL mode.")
        mc = MyCobot280(PI_PORT, PI_BAUD)
    
    # Define Target Coordinates [x, y, z, rx, ry, rz]
    pick_point = [155.0, -40.0, 100.0, -180.0, 0.0, 0.0]
    place_point = [155.0, 150.0, 100.0, -180.0, 0.0, 0.0]

    # Check if the controller is connected
    if mc.is_controller_connected() == 1:

        run_pick_and_place(mc, pick_point, place_point)
    else:
        print("ERROR: Cannot connect to MyCobot. Check cables and power.")

if __name__ == "__main__":
    main()
