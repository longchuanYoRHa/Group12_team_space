from pymycobot import MyCobot280
from pymycobot import PI_PORT, PI_BAUD
import time

# 1. Initialize the MyCobot object
# For Raspberry Pi version, PI_PORT and PI_BAUD are pre-defined
mc = MyCobot280(PI_PORT, PI_BAUD)

pick_point = [100, 170, 200, -45, -90.0, 180]
safe_z = 220.0
move_speed = 50
gripper_speed = 80

coordinate=mc.get_coords()
print(coordinate)

# # --- Phase 2: Pick Sequence ---
# # Move to the safe height above the pick point
# print(f"Moving to safe height above pick point: {safe_z}mm")
# mc.send_coord(3, safe_z, move_speed) # 3 corresponds to Z-axis
# print(coordinate) 
# time.sleep(2)

# Descend to the pick coordinate
print("Descending to pick object.")
mc.send_coords(pick_point, move_speed, 1) # 1 is Linear move for accuracy

time.sleep(3)

print(coordinate)

