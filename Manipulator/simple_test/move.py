from pymycobot import MyCobot280
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of MyCobot, you can reference these two variables for MyCobot initialization
import time

# move to origin position by angles

# Init MyCobot280 object
mc = MyCobot280(PI_PORT, PI_BAUD)

mc.get_angles()
time.sleep(1)
mc.send_angles([0, 0, 0, 0, 0, 0], 30)
time.sleep(3)
mc.get_angles()
time.sleep(1)

print("Move to origin position...")
print(mc.get_angles())

