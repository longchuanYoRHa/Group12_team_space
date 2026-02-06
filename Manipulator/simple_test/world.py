from pymycobot.mycobot280 import MyCobot280
import time

# MyCobot280 类初始化需要两个参数：串口号和波特率

# 初始化一个MyCobot280对象
# 下面为 PI版本创建对象代码
mc = MyCobot280("/dev/ttyAMA0", 1000000)

if mc.get_fresh_mode() != 1:
  mc.set_fresh_mode(1)

# 获取当前头部的坐标以及姿态
coords = mc.get_coords()
print(coords)

# 智能规划路线，让头部以线性的方式到达[57.0, -107.4, 316.3]这个坐标，以及保持[-93.81, -12.71, -163.49]这个姿态，速度为80mm/s
# mc.send_coords([100, 100, 100, 100, 100, 100], 20, 1)

# 设置等待时间1.5秒
time.sleep(1.5)

# # 智能规划路线，让头部以线性的方式到达[-13.7, -107.5, 223.9]这个坐标，以及保持[165.52, -75.41, -73.52]这个姿态，速度为80mm/s
# mc.send_coords([-13.7, -107.5, 223.9, 165.52, -75.41, -73.52], 80, 1)

# 设置等待时间1.5秒
time.sleep(1.5)

# 仅改变头部的x坐标，设置头部的x坐标为-40。让其智能规划路线让头部移动到改变后的位置，，速度为70mm/s
mc.send_coord(1, -40, 70)
time.sleep(1.5)
