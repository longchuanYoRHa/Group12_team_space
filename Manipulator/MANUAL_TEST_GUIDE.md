# MyCobot 手动测试指南

在没有连接NUC的情况下，可以使用以下两种方式手动发送pickup和place命令进行测试。

## 方法一：使用Python测试脚本（推荐）

已创建 `manual_test.py` 脚本，包含了您提供的标定坐标。

### 使用方法

```bash
# 确保已source工作空间
source /home/student42/mycobot_ws/install/setup.bash
# 激活虚拟环境
source /home/student42/mycobot_ws/venv_mycobot/bin/activate

# 发送pickup命令 (坐标: x=202.2, y=-129.3, z=237.9)
python3 src/my_cobot_control/my_cobot_control/manual_test.py pick

# 发送place命令 (坐标: x=66.7, y=-218.2, z=123.7)
python3 src/my_cobot_control/my_cobot_control/manual_test.py place

# 发送自定义坐标
python3 src/my_cobot_control/my_cobot_control/manual_test.py custom 150.0 -100.0 200.0 pick
python3 src/my_cobot_control/my_cobot_control/manual_test.py custom 50.0 -200.0 150.0 place
```

## 方法二：直接使用ROS2命令行

```bash
# 发送pickup命令（使用标定的pickup坐标）
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 202.2, y: -129.3, z: 237.9}"
---
data: releasing
---

# 发送place命令（使用标定的place坐标）
ros2 topic pub --once /arm/target_place geometry_msgs/msg/Point "{x: 66.7, y: -218.2, z: 123.7}"

# 自定义坐标示例
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 150.0, y: -100.0, z: 200.0}"
```

## 完整测试流程

### 1. 启动控制器

在第一个终端：
```bash
cd /home/student42/mycobot_ws
source install/setup.bash
source venv_mycobot/bin/activate
ros2 launch my_cobot_control arm_control.launch.py
```

### 2. 监控状态

在第二个终端（可选）：
```bash
source /home/student42/mycobot_ws/install/setup.bash

# 监控arm状态
ros2 topic echo /arm/status

# 监控gripper状态
ros2 topic echo /arm/gripper_status

# 监控关节状态
ros2 topic echo /arm/joint_states
```

### 3. 发送命令

在第三个终端：
```bash
cd /home/student42/mycobot_ws
source install/setup.bash
source venv_mycobot/bin/activate

# 发送pickup命令
python3 src/my_cobot_control/my_cobot_control/manual_test.py pick

# 等待机械臂完成pick动作并进入HOLDING状态

# 发送place命令
python3 src/my_cobot_control/my_cobot_control/manual_test.py place
```

## 预定义的测试坐标

根据您的标定数据，脚本中预设了以下坐标：

**Pickup点:**
- x: 202.2 mm
- y: -129.3 mm
- z: 237.9 mm

**Place点:**
- x: 66.7 mm
- y: -218.2 mm
- z: 123.7 mm

## 状态流程

1. **IDLE** → 发送 `pick` 命令
2. **MOVING_TO_PICK** → 移动到pickup点上方
3. **DESCENDING_PICK** → 下降到pickup高度
4. **GRIPPING** → 夹取物体
5. **GRIP_CHECK** → 验证夹取
6. **LIFTING** → 提升物体
7. **RETURNING_HOME** → 返回home位置
8. **HOLDING** → 持有物体，等待place命令
9. 发送 `place` 命令 → **MOVING_TO_PLACE**
10. **RELEASING** → 释放物体
11. **RETURNING_HOME** → 返回home
12. **IDLE** → 完成

## 注意事项

- 在发送 `place` 命令前，必须先成功执行 `pick` 命令（状态为HOLDING）
- 如果使用MOCK模式（无实际硬件），所有动作会模拟执行
- 坐标范围限制：
  - x: -281.45 到 281.45 mm
  - y: -281.45 到 281.45 mm
  - z: -70.0 到 450.0 mm
