# Package Structure
my_cobot_control -- Rviz Control Package
mycobot_ros2 -- Official ROS 2 Package for myCobot Manipulator
mycobot280_pi -- Gazebo Simulation Package

# 任务
总项目是一个由迷你主机NUC，leo rover，机械臂，雷达和摄像头组成的机器人，目标是在迷宫中寻找对应颜色的积木块，并运输到对应颜色的桶里。
现在开发机械臂子项目，使用mycobot280pi机械臂+配套的自适应夹爪抓取积木块，然后将积木块放置到指定位置。
1.使用ros2通信接受来自主机的三维坐标（x，y，z），这个坐标是通过目标识别算法和深度摄像头获得的，相对位置将会固定在机器人上（最终需要tf变换）。
2. 考虑ros2设备通信，同topic占用问题，除了机械臂，leo rover也使用ros2与主机通信。
3.积木块在被夹取时，要检测是否有力反馈确定积木块已经被夹紧。
4.运送期间需要持续被夹住，直到放入桶中。
怎么设置ros2包？还有那些需要注意的技术细节？

暂不包含TF
- 先假设收到的坐标已经是机械臂基座坐标系下的，后续再加TF
Leo Rover和机械臂的ROS2隔离方式偏好？两者都连接到同一个NUC主机。
ROS2 Namespace
- 使用 /arm 和 /rover 命名空间隔离topic，共享同一个 ROS_DOMAIN_ID
放置位置（桶的位置）是固定的预设坐标，还是也通过ROS2 topic动态接收？
动态接收
- 放置位置也通过ROS2 topic从主机接收
主机与机械臂的通信模式偏好？这决定了抓取任务的触发方式。
Topic订阅
- 保持当前设计，订阅/target_object topic，发布/arm_status

////////////////////////////////////////////////////////////////////////////////
我现在想实现在gazebo上实现模拟，然后实现实际中的机械臂的任务：
在目前的gazebo模拟在，设置某个坐标位置，转换成角度坐标，实现模拟，可以用到官方提供的API：from pymycobot import MyCobot280 实现逆运动学，这是可行的么？


# 1 Build & Run Instructions
```
# Base on Ubuntu 22.04, to install pymycobot on your system, so we will create a virtual environment for pymycobot to avoid conflicts with ROS dependencies.

# Activate virtual environment
cd mycobot_ws/
source venv_mycobot/bin/activate

# Deactivate virtual environment
deactivate

# Rviz Build & Source
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
#Or
colcon build --symlink-install
source install/setup.bash

# Start Rviz
ros2 launch my_cobot_control pick_and_place_demo.launch.py

# Gazebo Build & Source
cd ~/mycobot_ws
colcon build --packages-select mycobot280_pi mycobot_description
source install/setup.bash
```

# 2 Start the Gazebo simulation environment

## 2.1 Without Gripper
```
# Start Gazebo Simulation
ros2 launch mycobot280_pi mycobot.launch.py
# Run control script
ros2 run mycobot280_pi move_mycobot.py
```

## 2.2 With Gripper
```
# Start Gazebo Simulation with Gripper
ros2 launch mycobot280_pi mycobot_gripper.launch.py
```
You should see the gripper in Gazebo like this:
![alt text](demo/gripper_gazebo.png)

Run gripper control script:
交互模式（默认）——用于 Gazebo 坐标校准
```
ros2 run mycobot280_pi move_mycobot_gripper.py
```
![alt text](demo/gripper_interactive.png)
自动任务模式
```
ros2 run mycobot280_pi move_mycobot_gripper.py --auto
```

Killall Gazebo Processes for Clean Exit
```
killall -9 ruby gz
```


target_pick ──►  IDLE → MOVING_TO_PICK → DESCENDING_PICK → GRIPPING
                          → GRIP_CHECK → LIFTING → RETURNING_HOME → HOLDING
                                                                      │
                  (rover moves freely, arm locked at home with block) │
                                                                      ▼
target_place ──►  HOLDING → MOVING_TO_PLACE → RELEASING → RETURNING_HOME → IDLE


# 3 Hardware Controller (Real Arm)

## 3.1 Architecture Overview

`mycobot_controller` node runs under the `/arm` namespace and controls the MyCobot 280 Pi + adaptive gripper via `pymycobot`. It implements a **split pick/place** workflow:

- **PICK phase**: receive 3D pick coordinates → grab block → verify grip → return to home → enter **HOLDING** state
- **HOLDING**: arm locked at home with block held; rover can move freely to find the target bin
- **PLACE phase**: receive 3D bin coordinates → move above bin → release (drop) → return to home → back to **IDLE**

### ROS 2 Topics (under `/arm` namespace)

| Topic | Type | Direction | Description |
|---|---|---|---|
| `target_pick` | `geometry_msgs/Point` | Subscribe | 3D pick coordinates (triggers pick phase, only accepted in IDLE) |
| `target_place` | `geometry_msgs/Point` | Subscribe | 3D bin coordinates (triggers place phase, only accepted in HOLDING) |
| `status` | `std_msgs/String` | Publish | Current state: `idle`, `holding`, `moving_to_pick`, `releasing`, etc. |
| `gripper_status` | `std_msgs/String` | Publish | Grip feedback: `object_held`, `no_object`, `released`, `object_dropped` |
| `joint_states` | `sensor_msgs/JointState` | Publish | Real-time joint angles at 10 Hz |

### Launch Parameters

| Parameter | Default | Description |
|---|---|---|
| `safe_z` | `220.0` | Safe travel height in mm |
| `move_speed` | `50` | Arm movement speed (1–100) |
| `gripper_speed` | `80` | Gripper speed (1–100) |
| `rviz` | `false` | Launch RViz2 for visualisation |

### NUC Integration Pattern

```
NUC                                           Pi (arm)
 │                                              │
 │── pub /arm/target_pick {x,y,z} ───────────► │  PICK phase starts
 │                                              │
 │◄── sub /arm/status == "holding" ────────────│  Block secured, arm at home
 │                                              │
 │  (rover drives to find matching bin)         │  (arm locked, waiting)
 │                                              │
 │── pub /arm/target_place {x,y,z} ──────────► │  PLACE phase starts
 │                                              │
 │◄── sub /arm/status == "idle" ───────────────│  Done, ready for next block
```

## 3.2 Build

```bash
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
source install/setup.bash
```

## 3.3 Run on Pi (Real Hardware)

### Deploy to Pi
```bash
# From NUC, copy the source to Pi
scp -r ~/mycobot_ws/src/my_cobot_control elephant@10.3.14.59:~/mycobot_ws/src/

# SSH into Pi
ssh elephant@10.3.14.59
# password: trunk

# Build on Pi
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
source install/setup.bash
```

### Launch the arm controller
```bash
# On Pi — start the controller (all nodes under /arm namespace)
ros2 launch my_cobot_control arm_controller.launch.py

# With custom parameters
ros2 launch my_cobot_control arm_controller.launch.py safe_z:=250.0 move_speed:=40

# With RViz (requires display)
ros2 launch my_cobot_control arm_controller.launch.py rviz:=true
```

### Or run the node directly (without launch file)
```bash
ros2 run my_cobot_control mycobot_controller --ros-args -r __ns:=/arm
```

## 3.4 Run on NUC (Mock Mode / Development)

When no hardware is connected (no `/dev/ttyAMA0`), the controller automatically enters **MOCK mode** — all hardware calls are simulated with print outputs.

```bash
# On NUC (dev machine)
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
source install/setup.bash
ros2 launch my_cobot_control arm_controller.launch.py
```

## 3.5 Test Commands

All test commands below should be run from a **separate terminal** on the NUC (or any machine on the same ROS_DOMAIN_ID).

### Monitor arm status (run first, keep open)
```bash
# Watch real-time state changes
ros2 topic echo /arm/status

# Watch gripper feedback
ros2 topic echo /arm/gripper_status
```

### Test 1: Full pick-and-place cycle
```bash
# Step 1: Send pick coordinates (block location)
ros2 topic pub --once /arm/target_pick geometry_msgs/Point "{x: 162.5, y: -134.8, z: 87.6}"

# Wait for status == "holding" (arm has block, at home position)
# The rover would drive to the bin during this time

# Step 2: Send place coordinates (bin location)
ros2 topic pub --once /arm/target_place geometry_msgs/Point "{x: 23.8, y: -245.6, z: 102.0}"

# Wait for status == "idle" (cycle complete)
```

### Test 2: Pick only (verify HOLDING state)
```bash
# Send pick command
ros2 topic pub --once /arm/target_pick geometry_msgs/Point "{x: 162.5, y: -134.8, z: 87.6}"

# Verify arm enters HOLDING state
ros2 topic echo /arm/status --once
# Expected output: data: "holding"

# Verify grip feedback
ros2 topic echo /arm/gripper_status --once
# Expected output: data: "object_held"
```

### Test 3: Rejected commands (state guard)
```bash
# Try to place when not holding — should be rejected
ros2 topic pub --once /arm/target_place geometry_msgs/Point "{x: 23.8, y: -245.6, z: 102.0}"
# Log: "Ignoring target_place (state=IDLE, need HOLDING)"

# Try to pick when already picking — should be rejected
ros2 topic pub --once /arm/target_pick geometry_msgs/Point "{x: 162.5, y: -134.8, z: 87.6}"
# (send twice quickly, second one rejected)
```

### Test 4: Out-of-range coordinate validation
```bash
# Send coordinates outside workspace limits
ros2 topic pub --once /arm/target_pick geometry_msgs/Point "{x: 999.0, y: 0.0, z: 100.0}"
# Log: "Coordinate x=999.0 out of range [-281.45, 281.45]"
```

### Test 5: Monitor joint states
```bash
# View real-time joint positions (10 Hz)
ros2 topic echo /arm/joint_states

# Check topic frequency
ros2 topic hz /arm/joint_states
```

### Test 6: List all arm topics
```bash
ros2 topic list | grep arm
# Expected:
#   /arm/gripper_status
#   /arm/joint_states
#   /arm/status
#   /arm/target_pick
#   /arm/target_place
```

## 3.6 Standalone Calibration Test (No ROS 2)

A standalone script is available for testing pick-and-place with calibrated coordinates directly on the Pi, **without ROS 2**:

```bash
# SSH into Pi
ssh elephant@10.3.14.59

# Run directly with Python (requires pymycobot)
cd ~/mycobot_ws/src/my_cobot_control/my_cobot_control
python3 test_calibration_pick_place.py
```

This script uses hardcoded calibration data (home/pickup/place) and runs a full cycle for hardware validation.

## 3.7 Calibration

Use the calibration tool to record new home/pickup/place positions by dragging the arm manually:

```bash
# On Pi
cd ~/mycobot_ws/src/my_cobot_control/my_cobot_control
python3 calibration_tool.py
```

Calibration files are saved to `my_cobot_control/calibration_data/` and automatically loaded by the controller (latest file used). You can also specify a calibration file explicitly via launch parameter:

```bash
ros2 run my_cobot_control mycobot_controller --ros-args \
  -r __ns:=/arm \
  -p calibration_file:=/path/to/calibration_2026-02-25_18-26-52.json
```

## 3.8 Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| `No hardware -- running in MOCK mode` | No `/dev/ttyAMA0` found | Run on Pi, or use mock mode for dev |
| `Ignoring target_place (need HOLDING)` | Place sent before pick completed | Wait for `status == "holding"` first |
| `Grip failed -- aborting pick` | Object not detected by gripper | Check gripper torque, object size, pick height |
| `Object lost during lift` | Grip too weak or object slipped | Increase `gripper_torque` parameter (up to 980) |
| `Coordinate x=... out of range` | Target outside workspace | Check coordinate frame, values must be in mm |
| Build error: `shebang too long` | venv path hardcoded on Pi | `setup.py` auto-detects venv; rebuild |

