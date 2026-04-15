# MyCobot 280 Pi Manipulator Control Package

**This package is still in development.**
ROS2 control package used for Elephant Robotics MyCobot 280pi Manipulator. Supports RViz visualization, Gazebo simulation and real hardware control.

## Features

- Supports MyCobot 280 Pi Manipulator (with Adaptive Gripper)
- RViz visualization simulation
- Gazebo physics simulation (with and without gripper)
- Pick and Place demonstration
- Mock mode local testing (no hardware required)
- ROS 2 distributed control (supports WiFi communication)
- Compatible with pymycobot API

## File Structure

```
scripts   -- Inculuding some calibration tools for arm and gripper 
my_cobot_control    -- Manipulator Control Package (URDF, TF2, Controllers, Pick-and-Place Logic)
mycobot280_pi       -- Gazebo Simulation Package (from mltejas88/Project_Mycobot_280pi_simulation)
```

## System Requirements

- **Operating System**: Ubuntu 22.04 / 24.04
- **ROS 2**: Jazzy
- **Python**: 3.12+
- **Dependencies**:
  - `pymycobot` (hardware control & inverse kinematics from Elephant Robotics)
  - `mycobot_ros2` (official URDF model and ROS 2 integration from Elephant Robotics)
  - `gazebo` / `gz-sim` (Gazebo simulation)
  - `ros-$ROS_DISTRO-gazebo-ros-pkgs` (Gazebo ROS bridge)
  - `ros-$ROS_DISTRO-joint-state-publisher`
  - `ros-$ROS_DISTRO-robot-state-publisher`
  - `ros-$ROS_DISTRO-controller-manager`
  - `ros-$ROS_DISTRO-ros2-control`
  - `ros-$ROS_DISTRO-ros2-controllers`

## Installation Steps

### 1. Create Workspace

```bash
mkdir -p ~/mycobot_ws/src
cd ~/mycobot_ws/src
```

### 2. Clone Repository

```bash
# Clone this package
git clone https://github.com/DissimilarTriangle/MyCobot280Pi_ROS2_Package

# Clone the official mycobot_ros2 package (provides URDF model)
# The official repository only supports Humble, but you can try to use it in Jazzy as well.
git clone -b humble https://github.com/elephantrobotics/mycobot_ros2.git

# The Gazebo simulation package (already included in this repo)
# Source: https://github.com/mltejas88/Project_Mycobot_280pi_simulation
```

### 3. Build Workspace

```bash
# (Optional) Create a Python virtual environment for pymycobot
cd ~/mycobot_ws
python3 -m venv venv_mycobot
source venv_mycobot/bin/activate

# Install PyMyCobot API (for hardware control & IK)
pip install pymycobot

# If you want to install from source:
# git clone https://github.com/elephantrobotics/pymycobot.git
# pip install ./pymycobot
```

> **Note:** Sometimes the virtual environment is required to avoid conflicts between `pymycobot` and ROS 2 Python dependencies.

### 4. Build ROS 2 Packages

```bash
cd ~/mycobot_ws
colcon build --packages-select mycobot280_pi mycobot_description
# Or build all packages
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### 1. Run RViz Simulation

```bash
source ~/mycobot_ws/install/setup.bash
ros2 launch my_cobot_control pick_and_place_demo.launch.py
```

This will:
1. Launch RViz to display the MyCobot 280 model (with Adaptive Gripper)
2. Loop pick and place presentation

This part is abandoned for now, as we are focusing on Gazebo simulation and real hardware control. The RViz launch file is still available for testing the TF2 setup and joint state publishers without Gazebo or hardware.

---

### 2. Run Gazebo Simulation

#### 2.1 Without Gripper

```bash
# Terminal 1 — Start Gazebo simulation
ros2 launch mycobot280_pi mycobot.launch.py

# Terminal 2 — Run control script
ros2 run mycobot280_pi move_mycobot.py
```

#### 2.2 With Gripper

```bash
# Start Gazebo simulation with gripper
ros2 launch mycobot280_pi mycobot_gripper.launch.py
```

You should see the gripper in Gazebo like this:

![Gripper in Gazebo](demo/gripper_gazebo.png)

Run the gripper control script:

**Interactive mode (default)** — used for Gazebo coordinate calibration:
```bash
ros2 run mycobot280_pi move_mycobot_gripper.py
```

![Interactive Mode](demo/gripper_interactive.png)

**Automatic task mode:**
```bash
ros2 run mycobot280_pi move_mycobot_gripper.py --auto
```

**Kill all Gazebo processes (clean exit):**
```bash
killall -9 ruby gz
```

---

# 3 Hardware Controller Test (Real Arm)

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
| `status` | `std_msgs/String` | Publish | Simplified state: `idle`, `holding`, `busy`, `error` |
| `gripper_status` | `std_msgs/String` | Publish | Simplified grip feedback: `object_held`, `no_object`, `unknown` |
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

```
TF2 Frame Hierarchy:
base_link
  └── arm_base_link
        └── joint6_flange
              └── gripper_tip
```

## 3.2 Run on Pi (Real Hardware)

### Deploy to Pi
```bash
# From NUC, copy the source to Pi
scp -r ~/Group12_team_space/Manipulator/src/my_cobot_control elephant@10.0.1.3:~/ros2_ws/src/
scp -r ~/mycobot_ws/src/my_cobot_control elephant@10.0.1.3:~/ros2_ws/src/

# On NUC — SSH into Pi
ssh elephant@10.0.1.3

# sync time
# sudo date -s "$(ssh leo-rover-12@10.0.1.4 'date -u +%Y-%m-%d\ %H:%M:%S.%N')"
# sudo date -s "$(ssh student42@10.0.1.5 'date -u +%Y-%m-%d\ %H:%M:%S.%N')"

# TS=$(ssh leo-rover-12@10.0.1.4 "date -u +%Y-%m-%d\ %H:%M:%S.%N") || { echo "SSH failed"; exit 1; }
# [ -n "$TS" ] || { echo "Empty time string"; exit 1; }
# sudo date -u -s "$TS"

# Check Sync Time
chronyc sources -v

# Build on Pi
cd ~/ros2_ws
colcon build --packages-select my_cobot_control
source install/setup.bash
```

### Launch the arm controller
```bash
# Control node without TF2 (use raw coordinates in arm frame)
ros2 launch my_cobot_control arm_controller.launch.py

# On Pi — start the controller (all nodes under /arm namespace)
ros2 launch my_cobot_control mycobot_with_tf2.launch.py

# With custom parameters
ros2 launch my_cobot_control mycobot_with_tf2.launch.py use_mock:=false safe_z:=250.0 move_speed:=40
```

## 3.3 Run on Dev Machine (Mock Mode / Development)

When no hardware is connected (no `/dev/ttyAMA0`), the controller automatically enters **MOCK mode** — all hardware calls are simulated with print outputs.

```bash
# On dev machine
cd ~/mycobot_ws # Or
cd ~/Group12_team_space/Manipulator
colcon build --packages-select my_cobot_control
source install/setup.bash

# Control node without TF2 (use raw coordinates in arm frame)
ros2 launch my_cobot_control arm_controller.launch.py

# Default: real hardware + base_link->g_base + base_link->camera_link
ros2 launch my_cobot_control mycobot_with_tf2.launch.py

# Mock mode (no hardware, with GUI for joint angles)
ros2 launch my_cobot_control mycobot_with_tf2.launch.py use_mock:=true

# With RViz visualization (optional, requires TF2 setup)
ros2 launch my_cobot_control mycobot_with_rviz.launch.py
```

## 3.4 Test Commands

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
# Step 1: Send pick coordinates (example block location) without tf2 transform.
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 202.2, y: -129.3, z: 237.9}"

# Wait for status == "holding" (arm has block, at home position)
# The rover would drive to the bin during this time

# Step 2: Send place coordinates (bin location)
ros2 topic pub --once /arm/target_place geometry_msgs/msg/Point "{x: 66.7, y: -218.2, z: 123.7}"
# Wait for status == "idle" (cycle complete)

```

### Test 2: TF2 Transform Validation
```bash
# Check TF2 tree structure
cd ~/mycobot_ws  #or cd ~/ros2_ws
ros2 run tf2_tools view_frames
evince frames.pdf

# Run the TF2 transform test script (requires camera TF setup)
cd ~/ros2_ws/scripts
python3 scripts/test_tf2_transform.py

# Visualize in RViz (optional, requires rviz launch)
ros2 launch my_cobot_control mycobot_with_rviz.launch.py

# Send pick command in camera frame (will be transformed to arm frame) in meter
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: -0.006982066202908754, y: -0.009772047400474548, z: 0.18200001120567322}"

# Send place command in camera frame in meter
ros2 topic pub --once /arm/target_place geometry_msgs/msg/Point "{x: -0.03592269495129585, y: 0.029966816306114197, z: 0.1600000113248825}"
```


## 3.5 Calibration

Use the calibration tool to record new home/pickup/place positions by dragging the arm manually:

```bash
# On Pi
cd ~/ros2_ws/scripts
python3 calibration_tool.py
```

## 3.6 Time Synchronization Check
```bash
# sync time
sudo date -s "$(ssh leo-rover-12@10.0.1.4 'date -u +%Y-%m-%d\ %H:%M:%S.%N')"
sudo date -s "$(ssh student42@10.0.1.4 'date -u +%Y-%m-%d\ %H:%M:%S.%N')"
# check NTP status for timesync
sudo systemctl status ntp
# check hardware clock
timedatectl  
```
