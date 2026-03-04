# Manipulator Control / MyCobot 280 Pi Control Package

**This package is still in development.**
ROS2 control package used for MyCobot 280pi Manipulator. Supports RViz visualization, Gazebo simulation and real hardware control.

## Features

- 🤖 Supports MyCobot 280 Pi Manipulator (with Adaptive Gripper)
- 📊 RViz visualization simulation
- 🌍 Gazebo physics simulation (with and without gripper)
- 🎮 Pick and Place demonstration
- 🔄 Mock mode local testing (no hardware required)
- 📡 ROS 2 distributed control (supports WiFi communication)
- 🐍 Compatible with pymycobot API

## Package Structure

```
my_cobot_control    -- Manipulator Control Package and Calibration Tools
mycobot_ros2        -- Official ROS 2 Package for myCobot Manipulator (URDF)
mycobot280_pi       -- Gazebo Simulation Package (mltejas88/Project_Mycobot_280pi_simulation)
```

## System Requirements

- **Operating System**: Ubuntu 22.04 / 24.04
- **ROS 2**: Jazzy (or Humble)
- **Python**: 3.12+
- **Dependencies**:
  - `pymycobot` (hardware control & inverse kinematics)
  - `mycobot_ros2` (official URDF model and ROS 2 integration)
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
git clone https://github.com/YOUR_USERNAME/my_cobot_control.git

# Clone the official mycobot_ros2 package (provides URDF model)
# The official repository only supports Humble, but you can try to use it in Jazzy as well.
git clone -b humble https://github.com/elephantrobotics/mycobot_ros2.git

# The Gazebo simulation package (already included in this repo)
# Source: https://github.com/mltejas88/Project_Mycobot_280pi_simulation
```

### 3. Build Workspace and Virtual Environment

```bash
cd ~/mycobot_ws
python3 -m venv venv_mycobot
source venv_mycobot/bin/activate

# Install PyMyCobot API (for hardware control & IK)
pip install pymycobot

# If you want to install from source:
# git clone https://github.com/elephantrobotics/pymycobot.git
# pip install ./pymycobot
```

> **Note:** The virtual environment is required to avoid conflicts between `pymycobot` and ROS 2 Python dependencies. Always activate it before running scripts that use `pymycobot`.

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


---

## Notes on Inverse Kinematics in Gazebo

The `pymycobot` API (`from pymycobot import MyCobot280`) provides built-in inverse kinematics. You can use it to convert Cartesian coordinates `(x, y, z)` to joint angles for Gazebo simulation **without** connecting to real hardware. This is the recommended approach for coordinate-to-angle conversion in simulation.

The workflow is:
1. Receive target `(x, y, z)` coordinates via ROS 2 topic (from depth camera + object detection)
2. Use `pymycobot` IK to compute joint angles
3. Send joint angles to Gazebo controllers via ROS 2 topics
4. Apply same pipeline to real hardware for deployment

