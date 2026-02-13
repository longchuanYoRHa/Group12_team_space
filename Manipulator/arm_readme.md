# Manipulator Control / MyCobot 280 Control Package

**This package is still in development.**
ROS2 control package used for MyCobot 280pi Manipulator. Supports RViz visualization and real hardware control.

## Features

- 🤖 Supports MyCobot 280 Pi Manipulator (with Adaptive Gripper)
- 📊 RViz visualization simulation
- 🎮 Pick and Place demonstration
- 🔄 Mock mode local testing (no hardware required)
- 📡 ROS 2 distributed control (supports WiFi communication)
- 🐍 Compatible with pymycobot API

## System Requirements

- **Operating System**: Ubuntu 22.04 / 24.04
- **ROS 2**: Jazzy (or Humble)
- **Python**: 3.12+
- **Dependencies**:
  - `pymycobot` (hardware control)
  - `mycobot_ros2` (official URDF model and ROS 2 integration)

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
# The official repository only supports for Humble, but you can try to use it in Jazzy as well.
git clone -b humble https://github.com/elephantrobotics/mycobot_ros2.git
```

### 3. Build Workspace and Virtual Environment

```bash 
cd ~/mycobot_ws
python3 -m venv venv_mycobot
source venv_mycobot/bin/activate 

# Install PyMyCobot API (for hardware control)
pip install pymycobot
# If you want to install from source, you can clone the repository and install it:
https://github.com/elephantrobotics/pymycobot.git
```

### 4. Build ROS 2 Packages

```bash
cd ~/mycobot_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1. Run RViz Simulation

```bash
source ~/mycobot_ws/install/setup.bash
ros2 launch my_cobot_control pick_and_place_demo.launch.py
```
This will:
1. Launch RViz to display the MyCobot 280 model (with Adaptive Gripper)
2. Loop presentation

## 2. Hardware Control with RViz Feedback
Terminal 1: Start Rviz Node (connect to hardware)
```bash
ros2 launch mycobot_280pi test.launch.py
```
Terminal 2: Start Pick and Place Demo node
```bash
ros2 run my_cobot_control pick_and_place_with_feedback
```