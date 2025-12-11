# Software Control Development Notes

This document tracks the development progress and setup procedures for the Leo Rover software control system. It is maintained to allow team members to understand current work and take over if needed.

## Chassis Basics

### General Problems

Some of the general problems for the Leo Rover development:

- [Known Issues Documentation](https://docs.fictionlab.pl/leo-rover/1.8/documentation/known-issues)
- **Note**: Watch out for the version.

### Support Documents

- [FAQ Documentation](https://docs.fictionlab.pl/leo-rover/1.8/documentation/faq)

### Robot Description Package Installation

To install the robot description package:

```bash
sudo apt update
sudo apt install ros-jazzy-leo
sudo apt install ros-jazzy-leo-description ros-jazzy-leo-msgs ros-jazzy-leo-teleop
```

### Starting Robot Description (without physical robot)

To start up the robot description without physical robot (check availability):

```bash
ros2 run robot_state_publisher robot_state_publisher \
--ros-args -p robot_description:="$(xacro $(ros2 pkg prefix \
leo_description)/share/leo_description/urdf/leo.urdf.xacro)"
```

### Viewing Robot Frame

To view robot frame:

```bash
ros2 run tf2_tools view_frames
```

## Gazebo Simulation

### Gazebo Installation

To install Gazebo:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

### World List Problem

**Issue**: Waiting for world list problem

- Reference: [GitHub Issue #2285](https://github.com/gazebosim/gz-sim/issues/2285)

### Firewall Rules

To fix network issues with Gazebo:

```bash
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

### Gazebo GZ Bridge

- [ros_gz_bridge GitHub Repository](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
- [Gazebo ROS Launch Tutorial](https://classic.gazebosim.org/tutorials?tut=ros_roslaunch)

## Navigation Solutions

### Nav2 Installation

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### SLAM Demo Execution

To run SLAM demo:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True nav:=True headless:=False use_sim_time:=True
```

### Nav2 Troubleshooting

**Problem**: Nav2 stuck at waiting for route server/get state and Message Filter dropping message.

**Solution**:

```bash
sudo apt-get install ros-jazzy-nav2-route
```

## Lidar Setup

### Lidar Problem Reference

- [Slamtec RPLidar A2 Documentation](https://docs.fictionlab.pl/integrations/lidars/slamtec-rplidar-a2)

### Lidar Port Fix (udev Rule)

To create a udev rule for the lidar:

```bash
sudo bash -c 'cat >/etc/udev/rules.d/lidar.rules' <<'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="lidar"
EOF
```

Apply the udev rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Replug the lidar, then confirm:

```bash
ls -l /dev/lidar
```

## Static Lidar Mapping (without odometry)

This setup requires multiple terminals running simultaneously:

### Terminal 1: SLAM Toolbox

```bash
ros2 run slam_toolbox async_slam_toolbox_node \
--ros-args \
-p use_sim_time:=false \
-p base_frame:=base_link \
-p map_frame:=map \
-p odom_frame:=odom \
-p provide_odom_frame:=true \
-p use_odom:=false \
-p scan_topic:=/scan \
--log-level slam_toolbox:=debug
```

### Terminal 2: RPLidar Launch

```bash
ros2 launch rplidar_ros rplidar_a2m12_launch.py
```

### Terminal 3: Static Transform Publishers

```bash
ros2 run tf2_ros static_transform_publisher 0.20 0.0 0.15 0 0 0 base_link laser
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```

### Terminal 4: SLAM Toolbox Lifecycle

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

## NAV2 + SLAM + Explore Reference

- [Husarion ROS2 Tutorial - Exploration](https://husarion.com/tutorials/ros2-tutorials/10-exploration/)
