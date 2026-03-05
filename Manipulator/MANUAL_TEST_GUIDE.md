# MyCobot Manual Testing Guide

When the NUC is not connected, you can use the following two methods to manually send pickup and place commands for testing.

## Method 1: Using Python Test Script (Recommended)

The `manual_test.py` script has been created with your calibrated coordinates.

### Usage (Testing on NUC)

```bash
# Activate virtual environment
source ~/mycobot_ws/venv_mycobot/bin/activate
# Source the workspace
source ~/mycobot_ws/install/setup.bash

# Send pickup command (coordinates: x=202.2, y=-129.3, z=237.9)
python3 src/my_cobot_control/my_cobot_control/manual_test.py pick

# Send place command (coordinates: x=66.7, y=-218.2, z=123.7)
python3 src/my_cobot_control/my_cobot_control/manual_test.py place

# Send custom coordinates
python3 src/my_cobot_control/my_cobot_control/manual_test.py custom 150.0 -100.0 200.0 pick
python3 src/my_cobot_control/my_cobot_control/manual_test.py custom 50.0 -200.0 150.0 place
```

## Method 2: Using ROS2 Command Line Directly

```bash
# Send pickup command (using calibrated pickup coordinates)
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 202.2, y: -129.3, z: 237.9}"
---
data: releasing
---

# Send place command (using calibrated place coordinates)
ros2 topic pub --once /arm/target_place geometry_msgs/msg/Point "{x: 66.7, y: -218.2, z: 123.7}"

# Custom coordinates example
ros2 topic pub --once /arm/target_pick geometry_msgs/msg/Point "{x: 150.0, y: -100.0, z: 200.0}"
```

## Complete Testing Workflow

### 1. Launch Controller

In the first terminal:
```bash
cd ~/mycobot_ws
source install/setup.bash
source venv_mycobot/bin/activate
ros2 launch my_cobot_control arm_control.launch.py
```

### 2. Monitor Status

In the second terminal (optional):
```bash
source ~/mycobot_ws/install/setup.bash

# Monitor arm status
ros2 topic echo /arm/status

# Monitor gripper status
ros2 topic echo /arm/gripper_status

# Monitor joint states
ros2 topic echo /arm/joint_states
```

### 3. Send Commands

In the third terminal:
```bash
cd ~/mycobot_ws
source install/setup.bash
source venv_mycobot/bin/activate

# Send pickup command
python3 src/my_cobot_control/my_cobot_control/manual_test.py pick

# Wait for the arm to complete the pick action and enter HOLDING state

# Send place command
python3 src/my_cobot_control/my_cobot_control/manual_test.py place
```

## Predefined Test Coordinates

Based on your calibration data, the script has the following preset coordinates:

**Pickup Point:**
- x: 202.2 mm
- y: -129.3 mm
- z: 237.9 mm

**Place Point:**
- x: 66.7 mm
- y: -218.2 mm
- z: 123.7 mm

## State Flow

1. **IDLE** → Send `pick` command
2. **MOVING_TO_PICK** → Move above pickup point
3. **DESCENDING_PICK** → Descend to pickup height
4. **GRIPPING** → Grip object
5. **GRIP_CHECK** → Verify grip
6. **LIFTING** → Lift object
7. **RETURNING_HOME** → Return to home position
8. **HOLDING** → Holding object, waiting for place command
9. Send `place` command → **MOVING_TO_PLACE**
10. **RELEASING** → Release object
11. **RETURNING_HOME** → Return to home
12. **IDLE** → Complete

## Important Notes

- Before sending the `place` command, you must first successfully execute the `pick` command (state is HOLDING)
- If using MOCK mode (no actual hardware), all actions will be simulated
- Coordinate range limits:
  - x: -281.45 to 281.45 mm
  - y: -281.45 to 281.45 mm
  - z: -70.0 to 450.0 mm
