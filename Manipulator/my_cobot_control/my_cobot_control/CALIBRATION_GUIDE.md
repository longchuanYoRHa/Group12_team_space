# MyCobot280Pi Calibration Tool Guide

## Table of Contents
1. [How the Official Drag-Teaching Code Works](#how-the-official-drag-teaching-code-works)
2. [How to Read Coordinates](#how-to-read-coordinates)
3. [Using the Calibration Tool](#using-the-calibration-tool)
4. [Quick Calibration Workflow](#quick-calibration-workflow)
5. [FAQ](#faq)

---

## How the Official Drag-Teaching Code Works

### 1. Servo Unlock Mechanism

**Key Function**: `release_all_servos()`

```python
# In drag_trial_teaching.py
def record(self):
    # Unlock all servos, disconnect torque control
    self.mc.release_all_servos()
```

**Working Principle**:
- Calling `release_all_servos()` disconnects motor torque from all joints
- The robotic arm enters "free mode" and can be manually dragged
- Suitable for manual teaching and position calibration

**Reverse Operation**: `focus_all_servos()` or `power_on()` locks servos

---

### 2. Position Recording Mechanism

**Key Functions**: `get_encoders()`, `get_angles()`, `get_coords()`

```python
def _record():
    while self.recording:
        # Get encoder values (raw values)
        encoders = self.mc.get_encoders()
        # Get servo speeds
        speeds = self.mc.get_servo_speeds()
        # Save record
        record = [encoders, speeds, interval_time]
        self.record_list.append(record)
```

**Data Types**:
- **Encoders**: Raw encoder values (integers)
- **Angles**: Joint angles in degrees (°)
- **Coords**: Cartesian coordinates [x, y, z, rx, ry, rz]

---

## How to Read Coordinates

### Method 1: Direct Reading Using Python API

```python
from pymycobot.mycobot280 import MyCobot280

mc = MyCobot280('/dev/ttyAMA0', 1000000)
mc.power_on()

# Get joint angles (degrees)
angles = mc.get_angles()
print(f"Joint angles: {angles}")
# Output example: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Get Cartesian coordinates (mm and degrees)
coords = mc.get_coords()
print(f"Cartesian coordinates: {coords}")
# Output example: [200.5, 0.0, 250.3, 0.0, 0.0, 0.0]
# Format: [X(mm), Y(mm), Z(mm), RX(°), RY(°), RZ(°)]
```

### Method 2: Using Calibration Tool

```bash
# Activate virtual environment
source /home/student42/mycobot_ws/venv_mycobot/bin/activate

# Run calibration tool
cd /home/student42/mycobot_ws/src/my_cobot_control/my_cobot_control
python3 calibration_tool.py
```

**Operation Steps**:
1. Press `r` to unlock servos
2. Manually drag the robotic arm to target position
3. Press `l` to lock servos
4. Press `d` to display current position coordinates
5. Press `s` to save as calibration point

---

## Using the Calibration Tool

### Complete Calibration Tool (`calibration_tool.py`)

**Features**:
- ✅ Interactive operation interface
- ✅ Real-time coordinate display
- ✅ Save/load calibration points
- ✅ Move to saved points
- ✅ JSON format storage

**Usage Steps**:

```bash
# 1. Enter directory
cd /home/student42/mycobot_ws/src/my_cobot_control/my_cobot_control

# 2. Run calibration tool
python3 calibration_tool.py

# 3. Follow prompts to select serial port and baud rate
# Default: /dev/ttyAMA0, 1000000

# 4. Enter interactive interface
```

**Command List**:
| Command | Function |
|---------|----------|
| `r` | Unlock servos (allow dragging) |
| `l` | Lock servos |
| `d` | Display current position coordinates |
| `s` | Save current position as calibration point |
| `v` | View all saved calibration points |
| `m` | Move to specified calibration point |
| `w` | Save calibration data to file |
| `o` | Load calibration data from file |
| `x` | Delete calibration point |
| `q` | Exit program |

**Example Operations**:
```
Enter command: r          # Unlock servos
✓ Servos unlocked, you can now manually drag the robotic arm

[Manually drag to target position]

Enter command: l          # Lock servos
✓ Servos locked

Enter command: d          # Display current coordinates
Current robotic arm position:
Joint angles (degrees):    [10.5, -20.3, 45.6, 0.0, 15.2, 0.0]
Cartesian coordinates (mm):  [180.5, 50.3, 220.8, 0.0, 15.2, 0.0]

Enter command: s          # Save calibration point
Enter calibration point name: pickup_point
Enter description: Block pickup position
✓ Calibration point saved: pickup_point

Enter command: w          # Save to file
✓ Calibration data saved to: calibration_data/calibration_2026-02-26_14-30-15.json
```

---

## Quick Calibration Workflow

### Quick Calibration Script (`quick_calibration.py`)

Suitable for quickly calibrating a few fixed key positions.

```bash
python3 quick_calibration.py
```

**Automatically guides calibration of the following positions**:
1. **Home Position** - Initial/safe position
2. **Pickup Preparation Position** - Above the block
3. **Pickup Position** - Gripper closed in contact with block
4. **Place Preparation Position** - Above target point
5. **Place Position** - Block release position

**Output File**: `quick_calibration.json`

---

## Coordinate System Description

### Joint Space
- 6 joint angles: [J1, J2, J3, J4, J5, J6]
- Unit: degrees (°)
- Range: 
  - J1: -165° ~ +165°
  - J2: -165° ~ +165°
  - J3: -165° ~ +165°
  - J4: -165° ~ +165°
  - J5: -165° ~ +165°
  - J6: -175° ~ +175°

### Cartesian Space
- Position and orientation: [X, Y, Z, RX, RY, RZ]
- X, Y, Z: millimeters (mm) - End effector position
- RX, RY, RZ: degrees (°) - End effector orientation (Euler angles)

**Base Coordinate System**:
```
         Z ↑
           |
           |
           O----→ X
          /
         /
        Y
```

---

## Example Code Using Calibration Data

### Load and Use Calibration Points

```python
import json
from pymycobot.mycobot280 import MyCobot280

# Connect to robotic arm
mc = MyCobot280('/dev/ttyAMA0', 1000000)
mc.power_on()

# Load calibration data
with open('calibration_data/calibration_2026-02-26_14-30-15.json', 'r') as f:
    calib_points = json.load(f)

# Move to Home position
home_angles = calib_points['home']['angles']
mc.send_angles(home_angles, 50)  # Speed 50
time.sleep(3)

# Move to pickup position
pickup_angles = calib_points['pickup_point']['angles']
mc.send_angles(pickup_angles, 30)  # Speed 30, slower and more precise
time.sleep(3)

# Move using Cartesian coordinates
pickup_coords = calib_points['pickup_point']['coords']
mc.send_coords(pickup_coords, 30)
```

---

## FAQ

### Q1: Why is the robotic arm too loose/stiff when dragging?
**A**: 
- Too loose: `release_all_servos()` has been called, normal behavior
- Too stiff: Servos not unlocked, need to call `release_all_servos()`

### Q2: How to ensure calibration accuracy?
**A**: 
1. Save coordinates after locking servos
2. Move to the same position multiple times to verify repeatability
3. Use slower speed (20-50) for movement

### Q3: Why do coordinate values jump?
**A**: 
- Communication interference: Check serial port connection
- Servos not locked: Lock before reading
- Add delay: Wait 0.5-1 second before reading

### Q4: Cannot move to saved calibration point?
**A**: 
- Check if position is outside workspace
- Check if joint angles are within valid range
- Use `send_angles()` instead of `send_coords()`

### Q5: Serial port permission issues?
**A**: 
```bash
# Check what is using the serial port
sudo fuser /dev/ttyAMA0

# If a process is using it, stop it first
sudo kill <PID>

# Or stop ROS related services
systemctl --user stop ros.target
# Or
systemctl --user stop ros-nodes.service

# Add user to dialout group
sudo usermod -aG dialout $USER
# Re-login to take effect

# Or temporarily modify permissions
sudo chmod 666 /dev/ttyAMA0
```

---

## Recommended Workflow

### Block Pickup Project Calibration Workflow

1. **Define Key Positions**:
   - Home (safe position)
   - Block detection position (camera capture)
   - Pickup preparation position (50mm above block)
   - Pickup position (in contact with block)
   - Place preparation position (50mm above target)
   - Place position (release block)
   - Multiple placement points (if there are multiple target positions)

2. **Use quick calibration script to obtain initial positions**

3. **Fine-tune using complete calibration tool**:
   - Add intermediate points to avoid collisions
   - Add safe return positions

4. **Test and Verify**:
   - Test movement between all points
   - Check if trajectory is safe
   - Verify pickup reliability

5. **Save Multiple Versions**:
   - Calibration for different block sizes
   - Calibration for different placement positions

---

## Advanced Techniques

### 1. Automatically Generate Grid Calibration Points

```python
# Generate grid points on a plane for placement
def generate_grid_points(start_x, start_y, z, rows, cols, spacing):
    points = {}
    for r in range(rows):
        for c in range(cols):
            x = start_x + c * spacing
            y = start_y + r * spacing
            point_name = f"place_r{r}_c{c}"
            points[point_name] = {
                "coords": [x, y, z, 0.0, 0.0, 0.0],
                "description": f"Placement point row{r} col{c}"
            }
    return points
```

### 2. Safety Zone Check

```python
def is_safe_position(coords, safe_zone):
    """Check if coordinates are within safety zone"""
    x, y, z = coords[:3]
    return (safe_zone['x_min'] <= x <= safe_zone['x_max'] and
            safe_zone['y_min'] <= y <= safe_zone['y_max'] and
            safe_zone['z_min'] <= z <= safe_zone['z_max'])
```

---

## File Description

| File | Purpose |
|------|---------|
| `calibration_tool.py` | Complete interactive calibration tool |
| `quick_calibration.py` | Quick calibration script |
| `calibration_data/*.json` | Saved calibration data |
| `drag_trial_teaching.py` | Official drag-teaching code |

---

## References

- MyCobot280 Official Documentation: https://docs.elephantrobotics.com/
- Python API Documentation: https://github.com/elephantrobotics/pymycobot

---

**Created**: 2026-02-26  
**Version**: 1.0
CALIBRATION_GUIDE.md