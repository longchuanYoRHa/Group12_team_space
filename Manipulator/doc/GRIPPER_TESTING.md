# Gripper Threshold Testing Guide

## Overview

Determining the correct `grip_threshold` is crucial for reliable block detection. This guide explains how MyCobot's adaptive gripper works and how to find the optimal threshold value.

## How the Adaptive Gripper Works

Your MyCobot uses an **Adaptive Gripper** with the following behavior:

### When Closing WITHOUT an Object:
- ✅ Gripper jaws close completely
- ✅ `get_gripper_value()` returns **low value** (typically 0-3)
- ✅ This indicates: **NO object held**

### When Closing WITH an Object:
- ✅ Gripper jaws stop when blocked by object
- ✅ `get_gripper_value()` returns **higher value** (typically 10-100)
- ✅ This indicates: **Object is held**

### Detection Logic:
```python
gripper_value = mc.get_gripper_value()

if gripper_value > grip_threshold:
    # Object detected! ✓
    print("Block is held")
else:
    # No object / gripper fully closed ✗
    print("No block detected")
```

## Why Threshold Matters

**Too Low (e.g., threshold = 1)**:
- ❌ May have false positives
- ❌ Noise could trigger false detection

**Too High (e.g., threshold = 50)**:
- ❌ May miss objects
- ❌ Small/thin objects might not trigger detection

**Just Right (e.g., threshold = 5-10)**:
- ✅ Reliable detection
- ✅ Margin for variation
- ✅ Works with different block sizes

## Testing Tools Provided

### 1. `test_gripper_threshold.py` (Recommended)

**Comprehensive testing tool** that guides you through the calibration process.

**What it does:**
1. Tests gripper closing **without** object (3 samples)
2. Tests gripper closing **with** object (3 samples)
3. Optionally tests different sized objects
4. Analyzes data and recommends threshold
5. Provides interactive testing mode

**How to use:**
```bash

# Navigate to directory
cd ~/ros2_ws/src/my_cobot_control/my_cobot_control

# Run the test
python3 test_gripper_threshold.py
```

**Expected output:**
```
=========================================================
MyCobot Gripper Threshold Testing Tool
=========================================================

This tool will help you determine the optimal gripper threshold.
You will need:
  - Your robot arm powered and connected
  - One or more test blocks
  - About 5-10 minutes

Ready to begin? (y/n): y

Connecting to /dev/ttyAMA0 @ 1000000...
✓ Gripper torque set to 300
✓ Connection established

=========================================================
TEST 1: Empty Gripper (No Object)
=========================================================

Make sure there is NO object in the gripper.
Press ENTER when ready...

--- Sample 1/3 ---
  Opening gripper...
  Closing gripper (empty)...
  ✓ Gripper value (empty): 0

--- Sample 2/3 ---
  Opening gripper...
  Closing gripper (empty)...
  ✓ Gripper value (empty): 1

--- Sample 3/3 ---
  Opening gripper...
  Closing gripper (empty)...
  ✓ Gripper value (empty): 0

📊 Empty grip statistics:
   Average: 0.3
   Range:   0 - 1

=========================================================
TEST 2: Gripper with Object
=========================================================

Place a block in the gripper when prompted.

--- Sample 1/3 ---
  Opening gripper...
  Place block in gripper, then press ENTER...
  Closing gripper on object...
  ✓ Gripper value (with object): 25

[... continues ...]

=========================================================
ANALYSIS & RECOMMENDATIONS
=========================================================

📊 Summary:
   Empty grip:  avg=0.3, max=1
   With object: avg=27.7, min=25

💡 Recommended Thresholds:
   Conservative (safer):  3
   Midpoint:             13.0

✓ Threshold 3 should work well:
   - Above empty grip max (1)
   - Below object grip min (25)

📝 Configuration for mycobot_controller:
   grip_threshold: 3

   Add to your launch file or as ROS parameter:
   <param name="grip_threshold" value="3"/>
```

### 2. `monitor_gripper.py`

**Real-time monitoring tool** for quick visual feedback.

**What it does:**
- Continuously displays gripper value
- Visual bar graph
- Interactive open/close commands

**How to use:**
```bash
source /home/student42/mycobot_ws/venv_mycobot/bin/activate
cd /home/student42/mycobot_ws/src/my_cobot_control/my_cobot_control
python3 monitor_gripper.py
```

**Example output:**
```
Real-time Gripper Monitor

Commands (type and press ENTER):
  o  - Open gripper
  c  - Close gripper
  q  - Quit

Gripper values will be displayed continuously...

Gripper:  25 [█████████████████████████                         ]
```

## Step-by-Step Process

### Step 1: Prepare Hardware
```bash
# Ensure robot is powered
# Check serial port is available
ls -l /dev/ttyAMA0

# Activate virtual environment
source /home/student42/mycobot_ws/venv_mycobot/bin/activate
```

### Step 2: Run Threshold Test
```bash
cd /home/student42/mycobot_ws/src/my_cobot_control/my_cobot_control
python3 test_gripper_threshold.py
```

### Step 3: Follow Interactive Prompts
1. **Empty Test**: Let gripper close with nothing inside (3 times)
2. **Object Test**: Place block and let gripper close (3 times)
3. **Different Objects** (optional): Test various block sizes
4. **Review Results**: Check recommended threshold

### Step 4: Update Configuration

Edit your launch file or controller parameters:

**Option A: In launch file**
```xml
<param name="grip_threshold" value="3"/>
```

**Option B: Command line**
```bash
ros2 run my_cobot_control mycobot_controller --ros-args -p grip_threshold:=3
```

**Option C: In mycobot_controller.py**
```python
self.declare_parameter('grip_threshold', 3)  # Update default value
```

### Step 5: Test with Real Workflow
```bash
# Launch controller with new threshold
ros2 launch my_cobot_control mycobot_control.launch.py

# In another terminal, test pick operation
ros2 topic pub --once /arm/target_pick geometry_msgs/Point "{x: 150.0, y: 0.0, z: 50.0}"

# Watch status and gripper_status topics
ros2 topic echo /arm/status
ros2 topic echo /arm/gripper_status
```

## Troubleshooting

### Problem: Gripper values overlap (empty max ≈ object min)

**Symptoms:**
```
⚠ WARNING: Overlap detected!
   Empty max (15) >= Object min (12)
```

**Solutions:**
1. **Increase gripper torque**:
   ```python
   self.declare_parameter('gripper_torque', 400)  # Try higher value
   ```

2. **Use different test objects**: Try blocks with different textures/materials

3. **Check gripper calibration**: Mechanical issue may need hardware check

### Problem: False negatives (object held but not detected)

**Symptoms:**
- Gripper clearly holding object
- But `gripper_status` says "no_object"

**Solution:**
- **Lower threshold**: Current value too high
- Re-run test and use **conservative** recommended value

### Problem: False positives (no object but detected)

**Symptoms:**
- Gripper empty
- But `gripper_status` says "object_held"

**Solution:**
- **Raise threshold**: Current value too low
- Check for mechanical interference preventing full closure

### Problem: Inconsistent readings

**Symptoms:**
- Same scenario gives widely different values

**Solution:**
1. Check gripper torque setting
2. Ensure object is positioned consistently
3. Add delay before reading: `time.sleep(2.0)`
4. Take more samples for averaging

## Understanding the Numbers

### Typical Value Ranges:

| Scenario | Gripper Value | Interpretation |
|----------|---------------|----------------|
| Fully closed, no object | 0-3 | Normal, no grip |
| Small/thin object | 5-15 | Weak grip |
| Medium block | 15-40 | Good grip |
| Large/thick object | 40-80 | Strong grip |
| Mechanical jam | 80-100 | Error / obstruction |

### Recommended Thresholds:

| Block Type | Suggested Threshold |
|------------|---------------------|
| Small cubes (20mm) | 3-5 |
| Medium blocks (30-40mm) | 5-8 |
| Mixed sizes | 5-10 |
| Large objects | 8-12 |

## Advanced: Adaptive Threshold

For advanced users, you can implement dynamic thresholding:

```python
def adaptive_grip_check(self):
    """Calculate threshold based on recent history."""
    # Read gripper value
    current_val = self._hw_get_gripper_value()
    
    # Dynamic threshold: 150% of baseline
    baseline_closed = 2.0  # From calibration
    dynamic_threshold = baseline_closed * 1.5
    
    if current_val > dynamic_threshold:
        return True  # Object held
    else:
        return False  # No object
```

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────┐
│  GRIPPER THRESHOLD QUICK REFERENCE                      │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  🔧 Test Threshold:                                      │
│     python3 test_gripper_threshold.py                   │
│                                                          │
│  📊 Monitor Values:                                      │
│     python3 monitor_gripper.py                          │
│                                                          │
│  ⚙️  Update Config:                                      │
│     Edit launch file or controller parameter            │
│     grip_threshold: <recommended_value>                 │
│                                                          │
│  ✅ Detection Logic:                                     │
│     value > threshold  →  Object held ✓                 │
│     value ≤ threshold  →  No object ✗                   │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

## Files Created

1. **test_gripper_threshold.py** - Comprehensive calibration tool
2. **monitor_gripper.py** - Real-time value monitor
3. **GRIPPER_TESTING.md** - This documentation

## Next Steps

After determining your threshold:

1. ✅ Update `grip_threshold` in your configuration
2. ✅ Test full pick-and-place workflow
3. ✅ Monitor success rate over multiple cycles
4. ✅ Fine-tune if needed based on real performance
5. ✅ Document final value in your project notes

---

**Last Updated**: 2026-03-04  
**Version**: 1.0
