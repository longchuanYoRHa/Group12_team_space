# ROS2 Communication Interface Reference for Arm and NUC

> This section lists all ROS2 interfaces exposed by the `mycobot_controller_tf2` node, for NUC-side integration.

## 1. Node Info

| Item | Value |
|---|---|
| Node name | `mycobot_controller` |
| Namespace | `/arm` |
| Executable | `mycobot_controller_tf2` |
| Package | `my_cobot_control` |

---

## 2 Subscribed Topics (NUC → Arm)

| Full Topic | Message Type | Frame | Description |
|---|---|---|---|
| `/arm/target_pick` | `geometry_msgs/msg/Point` | `base_link` | 3D pick coords (mm). Only accepted in `IDLE` state. Triggers full pick sequence. |
| `/arm/target_place` | `geometry_msgs/msg/Point` | `base_link` | 3D place coords (mm). Only accepted in `HOLDING` state. Triggers place + return. |

**Message field layout (`geometry_msgs/msg/Point`):**

```yaml
x: float64   # X coordinate (m) in base_link frame
y: float64   # Y coordinate (m) in base_link frame
z: float64   # Z coordinate (m) in base_link frame
```

**Coordinate Limits (arm base frame, after TF2 transform):**

| Axis | Min (m) | Max (m) |
|---|---|---|
| x | 0.015 | 0.295 |
| y | -0.130 | 0.070 |
| z | 0.015 | 0.430 |

---

## 3 Published Topics (Arm → NUC)

| Full Topic | Message Type | Rate | Description |
|---|---|---|---|
| `/arm/status` | `std_msgs/msg/String` | On state change | Current arm state (see state list below) |
| `/arm/gripper_status` | `std_msgs/msg/String` | On grip event | Gripper feedback (see gripper states below) |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | 10 Hz | Real-time joint angles (radians) |

**`/arm/status` possible values:**

| Value | Meaning |
|---|---|
| `idle` | Ready to accept new pick command |
| `busy` | Intermediate execution states (moving, gripping, lifting, releasing, etc.). See ROS logs for details |
| `holding` | Block secured, arm locked at home. Ready to accept place command |
| `error` | Error occurred, check logs |

**`/arm/gripper_status` possible values:**

| Value | Meaning |
|---|---|
| `object_held` | Grip confirmed, object in gripper |
| `no_object` | No object (includes grip failed, object lost, or gripper released) |
| `unknown` | Gripper value unreadable |

**`/arm/joint_states` field layout (`sensor_msgs/msg/JointState`):**

```yaml
header:
  stamp: <timestamp>
name: [
  'joint2_to_joint1',
  'joint3_to_joint2',
  'joint4_to_joint3',
  'joint5_to_joint4',
  'joint6_to_joint5',
  'joint6output_to_joint6'
]
position: [j1, j2, j3, j4, j5, j6]   # radians
```

---

## 4 TF2 Frames

| Frame | Parent | Source | Description |
|---|---|---|---|
| `base_link` | — | NUC / Camera node | Camera optical origin. Input coords are in this frame |
| `g_base` | `base_link` | Static TF (launch file) | Arm base (`arm_base_link` in URDF). Fixed offset from base_link (rover) |
| `joint6_flange` | `g_base` | `robot_state_publisher` | End-effector flange |
| `gripper_tip` | `joint6_flange` | Static TF (launch file) | Gripper finger tip. Z offset = 79 mm |

Default static transform (`base_link` → `g_base`):

| Param | Default | Description |
|---|---|---|
| `camera_x` | `0.0379` m | X offset |
| `camera_y` | `0.0641` m | Y offset |
| `camera_z` | `-0.0486` m | Z offset |
| `camera_pitch` | `-0.5236` rad | Pitch (-30°) |

---

## 5 Launch Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `safe_z` | float | `250.0` | Safe travel height (mm) |
| `move_speed` | int | `40` | Arm speed (1–100) |
| `gripper_speed` | int | `80` | Gripper speed (1–100) |
| `gripper_torque` | int | `300` | Gripper holding torque (1–980) |
| `grip_threshold` | int | `25` | Min gripper value to confirm grip |
| `grip_check_retries` | int | `2` | Number of grip check retries |
| `end_rx` | float | `-178.0` | End-effector roll (deg) |
| `end_ry` | float | `0.0` | End-effector pitch (deg) |
| `end_rz` | float | `0.0` | End-effector yaw (deg) |
| `home_angles` | float[] | `[0,0,0,0,0,0]` | Home joint angles (deg) |
| `calibration_file` | string | `''` | Path to calibration JSON |
| `camera_frame` | string | `camera_link` | TF2 source frame for input coords |
| `arm_base_frame` | string | `g_base` | TF2 arm base frame |
| `tf_timeout` | float | `1.0` | TF2 lookup timeout (s) |
| `compensate_gripper_offset` | bool | `true` | Auto subtract gripper Z offset |
| `gripper_offset_z` | float | `79.0` | Fallback gripper Z offset (mm) |
| `use_mock` | bool | `true` | Launch joint GUI (dev) or real hw |

---

## 6 ROS_DOMAIN_ID

Make sure the same `ROS_DOMAIN_ID` is set on both NUC and Pi:

```bash
export ROS_DOMAIN_ID=12   # or any agreed value, default 0
```

For persistent setting, add to `~/.bashrc` on both machines.
