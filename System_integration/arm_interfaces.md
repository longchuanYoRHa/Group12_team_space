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
x: float64   # X coordinate (mm) in base_link frame
y: float64   # Y coordinate (mm) in base_link frame
z: float64   # Z coordinate (mm) in base_link frame
```

**Coordinate Limits (arm base frame, after TF2 transform):**

| Axis | Min (mm) | Max (mm) |
|---|---|---|
| x | -281.45 | 281.45 |
| y | -281.45 | 281.45 |
| z | -70.0 | 450.0 |

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
| `moving_to_pick` | Moving above pick target |
| `descending_pick` | Descending to pick height |
| `gripping` | Closing gripper |
| `grip_check` | Verifying grip |
| `lifting` | Lifting object to safe height |
| `returning_home` | Moving back to home angles |
| `holding` | Block secured, arm locked at home. Ready to accept place command |
| `moving_to_place` | Moving above bin |
| `releasing` | Opening gripper to drop block |
| `error` | Error occurred, check logs |

**`/arm/gripper_status` possible values:**

| Value | Meaning |
|---|---|
| `object_held` | Grip confirmed, object in gripper |
| `no_object` | Grip attempt failed, nothing detected |
| `released` | Gripper opened (after place) |
| `object_dropped` | Object lost during lift or transit |
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