# Central Controller

## Rover bring-up: time sync and topic check

Accurate time on the base and NUC avoids TF extrapolation warnings and bad sensor fusion. After you SSH into the **rover**, open a session on the **NUC** from that shell and sync time there before running ROS 2.

### 1. SSH chain

From your laptop (example hostnames—use your team’s addresses):

```bash
ssh <user>@<rover-hostname-or-ip>
```

On the rover, SSH into the NUC:

```bash
ssh <user>@<nuc-hostname-or-ip>
```

### 2. Time sync on the NUC (command line)

On the **NUC** shell, set the NUC clock from the rover time (SSH must reach `leo-rover-12@10.0.0.2`; you may be prompted for a password unless keys are set up):

```bash
sudo date -s "$(ssh leo-rover-12@10.0.0.2 date '+%Y-%m-%d %H:%M:%S')"
```

Check with `date` or `date -u`. Exit the NUC SSH session when done if you connected via the rover; otherwise continue on the NUC for ROS commands.

### 3. Check ROS 2 topics on the NUC

With the base firmware / driver stack running, on a terminal where the NUC ROS environment is sourced, list topics:

```bash
ros2 topic list
```

You should see at least:

- `/imu/data`
- `/imu/data_raw`
- `/imu/rpy`
- `/joint_states`
- `/merged_odom`
- `/robot_description`
- `/tf`
- `/wheel_odom`

If any of these are missing, the chassis stack is probably not fully started or the wrong workspace/domain is sourced.

---

`central_controller/task_manager_node_v2.py` is the main scheduling node. It runs a **topic-driven state machine** that ties together exploration, navigation, precision alignment, manipulation phases, and post-exploration map fallback.

The current implementation no longer uses the older intermediate states described in the previous README (`OBJECT_FOUND`, `BIN_FOUND`, `PAUSE_EXPLORE`). Transitions are driven in callbacks: pause exploration, send navigation goals, and advance the state machine directly.

---

## Architecture

### 1. Initialization and exploration start

- `INIT`
  - Wait for the Nav2 action server.
  - Call `/reset_odometry`.
  - Read TF and store `home_pose`.
- `PRE_EXPLORE_SPIN`
  - If `pre_explore_spin_enable=true`, the robot first navigates in the `map` frame to a pose offset from the start.
  - Default offset is `+0.3 m` along x, facing `-x` (`yaw = pi`).
  - Frontier exploration starts only after this step completes.
- `EXPLORE`
  - Publish `explore/resume=true` to start or resume `custom_explore_node`.

### 2. Detections trigger navigation

- The node counts consecutive detections; by default **5** frames are required before follow-up behavior runs.
- When the threshold is met, the node:
  - Pauses exploration;
  - Cancels the active Nav2 goal if any;
  - Computes pre-grasp / pre-place goals from the robot and target poses;
  - Sends a Nav2 `NavigateToPose` goal.

### 3. Precision alignment

- After Nav2 reaches the pre-grasp pose, pre-place pose, or an interest point, the state becomes `PRECISION_ALIGN`.
- In this phase the node:
  - Waits for a new detection to trigger alignment;
  - Temporarily disables local costmap inflation;
  - Uses the `DockRobot` action for short-range docking-style alignment.
- On success, the state advances to the next manipulation state.

### 4. Post-manipulation behavior

- After grasp or place, the node does not return to exploration immediately; it enters `BACKUP_AFTER_ACTION`:
  - Drive backward for a fixed distance at the configured linear speed;
  - Restore local costmap inflation;
  - Then transition to the next state.
- If full-map exploration is not finished yet, the flow eventually returns to `EXPLORE`.

### 5. After exploration finishes

- On `explore/finished=true`, the node:
  - Stops exploration;
  - Runs `map_saver_cli` to save the map;
  - Extracts interest points from the generated `.pgm`;
  - Filters blacklisted targets;
  - Navigates through interest points to keep searching for targets.
- Related states include:
  - `RUN_MAP_DETECTION`
  - `NAV_TO_INTEREST_POINT`
  - `WAIT_AT_INTEREST_POINT`
- In practice, after reaching an interest point the node often enters `PRECISION_ALIGN` and, on timeout, skips to the next interest point.

---

## State overview

| State | Description |
|---|---|
| `INIT` | Wait for Nav2 / TF, save home pose, run startup preparation. |
| `PRE_EXPLORE_SPIN` | Pre-navigation before exploration: move to a fixed offset from home. |
| `EXPLORE` | Frontier exploration is running. |
| `NAV_TO_OBJECT_PREGRASP` | Navigate to the pre-grasp pose in front of the object. |
| `PRECISION_ALIGN` | Fine alignment after reaching the pre-pose. |
| `GRASP` | Grasp phase. |
| `RESUME_EXPLORE_FOR_BIN` | Resume exploration to search for the next target (e.g. bin). |
| `NAV_TO_BIN_PREPLACE` | Navigate to the pre-place pose in front of the bin. |
| `PLACE_IN_BIN` | Place phase. |
| `BACKUP_AFTER_ACTION` | Back up after manipulation, then restore navigation settings. |
| `POST_ACTION` | In the current version, returns to `EXPLORE`. |
| `EXPLORE_FINISHED_FALLBACK` | Entry to the post-exploration fallback path. |
| `RUN_MAP_DETECTION` | Interest-point detection on the saved map. |
| `NAV_TO_INTEREST_POINT` | Navigate to a candidate interest point. |
| `WAIT_AT_INTEREST_POINT` | Wait at an interest point for a trigger; often superseded by `PRECISION_ALIGN` timeout logic. |

---

## External interfaces

### Topics

**Published**

- `explore/resume` (`std_msgs/Bool`)
- `task_manager/state` (`std_msgs/String`)
- `task_manager/cargo_state` (`std_msgs/String`)

**Subscribed**

- `explore/finished` (`std_msgs/Bool`)

### Services

- `task_manager/get_state` (`std_srvs/Trigger`) — returns current `state` and `cargo`.
- `/reset_odometry` (`std_srvs/Trigger`) — called during initialization.

---

## How to launch

From workspace `System_integration/Ver01_ws`:

```bash
cd /home/student21/Desktop/AERO62520_ws/personal_branch/Group12_team_space/System_integration/Ver01_ws
colcon build
source install/setup.bash
ros2 launch central_controller starter_launch
```

To build only `central_controller`:

```bash
colcon build --packages-select central_controller
source install/setup.bash
ros2 launch central_controller starter_launch
```
