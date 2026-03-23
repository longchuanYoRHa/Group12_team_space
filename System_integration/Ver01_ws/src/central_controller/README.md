# Central Controller

ROS 2 node that acts as the core scheduler for the Explore–Pick–Stow–SearchBin–Place workflow. It coordinates exploration, vision-based target detection, Nav2 navigation, and (placeholder) arm manipulation.

---

## Main Functionality

- **Explore control**  
  Publishes to `explore/resume` (`std_msgs/Bool`) to start or pause the exploration behaviour (`custom_explore` / `custom_explore_node`). Only publishes when the value changes.

- **Vision integration**  
  Subscribes to detection topics from `rover_vision_node`:
  - **Pick targets:** `/target_pick/red`, `/target_pick/green`, `/target_pick/blue` — `geometry_msgs/Point` (object position in camera frame).
  - **Place targets:** `/target_place/red`, `/target_place/green`, `/target_place/blue` — `geometry_msgs/Point` (bin position in camera frame).  
  Points are transformed to the `map` frame using TF (configurable `camera_frame_id`, default `camera_depth_optical_frame`).

- **Target stability**  
  The **main program** performs target stability: it requires **N consecutive detections** (default `required_detection_frames = 5`) before transitioning to `OBJECT_FOUND` or `BIN_FOUND`. Single-frame detections are ignored for state transitions.

- **Navigation**  
  Uses Nav2 `NavigateToPose` to drive to pregrasp/preplace poses computed in front of the target at configurable distances (`pregrasp_distance`, `preplace_distance`).

- **Blacklist**  
  Failed grasp poses can be added to a blacklist (configurable `blacklist_radius`) so the same object is not retried repeatedly.

- **State and cargo publishing**  
  Publishes current task state and cargo state for monitoring: `task_manager/state`, `task_manager/cargo_state` (`std_msgs/String`).

---

## Task State Machine (Transition Logic)

The workflow is implemented as a state machine with the following states and transitions:

| State | Description |
|-------|-------------|
| `INIT` | Wait for Nav2 and TF; save home pose, then go to `EXPLORE`. |
| `EXPLORE` | Exploration is running; listen for object detections on `/target_pick/*`. |
| `OBJECT_FOUND` | Object stably detected (N-frame confirm); object pose available → go to `PAUSE_EXPLORE`. |
| `PAUSE_EXPLORE` | Cancel Nav2 goal, stop exploration; then go to `NAV_TO_OBJECT_PREGRASP` (if empty) or `NAV_TO_BIN_PREPLACE` (if carrying). |
| `NAV_TO_OBJECT_PREGRASP` | Send Nav2 goal to pregrasp pose; on success → `PRECISION_ALIGN_OBJECT`. |
| `PRECISION_ALIGN_OBJECT` | (Placeholder) Optional fine alignment; currently jumps to `GRASP`. |
| `GRASP` | (Placeholder) Execute grasp; on success set cargo to HAS_OBJECT → `RESUME_EXPLORE_FOR_BIN`. Retry/blacklist on failure. |
| `RESUME_EXPLORE_FOR_BIN` | Exploration running again; listen for bin detections on `/target_place/*`. |
| `BIN_FOUND` | Bin stably detected (N-frame confirm) → `PAUSE_EXPLORE`. |
| `NAV_TO_BIN_PREPLACE` | Send Nav2 goal to preplace pose; on success → `PRECISION_ALIGN_BIN`. |
| `PRECISION_ALIGN_BIN` | (Placeholder) Optional fine alignment; currently jumps to `PLACE_IN_BIN`. |
| `PLACE_IN_BIN` | (Placeholder) Execute place; on success set cargo to EMPTY → `POST_ACTION`. Retry on failure. |
| `POST_ACTION` | Currently: return to `EXPLORE`. |

Cargo state (`EMPTY` / `HAS_OBJECT`) restricts which detections are accepted: only object detections when `EMPTY` and in `EXPLORE`, and only bin detections when `HAS_OBJECT` and in `RESUME_EXPLORE_FOR_BIN`.

---

## Vision: When Does It Publish?

The vision node (`rover_vision_node`) publishes to `/target_pick/*` and `/target_place/*` **whenever** the current frame has a detection that passes its internal checks (e.g. confidence threshold and size filter for cube/bin). There is **no stability or filtering over time inside the vision node**; it is per-frame.

**Target stability is done in the central controller:** the task manager only advances to `OBJECT_FOUND` or `BIN_FOUND` after `required_detection_frames` (default 5) consecutive messages on the relevant topic. So the main program is responsible for “target stability”; the vision node only provides raw, frame-by-frame detections.

---

## TODOs and Current Limitations

### Arm (grasp / place)

- **Grasp** and **place** are **placeholders** only. The real arm/gripper actions (e.g. calling a grasp/place action server) still need to be integrated later.
- Precision align states (`PRECISION_ALIGN_OBJECT`, `PRECISION_ALIGN_BIN`) are also placeholders (e.g. for D435i-based alignment).

### Vision

- Vision is **only partially integrated**; **module-level testing** (e.g. unit/integration tests for vision + central_controller) has **not** been done yet.
- The current vision node is built for **real hardware** (RealSense, OpenVINO/YOLO). For **simulation**:
  - A **simulation adapter** should be implemented that:
    - Subscribes to a **simulation image topic** (e.g. camera image + depth or simulated depth).
    - Runs the same detection logic (or a simplified one) and publishes **the same message types** (`geometry_msgs/Point`) on the **same topic names** (`/target_pick/*`, `/target_place/*`).
  - The adapter must define:
    - **Simulation image topic(s)** and **message types** (e.g. `sensor_msgs/Image`, depth topic if used), and ensure **message type and frame_id** are compatible with the rest of the stack.
  - The central controller can stay unchanged as long as the adapter publishes `geometry_msgs/Point` on the existing topics and the TF tree (or sim equivalent) provides the configured `camera_frame_id` → `map` transform.

### Summary

- **Arm:** Grasp/place and precision alignment are placeholders; arm integration is pending.
- **Vision:** Partially integrated; no module test yet; simulation requires a dedicated sim adapter with a clear sim image topic interface and message type contract.
- **Stability:** Handled in the central controller (N-frame confirmation); no change needed in vision for that.
