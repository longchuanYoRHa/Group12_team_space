# ROS2 System Architecture Design: Navigation, Vision and Manipulation Coordination

## 1. Background

In a **ROS2-based robotic system**, it is common to have multiple functional modules such as:

- Navigation
- Vision / Perception
- Manipulation (Grasping)

When designing the system architecture, a common question arises:

> Should we design a **central controller node** that receives all information and makes all decisions, or should modules be allowed to **communicate and make decisions directly**?

This document summarizes the key considerations and proposes a recommended architecture.

---

# 2. Non-Recommended Architectures

## 2.1 Monolithic Central Controller

A straightforward approach is to build a single large controller node:


All module outputs are sent to a **single controller node**, which handles all decisions.

### Disadvantages

#### 1. Rapidly Increasing Complexity

The controller must understand:

- Nav2 action states
- Vision detection outputs
- Camera-to-robot coordinate transforms
- Grasp feasibility checks
- Manipulator execution feedback
- Failure recovery
- Timeout and retry strategies

This eventually becomes a **large monolithic state machine**, which is difficult to maintain.

---

#### 2. Reduced Module Reusability

If perception or manipulation algorithms change, for example:

- YOLO → another detector
- Different grasp planner

The central controller logic must also be modified.

---

#### 3. Difficult Debugging

Debugging requires tracking:

- many topics
- multiple actions
- complex internal states

This significantly increases debugging complexity.

---

## 2.2 Fully Decentralized Architecture

Another extreme is allowing modules to communicate freely:


Examples:

- Vision detects an object → directly triggers grasping
- Manipulation failure → directly commands navigation
- Navigation completion → vision decides the next step

### Disadvantages

#### 1. Unclear Responsibilities

The vision module should mainly provide:

- object detection
- object pose estimation

It should not decide:

- whether to grasp now
- whether the robot should move
- whether to abandon a target

---

#### 2. Hidden Dependencies

Modules gradually become tightly coupled.

For example:

- Manipulation node assumes a specific message format from Vision
- Vision node assumes a certain behavior from Navigation

This reduces system flexibility.

---

#### 3. Unpredictable System Behavior

If multiple nodes can make decisions:

- Who controls task flow?
- Who handles recovery?
- Who resolves conflicts?

The system becomes difficult to reason about.

---

# 3. Recommended Architecture: Layered Design

A more robust approach is a **layered architecture**.


The system is divided into two layers.

---

## 3.1 Execution Layer

Each module focuses on its **core capability**.

### Navigation

Responsibilities:

- Receive navigation goals
- Call Nav2 actions
- Return execution status

Example interface:
navigate_to_pose(goal_pose)

Return status:
SUCCESS
FAILED
CANCELED


---

### Vision

Responsibilities:

- Detect objects
- Estimate object poses
- Provide grasp candidates

Outputs:
object_id
object_pose
grasp_pose
confidence


---

### Manipulation

Responsibilities:

- Motion planning
- Execute grasp
- Control gripper

Interface:
pick_object(grasp_pose)

Return:
SUCCESS
FAILED
REASON


---

## 3.2 Coordination Layer

The **Task Coordinator** manages:

- task flow
- state transitions
- module invocation
- failure recovery
- retry strategies

Example workflow:
1 Navigate to search area
2 Trigger object detection
3 Evaluate grasp feasibility
4 Execute grasp
5 If failed -> retry or reposition
6 If success -> navigate to placement area

---

# 4. Direct Communication Between Modules

While the system uses a **central task coordinator** to manage high-level task execution, limited **direct communication between functional modules** is acceptable when necessary.

The overall design principle is:

- **High-level decisions** (task flow, sequencing, recovery strategies) should be handled by the **Task Coordinator**.
- **Low-level capability outputs** (e.g., perception results, control feedback, pose information) may be shared directly between modules.

This approach maintains:

- clear responsibility boundaries between modules  
- reduced coupling between perception, navigation, and manipulation  
- efficient data flow for time-sensitive operations

In practice, modules should expose **well-defined ROS2 interfaces** (topics, services, or actions), allowing the coordinator to orchestrate task execution while still permitting necessary data exchange between modules.
