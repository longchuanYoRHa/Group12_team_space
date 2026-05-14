# Autonomous Object Retrieval & Delivery Rover

## Project Overview
Welcome to the GitHub repository of the Manchester Robotics Team 12! We are a group of five passionate Robotics students from the University of Manchester: Aritra Bag, Nicholas Lesseps, Qikai Sun, Shaotong Hong, Zhikun Peng.

This project focuses on the design and development of an autonomous rover capable of retrieving and delivering objects in a structured environment. The rover is equipped with a robotic arm for manipulation tasks, a vision system for object detection and localization, and a navigation system for autonomous movement. The project encompasses mechanical design, software control, system integration, and testing to ensure the rover can perform its intended functions effectively.

## Git Workspace Structure

- [Mechanical_design](Mechanical_design): Mechanical design of modification for chassis and the robotic arm, and linking structures with some general draft and the design analysis.
- [Software_control](Software_control): Software control of the rover and the robotic arm, with some general glance and the software control analysis.
- [Equipment](Equipment): Equipment datasheets
- [Manipulator](Manipulator): Software control of the robotic arm, with some general glance and the software control analysis.
- [Vision](Vision): Vision related code of the rover, with some general glance and the vision system analysis.
- [System_integration](System_integration): System integration of the rover, with core control logic and integration strategies.
- [Integration_sanity_check](Integration_sanity_check): Integration sanity check script of whole system, with remote control and the sanity check report.
- [CW1_workplace_charter](CW1_workplace_charter): Workplace charter of the team, with the statement of purpose and the statements of principles and commitments.
- [CW3_design_requirements_analysis](CW3_design_requirements_analysis): Design requirements analysis of the project, containing related requirements, support documents and draft of the design requirements analysis.
- [CW4_preliminary_design_review](CW4_preliminary_design_review): Preliminary design review of the project, containing the preliminary design review report and the block diagrams.
- [CW6_final_design_review](CW6_final_design_review): Final design review of the project, containing the final design review report and the block diagrams.

## Technical Overview

- **Vision System**: Integrated YOLO11 (accelerated by OpenVINO) achieving 58 FPS for real-time object detection and spatial reasoning. Uses RGB-D input for colour classification and approximate depth estimation to support grasp planning.
- **Navigation**: Utilizing `slam_toolbox` for mapping and the `Nav2` stack for autonomous path planning and obstacle avoidance, with exploration and recovery behaviours.
- **Manipulation**: Precise 6-DOF control with a state machine for accurate grasping and depositing, including gripper control, pick/place sequences, and calibration routines.
- **Software Architecture**: Modular, decentralized framework built on ROS 2 with separate packages for perception, planning, control, and integration; communication via topics/services/actions and lifecycle-managed nodes.