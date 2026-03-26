# ROS2 Vision Package (`vision_pkg`)

This repository contains the **ROS2 package** responsible for integrating YOLOv11 and OpenVINO into a robotic middleware ecosystem. It serves as the bridge between raw camera data and actionable spatial coordinates for the Red, Blue, and Green cube detection system.

## 📂 Package Components

*   **`src/`**: The core source directory containing the ROS2 node implementations. These nodes handle the subscription to the RealSense image streams and the publication of detection metadata.
*   **`ros_deployment_instructions.txt`**: A specialized guide focused on the environment integration required to run high-performance AI within ROS2.

## 📋 Key Features

### 1. Unified Environment Bridge
The package includes documentation on how to configure a **Global ROS2 installation** to interface with a **Python Virtual Environment**. This allows the ROS2 nodes to access the specific versions of `ultralytics` and `openvino` required for inference without causing system-wide dependency conflicts.

### 2. OpenVINO Inference Node
The included nodes are designed to load the optimized `best_openvino_model/`. They are programmed to leverage **Intel hardware acceleration** (CPU/GPU) to ensure low-latency processing of the 720p image stream.

### 3. Spatial Data Publishing
Beyond simple 2D bounding boxes, this package is designed to output:
*   **3D Coordinates**: Real-world X, Y, Z positions for the cubes.
*   **Status Flags**: Safety alerts based on the "Stopping Distance" logic and "Size Estimation" (filtering 2cm cubes from 20cm bins).

## 🏗️ Build & Deployment Logic
The package follows the standard **Colcon build system** architecture. The deployment instructions provided within the folder detail the specific `PYTHONPATH` and environment sourcing sequence necessary to ensure the detection scripts (v1–v4) function correctly within a ROS2 workspace.

---

**Note:** This package is intended to be used in conjunction with the **RealSense D435** drivers and the **OpenVINO** runtime environment.

