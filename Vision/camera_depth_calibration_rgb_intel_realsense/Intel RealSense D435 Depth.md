# Intel RealSense D435 Depth Calibration & Detection

This repository provides two distinct methods for calibrating the Intel RealSense D435 camera to ensure accurate spatial coordinates for cube detection. It bridges the gap between manual RGB calibration and Intel’s native on-chip depth hardware calibration.

## 📂 Repository Structure

### 1. RGB Manual Calibration (`/rgb_manual_calibration`)
This folder contains tools to calibrate the RGB camera intrinsics, which is a prerequisite for mapping 2D detections to 3D space.

*   **`calibrate.py`**: A Python script that uses a physical **checkerboard** pattern to calculate the camera matrix and distortion coefficients.
*   **Manual Depth Implementation**: Includes a deployment script that combines the trained YOLO model with manual RGB-to-Depth mapping. 
    *   *Note:* While functional for detection, this method's depth accuracy is limited by the manual calibration parameters and is less precise than hardware-level alignment.

### 2. Intel RealSense Viewer & On-Chip Calibration (`/realsense_viewer_instructions`)
This folder provides the workflow for the highly accurate, factory-grade calibration method.

*   **Setup Instructions**: Step-by-step guide to installing the `intel-realsense-viewer` on Linux/Windows.
*   **On-Chip Calibration**: Documentation on how to trigger the D435's internal self-calibration. This process uses the camera's internal processors to optimize the stereo depth map without requiring a checkerboard.
*   **Firmware Updates**: Instructions on keeping the D435 firmware current to support the latest depth algorithms.

## 🛠 Calibration Workflow

### Step 1: Hardware Calibration (Recommended)
Follow the instructions in the Realsense Viewer folder to perform an **On-Chip Calibration**. This ensures the IR projectors and stereo sensors are perfectly aligned at the hardware level.

### Step 2: RGB Alignment
Use the `calibrate.py` script if your application requires custom lens correction or specific alignment between the RGB sensor and a custom coordinate system:
```bash
python calibrate.py --width 9 --height 6 --square_size 0.025

