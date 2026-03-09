# Cube Detection: YOLOv8 vs YOLOv11

This repository features a complete pipeline for detecting **Red, Blue, and Green cubes** (individual or combinations). It covers data acquisition, structured dataset management, and high-performance inference using **OpenVINO** on **Intel CPU and GPU**.

## 📸 Data Acquisition
The `capture.py` script is included to streamline the creation of new training data. 
- **Functionality**: Captures frames from your camera and saves them directly to your project directory.
- **Resolution**: Configured to match the **720p** "New Set" for consistency.

## 📂 Dataset Organization

The repository distinguishes between raw captures and training-ready data:
*   **Old Set (640p):** Legacy images for baseline testing.
*   **New Set (720p):** High-definition images featuring single cubes, pairs, and triplets.

### `/yolo_ready` Directory
The `yolo_ready/` folder is pre-sorted for direct YOLO training using the 720p dataset:

```text
yolo_ready/
├── images/
│   ├── train/  # Training photos (Single & mixed cubes)
│   └── val/    # Validation photos
└── labels/
    ├── train/  # Corresponding YOLO .txt labels
    └── val/    # Corresponding YOLO .txt labels

