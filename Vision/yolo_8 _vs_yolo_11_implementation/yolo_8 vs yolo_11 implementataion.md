# YOLOv8 vs YOLOv11 OpenVINO Deployment

This repository compares the implementation and performance of **YOLOv8** and **YOLOv11** optimized with the **OpenVINO Toolkit**. It provides a unified pipeline for running inference on both **Intel CPUs** and **GPUs** (integrated or discrete).

## 🚀 Overview


| Feature | YOLOv8 | YOLOv11 |
| :--- | :--- | :--- |
| **Architecture** | C2f based | C3k2 & C2PSA (Spatial Attention) |
| **Parameters** | Baseline | ~22% fewer (m-variant) |
| **Focus** | Stability & mature ecosystem | Efficiency & small object detection |
| **OpenVINO Support** | Fully Mature | Optimized via latest `ultralytics` |

## 🛠️ Implementation Strategy

### 1. Exporting Models
Both versions utilize the OpenVINO Intermediate Representation (IR) for maximum hardware acceleration.

```python
from ultralytics import YOLO

# For YOLOv8
model_v8 = YOLO("yolov8n.pt")
model_v8.export(format="openvino", dynamic=True)

# For YOLOv11
model_v11 = YOLO("yolo11n.pt")
model_v11.export(format="openvino", dynamic=True)

