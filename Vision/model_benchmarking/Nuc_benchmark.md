# NUC Model Benchmark & Deployment Guide

It combines Environment Setup, Model Deployment (Downloading & Exporting), and Benchmarking Instructions into one smooth workflow.

This project benchmarks **YOLOv8**, **YOLO11**, and **RT-DETR** object detection models on the Intel NUC.

To ensure real-time performance on the NUC CPU/iGPU, this guide includes steps to automatically download these models and convert them to **OpenVINO** format.

---

1. Environment Setup

Before running the scripts, ensure your NUC has the necessary Python libraries installed.
Open your terminal and run:

```bash
# 1. Update pip
pip install --upgrade pip

# 2. Install core libraries (Ultralytics includes YOLO & RT-DETR)
pip install ultralytics opencv-python numpy pandas

# 3. Install OpenVINO (Critical for NUC CPU acceleration)
pip install openvino
 2. Model Deployment (Download & Export)
CRITICAL STEP: To get high FPS, we must export the PyTorch (.pt) models to OpenVINO format.

Run the following 3 commands one by one in your project folder. (Note: These commands will automatically download the weights from GitHub if you don't have them, and then convert them.)

A. Deploy YOLOv8 (Nano)
The standard baseline model.

Bash

yolo export model=yolov8n.pt format=openvino
Result: Creates a folder named yolov8n_openvino_model/.

B. Deploy YOLO11 (Nano)
Newer architecture, higher accuracy.

Bash

yolo export model=yolo11n.pt format=openvino
Result: Creates a folder named yolo11n_openvino_model/.

C. Deploy RT-DETR (Large)
Transformer-based model. High accuracy but heavy resource usage.

Bash

yolo export model=rtdetr-l.pt format=openvino
Result: Creates a folder named rtdetr-l_openvino_model/.

3. Configuration
Open the python script benchmark_nuc.py.

Locate the MODEL_CONFIG section at the top.

Ensure the paths match the folders created in Step 2. (If you haven't moved the folders, the default relative paths below will work perfectly.)

Python

MODEL_CONFIG = [
    # Format: ("Display Name", "Path to OpenVINO Folder", "Architecture")
    
    ("YOLOv8n-VINO",  "./yolov8n_openvino_model/",  "CNN"),
    ("YOLO11n-VINO",  "./yolo11n_openvino_model/",  "CNN"),
    ("RT-DETR-VINO",  "./rtdetr-l_openvino_model/", "Transformer")
]
4. Run the Benchmark
Once the models are deployed and the config is checked, run the benchmark script:

Bash

python benchmark_nuc.py
Expected Output Example
The script will warm up each model to compile the OpenVINO graph, then run a stress test to calculate the average FPS.

Plaintext

=====================================================================================
Model Name         | Arch         | FPS        | Latency(ms)  | Conf      
-------------------------------------------------------------------------------------
YOLOv8n-VINO       | CNN          | 65.20      | 15.33        | 0.89      
YOLO11n-VINO       | CNN          | 62.10      | 16.10        | 0.94      
RT-DETR-VINO       | Transformer  | 12.50      | 80.20        | 0.87      
=====================================================================================


