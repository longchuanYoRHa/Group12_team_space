# OpenVINO Optimized Deployment Model

This directory contains the production-ready **OpenVINO Intermediate Representation (IR)** files for the cube detection system. These files are optimized for high-performance inference on **Intel CPUs** and **GPUs**.

## 📂 Folder Contents: `best_openvino_model/`

This folder is designed to be cloned directly into your deployment environment or **ROS workspace**.

*   **`best.xml`**: The network topology (graph structure).
*   **`best.bin`**: The trained weights and biases in optimized binary format.
*   **`metadata.yaml`**: Contains model configuration, including class names (`Red`, `Blue`, `Green`) and input dimensions (720p).

## 🚀 Deployment & ROS Integration

These files are intended for use with the `launching_scripts/` or your custom **ROS node**. Because the model is already in OpenVINO format, it bypasses the need for PyTorch, significantly reducing startup time and memory overhead.

### Hardware Execution
You can target specific hardware by passing the device string to your launching scripts:


| Hardware | Device String | Usage |
| :--- | :--- | :--- |
| **Intel CPU** | `CPU` | General purpose execution |
| **Intel iGPU** | `GPU` | Integrated Graphics acceleration |
| **Intel Arc/dGPU** | `GPU.1` | Discrete Graphics (if available) |

### Loading via Ultralytics (Python)
If using the provided scripts, load the model by pointing to the **folder path**:

```python
from ultralytics import YOLO

# Load the OpenVINO folder directly
model = YOLO("best_openvino_model/")

# Run accelerated inference on GPU
results = model.predict(source=0, device="gpu")

