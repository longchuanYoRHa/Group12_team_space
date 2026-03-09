# Installation & Environment Setup

This directory contains the necessary documentation to set up the execution environment for the cube detection system. The instructions are split into two phases: core hardware acceleration and the high-level vision framework.

## 📂 Installation Guides

The environment is configured using the following two instruction sets:

*   **`openvino_installation.txt`**: 
    *   **Purpose**: Provides the step-by-step process for installing the **Intel OpenVINO Toolkit**.
    *   **Key Components**: Setup of the OpenVINO Runtime, development tools, and the necessary drivers to enable **GPU-accelerated** inference on Intel Iris Xe or Arc graphics.
    *   **Focus**: Low-level hardware optimization and model optimization (IR format support).

*   **`yolo_openvino_installation.txt`**: 
    *   **Purpose**: Configures the **Ultralytics YOLO** framework and the deployment dependencies.
    *   **Key Components**: Installation of `ultralytics`, `opencv-python`, and the specific libraries required to bridge YOLO logic with the OpenVINO backend.
    *   **Focus**: High-level application logic, including the execution of the `v1` through `v4` deployment scripts.

## 🛠️ Environment Workflow

To successfully run the project, follow the scripts in order:

1.  **Hardware Level**: Complete `openvino_installation.txt` first to ensure your system can communicate with the Intel CPU/GPU.
2.  **Application Level**: Complete `yolo_openvino_installation.txt` to install the detection engine.
3.  **Verification**: Once both are finished, you can run the deployment scripts using the `best_openvino_model/` folder.

## 🚀 Compatibility Table


| Installation Task | Target Hardware | Primary Library |
| :--- | :--- | :--- |
| **OpenVINO Setup** | Intel CPU/GPU | `openvino`, `openvino-dev` |
| **YOLO Setup** | All Backends | `ultralytics`, `opencv` |

---

**Note:** It is highly recommended to use a single virtual environment (e.g., `venv` or `conda`) for both sets of instructions to ensure all dependencies are linked correctly for the deployment scripts.

