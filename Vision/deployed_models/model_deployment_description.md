# Cube Detection: Model Deployment & Evaluation

This repository tracks the evolution of our detection models, comparing legacy **YOLOv8** iterations against the current **YOLOv11** implementation. 

## 📂 Deployed Models Hierarchy

The `deployed_models/` directory serves as the versioned archive for weights and performance analytics. 

*   **`old_models/` (Version 1 & 2):** These iterations were trained exclusively on the **640p** "Old Set." They serve as the baseline for lower-resolution performance.
*   **`latest_model/`:** Our flagship model trained on the **720p** "New Set." This version leverages higher pixel density for superior accuracy in multi-cube scenarios.

### Directory Structure
```text
deployed_models/
├── latest_model/             # 720p Optimized (YOLOv11)
│   ├── weights/
│   │   ├── best.pt           # <--- Actual deployment file
│   │   └── last.pt
│   ├── confusion_matrix.png  # Accuracy breakdown
│   ├── results.png           # Training loss & mAP curves
│   └── val_batch_labels.jpg  # Visual validation proofs
└── old_models/               # 640p Legacy (YOLOv8)
    ├── version_1/
    │   └── weights/best.pt
    └── version_2/
        └── weights/best.pt

