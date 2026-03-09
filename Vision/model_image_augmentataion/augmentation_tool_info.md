# Data Augmentation Tool for Object Detection

A dedicated Python utility for expanding image datasets in Robotic Systems Design – Group 12.
This tool applies geometric and pixel-level transformations to significantly increase dataset diversity, improving YOLOv8/YOLO11 model robustness under real-world conditions.

Result: Converts 300 raw images into 2,400+ training samples (8× dataset growth).

## Critical Workflow Notice

Run this augmentation script before labeling your dataset.

Geometric operations (rotation/flip) change object coordinates.
Correct workflow:

Collect raw images

Run this augmentation tool

Upload the augmented dataset to Roboflow / LabelImg

Label all generated images (recommended: label once → auto-propagate in Roboflow)

Following this order prevents annotation misalignment.

## Augmentation Features

For every input image, this tool produces 7 additional variants, totaling 8 versions per image:

```text
Type         Suffix          Description                           Purpose
Original     _original       Raw untouched image                   Baseline
Geometric    _flip_h         Horizontal flip                       Mirrors viewpoint
Geometric    _flip_v         Vertical flip                         Handles upside-down or low-angle shots
Geometric    _rot90          Rotate 90° clockwise                  Supports rotated camera mountings
Geometric    _rot180         Rotate 180°                           Full image inversion
Pixel        _bright         +60% brightness                       Strong light / glare simulation
Pixel        _dark           −40% brightness                       Low-light / shadow conditions
Hybrid       _rot90_noise    Rotate 90° + Gaussian noise           Models sensor noise under difficult angles
```

These transformations improve robustness against lighting shifts, sensor artifacts, and camera placement variability.

## Installation

Make sure Python is installed. Install dependencies using pip:

pip install opencv-python numpy
