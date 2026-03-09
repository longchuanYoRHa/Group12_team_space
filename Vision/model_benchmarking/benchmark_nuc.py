import time
import os
import numpy as np
from ultralytics import YOLO, RTDETR

# ==========================================
#  Configuration Section
# ==========================================

# 1. Model Paths
#    IMPORTANT: Update these paths to match your actual NUC file structure.
#    For OpenVINO, point to the FOLDER containing .xml and .bin files.
#    For PyTorch, point to the .pt file.
MODEL_CONFIG = [
    # Format: ("Display Name", "Absolute Path to Model", "Architecture Type")
    
    ("YOLOv8n-VINO",  "/home/leo-rover-12/yolo_openvino_gpu/yolov8n_openvino_model/",  "CNN"),
    ("YOLO11n-VINO",  "/home/leo-rover-12/yolo_openvino_gpu/yolo11n_openvino_model/",  "CNN"),
    
    # If RT-DETR is not exported to OpenVINO, point to the .pt file
    ("RT-DETR-L",     "/home/leo-rover-12/yolo_openvino_gpu/rtdetr-l.pt",              "Transformer")
]

# 2. Test Image Source
#    Use a local path if available (e.g., 'data/test.jpg') to avoid network latency.
#    Using a URL for default demonstration.
TEST_IMAGE = 'https://ultralytics.com/images/bus.jpg' 

# 3. Number of inference runs per model to calculate average
TEST_RUNS = 30

# ==========================================

def run_benchmark():
    print(f"\n Starting NUC Model Benchmark (Runs per model: {TEST_RUNS})\n")
    
    results = []

    for name, path, arch in MODEL_CONFIG:
        # --- 0. Path Validation ---
        if not os.path.exists(path):
            print(f" Error: Path not found -> {path}")
            continue

        print(f" Loading: {name} ...")

        # --- 1. Load Model ---
        try:
            # Ultralytics automatically detects if path is a .pt file or OpenVINO folder
            if "rtdetr" in name.lower() or "rtdetr" in path.lower():
                try:
                    model = RTDETR(path)
                except:
                    # Fallback to YOLO class which often handles exported RT-DETR
                    model = YOLO(path)
            else:
                model = YOLO(path) 
        except Exception as e:
            print(f" Load Failed: {e}")
            continue

        # --- 2. Warmup ---
        # Critical for OpenVINO to compile the computation graph
        print(f"    Warming up model...")
        try:
            model.predict(TEST_IMAGE, verbose=False)
        except Exception as e:
            print(f"    Warmup failed: {e}")
            continue

        # --- 3. Inference Loop ---
        latencies = []
        confidences = []

        print(f"    Running inference...")
        for _ in range(TEST_RUNS):
            start_time = time.time()
            
            # Run prediction
            res = model.predict(TEST_IMAGE, verbose=False)[0]
            
            end_time = time.time()
            latencies.append((end_time - start_time) * 1000) # Convert to ms
            
            # Record average confidence of detected objects
            if res.boxes:
                confidences.append(res.boxes.conf.cpu().numpy().mean())
            else:
                confidences.append(0.0)

        # --- 4. Calculate Statistics ---
        avg_latency = np.mean(latencies)
        avg_fps = 1000 / avg_latency
        avg_conf = np.mean(confidences) if confidences else 0.0

        results.append({
            "model": name,
            "arch": arch,
            "fps": avg_fps,
            "latency": avg_latency,
            "conf": avg_conf
        })
        
        print(f"    Finished. Average FPS: {avg_fps:.2f}")

    # ================= Print Final Table =================
    if not results:
        print("\n No models were tested successfully. Check your paths.")
        return

    print("\n" + "="*85)
    # Formatted string for table alignment
    header = f"{'Model Name':<18} | {'Arch':<12} | {'FPS':<10} | {'Latency(ms)':<12} | {'Conf':<10}"
    print(header)
    print("-" * 85)

    for res in results:
        row = f"{res['model']:<18} | {res['arch']:<12} | {res['fps']:<10.2f} | {res['latency']:<12.2f} | {res['conf']:<10.2f}"
        print(row)
    
    print("="*85)
    
    # Identify the fastest model
    best_model = max(results, key=lambda x: x['fps'])
    print(f"\n Winner: [{best_model['model']}] is the fastest on this NUC ({best_model['fps']:.2f} FPS).")

if __name__ == "__main__":
    run_benchmark()