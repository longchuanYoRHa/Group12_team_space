
from ultralytics import YOLO
import cv2
import time
from collections import deque

# ---------- Settings ----------
CAM_INDEX = 0            # default webcam
WINDOW_NAME = "YOLOv8 Real-Time (with FPS)"
MODEL_PATH = "yolov8n.pt"  # nano model for speed
IMG_SIZE = 640            # inference size (reduce for more speed, e.g., 480 or 416)
CONF_THRES = 0.25         # confidence threshold for drawing
FPS_SMOOTH_N = 30         # moving average window for FPS display
# ------------------------------

# Load YOLOv8 model
model = YOLO(MODEL_PATH)

# Open webcam
cap = cv2.VideoCapture(CAM_INDEX)  # 0 = default webcam
# (Optional) constrain capture resolution for speed
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# FPS tracking
fps_times = deque(maxlen=FPS_SMOOTH_N)
prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Measure time since last frame
    now = time.time()
    dt = now - prev_time
    prev_time = now
    if dt > 0:
        fps_times.append(1.0 / dt)

    # Run YOLO inference on the frame (no recording, just inference + display)
    results = model.predict(
        source=frame,
        imgsz=IMG_SIZE,
        conf=CONF_THRES,
        verbose=False
    )

    # Draw detections
    annotated_frame = results[0].plot()

    # Compute smoothed FPS
    if fps_times:
        fps = sum(fps_times) / len(fps_times)
    else:
        fps = 0.0

    # Overlay FPS (top-left corner)
    cv2.putText(
        annotated_frame,
        f"FPS: {fps:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA
    )

    # Display
    cv2.imshow(WINDOW_NAME, annotated_frame)

    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
