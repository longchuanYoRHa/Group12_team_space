
from ultralytics import YOLO
import cv2
import time
from collections import deque
import sys

# ---------- Settings ----------
CAM_INDEX = 0            # default webcam
WINDOW_NAME = "YOLO11 Real-Time (with FPS)"
MODEL_PATH = "yolo11n.pt"  # nano model for speed (correct name; no 'v')
IMG_SIZE = 640           # inference size (reduce for more speed, e.g., 480 or 416)
CONF_THRES = 0.25        # confidence threshold for drawing
FPS_SMOOTH_N = 30        # moving average window for FPS display
TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080
# ------------------------------


def open_camera(index=0, width=1920, height=1080, fps=30):
    """
    Try to open a webcam and request given resolution/fps.
    Returns an opened cv2.VideoCapture or None.
    """
    # Try default backend first; on Linux OpenCV uses V4L2 automatically
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None

    # Request resolution and fps (may not be guaranteed)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Check what we actually got
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[Camera] Requested {width}x{height}@{fps}, got {got_w}x{got_h}@{got_fps:.1f}")

    # Test a read
    ret, frame = cap.read()
    if not ret or frame is None:
        cap.release()
        return None

    return cap


def main():
    # ---- Load YOLO11 nano model (auto-downloads if not found) ----
    try:
        model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"[Error] Could not load model '{MODEL_PATH}': {e}")
        print("Tip: ensure 'ultralytics' is up to date: pip install -U ultralytics")
        sys.exit(1)

    # ---- Open webcam at 1080p. If it fails, print and exit cleanly ----
    cap = open_camera(index=CAM_INDEX, width=TARGET_WIDTH, height=TARGET_HEIGHT, fps=30)
    if cap is None:
        print("[Error] Failed to open camera at 1920x1080. "
              "Try a lower resolution or check camera permissions/devices.")
        sys.exit(1)

    # FPS tracking
    fps_times = deque(maxlen=FPS_SMOOTH_N)
    prev_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[Error] Failed to read from camera.")
                break

            # Measure time since last frame
            now = time.time()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps_times.append(1.0 / dt)

            # Run YOLO inference on the frame (CPU-only here)
            results = model.predict(
                source=frame,
                imgsz=IMG_SIZE,
                conf=CONF_THRES,
                device="cpu",   # force CPU
                half=False,     # half precision is GPU-only
                verbose=False
            )

            # Draw detections
            annotated_frame = results[0].plot()

            # Compute smoothed FPS
            fps = (sum(fps_times) / len(fps_times)) if fps_times else 0.0

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

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

