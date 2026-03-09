from ultralytics import YOLO
import cv2
import time
import os
import threading
from collections import deque

# ---------- Settings ----------
CAM_INDEX = 6

WINDOW_NAME = "YOLO11 - Optimized Intel iGPU"
PT_MODEL_PATH = "best.pt"  # Your original PyTorch model
OV_MODEL_DIR = "best_openvino_model" # Exported version folder

IMG_SIZE = 640           
CONF_THRES = 0.50        
TARGET_WIDTH = 1280     
TARGET_HEIGHT = 720

class CameraStream:
    def __init__(self, index, width, height):
        # V4L2 is standard for Linux stability with MJPG
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Keep buffer minimal for real-time
        
        self.ret, self.frame = self.cap.read()
        self.running = self.ret
        if not self.ret:
            print(f"[ERROR] Camera {index} failed.")
        else:
            self.thread = threading.Thread(target=self.update, daemon=True)

    def start(self):
        if self.running: self.thread.start()
        return self

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret: self.frame = frame
            else: time.sleep(0.01)

    def read(self):
        return self.frame

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'): self.thread.join(timeout=1.0)
        self.cap.release()

def prepare_model():
    """Ensures the model is exported to OpenVINO format for iGPU speed."""
    if not os.path.exists(OV_MODEL_DIR):
        print(f"Exporting {PT_MODEL_PATH} to OpenVINO...")
        model = YOLO(PT_MODEL_PATH)
        # half=True (FP16) is essential for iGPU performance
        model.export(format="openvino", half=True, imgsz=IMG_SIZE)
        print("Export complete.")
    
    # Load the optimized OpenVINO folder
    return YOLO(OV_MODEL_DIR, task="detect")

def main():
    os.environ["QT_QPA_PLATFORM"] = "xcb" # Prevent Wayland GUI crashes
    
    try:
        model = prepare_model()
    except Exception as e:
        print(f"[ERROR] Model setup failed: {e}")
        return

    stream = CameraStream(CAM_INDEX, TARGET_WIDTH, TARGET_HEIGHT).start()
    if not stream.running: return

    fps_times = deque(maxlen=30)
    prev_time = time.time()

    print("Running inference on Intel iGPU...")

    try:
        while True:
            frame = stream.read()
            if frame is None: continue

            # 'intel:gpu' specifically targets the integrated graphics
            results = model.predict(
                source=frame,
                imgsz=IMG_SIZE,
                conf=CONF_THRES,
                device="intel:gpu", 
                half=True, 
                verbose=False,
                stream=False
            )

            # Draw results on the frame
            annotated_frame = results[0].plot() if results else frame

            # FPS Calculation
            now = time.time()
            fps_times.append(1.0 / (now - prev_time))
            prev_time = now
            avg_fps = sum(fps_times) / len(fps_times)

            cv2.putText(annotated_frame, f"iGPU FPS: {int(avg_fps)}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow(WINDOW_NAME, annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        stream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
