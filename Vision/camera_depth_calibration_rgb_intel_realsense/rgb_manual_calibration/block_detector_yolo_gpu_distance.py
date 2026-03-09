from ultralytics import YOLO
import cv2
import time
import os
import threading
import numpy as np
from collections import deque

# ---------- Settings ----------
CAM_INDEX = 6          
WINDOW_NAME = "YOLO11 - High Confidence Single Detection"
PT_MODEL_PATH = "best.pt"  
OV_MODEL_DIR = "best_openvino_model" 
CALIB_FILE = "d435_720p_setup.yml"  

# Physical height of the block (Update this for exact accuracy!)
OBJECT_REAL_HEIGHT_CM = 5.0  
CONF_CUTOFF = 0.90  # 90% Confidence Threshold

TARGET_WIDTH, TARGET_HEIGHT = 1280, 720
IMG_SIZE = 640           

# ---------- Load Calibration ----------
if os.path.exists(CALIB_FILE):
    fs = cv2.FileStorage(CALIB_FILE, cv2.FileStorage_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    FOCAL_LENGTH_Y = camera_matrix[1, 1] 
    fs.release()
else:
    print(f"[ERROR] {CALIB_FILE} not found!")
    exit()

class CameraStream:
    def __init__(self, index, width, height):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = self.ret
        self.thread = threading.Thread(target=self.update, daemon=True)

    def start(self):
        if self.running: self.thread.start()
        return self

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret: self.frame = frame
            else: time.sleep(0.01)

    def read(self): return self.frame

    def stop(self):
        self.running = False
        self.cap.release()

def prepare_model():
    if not os.path.exists(OV_MODEL_DIR):
        model = YOLO(PT_MODEL_PATH)
        model.export(format="openvino", half=True, imgsz=IMG_SIZE)
    return YOLO(OV_MODEL_DIR, task="detect")

def main():
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    model = prepare_model()
    stream = CameraStream(CAM_INDEX, TARGET_WIDTH, TARGET_HEIGHT).start()
    
    fps_times = deque(maxlen=30)
    prev_time = time.time()

    try:
        while True:
            frame = stream.read()
            if frame is None: continue

            results = model.predict(
                source=frame, imgsz=IMG_SIZE, conf=CONF_CUTOFF, # Apply 90% filter
                device="intel:gpu", half=True, verbose=False
            )

            # --- PROCESS ONLY THE HIGHEST ACCURACY ---
            if results and len(results[0].boxes) > 0:
                # YOLO results are usually sorted by confidence, but we force it to be sure
                boxes = results[0].boxes
                best_box_idx = np.argmax(boxes.conf.cpu().numpy())
                box = boxes[best_box_idx]
                
                conf = float(box.conf)
                
                # Double-check cutoff (though model.predict(conf=0.9) handles it)
                if conf >= CONF_CUTOFF:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = model.names[int(box.cls[0])]

                    # Distance Math
                    pixel_height = y2 - y1
                    distance_cm = (OBJECT_REAL_HEIGHT_CM * FOCAL_LENGTH_Y) / pixel_height if pixel_height > 0 else 0

                    # Draw Overlay
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    info_text = f"BEST: {label} {conf:.2f} | {distance_cm:.1f} cm"
                    
                    # Label Background
                    cv2.rectangle(frame, (x1, y1 - 35), (x1 + 450, y1), (0, 255, 0), -1)
                    cv2.putText(frame, info_text, (x1 + 10, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

            # FPS Display
            now = time.time()
            fps_times.append(1.0 / (now - prev_time))
            prev_time = now
            cv2.putText(frame, f"iGPU FPS: {int(sum(fps_times)/len(fps_times))}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        stream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
