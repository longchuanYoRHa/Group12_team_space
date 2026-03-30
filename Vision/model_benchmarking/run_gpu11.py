import cv2
from ultralytics import YOLO
import time

# 1. 加载刚才导出的 OpenVINO 模型
# 这里的路径一定要对应你截图里生成的那个文件夹名
model_path = "yolo11n_openvino_model/" 

print(f"正在加载模型: {model_path}...")
model = YOLO(model_path)
print("模型加载完成！")

# 2. 打开摄像头 (0 代表第一个 USB 摄像头)
cap = cv2.VideoCapture(4)

# 如果打不开摄像头，尝试把 0 改成 1 或 2，或者检查插拔
if not cap.isOpened():
    print("❌ 无法打开摄像头，请检查连接！")
    exit()

print("🚀 开始推理... (第一次运行需要几秒钟时间编译模型到 GPU，请耐心等待)")

# 用于计算帧率
prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取画面")
        break

    # --- 🔥 核心：调用 iGPU 进行推理 🔥 ---
    # device='GPU' -> 告诉 OpenVINO 使用 Iris Xe 核显
    # verbose=False -> 不打印啰嗦的日志，速度更快
    results = model(frame, verbose=False)
    # ------------------------------------

    # 计算 FPS (每秒帧数)
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time

    # 画框框
    annotated_frame = results[0].plot()

    # 在画面上显示 FPS
    cv2.putText(annotated_frame, f"FPS: {fps:.1f} (Iris Xe GPU)", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 显示画面
    cv2.imshow("YOLO11 OpenVINO Inference", annotated_frame)

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
