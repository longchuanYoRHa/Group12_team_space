import cv2
import numpy as np
import openvino.runtime as ov
import time

# === 1. 配置 ===
MODEL_PATH = "faster_rcnn_openvino_model.xml"
CONF_THRESHOLD = 0.5  # 置信度阈值

# COCO 91类 (Torchvision 的索引 1 是 Person，0 是背景)
COCO_CLASSES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

# === 2. 初始化 OpenVINO ===
print("加载模型中...")
core = ov.Core()
model = core.read_model(MODEL_PATH)
compiled_model = core.compile_model(model, "AUTO")
infer_request = compiled_model.create_infer_request()

# 获取输入层信息 (用于自动调整尺寸)
input_layer = compiled_model.input(0)
n, c, h, w = input_layer.shape

# 如果模型是动态输入，手动指定一个尺寸
if h == -1 or w == -1:
    h, w = 640, 640

print(f"模型输入尺寸: {w}x{h}")

# === 3. 打开摄像头 ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ 无法打开摄像头")
    exit()

print("🚀 开始推理... (Faster R-CNN 会比较慢)")

while True:
    ret, frame = cap.read()
    if not ret: break

    start_time = time.time()

    # === 预处理 ===
    # Resize -> 归一化 (0-1) -> 转为 NCHW -> 增加 Batch 维度
    resized_img = cv2.resize(frame, (w, h))
    input_tensor = resized_img.astype(np.float32) / 255.0
    input_tensor = input_tensor.transpose(2, 0, 1) # HWC -> CHW
    input_tensor = np.expand_dims(input_tensor, 0) # -> 1CHW

    # === 推理 ===
    # Faster R-CNN 输出通常包含三个 Tensor: boxes, labels, scores
    results = infer_request.infer({0: input_tensor})

    # 从结果中提取数据 (注意：OpenVINO 输出顺序可能变化，最好通过名称获取)
    # 这里假设输出名称就是导出时的 names，如果报错需要打印 results.keys() 查看
    # 通常是: 'boxes', 'labels', 'scores'
    
    # 获取输出 Tensor (这里尝试按名称获取，如果导出时没指定名称，可能是 output0, output1...)
    # 我们可以通过遍历 results 来找到对应的 shape
    
    boxes = None
    scores = None
    labels = None

    for output_tensor, data in results.items():
        # 这里用简单的逻辑判断哪个是哪个
        # boxes 的 shape 通常是 (N, 4)
        # scores 的 shape 通常是 (N,) 且是浮点数
        # labels 的 shape 通常是 (N,) 且是整数
        
        if len(data.shape) == 2 and data.shape[1] == 4:
            boxes = data
        elif len(data.shape) == 1:
            # 区分 scores 和 labels
            if data.dtype == np.float32 or data.dtype == np.float64:
                scores = data
            else:
                labels = data
    
    # === 后处理与画框 ===
    if boxes is not None and scores is not None and labels is not None:
        # 将 box 坐标映射回原图尺寸
        scale_x = frame.shape[1] / w
        scale_y = frame.shape[0] / h

        for i in range(len(scores)):
            score = scores[i]
            if score > CONF_THRESHOLD:
                label_idx = int(labels[i])
                box = boxes[i]
                
                # 还原坐标
                x1 = int(box[0] * scale_x)
                y1 = int(box[1] * scale_y)
                x2 = int(box[2] * scale_x)
                y2 = int(box[3] * scale_y)

                # 获取类别名
                class_name = COCO_CLASSES[label_idx] if label_idx < len(COCO_CLASSES) else f"ID {label_idx}"
                
                # 画框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"{class_name} {score:.2f}", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # 计算 FPS
    end_time = time.time()
    fps = 1 / (end_time - start_time)
    cv2.putText(frame, f"Faster R-CNN FPS: {fps:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("Faster R-CNN OpenVINO", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
