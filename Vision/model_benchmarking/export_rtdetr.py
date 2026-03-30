from ultralytics import YOLO

# 1. 下载并加载 RT-DETR Large 模型
print("正在加载 RT-DETR-l ...")
model = YOLO('rtdetr-l.pt') 

# 2. 导出为 OpenVINO 格式
# 🛑 关键修改：改为 False (使用 FP32 全精度)
# Transformer 架构对精度非常敏感，FP16 容易导致数值溢出，从而检测不到任何物体。
print("正在导出为 OpenVINO (FP32 全精度)...")
model.export(format='openvino', half=False)

print("导出完成！请运行测试脚本。")
