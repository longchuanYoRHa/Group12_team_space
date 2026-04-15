import torch
import torchvision
import torch.nn as nn

# 1. 定义 Wrapper 类 (保持不变，解决字典输出问题)
class TraceableFasterRCNN(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, x):
        if isinstance(x, torch.Tensor):
            images = [x[0]] 
        else:
            images = x
        
        predictions = self.model(images)
        pred = predictions[0]
        
        # 拆解字典为 Tuple
        return pred['boxes'], pred['labels'], pred['scores']

# 2. 加载和包装模型
print("正在加载 Faster R-CNN...")
original_model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights='DEFAULT')
original_model.eval()
wrapped_model = TraceableFasterRCNN(original_model)

# 3. 创建虚拟输入
# 注意：这个尺寸(640x640)将成为模型的固定输入尺寸
dummy_input = torch.randn(1, 3, 640, 640)

# 4. 手动执行 JIT Trace
print("正在强制执行 JIT 追踪...")
try:
    # strict=False 忽略非关键警告
    traced_model = torch.jit.trace(wrapped_model, dummy_input, strict=False)
    print("✅ JIT 追踪成功！")
except Exception as e:
    print(f"❌ JIT 追踪失败: {e}")
    exit()

# 5. 导出为 ONNX
print("正在导出为 ONNX...")
torch.onnx.export(
    traced_model,
    dummy_input,
    "faster_rcnn.onnx",
    opset_version=11,
    input_names=['input'],
    output_names=['boxes', 'labels', 'scores'],
    # 🛑 关键修改：删除了 dynamic_axes
    # 在 Python 3.12 + PyTorch 2.x 中，对 ScriptModule 使用动态轴会导致崩溃
    # 删除后模型将固定为 1x3x640x640，但这能保证导出成功！
    do_constant_folding=True
)

print("🎉 导出完成！文件名为: faster_rcnn.onnx")
