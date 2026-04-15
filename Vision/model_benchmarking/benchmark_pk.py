import time
import cv2
import numpy as np
from ultralytics import YOLO
import psutil 
import os

# === 1. 配置区域 ===
# 请确保这三个文件夹都在当前目录下
path_v8 = 'yolov8n_openvino_model'
path_v11 = 'yolo11n_openvino_model'
path_rtdetr = 'rtdetr-l_openvino_model' 

# 测试用的图片 (如果网络不好下载失败，请换成本地图片路径，比如 'bus.jpg')
test_image = 'https://ultralytics.com/images/bus.jpg'

# 循环次数：因为 RT-DETR 很慢，建议设为 50，否则要等很久
loops = 50 

# === 2. 核心测试函数 (之前缺失的部分) ===
def benchmark_model(model_name, model_path):
    print(f"\n⚡ 正在测试: {model_name} ...")
    
    # 检查路径是否存在
    if not os.path.exists(model_path):
        print(f"❌ 错误: 找不到路径 '{model_path}'，跳过此模型。")
        return None

    # 加载模型
    try:
        model = YOLO(model_path, task='detect')
    except Exception as e:
        print(f"❌ 加载失败: {e}")
        return None
    
    # 预热 (Warmup)
    print("   预热中 (10次推理)...")
    try:
        for _ in range(10):
            model(test_image, verbose=False)
    except Exception as e:
        print(f"❌ 预热失败 (可能是网络下载图片超时): {e}")
        return None
        
    # 正式测速
    print(f"   开始跑 {loops} 圈...")
    start_time = time.time()
    cpu_usage_start = psutil.cpu_percent(interval=None)
    
    scores = []
    
    for _ in range(loops):
        results = model(test_image, verbose=False)
        # 获取第一张图的最高置信度
        if len(results[0].boxes) > 0:
            scores.append(float(results[0].boxes.conf[0]))
            
    end_time = time.time()
    cpu_usage_end = psutil.cpu_percent(interval=None)
    
    # 计算指标
    total_time = end_time - start_time
    fps = loops / total_time
    latency = (total_time / loops) * 1000
    avg_conf = sum(scores) / len(scores) if scores else 0
    
    print(f"   ✅ {model_name} 结果:")
    print(f"      FPS: {fps:.2f}")
    print(f"      延迟: {latency:.2f} ms")
    print(f"      平均置信度: {avg_conf:.2f}")
    
    return fps, latency, avg_conf

# === 3. 主程序入口 ===
if __name__ == '__main__':
    print("=== 🚀 巅峰对决: CNN (YOLO) vs Transformer (RT-DETR) ===")
    print(f"测试设备: Intel NUC (OpenVINO加速)")
    
    # 运行对比
    res_v8 = benchmark_model("YOLOv8 Nano", path_v8)
    res_v11 = benchmark_model("YOLO11 Nano", path_v11)
    res_rt = benchmark_model("RT-DETR Large", path_rtdetr)
    
    # 打印最终对比表
    print("\n" + "="*65)
    print(f"{'模型 (Model)':<15} | {'架构':<12} | {'FPS':<8} | {'延迟(ms)':<10} | {'置信度':<8}")
    print("-" * 65)
    
    if res_v8:
        print(f"{'YOLOv8n':<15} | {'CNN':<12} | {res_v8[0]:<8.2f} | {res_v8[1]:<10.2f} | {res_v8[2]:<8.2f}")
    
    if res_v11:
        print(f"{'YOLO11n':<15} | {'CNN':<12} | {res_v11[0]:<8.2f} | {res_v11[1]:<10.2f} | {res_v11[2]:<8.2f}")
        
    if res_rt:
        print(f"{'RT-DETR-l':<15} | {'Transformer':<12} | {res_rt[0]:<8.2f} | {res_rt[1]:<10.2f} | {res_rt[2]:<8.2f}")
        
    print("="*65)
