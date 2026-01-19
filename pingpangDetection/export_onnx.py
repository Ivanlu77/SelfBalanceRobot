from ultralytics import YOLO

# 修改为你训练好的模型路径
MODEL_PATH = 'pingpangDetection/models/best.pt'  # 或者 dataset/runs/detect/xxx/weights/best.pt

# 加载模型
model = YOLO(MODEL_PATH)

# 导出为ONNX格式
from ultralytics import YOLO

model.export(format='onnx', simplify=True, opset=11, dynamic=False)

print('模型已导出为ONNX格式！') 