from ultralytics import YOLO

def train_custom_model():
    # 加载预训练模型
    model = YOLO('yolov8n.pt')
    
    # 开始训练，使用 Roboflow 导出的 data.yaml
    results = model.train(
        data='dataset/data.yaml',  # 使用 Roboflow 导出的 data.yaml
        epochs=100,  # 训练轮数
        imgsz=320,  # 图像大小（与您的图片一致）
        batch=-1,   # 批次大小
        name='custom_model'  # 实验名称
    )
    
    print("训练完成！模型保存在 runs/detect/custom_model/weights/best.pt")
    print("训练和验证集、测试集的划分由 data.yaml 自动适配，无需手动修改。")

if __name__ == "__main__":
    train_custom_model() 