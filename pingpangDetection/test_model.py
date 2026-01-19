from ultralytics import YOLO
import cv2

# 修改为你训练好的模型路径
MODEL_PATH = 'pingpangDetection/models/best.pt'  # 或者 dataset/runs/detect/xxx/weights/best.pt

# 初始化模型
model = YOLO(MODEL_PATH)

# 打开摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

TARGET_SIZE = (320, 240)  # 目标分辨率
CONF_THRES = 0.75  # 只显示置信度大于0.5的目标

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法获取画面")
        break

    # 缩放图像到目标分辨率
    resized_frame = cv2.resize(frame, TARGET_SIZE, interpolation=cv2.INTER_AREA)

    # 检测
    results = model(resized_frame)
    annotated_frame = results[0].plot()

    # 显示检测结果
    cv2.imshow("检测效果", annotated_frame)

    # 打印检测到的目标位置和置信度
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            confidence = box.conf[0]
            if confidence < CONF_THRES:
                continue
            print(f"检测到目标: x1={x1:.0f}, y1={y1:.0f}, x2={x2:.0f}, y2={y2:.0f}, 置信度={confidence:.2f}")

    # 按q退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows() 