import cv2
import time
import numpy as np

MODEL_PATH = 'pingpangDetection/models/best.onnx'
INPUT_SIZE = (320, 320)
CONF_THRES = 0.75

net = cv2.dnn.readNetFromONNX(MODEL_PATH)


def pingpang_detect(img):
    original_h, original_w = img.shape[:2]
    img_resized = cv2.resize(img, INPUT_SIZE, interpolation=cv2.INTER_AREA)
    scale_x = original_w / INPUT_SIZE[0]
    scale_y = original_h / INPUT_SIZE[1]

    blob = cv2.dnn.blobFromImage(img_resized, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward()
    detections = outputs[0].transpose(1, 0)  # (5, 2100) → (2100, 5)

    best_center = None
    best_conf = 0

    for detection in detections:
        x, y, w, h, conf = detection
        if conf < CONF_THRES:
            continue
        x1 = int((x - w / 2) * scale_x)
        y1 = int((y - h / 2) * scale_y)
        x2 = int((x + w / 2) * scale_x)
        y2 = int((y + h / 2) * scale_y)
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        if conf > best_conf:
            best_conf = conf
            best_center = (center_x, center_y)
    return best_center


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame")
            break
        center = pingpang_detect(frame)
        if center:
            print(f"Pingpang center: {center}")
        else:
            print("No pingpang detected.")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()