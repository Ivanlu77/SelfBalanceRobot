import cv2
import time
import numpy as np
from picamera2 import Picamera2

MODEL_PATH = 'models/best.onnx'
INPUT_SIZE = (320, 320)
CONF_THRES = 0.75

net = cv2.dnn.readNetFromONNX(MODEL_PATH)

def pingpang_detect(img):
    blob = cv2.dnn.blobFromImage(img, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward()

    detections = np.squeeze(outputs, axis=0).T  # shape: (2100, 5)

    best_center = None
    best_conf = 0

    for det in detections:
        x, y, w, h, conf = det
        if conf < CONF_THRES:
            continue
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        if conf > best_conf:
            best_conf = conf
            best_center = (center_x, center_y)

    return best_center, w, h


def main():
    frame_time = 1 / 5
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (320, 320),"format": "BGR888"},
        raw={"size": picam2.sensor_modes[-1]["size"]}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    while True:
        start_time = time.time()
        try:
            frame = picam2.capture_array()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            center, w, h = pingpang_detect(frame)
            if center:
                area = w*h
                print(f"Tennis center: {center}")
                print(f"Tennis area: {area}" )
            else:
                print("No tennis detected")
        except Exception as e:
            print(f"Frame error: {e}")

        t = time.time() - start_time
        if t < frame_time:
            time.sleep(frame_time - t)

if __name__ == "__main__":
    main()
