import cv2
import time
import numpy as np

MODEL_PATH = 'pingpangDetection/models/best1.onnx'
INPUT_SIZE = (320, 320)
CONF_THRES = 0.75

def pingpang_detect(img):
    original_h, original_w = img.shape[:2]
    img_resized = cv2.resize(img, INPUT_SIZE, interpolation=cv2.INTER_AREA)
    scale_x = original_w / INPUT_SIZE[0]
    scale_y = original_h / INPUT_SIZE[1]
    blob = cv2.dnn.blobFromImage(img_resized, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward()
    detections = outputs[0].transpose(1, 0)
    best_center = None
    best_area = 0
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
            best_area = w * h
    return best_center, best_area

net = cv2.dnn.readNetFromONNX(MODEL_PATH)

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot open camera")
        return
    TARGET_AREA = 4000
    FRAME_CENTER_X = 320
    turn_last_error = 0
    speed_last_error = 0
    turn_kp = 0.3
    turn_kd = 0.1
    speed_kp = 6/2000
    speed_kd = 2/2000
    turn_history = []
    speed_history = []
    filter_window = 5
    # not detected pingpang
    last_turn_filtered = 0.0
    last_speed_filtered = 0.0
    decay = 0.9
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame")
            break
        center, area = pingpang_detect(frame)
        if center:
            position_error = center[0] - FRAME_CENTER_X
            area_error = TARGET_AREA - area
            # turn control
            turn_derivative = position_error - turn_last_error
            turn = - (turn_kp * position_error + turn_kd * turn_derivative) / 100
            turn_last_error = position_error
            turn = np.clip(turn, -0.25, 0.25)
            # filter
            if not turn_history:
                turn_filtered = turn
            else:
                turn_filtered = (turn_history[-1] * (filter_window - 1) + turn) / filter_window
            turn_history.append(turn_filtered)
            if len(turn_history) > filter_window:
                turn_history.pop(0)
            # speed control
            speed_derivative = area_error - speed_last_error
            speed = - (speed_kp * area_error + speed_kd * speed_derivative)
            speed_last_error = area_error
            speed = np.clip(speed, -10, 10)
            # filter
            if not speed_history:
                speed_filtered = speed
            else:
                speed_filtered = (speed_history[-1] * (filter_window - 1) + speed) / filter_window
            speed_history.append(speed_filtered)
            if len(speed_history) > filter_window:
                speed_history.pop(0)
            last_turn_filtered = turn_filtered
            last_speed_filtered = speed_filtered
            cv2.circle(frame, center, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Center: {center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Area: {area}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Speed: {speed_filtered:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn: {turn_filtered:.2f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print(f"Position error: {position_error}, Area error: {area_error}")
            print(f"Speed control: {speed_filtered:.2f}, Turn control: {turn_filtered:.2f}")
        cv2.imshow("Pingpang Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()