import cv2
import time
import numpy as np
from picamera2 import Picamera2
import os

class PingpangController:
    def __init__(self):
        self.TARGET_AREA = 600
        self.FRAME_CENTER_X= 160
        self.frame_time = 1/10
        self.is_following = False
        self.picam2 = None
        self.net = None
        self.turn_last_error = 0
        self.speed_last_error = 0
        self.turn_kp = 0.4
        self.turn_kd = 0.1
        self.speed_kp = 3/380
        self.speed_kd = 0.5/380
        self.turn_history = []
        self.speed_history = []
        self.filter_window = 5
        self.position_deadzone = 6
        self.area_deadzone = 80
        self.setup_camera()
        self.setup_model()

    def setup_camera(self):
        self.picam2 = Picamera2(1)
        config = self.picam2.create_preview_configuration(
            main={"size": (320, 320), "format": "BGR888"},
            raw={"size": self.picam2.sensor_modes[-1]["size"]}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)

    def setup_model(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        MODEL_PATH = os.path.join(current_dir, 'models', 'best.onnx')
        if not os.path.exists(MODEL_PATH):
            raise FileNotFoundError(f"Model file not found at: {MODEL_PATH}")
        self.net = cv2.dnn.readNetFromONNX(MODEL_PATH)

    def start_following(self):
        self.is_following = True

    def stop_following(self):
        self.is_following = False

    def get_control_values(self):
        
        global speed_stop_counter
        
        if not self.is_following:
            return "0", "0"
        try:
            speed = 0
            turn = 0
            frame = self.picam2.capture_array()
            if frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            center, area = self.pingpang_detect(frame)
            if center:
                position_error = center[0] - self.FRAME_CENTER_X
                area_error = self.TARGET_AREA - area
                # deadzone for position and area
                if abs(position_error) <= self.position_deadzone:
                    position_error = 0     
                if abs(area_error) <= self.area_deadzone:
                    speed = 0
                if abs(position_error) > self.position_deadzone:
                    # turn control
                    turn_derivative = position_error - self.turn_last_error
                    turn = - (self.turn_kp * position_error + self.turn_kd * turn_derivative)
                    self.turn_last_error = position_error
                    turn = np.clip(turn, -0.8, 0.8)
                if abs(area_error) > self.area_deadzone:
                    # speed control
                    if area_error > 400 :
                        speed = 12
                    elif area_error > 200 :
                        speed = 8
                    elif area_error > 50 :
                        speed = 6
                    elif area_error > -50 :
                        speed = 0
                    elif area_error > -200 :
                        speed = -6
                    elif area_error > -400 :
                        speed = -8
                    else: speed = -12
                    speed = np.clip(speed, -12, 12)
                # filter
                if not self.turn_history:
                    turn_filtered = turn
                else:
                    turn_filtered = (self.turn_history[-1] * (self.filter_window - 1) + turn) / self.filter_window
                self.turn_history.append(turn_filtered)
            if len(self.turn_history) > self.filter_window:
                self.turn_history.pop(0)
            if speed == 0 :
                speed_stop_counter = speed_stop_counter + 1
            else : speed_stop_counter = 0
            if not self.speed_history:
                speed_filtered = speed
            else:
                speed_filtered = (self.speed_history[-1] * (self.filter_window - 1) + speed) / self.filter_window
            if speed_stop_counter > 3 :
                speed_filtered = 0
            self.speed_history.append(speed_filtered)
            if len(self.speed_history) > self.filter_window:
                self.speed_history.pop(0)
            return str(speed_filtered), str(turn_filtered)
        except Exception:
            return "0", "0"

    def pingpang_detect(self, img):
        INPUT_SIZE = (320, 320)
        CONF_THRES = 0.75
        blob = cv2.dnn.blobFromImage(img, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward()
        detections = np.squeeze(outputs, axis=0).T
        best_center = None
        best_area = 0
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
                best_area = w * h
        return best_center, best_area

controller = PingpangController()
