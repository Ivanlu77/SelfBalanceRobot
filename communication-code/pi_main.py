#!/usr/bin/env python3
import socket
import threading
import serial
import time
import sys
import os
from flask import Flask, Response
import cv2
from picamera2 import Picamera2
from libcamera import Transform

current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(current_dir)
sys.path.append(root_dir)
from pingpangDetection.pingpangControl import controller  # Camera 0 


target_speed = "0"
target_yaw = "0"
serial_lock = threading.Lock()
is_following_mode = False
obstacle_avoidance_enabled = False


app = Flask(__name__)
stream_cam = Picamera2(0)


video_config = stream_cam.create_still_configuration(
    main={"format": "RGB888", "size": (3280, 2464)},
    transform=Transform()
)
stream_cam.configure(video_config)
stream_cam.start()


def generate_frames():
    while True:
        frame = stream_cam.capture_array()
        frame = cv2.resize(frame, (640, 480))  
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return '''
    <html style="margin:0; padding:0; overflow:hidden;">
    <body style="margin:0; padding:0; overflow:hidden; background:black;">
        <img src="/video_feed" style="width:100vw; height:100vh; object-fit:cover;">
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_camera_stream():
    print("📷 Camera stream running at http://<pi-ip>:8000")
    app.run(host='0.0.0.0', port=8000, threaded=True)


def setSpeedYaw(data):
    global target_speed, target_yaw
    target_speed, target_yaw = data.split(',', 1)
    return target_speed

def process_command(command, data):
    global target_speed, is_following_mode, obstacle_avoidance_enabled

    if command == 'move':
        try:
            if not is_following_mode:
                setSpeedYaw(data)
            else:
                return "Error: Cannot set speed in following mode"
            return f"Speed set to {data}"
        except ValueError:
            return "Error: Invalid speed value"

    elif command == 'follow':
        is_following_mode = True
        controller.start_following()
        return "Following mode activated"

    elif command == 'stop_follow':
        is_following_mode = False
        controller.stop_following()
        return "Following mode deactivated"

    elif command == 'obstacle_avoidance':
        if data == 'on':
            obstacle_avoidance_enabled = True
            return "Obstacle avoidance enabled"
        elif data == 'off':
            obstacle_avoidance_enabled = False
            return "Obstacle avoidance disabled"
        else:
            return "Error: Invalid obstacle_avoidance value"

    elif command == 'ping':
        return "success"

    elif command == 'echo':
        return data

    return f"Unknown command: {command}"


def handle_client(client_socket, addr):
    try:
        while True:
            message = client_socket.recv(1024).decode('utf-8')
            if not message:
                break
            if '::' in message:
                command, data = message.split('::', 1)
            else:
                command, data = message, ''
            response = process_command(command, data)
            client_socket.send(response.encode('utf-8'))
    except Exception as e:
        print(f"[ERROR] {addr} disconnected: {e}")
    finally:
        client_socket.close()

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except:
        return '0.0.0.0'
    finally:
        s.close()

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', 5000))
    server.listen(5)
    ip = get_local_ip()
    print(f"📡 TCP server running at {ip}:5000")
    print("Obstacle avoidance: OFF")
    print("Following mode: OFF")
    try:
        while True:
            client, addr = server.accept()
            threading.Thread(target=handle_client, args=(client, addr)).start()
    except KeyboardInterrupt:
        print("Server shutting down...")
    finally:
        server.close()


def serial_communication():
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    print("🔌 Serial to ESP32 started.")
    try:
        while True:
            global target_speed, target_yaw
            if is_following_mode:
                target_speed, target_yaw = controller.get_control_values()

            with serial_lock:
                msg = f"{target_speed},{target_yaw}\n".encode('utf-8')
                ser.write(msg)
                print(f"→ ESP32: {msg.strip()}")
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                print(f"← ESP32: {response}")
            time.sleep(0.1)
    except Exception as e:
        print(f"[ERROR] Serial: {e}")
    finally:
        ser.close()


def main():
    threading.Thread(target=start_camera_stream, daemon=True).start()
    threading.Thread(target=start_server, daemon=True).start()
    try:
        serial_communication()
    except KeyboardInterrupt:
        print("Exiting program...")

if __name__ == "__main__":
    main()
