#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import socket
import threading
import queue
import time

UBUNTU_IP = '192.168.1.104'
UBUNTU_PORT = 6666

status_queue = queue.Queue(maxsize=1)

last_lidar_time = time.time()
lidar_active = False
subscriber = None

def socket_worker():
   
    global status_queue
    sock = None
    connected = False

    while not rospy.is_shutdown():
        if not connected:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2)
                sock.connect((UBUNTU_IP, UBUNTU_PORT))
                connected = True
                rospy.loginfo("✅ Connected to Ubuntu socket server.")
            except Exception as e:
                rospy.logwarn("❌ Socket connection failed: %s", e)
                time.sleep(5)
                continue

        try:
            status = status_queue.get(timeout=1)
            sock.sendall((status + "\n").encode())
        except queue.Empty:
            pass
        except Exception as e:
            rospy.logwarn("❌ Socket send error: %s", e)
            connected = False
            if sock:
                sock.close()
                sock = None
            time.sleep(2)

def scan_callback(scan):
    global last_lidar_time
    last_lidar_time = time.time()

    ranges = np.array(scan.ranges)
    ranges[ranges == 0.0] = np.inf

    front = np.concatenate((ranges[0:30], ranges[-30:]))
    rear = ranges[150:210]

    f_danger_distance = 0.55
    r_danger_distance = 0.55
    
    
    front_finite = front[np.isfinite(front)]
    front_dangerous_ratio = np.sum(front_finite < f_danger_distance) / len(front_finite) if len(front_finite) > 0 else 0
    
    
    rear_finite = rear[np.isfinite(rear)]
    rear_dangerous_ratio = np.sum(rear_finite < r_danger_distance) / len(rear_finite) if len(rear_finite) > 0 else 0

    
    if front_dangerous_ratio > 0.3:
        status = "FRONT_OBSTACLE"
    elif rear_dangerous_ratio > 0.3:
        status = "REAR_OBSTACLE"
    else:
        status = "MOVE_FORWARD"

    rospy.loginfo("Status: %s", status)

    try:
        if status_queue.full():
            status_queue.get_nowait()
        status_queue.put_nowait(status)
    except queue.Full:
        pass

def monitor_lidar():
    global last_lidar_time, subscriber
    while not rospy.is_shutdown():
        now = time.time()
        if now - last_lidar_time > 5:
            rospy.logwarn("⚠️ No LiDAR data for 5s, attempting to resubscribe...")
            try:
                subscriber.unregister()
                time.sleep(1)
                subscriber = rospy.Subscriber("/LiDAR/LD06", LaserScan, scan_callback)
                rospy.loginfo("✅ Resubscribed to LiDAR topic.")
                last_lidar_time = time.time()  
            except Exception as e:
                rospy.logwarn("❌ Failed to resubscribe: %s", e)
        time.sleep(3)

def main():
    global subscriber
    rospy.init_node('laser_obstacle_avoider', anonymous=True)

    threading.Thread(target=socket_worker, daemon=True).start()
    threading.Thread(target=monitor_lidar, daemon=True).start()

    subscriber = rospy.Subscriber("/LiDAR/LD06", LaserScan, scan_callback)
    rospy.loginfo("✅ Laser obstacle avoidance node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
