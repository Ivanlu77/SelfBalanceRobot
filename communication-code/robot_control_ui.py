# -*- coding: utf-8 -*-
import sys
import socket
import time
import threading
import subprocess
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                            QFrame, QGridLayout, QGroupBox, QSizePolicy, QCheckBox,
                            QGraphicsView, QGraphicsScene, QGraphicsProxyWidget, QSlider)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, QPropertyAnimation, QEasingCurve, QRect, QUrl
from PyQt6.QtGui import QPainter, QColor, QPen, QFont, QTransform
from PyQt6.QtWebEngineWidgets import QWebEngineView
import pygame

class AnimatedLabel(QLabel):
    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self.animation = QPropertyAnimation(self, b"geometry")
        self.animation.setDuration(300)
        self.animation.setEasingCurve(QEasingCurve.Type.OutCubic)
        
    def animate_pulse(self):
        current_rect = self.geometry()
        self.animation.setStartValue(current_rect)
        self.animation.setEndValue(current_rect)
        self.animation.start()

class StatusIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(16, 16)
        self.connected = False
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.update)
        self.blink_state = True
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        if self.connected:
            color = QColor("#4CAF50")
        else:
            # Blinking red for disconnected
            if self.blink_state:
                color = QColor("#F44336")
            else:
                color = QColor("#FF8A80")
        
        painter.setBrush(color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(2, 2, 12, 12)
    
    def set_connected(self, connected):
        self.connected = connected
        if connected:
            self.blink_timer.stop()
            self.blink_state = True
        else:
            self.blink_timer.start(500)  # Blink every 500ms
        self.update()

class JoystickWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(80, 80)
        self.x = 0
        self.y = 0
        self.setStyleSheet("background-color: transparent;")
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # Draw background circle (smaller)
        painter.setPen(QPen(QColor("#404040"), 2))
        painter.setBrush(QColor("#2b2b2b"))
        painter.drawEllipse(12, 12, self.width()-24, self.height()-24)
        
        # Draw subtle crosshair
        painter.setPen(QPen(QColor("#64B5F6"), 1))
        painter.drawLine(center_x - 5, center_y, center_x + 5, center_y)
        painter.drawLine(center_x, center_y - 5, center_x, center_y + 5)
        
        # Draw joystick position as a glowing dot
        knob_x = center_x + int(self.x * 22)  # Reduced from 30 to 22
        knob_y = center_y + int(self.y * 22)
        
        # Glow effect
        painter.setPen(QPen(QColor("#FF5722"), 2))
        painter.setBrush(QColor("#FF5722"))
        painter.drawEllipse(knob_x-5, knob_y-5, 10, 10)
        
        # Inner bright core
        painter.setBrush(QColor("#FFC107"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(knob_x-2, knob_y-2, 4, 4)

    def update_position(self, x, y):
        self.x = max(-1, min(1, x))
        self.y = max(-1, min(1, y))
        self.update()

class JoystickThread(QThread):
    joystick_update = pyqtSignal(float, float, float, float)
    button_pressed = pyqtSignal(str)  # Add signal for button presses
    dpad_pressed = pyqtSignal(str)  # Add signal for D-pad presses
    
    def __init__(self):
        super().__init__()
        pygame.init()
        pygame.joystick.init()
        self.running = True
        self.prev_button_states = {}  # Track previous button states for edge detection
        self.prev_hat_states = {}  # Track previous hat (D-pad) states
        
    def run(self):
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            
            # Initialize button states
            for i in range(joystick.get_numbuttons()):
                self.prev_button_states[i] = False
                
            # Initialize hat (D-pad) states
            for i in range(joystick.get_numhats()):
                self.prev_hat_states[i] = (0, 0)
            
            while self.running:
                pygame.event.pump()
                left_x = joystick.get_axis(0)
                left_y = joystick.get_axis(1)
                right_x = joystick.get_axis(2)
                right_y = joystick.get_axis(3)
                self.joystick_update.emit(left_x, left_y, right_x, right_y)
                
                # Check button presses (only emit on button press, not hold)
                for i in range(joystick.get_numbuttons()):
                    current_state = joystick.get_button(i)
                    prev_state = self.prev_button_states.get(i, False)
                    
                    # Detect button press (transition from False to True)
                    if current_state and not prev_state:
                        button_name = self.get_button_name(i)
                        print(f"Button pressed: {button_name} (ID: {i})")
                        self.button_pressed.emit(button_name)
                    
                    self.prev_button_states[i] = current_state
                
                # Check D-pad (hat) presses
                for i in range(joystick.get_numhats()):
                    current_hat = joystick.get_hat(i)
                    prev_hat = self.prev_hat_states.get(i, (0, 0))
                    
                    # Detect D-pad press (transition to non-zero state)
                    if current_hat != prev_hat and current_hat != (0, 0):
                        dpad_direction = self.get_dpad_direction(current_hat)
                        print(f"D-pad pressed: {dpad_direction}")
                        self.dpad_pressed.emit(dpad_direction)
                    
                    self.prev_hat_states[i] = current_hat
                
                time.sleep(0.02)  # 50Hz update rate
    
    def get_button_name(self, button_id):
        """Map button ID to button name (Xbox controller layout)"""
        button_map = {
            0: "A",
            1: "B", 
            2: "X",
            3: "Y",
            4: "LB",
            5: "RB",
            6: "SELECT",
            7: "START",
            8: "LS",  # Left stick press
            9: "RS"   # Right stick press
        }
        return button_map.get(button_id, f"BUTTON_{button_id}")
    
    def get_dpad_direction(self, hat_value):
        """Map hat (D-pad) value to direction name"""
        x, y = hat_value
        if y == 1:
            return "DPAD_UP"
        elif y == -1:
            return "DPAD_DOWN"
        elif x == 1:
            return "DPAD_RIGHT"
        elif x == -1:
            return "DPAD_LEFT"
        return "DPAD_UNKNOWN"
                
    def stop(self):
        self.running = False
        self.wait()

class ObstacleStatusServer(QThread):
    obstacle_update = pyqtSignal(str)
    
    def __init__(self, host='0.0.0.0', port=6666):
        super().__init__()
        self.host = host
        self.port = port
        self.running = True
        self.latest_status = "UNKNOWN"
        self.lock = threading.Lock()

    def run(self):
        print(f"[OBSTACLE SERVER] Starting obstacle status listener on {self.host}:{self.port}...")
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((self.host, self.port))
                s.listen()
                s.settimeout(1.0)  # Add timeout for checking self.running
                
                while self.running:
                    try:
                        conn, addr = s.accept()
                        threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True).start()
                    except socket.timeout:
                        continue  # Check self.running again
                    except Exception as e:
                        if self.running:  # Only log if we're still supposed to be running
                            print(f"[OBSTACLE ERROR] Connection error: {e}")
        except Exception as e:
            print(f"[OBSTACLE ERROR] Server error: {e}")

    def handle_client(self, conn, addr):
        print(f"[OBSTACLE] Connected from {addr}")
        with conn:
            while self.running:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    message = data.decode().strip()
                    print(f"[OBSTACLE] Received: {message}")
                    
                    with self.lock:
                        self.latest_status = message
                        self.obstacle_update.emit(message)
                        
                except Exception as e:
                    print(f"[OBSTACLE ERROR] {e}")
                    break

    def get_status(self):
        with self.lock:
            return self.latest_status

    def stop(self):
        self.running = False

class SimplePositionIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(350, 280)  # Changed to rectangular: wider but shorter
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_angle = 0.0
        self.connected = False
        self.position_history = []  # Store position history for trail
        # Remove max_history limit - trail will never disappear
        self.auto_scale = True  # Enable auto-scaling
        self.min_scale = 1.0  # Minimum scale (1 meter radius)
        self.current_scale = 5.0  # Current scale (starts at 5 meter radius)
        self.setStyleSheet("background: transparent;")
        
    def calculate_auto_scale(self):
        """Calculate optimal scale based on trajectory bounds"""
        if len(self.position_history) < 2:
            return self.current_scale
        
        # Find bounds of all trajectory points
        x_coords = [pos[0] for pos in self.position_history] + [self.robot_x]
        y_coords = [pos[1] for pos in self.position_history] + [self.robot_y]
        
        if not x_coords or not y_coords:
            return self.current_scale
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Calculate required radius to fit all points
        max_distance = max(
            abs(min_x), abs(max_x),
            abs(min_y), abs(max_y),
            ((max_x - min_x) / 2), ((max_y - min_y) / 2)
        )
        
        # Add 20% padding
        required_scale = max_distance * 1.2
        
        # Ensure minimum scale
        required_scale = max(required_scale, self.min_scale)
        
        return required_scale
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        center_x = self.width() // 2
        center_y = self.height() // 2
        radius = min(self.width(), self.height()) // 2 - 20
        
        # Auto-scale based on trajectory
        if self.auto_scale:
            self.current_scale = self.calculate_auto_scale()
        
        scale = radius / self.current_scale
        
        # Draw outer circle (boundary)
        painter.setPen(QPen(QColor("#404040"), 2))
        painter.setBrush(QColor("#2b2b2b"))
        painter.drawEllipse(int(center_x - radius), int(center_y - radius), int(radius * 2), int(radius * 2))
        
        # Draw scale rings
        painter.setPen(QPen(QColor("#404040"), 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        ring_count = 4
        for i in range(1, ring_count):
            ring_radius = radius * i / ring_count
            painter.drawEllipse(int(center_x - ring_radius), int(center_y - ring_radius), 
                              int(ring_radius * 2), int(ring_radius * 2))
        
        # Draw grid lines
        painter.setPen(QPen(QColor("#64B5F6"), 1))
        painter.drawLine(center_x - radius, center_y, center_x + radius, center_y)  # Horizontal
        painter.drawLine(center_x, center_y - radius, center_x, center_y + radius)  # Vertical
        
        # Draw origin point
        painter.setPen(QPen(QColor("#64B5F6"), 2))
        painter.setBrush(QColor("#64B5F6"))
        painter.drawEllipse(int(center_x - 3), int(center_y - 3), 6, 6)
        
        if self.connected:
            # Draw trail/trajectory - all points, no limit
            if len(self.position_history) > 1:
                painter.setPen(QPen(QColor("#4CAF50"), 2))  # Green trail
                painter.setBrush(Qt.BrushStyle.NoBrush)
                
                # Draw lines connecting all trail points
                for i in range(1, len(self.position_history)):
                    prev_x, prev_y = self.position_history[i-1]
                    curr_x, curr_y = self.position_history[i]
                    
                    # Convert to screen coordinates
                    prev_screen_x = center_x + (prev_x * scale)
                    prev_screen_y = center_y + (prev_y * scale)
                    curr_screen_x = center_x + (curr_x * scale)
                    curr_screen_y = center_y + (curr_y * scale)
                    
                    # Only draw if both points are visible (with some margin)
                    margin = 50
                    if (abs(prev_screen_x - center_x) < radius + margin and 
                        abs(prev_screen_y - center_y) < radius + margin and
                        abs(curr_screen_x - center_x) < radius + margin and 
                        abs(curr_screen_y - center_y) < radius + margin):
                        painter.drawLine(int(prev_screen_x), int(prev_screen_y), 
                                       int(curr_screen_x), int(curr_screen_y))
                
                # Draw trail points as small dots (sample for performance)
                painter.setPen(QPen(QColor("#4CAF50"), 1))
                painter.setBrush(QColor("#4CAF50"))
                # Only draw every nth point if we have many points
                step = max(1, len(self.position_history) // 500)  # Limit to ~500 visible dots
                for i in range(0, len(self.position_history) - 1, step):
                    x, y = self.position_history[i]
                    screen_x = center_x + (x * scale)
                    screen_y = center_y + (y * scale)
                    
                    # Only draw dots within view
                    if (abs(screen_x - center_x) < radius + 20 and 
                        abs(screen_y - center_y) < radius + 20):
                        painter.drawEllipse(int(screen_x - 1), int(screen_y - 1), 2, 2)
            
            # Calculate robot position on the circle (scaled)
            robot_screen_x = center_x + (self.robot_x * scale)
            robot_screen_y = center_y + (self.robot_y * scale)
            
            # Draw robot as a triangle pointing in the direction
            painter.setPen(QPen(QColor("#FF5722"), 2))
            painter.setBrush(QColor("#FF5722"))
            
            # Calculate triangle points based on robot angle
            import math
            angle_rad = math.radians(self.robot_angle)
            triangle_size = 8
            
            # Front point (direction the robot is facing)
            front_x = robot_screen_x - triangle_size * math.sin(angle_rad)
            front_y = robot_screen_y - triangle_size * math.cos(angle_rad)
            
            # Back left point
            back_left_x = robot_screen_x + triangle_size * 0.6 * math.sin(angle_rad - 2.618)  # 150 degrees
            back_left_y = robot_screen_y + triangle_size * 0.6 * math.cos(angle_rad - 2.618)
            
            # Back right point
            back_right_x = robot_screen_x + triangle_size * 0.6 * math.sin(angle_rad + 2.618)  # 150 degrees
            back_right_y = robot_screen_y + triangle_size * 0.6 * math.cos(angle_rad + 2.618)
            
            # Draw the triangle
            from PyQt6.QtGui import QPolygonF
            from PyQt6.QtCore import QPointF
            triangle = QPolygonF([
                QPointF(front_x, front_y),
                QPointF(back_left_x, back_left_y),
                QPointF(back_right_x, back_right_y)
            ])
            painter.drawPolygon(triangle)
            
            # Draw position text
            painter.setPen(QColor("#ffffff"))
            painter.setFont(QFont("Arial", 10))
            position_text = f"X: {self.robot_x:.2f}m\nY: {self.robot_y:.2f}m\nAngle: {self.robot_angle:.1f}deg"
            painter.drawText(10, 20, position_text)
            
            # Draw trail info and scale info
            painter.setPen(QColor("#4CAF50"))
            painter.setFont(QFont("Arial", 9))
            trail_text = f"Trail: {len(self.position_history)} points"
            painter.drawText(10, self.height() - 50, trail_text)
            
            # Draw scale info
            painter.setPen(QColor("#64B5F6"))
            scale_text = f"Scale: {self.current_scale:.1f}m radius"
            painter.drawText(10, self.height() - 30, scale_text)
        else:
            # Draw disconnected message
            painter.setPen(QColor("#F44336"))
            painter.setFont(QFont("Arial", 12, QFont.Weight.Bold))
            painter.drawText(center_x - 50, center_y, "Disconnected")
    
    def clamp_to_circle(self, x, y, center_x, center_y, radius):
        """Clamp point to stay within circle bounds"""
        distance_from_center = ((x - center_x) ** 2 + (y - center_y) ** 2) ** 0.5
        if distance_from_center > radius - 10:
            ratio = (radius - 10) / distance_from_center
            x = center_x + (x - center_x) * ratio
            y = center_y + (y - center_y) * ratio
        return x, y
    
    def update_position(self, x, y, angle):
        # Update current position
        self.robot_x = x
        self.robot_y = y
        self.robot_angle = angle
        
        # Add to position history (no limit - trail never disappears)
        self.position_history.append((x, y))
        
        self.update()
    
    def clear_trail(self):
        """Clear the position trail"""
        self.position_history.clear()
        self.update()
    
    def set_connected(self, connected):
        self.connected = connected
        if not connected:
            # Don't clear trail when disconnected - keep it persistent
            pass
        self.update()
        
    def toggle_auto_scale(self):
        """Toggle auto-scale on/off"""
        self.auto_scale = not self.auto_scale
        if not self.auto_scale:
            self.current_scale = 5.0  # Reset to default
        self.update()
        return self.auto_scale

class PositionDataThread(QThread):
    position_update = pyqtSignal(float, float, float)  # x, y, angle
    connection_status = pyqtSignal(bool)
    power_battery_update = pyqtSignal(float, float)  # power, battery_percentage
    
    def __init__(self, host='192.168.137.172', port=7777):
        super().__init__()
        self.host = host
        self.port = port
        self.running = True
        self.socket = None
        self.connected = False
        
        # Position tracking (simplified from windows_position_viewer.py)
        self.current_angle = 0.0
        self.cumulative_x = 0.0
        self.cumulative_y = 0.0
        self.first_data = True
        
        # Distance reception control
        self.receive_distance = True  # Can be toggled by controller
        
        # Power and battery data
        self.current_power = 0.0
        self.current_battery = 0.0
        
    def set_distance_reception(self, enabled):
        """Enable or disable distance data reception"""
        self.receive_distance = enabled
        print(f"Distance reception: {'enabled' if enabled else 'disabled'}")
    
    def reset_position_origin(self):
        """Reset position to origin"""
        self.cumulative_x = 0.0
        self.cumulative_y = 0.0
        self.current_angle = 0.0  # Reset angle to 0 (pointing north)
        self.first_data = True
        # Immediately emit the reset position
        self.position_update.emit(self.cumulative_x, self.cumulative_y, self.current_angle)
        print("Position origin reset - angle set to (north)")
    
    def run(self):
        while self.running:
            try:
                if not self.connected:
                    self.connect_to_pi()
                
                if self.connected and self.socket:
                    self.receive_data()
                else:
                    time.sleep(2.0)
                
                if self.connected:
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"Position data thread error: {e}")
                self.connected = False
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                time.sleep(2.0)
    
    def connect_to_pi(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self.connection_status.emit(True)
            print(f"Connected to position server at {self.host}:{self.port}")
        except Exception as e:
            self.connected = False
            self.connection_status.emit(False)
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
    
    def receive_data(self):
        try:
            import json
            import math
            
            self.socket.settimeout(1.0)
            data = self.socket.recv(1024).decode('utf-8')
            if not data:
                self.connected = False
                self.connection_status.emit(False)
                return
            
            # Process complete lines
            lines = data.split('\n')
            for line in lines:
                if line.strip():
                    try:
                        json_data = json.loads(line.strip())
                        angle = json_data.get('angle', 0.0)
                        distance = json_data.get('distance', 0.0)
                        power = json_data.get('power', 0.0)
                        battery_level = json_data.get('batteryLevel', 0.0)
                        
                        # Always update angle
                        self.current_angle = angle
                        
                        # Update power and battery data
                        self.current_power = power
                        self.current_battery = battery_level
                        
                        # Only process distance if reception is enabled
                        if self.receive_distance:
                            if not self.first_data:
                                # Calculate movement
                                angle_rad = math.radians(angle)
                                delta_x = -distance * math.sin(angle_rad)  # Fixed direction
                                delta_y = -distance * math.cos(angle_rad)  # Fixed direction
                                
                                self.cumulative_x += delta_x
                                self.cumulative_y += delta_y
                            else:
                                self.cumulative_x = 0.0
                                self.cumulative_y = 0.0
                                self.first_data = False
                        
                        # Always emit position update (x,y might not change if distance reception disabled)
                        self.position_update.emit(self.cumulative_x, self.cumulative_y, self.current_angle)
                        
                        # Emit power and battery update
                        self.power_battery_update.emit(self.current_power, self.current_battery)
                        
                    except json.JSONDecodeError:
                        pass
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Data receive error: {e}")
            self.connected = False
            self.connection_status.emit(False)
    
    def stop(self):
        self.running = False
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.wait()

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Panel")
        self.setMinimumSize(1000, 700)
        
        # Set modern dark theme with better spacing
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #1a1a2e, stop:0.5 #16213e, stop:1 #1a1a2e);
                color: #ffffff;
            }
            QWidget {
                background-color: transparent;
                color: #ffffff;
                font-family: "Segoe UI", "Arial", sans-serif;
            }
            QGroupBox {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 rgba(255,255,255,8), stop:1 rgba(255,255,255,3));
                border: 1px solid rgba(255,255,255,20);
                border-radius: 12px;
                margin: 8px 2px;
                padding: 15px 8px 8px 8px;
                font-weight: 600;
                font-size: 13px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 8px 0 8px;
                color: #64B5F6;
                font-weight: bold;
            }
            QLabel {
                color: #ffffff;
                font-size: 13px;
                margin: 2px;
            }
            QLineEdit {
                padding: 10px;
                border: 2px solid rgba(100,181,246,50);
                border-radius: 8px;
                background: rgba(255,255,255,10);
                color: #ffffff;
                font-size: 13px;
                margin: 2px;
            }
            QLineEdit:focus {
                border: 2px solid #64B5F6;
                background: rgba(100,181,246,20);
            }
            QPushButton {
                padding: 12px 16px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #64B5F6, stop:1 #42A5F5);
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: 600;
                font-size: 13px;
                margin: 2px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #90CAF9, stop:1 #64B5F6);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #42A5F5, stop:1 #2196F3);
            }
            QPushButton:disabled {
                background: rgba(255,255,255,20);
                color: rgba(255,255,255,100);
            }
            .follow-button-on {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4CAF50, stop:1 #388E3C);
            }
            .follow-button-on:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #66BB6A, stop:1 #4CAF50);
            }
            .follow-button-off {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #F44336, stop:1 #D32F2F);
            }
            .follow-button-off:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #EF5350, stop:1 #F44336);
            }
            .obstacle-button-on {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FF9800, stop:1 #F57C00);
            }
            .obstacle-button-on:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FFB74D, stop:1 #FF9800);
            }
            .obstacle-button-off {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #9E9E9E, stop:1 #757575);
            }
            .obstacle-button-off:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #BDBDBD, stop:1 #9E9E9E);
            }
            .command-label {
                background: rgba(76,175,80,20);
                border: 1px solid #4CAF50;
                border-radius: 6px;
                padding: 8px 12px;
                color: #4CAF50;
                font-family: "Consolas", "Monaco", monospace;
                font-size: 12px;
                font-weight: bold;
                margin: 4px;
            }
            .value-display {
                background: rgba(255,193,7,20);
                border: 1px solid #FFC107;
                border-radius: 6px;
                padding: 6px 10px;
                color: #FFC107;
                font-size: 14px;
                font-weight: bold;
                margin: 2px;
                min-width: 60px;
                text-align: center;
            }
        """)
        
        # Initialize variables
        self.server_ip = ""
        self.connected = False
        self.joystick_thread = None
        self.send_timer = QTimer()
        self.send_timer.timeout.connect(self.send_control_data)
        self.position_viewer_process = None

        # Feature states
        self.following_mode = False
        self.obstacle_avoidance_enabled = False
        self.obstacle_status = "UNKNOWN"
        
        # Speed control
        self.speed_multiplier = 1.0
        
        # Initialize obstacle status server
        self.obstacle_server = ObstacleStatusServer()
        self.obstacle_server.obstacle_update.connect(self.update_obstacle_status)
        self.obstacle_server.start()
        
        self.setup_ui()
        self.init_joystick()
        
    def setup_ui(self):
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)  # Changed to vertical layout
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)
        
        # Top section - Camera and Position side by side
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        
        # Camera feed (left side of top)
        camera_group = QGroupBox("Camera Feed")
        camera_layout = QVBoxLayout(camera_group)
        # Create the camera view
        self.camera_view = QWebEngineView()
        self.camera_view.setMinimumHeight(300)
        self.camera_view.setMinimumWidth(450)
        self.camera_view.setStyleSheet("""
            QWebEngineView {
                background: rgba(100,181,246,10);
                border: 1px solid rgba(100,181,246,30);
                border-radius: 10px;
                margin: 5px;
            }
        """)
        
        # Load the page and inject CSS to rotate the video
        self.camera_view.setUrl(QUrl("http://192.168.137.172:8000"))
        
        # Inject JavaScript to rotate the video when page loads
        def inject_rotation():
            script = """
            document.addEventListener('DOMContentLoaded', function() {
                var video = document.querySelector('img') || document.querySelector('video') || document.body;
                if (video) {
                    video.style.transform = 'rotate(180deg)';
                    video.style.webkitTransform = 'rotate(180deg)';
                }
            });
            
            // Also try to rotate immediately
            setTimeout(function() {
                var video = document.querySelector('img') || document.querySelector('video') || document.body;
                if (video) {
                    video.style.transform = 'rotate(180deg)';
                    video.style.webkitTransform = 'rotate(180deg)';
                }
            }, 1000);
            """
            self.camera_view.page().runJavaScript(script)
        
        # Connect to load finished signal
        self.camera_view.loadFinished.connect(lambda: inject_rotation())
        
        camera_layout.addWidget(self.camera_view)
        
        # Position area (right side of top)
        position_widget = QWidget()
        position_widget_layout = QVBoxLayout(position_widget)
        position_widget_layout.setContentsMargins(0, 0, 0, 0)
        position_widget_layout.setSpacing(8)
        
        # Position title and status
        position_title_layout = QHBoxLayout()
        position_title = QLabel("Position & Trail")
        position_title.setStyleSheet("""
            QLabel {
                color: #64B5F6;
                font-size: 14px;
                font-weight: bold;
                margin: 5px;
            }
        """)
        
        self.position_viewer_status = QLabel("Viewer: Stopped")
        self.position_viewer_status.setStyleSheet("""
            QLabel {
                color: #F44336;
                font-size: 10px;
                font-weight: bold;
                padding: 4px 8px;
                background: rgba(244,67,54,20);
                border: 1px solid #F44336;
                border-radius: 4px;
                margin: 2px;
            }
        """)
        
        # Distance reception status
        self.distance_status = QLabel("Distance: ON")
        self.distance_status.setStyleSheet("""
            QLabel {
                color: #4CAF50;
                font-size: 10px;
                font-weight: bold;
                padding: 4px 8px;
                background: rgba(76,175,80,20);
                border: 1px solid #4CAF50;
                border-radius: 4px;
                margin: 2px;
            }
        """)
        
        position_title_layout.addWidget(position_title)
        position_title_layout.addStretch()
        position_title_layout.addWidget(self.distance_status)
        position_title_layout.addWidget(self.position_viewer_status)
        
        # Position control buttons (compact)
        position_controls_layout = QHBoxLayout()
        position_controls_layout.setSpacing(5)
        
        self.start_viewer_btn = QPushButton("Full View")
        self.start_viewer_btn.clicked.connect(self.start_position_viewer)
        self.start_viewer_btn.setStyleSheet("""
            QPushButton {
                padding: 4px 8px;
                font-size: 9px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #4CAF50, stop:1 #388E3C);
                border-radius: 4px;
                margin: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #66BB6A, stop:1 #4CAF50);
            }
        """)
        
        self.stop_viewer_btn = QPushButton("Close")
        self.stop_viewer_btn.clicked.connect(self.stop_position_viewer)
        self.stop_viewer_btn.setEnabled(False)
        self.stop_viewer_btn.setStyleSheet("""
            QPushButton {
                padding: 4px 8px;
                font-size: 9px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #F44336, stop:1 #D32F2F);
                border-radius: 4px;
                margin: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #EF5350, stop:1 #F44336);
            }
        """)
        
        self.clear_trail_btn = QPushButton("Clear")
        self.clear_trail_btn.clicked.connect(self.clear_position_trail)
        self.clear_trail_btn.setStyleSheet("""
            QPushButton {
                padding: 4px 8px;
                font-size: 9px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FF9800, stop:1 #F57C00);
                border-radius: 4px;
                margin: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FFB74D, stop:1 #FF9800);
            }
        """)
        
        self.reset_origin_btn = QPushButton("Reset Origin")
        self.reset_origin_btn.clicked.connect(self.reset_position_origin)
        self.reset_origin_btn.setStyleSheet("""
            QPushButton {
                padding: 4px 8px;
                font-size: 9px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #9C27B0, stop:1 #7B1FA2);
                border-radius: 4px;
                margin: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #BA68C8, stop:1 #9C27B0);
            }
        """)
        
        self.auto_scale_btn = QPushButton("Auto Scale: ON")
        self.auto_scale_btn.clicked.connect(self.toggle_auto_scale)
        self.auto_scale_btn.setStyleSheet("""
            QPushButton {
                padding: 4px 8px;
                font-size: 9px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #00BCD4, stop:1 #0097A7);
                border-radius: 4px;
                margin: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #26C6DA, stop:1 #00BCD4);
            }
        """)
        
        position_controls_layout.addWidget(self.start_viewer_btn)
        position_controls_layout.addWidget(self.stop_viewer_btn)
        position_controls_layout.addWidget(self.clear_trail_btn)
        position_controls_layout.addWidget(self.reset_origin_btn)
        position_controls_layout.addWidget(self.auto_scale_btn)
        position_controls_layout.addStretch()
        
        # Position indicator (rectangular)
        self.position_indicator = SimplePositionIndicator()
        
        position_widget_layout.addLayout(position_title_layout)
        position_widget_layout.addLayout(position_controls_layout)
        position_widget_layout.addWidget(self.position_indicator)
        position_widget_layout.addStretch()
        
        # Initialize position data thread
        self.position_data_thread = PositionDataThread()
        self.position_data_thread.position_update.connect(self.position_indicator.update_position)
        self.position_data_thread.connection_status.connect(self.position_indicator.set_connected)
        self.position_data_thread.power_battery_update.connect(self.update_power_battery_display)
        self.position_data_thread.start()
        
        # Add to top layout
        top_layout.addWidget(camera_group, 2)  # Camera gets more space
        top_layout.addWidget(position_widget, 1)  # Position gets less space
        
        # Bottom section - Control panel (horizontal)
        control_group = QGroupBox("Robot Control Panel")
        control_main_layout = QHBoxLayout(control_group)
        control_main_layout.setSpacing(20)
        
        # Connection section
        connection_section = QVBoxLayout()
        connection_group = QGroupBox("Connection")
        connection_layout = QVBoxLayout(connection_group)
        connection_layout.setSpacing(8)
        
        # Status with indicator
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        self.connection_indicator = StatusIndicator()
        self.connection_status = AnimatedLabel("Disconnected")
        self.connection_status.setStyleSheet("font-weight: bold; margin-left: 5px;")
        status_layout.addWidget(self.connection_indicator)
        status_layout.addWidget(self.connection_status)
        status_layout.addStretch()
        
        # IP input layout
        ip_layout = QHBoxLayout()
        ip_layout.setContentsMargins(0, 0, 0, 0)
        ip_layout.addWidget(QLabel("Robot IP:"))
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("Robot IP")
        self.ip_input.setText("192.168.137.172")
        ip_layout.addWidget(self.ip_input)
        
        connect_button = QPushButton("Connect")
        connect_button.clicked.connect(self.connect_to_server)
        
        connection_layout.addLayout(status_layout)
        connection_layout.addLayout(ip_layout)
        connection_layout.addWidget(connect_button)
        
        # Features section
        features_section = QVBoxLayout()
        features_group = QGroupBox("Features")
        features_layout = QVBoxLayout(features_group)
        features_layout.setSpacing(8)
        
        self.follow_button = QPushButton("Visual Following: OFF")
        self.follow_button.setProperty("class", "follow-button-off")
        self.follow_button.clicked.connect(self.toggle_following)
        self.follow_button.setEnabled(False)
        
        self.obstacle_button = QPushButton("Obstacle Avoidance: OFF")
        self.obstacle_button.setProperty("class", "obstacle-button-off")
        self.obstacle_button.clicked.connect(self.toggle_obstacle_avoidance)
        self.obstacle_button.setEnabled(False)
        
        # Obstacle status
        obstacle_status_layout = QHBoxLayout()
        obstacle_status_layout.setContentsMargins(0, 0, 0, 0)
        self.obstacle_indicator = StatusIndicator()
        self.obstacle_status_label = AnimatedLabel("Obstacle: Unknown")
        self.obstacle_status_label.setStyleSheet("font-weight: bold; margin-left: 5px;")
        obstacle_status_layout.addWidget(self.obstacle_indicator)
        obstacle_status_layout.addWidget(self.obstacle_status_label)
        obstacle_status_layout.addStretch()
        
        features_layout.addWidget(self.follow_button)
        features_layout.addWidget(self.obstacle_button)
        features_layout.addLayout(obstacle_status_layout)
        
        # Control values section
        control_section = QVBoxLayout()
        control_group_inner = QGroupBox("Control Values")
        control_layout = QVBoxLayout(control_group_inner)
        control_layout.setSpacing(6)
        
        # Joystick displays in horizontal layout
        joystick_layout = QHBoxLayout()
        joystick_layout.setSpacing(10)
        
        # Left joystick (Speed)
        left_joy_layout = QVBoxLayout()
        left_joy_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_joy_layout.setSpacing(3)
        
        left_joy_label = QLabel("Speed")
        left_joy_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left_joy_label.setStyleSheet("font-weight: bold; color: #64B5F6; font-size: 12px;")
        
        self.left_joystick = JoystickWidget()
        
        left_joy_layout.addWidget(left_joy_label)
        left_joy_layout.addWidget(self.left_joystick)
        
        # Right joystick (Turn)
        right_joy_layout = QVBoxLayout()
        right_joy_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_joy_layout.setSpacing(3)
        
        right_joy_label = QLabel("Turn")
        right_joy_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_joy_label.setStyleSheet("font-weight: bold; color: #64B5F6; font-size: 12px;")
        
        self.right_joystick = JoystickWidget()
        
        right_joy_layout.addWidget(right_joy_label)
        right_joy_layout.addWidget(self.right_joystick)
        
        joystick_layout.addLayout(left_joy_layout)
        joystick_layout.addLayout(right_joy_layout)
        control_layout.addLayout(joystick_layout)
        
        # Speed control slider
        speed_control_layout = QHBoxLayout()
        speed_control_layout.setSpacing(10)
        
        speed_label = QLabel("Speed:")
        speed_label.setStyleSheet("font-weight: bold; color: #64B5F6; font-size: 12px;")
        speed_control_layout.addWidget(speed_label)
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setMinimum(10)  # 0.1x speed
        self.speed_slider.setMaximum(200)  # 2.0x speed
        self.speed_slider.setValue(100)  # 1.0x speed (default)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setTickInterval(25)
        self.speed_slider.valueChanged.connect(self.update_speed_multiplier)
        self.speed_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid rgba(100,181,246,50);
                height: 6px;
                background: rgba(100,181,246,20);
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #64B5F6, stop:1 #42A5F5);
                border: 1px solid #42A5F5;
                width: 16px;
                height: 16px;
                border-radius: 8px;
                margin: -6px 0;
            }
            QSlider::handle:horizontal:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #90CAF9, stop:1 #64B5F6);
            }
        """)
        speed_control_layout.addWidget(self.speed_slider)
        
        self.speed_value_display = QLabel("1.0x")
        self.speed_value_display.setProperty("class", "value-display")
        self.speed_value_display.setMinimumWidth(50)
        speed_control_layout.addWidget(self.speed_value_display)
        
        control_layout.addLayout(speed_control_layout)
        
        # Command display
        self.command_display = QLabel("Awaiting commands...")
        self.command_display.setProperty("class", "command-label")
        self.command_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        control_layout.addWidget(self.command_display)
        
        # Power and Battery status
        power_section = QVBoxLayout()
        power_group = QGroupBox("Power & Battery")
        power_layout = QVBoxLayout(power_group)
        
        # Power display
        power_layout_inner = QHBoxLayout()
        power_layout_inner.setContentsMargins(0, 0, 0, 0)
        power_label = QLabel("Power:")
        power_label.setStyleSheet("font-weight: bold; color: #64B5F6;")
        self.power_display = QLabel("0.00W")
        self.power_display.setProperty("class", "value-display")
        power_layout_inner.addWidget(power_label)
        power_layout_inner.addWidget(self.power_display)
        power_layout_inner.addStretch()
        
        # Battery display
        battery_layout_inner = QHBoxLayout()
        battery_layout_inner.setContentsMargins(0, 0, 0, 0)
        battery_label = QLabel("Battery:")
        battery_label.setStyleSheet("font-weight: bold; color: #64B5F6;")
        self.battery_display = QLabel("0.0%")
        self.battery_display.setProperty("class", "value-display")
        battery_layout_inner.addWidget(battery_label)
        battery_layout_inner.addWidget(self.battery_display)
        battery_layout_inner.addStretch()
        
        # Controller button mapping info (simplified)
        button_info = QLabel("A: Distance Toggle ? B: Reset Origin ? X: Clear Trail\nY: Follow ? LB: Obstacle ? RB: Auto Scale ? START/SELECT: Viewer\nDPAD Up/Down: Speed +/- ? LS/RS: Speed Min/Max")
        button_info.setStyleSheet("""
            QLabel {
                color: #64B5F6;
                font-size: 9px;
                background: rgba(100,181,246,10);
                border: 1px solid rgba(100,181,246,30);
                border-radius: 4px;
                padding: 4px;
                margin: 2px;
            }
        """)
        button_info.setWordWrap(True)
        
        power_layout.addLayout(power_layout_inner)
        power_layout.addLayout(battery_layout_inner)
        power_layout.addWidget(button_info)
        
        # Add all sections to control main layout
        control_main_layout.addWidget(connection_group)
        control_main_layout.addWidget(features_group)
        control_main_layout.addWidget(control_group_inner)
        control_main_layout.addWidget(power_group)
        
        # Add top and bottom to main layout
        main_layout.addLayout(top_layout, 2)  # Top gets more space
        main_layout.addWidget(control_group, 1)  # Bottom gets less space
        
    def init_joystick(self):
        self.joystick_thread = JoystickThread()
        self.joystick_thread.joystick_update.connect(self.update_joystick_display)
        self.joystick_thread.button_pressed.connect(self.handle_controller_button)
        self.joystick_thread.dpad_pressed.connect(self.handle_dpad_press)
        self.joystick_thread.start()
        
    def update_joystick_display(self, left_x, left_y, right_x, right_y):
        # Update joystick widgets
        self.left_joystick.update_position(left_x, left_y)
        self.right_joystick.update_position(right_x, right_y)
        
    def update_speed_multiplier(self, value):
        """Update speed multiplier from slider value"""
        self.speed_multiplier = value / 100.0  # Convert 10-200 to 0.1-2.0
        self.speed_value_display.setText(f"{self.speed_multiplier:.1f}x")
        
        # Update display color based on speed
        if self.speed_multiplier > 1.5:
            color = "#F44336"  # Red for high speed
        elif self.speed_multiplier > 1.0:
            color = "#FF9800"  # Orange for medium-high speed
        elif self.speed_multiplier >= 0.5:
            color = "#4CAF50"  # Green for normal speed
        else:
            color = "#2196F3"  # Blue for low speed
            
        self.speed_value_display.setStyleSheet(f"""
            QLabel {{
                background: rgba({self.hex_to_rgb(color)},20);
                border: 1px solid {color};
                border-radius: 6px;
                padding: 6px 10px;
                color: {color};
                font-size: 14px;
                font-weight: bold;
                margin: 2px;
                min-width: 50px;
                text-align: center;
            }}
        """)
        
    def update_obstacle_status(self, status):
        """Update obstacle status from the obstacle server"""
        self.obstacle_status = status
        self.obstacle_status_label.setText(f"Obstacle: {status}")
        self.obstacle_status_label.animate_pulse()
        
        # Update obstacle indicator
        if status == "CLEAR":
            self.obstacle_indicator.set_connected(True)
        else:
            self.obstacle_indicator.set_connected(False)
    
    def update_power_battery_display(self, power, battery_percentage):
        """Update power and battery display"""
        self.power_display.setText(f"{power:.2f}W")
        self.battery_display.setText(f"{battery_percentage:.1f}%")
        
        # Update battery color based on level
        if battery_percentage > 50:
            color = "#4CAF50"  # Green
        elif battery_percentage > 20:
            color = "#FF9800"  # Orange
        else:
            color = "#F44336"  # Red
            
        self.battery_display.setStyleSheet(f"""
            QLabel {{
                background: rgba({self.hex_to_rgb(color)},20);
                border: 1px solid {color};
                border-radius: 6px;
                padding: 6px 10px;
                color: {color};
                font-size: 14px;
                font-weight: bold;
                margin: 2px;
                min-width: 60px;
                text-align: center;
            }}
        """)
    
    def hex_to_rgb(self, hex_color):
        """Convert hex color to RGB values"""
        hex_color = hex_color.lstrip('#')
        return ','.join(str(int(hex_color[i:i+2], 16)) for i in (0, 2, 4))
        
    def connect_to_server(self):
        self.server_ip = self.ip_input.text()
        try:
            # Test connection
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((self.server_ip, 5000))
            client.send("ping".encode('utf-8'))
            response = client.recv(1024).decode('utf-8')
            client.close()
            
            if response == "success":
                self.connected = True
                self.connection_status.setText("Connected")
                self.connection_status.animate_pulse()
                self.connection_indicator.set_connected(True)
                self.send_timer.start(100)  # 10Hz
                # Enable feature buttons when connected
                self.follow_button.setEnabled(True)
                self.obstacle_button.setEnabled(True)
            else:
                self.connection_status.setText("Connection failed")
                self.connection_indicator.set_connected(False)
        except Exception as e:
            self.connection_status.setText(f"Error: {str(e)[:20]}...")
            self.connection_indicator.set_connected(False)
            
    def send_message(self, message):
        """Send a message to the robot server and return response"""
        if not self.connected:
            print("Not connected to server")
            return ""
            
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((self.server_ip, 5000))
            client.send(message.encode('utf-8'))
            response = client.recv(1024).decode('utf-8')
            client.close()
            return response
        except Exception as e:
            print(f"Error sending message: {e}")
            return ""
    
    def toggle_following(self):
        """Toggle visual following mode"""
        if not self.connected:
            return
            
        if self.following_mode:
            # Turn off following
            response = self.send_message("stop_follow")
            if response:
                self.following_mode = False
                self.follow_button.setText("Visual Following: OFF")
                self.follow_button.setProperty("class", "follow-button-off")
                self.follow_button.setStyleSheet("")
                self.follow_button.style().unpolish(self.follow_button)
                self.follow_button.style().polish(self.follow_button)
                print("Visual following disabled")
                # Resume joystick control
                self.send_timer.start(100)
        else:
            # Turn on following
            response = self.send_message("follow")
            if response:
                self.following_mode = True
                self.follow_button.setText("Visual Following: ON")
                self.follow_button.setProperty("class", "follow-button-on")
                self.follow_button.setStyleSheet("")
                self.follow_button.style().unpolish(self.follow_button)
                self.follow_button.style().polish(self.follow_button)
                print("Visual following enabled")
                # Stop joystick control when following is active
                self.send_timer.stop()
                self.command_display.setText("Visual tracking active")
            
    def toggle_obstacle_avoidance(self):
        """Toggle obstacle avoidance mode"""
        if not self.connected:
            return
            
        if self.obstacle_avoidance_enabled:
            # Turn off obstacle avoidance
            response = self.send_message("obstacle_avoidance::off")
            if response:
                self.obstacle_avoidance_enabled = False
                self.obstacle_button.setText("Obstacle Avoidance: OFF")
                self.obstacle_button.setProperty("class", "obstacle-button-off")
                self.obstacle_button.setStyleSheet("")
                self.obstacle_button.style().unpolish(self.obstacle_button)
                self.obstacle_button.style().polish(self.obstacle_button)
                print("Obstacle avoidance disabled")
        else:
            # Turn on obstacle avoidance
            response = self.send_message("obstacle_avoidance::on")
            if response:
                self.obstacle_avoidance_enabled = True
                self.obstacle_button.setText("Obstacle Avoidance: ON")
                self.obstacle_button.setProperty("class", "obstacle-button-on")
                self.obstacle_button.setStyleSheet("")
                self.obstacle_button.style().unpolish(self.obstacle_button)
                self.obstacle_button.style().polish(self.obstacle_button)
                print("Obstacle avoidance enabled")
            
    def send_control_data(self):
        if not self.connected or self.following_mode:
            return
            
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((self.server_ip, 5000))
            
            # Convert joystick values to control ranges
            left_y = self.left_joystick.y
            right_x = self.right_joystick.x
            
            # Apply dead zone
            if abs(left_y) < 0.1:
                left_y = 0
            if abs(right_x) < 0.1:
                right_x = 0
                
            # Calculate speed and steering with multiplier
            base_speed = round(-(left_y) * 12, 2)  # Base -12 to 12
            speed = round(base_speed * self.speed_multiplier, 2)  # Apply speed multiplier
            steering = round(-(right_x) * 1.3, 2)  # -1.3 to 1.3
            
            # Apply obstacle avoidance logic if enabled
            if self.obstacle_avoidance_enabled:
                if self.obstacle_status == "FRONT_OBSTACLE" and speed > 0:
                    speed = 0  # Block forward movement
                    print("Obstacle avoidance: Blocking forward movement")
                elif self.obstacle_status == "REAR_OBSTACLE" and speed < 0:
                    speed = 0  # Block backward movement
                    print("Obstacle avoidance: Blocking backward movement")
                # Always allow turning (steering) regardless of obstacles
            
            message = f"move::{speed},{steering}"
            self.command_display.setText(f"Speed: {speed} ({self.speed_multiplier:.1f}x) | Turn: {steering}")
            client.send(message.encode('utf-8'))
            client.close()
            
        except Exception as e:
            self.connected = False
            self.connection_status.setText(f"Error: {str(e)[:20]}...")
            self.connection_indicator.set_connected(False)
            self.send_timer.stop()
            # Disable feature buttons when disconnected
            self.follow_button.setEnabled(False)
            self.obstacle_button.setEnabled(False)
            
    def clear_position_trail(self):
        """Clear the position trail on the indicator"""
        if hasattr(self, 'position_indicator'):
            self.position_indicator.clear_trail()
            
    def reset_position_origin(self):
        """Reset position origin to current location"""
        if hasattr(self, 'position_data_thread'):
            self.position_data_thread.reset_position_origin()
        if hasattr(self, 'position_indicator'):
            self.position_indicator.clear_trail()
        print("Position origin reset via controller - angle set to north")
            
    def start_position_viewer(self):
        """Start the full position viewer window"""
        if self.position_viewer_process is None or self.position_viewer_process.poll() is not None:
            try:
                script_path = os.path.join(os.path.dirname(__file__), '..', 'graphing', 'windows_position_viewer.py')
                self.position_viewer_process = subprocess.Popen([sys.executable, script_path])
                
                self.start_viewer_btn.setEnabled(False)
                self.stop_viewer_btn.setEnabled(True)
                self.position_viewer_status.setText("Position Viewer: Running")
                self.position_viewer_status.setStyleSheet("""
                    QLabel {
                        color: #4CAF50;
                        font-size: 12px;
                        font-weight: bold;
                        padding: 8px;
                        background: rgba(76,175,80,20);
                        border: 1px solid #4CAF50;
                        border-radius: 6px;
                        margin: 5px;
                    }
                """)
                
                print("Full position viewer started")
                
            except Exception as e:
                print(f"Failed to start position viewer: {e}")
        
    def stop_position_viewer(self):
        """Stop the full position viewer window"""
        if self.position_viewer_process and self.position_viewer_process.poll() is None:
            try:
                self.position_viewer_process.terminate()
                self.position_viewer_process.wait(timeout=3)
                
                self.start_viewer_btn.setEnabled(True)
                self.stop_viewer_btn.setEnabled(False)
                self.position_viewer_status.setText("Position Viewer: Stopped")
                self.position_viewer_status.setStyleSheet("""
                    QLabel {
                        color: #F44336;
                        font-size: 12px;
                        font-weight: bold;
                        padding: 8px;
                        background: rgba(244,67,54,20);
                        border: 1px solid #F44336;
                        border-radius: 6px;
                        margin: 5px;
                    }
                """)
                
                print("Position viewer stopped")
                
            except Exception as e:
                print(f"Error stopping position viewer: {e}")
                
        self.position_viewer_process = None
            
    def handle_controller_button(self, button_name):
        """Handle controller button presses"""
        print(f"Controller button pressed: {button_name}")
        
        if button_name == "A":
            # Toggle distance reception
            if hasattr(self, 'position_data_thread'):
                current_state = self.position_data_thread.receive_distance
                self.position_data_thread.set_distance_reception(not current_state)
                
                # Update UI feedback
                if self.position_data_thread.receive_distance:
                    self.distance_status.setText("Distance: ON")
                    self.distance_status.setStyleSheet("""
                        QLabel {
                            color: #4CAF50;
                            font-size: 10px;
                            font-weight: bold;
                            padding: 4px 8px;
                            background: rgba(76,175,80,20);
                            border: 1px solid #4CAF50;
                            border-radius: 4px;
                            margin: 2px;
                        }
                    """)
                    print("Distance tracking enabled")
                else:
                    self.distance_status.setText("Distance: OFF")
                    self.distance_status.setStyleSheet("""
                        QLabel {
                            color: #FF9800;
                            font-size: 10px;
                            font-weight: bold;
                            padding: 4px 8px;
                            background: rgba(255,152,0,20);
                            border: 1px solid #FF9800;
                            border-radius: 4px;
                            margin: 2px;
                        }
                    """)
                    print("Distance tracking disabled (angle only)")
        
        elif button_name == "B":
            # Reset position origin
            if hasattr(self, 'position_data_thread'):
                self.position_data_thread.reset_position_origin()
            if hasattr(self, 'position_indicator'):
                self.position_indicator.clear_trail()
            print("Position origin reset via controller - angle set to north")
        
        elif button_name == "X":
            # Clear trail only
            if hasattr(self, 'position_indicator'):
                self.position_indicator.clear_trail()
            print("Trail cleared")
        
        elif button_name == "Y":
            # Toggle visual following
            self.toggle_following()
        
        elif button_name == "LB":
            # Toggle obstacle avoidance
            self.toggle_obstacle_avoidance()
        
        elif button_name == "RB":
            # Toggle auto-scale
            self.toggle_auto_scale()
        
        elif button_name == "START":
            # Open full position viewer
            self.start_position_viewer()
        
        elif button_name == "SELECT":
            # Close position viewer
            self.stop_position_viewer()
        
        elif button_name == "LS":
            # Set minimum speed (摇杆按下)
            self.speed_slider.setValue(10)  # 0.1x speed
            print("Speed set to minimum (0.1x)")
        
        elif button_name == "RS":
            # Set maximum speed (摇杆按下)
            self.speed_slider.setValue(200)  # 2.0x speed
            print("Speed set to maximum (2.0x)")
    
    def handle_dpad_press(self, direction):
        """Handle D-pad presses for speed adjustment"""
        current_value = self.speed_slider.value()
        
        if direction == "DPAD_UP":
            # Increase speed by 10% (10 units)
            new_value = min(200, current_value + 10)
            self.speed_slider.setValue(new_value)
            print(f"Speed increased to {new_value/100.0:.1f}x")
            
        elif direction == "DPAD_DOWN":
            # Decrease speed by 10% (10 units)
            new_value = max(10, current_value - 10)
            self.speed_slider.setValue(new_value)
            print(f"Speed decreased to {new_value/100.0:.1f}x")

    def toggle_auto_scale(self):
        """Toggle auto-scale on/off"""
        if hasattr(self, 'position_indicator'):
            auto_scale_enabled = self.position_indicator.toggle_auto_scale()
            # Update button text
            if auto_scale_enabled:
                self.auto_scale_btn.setText("Auto Scale: ON")
                self.auto_scale_btn.setStyleSheet("""
                    QPushButton {
                        padding: 4px 8px;
                        font-size: 9px;
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #00BCD4, stop:1 #0097A7);
                        border-radius: 4px;
                        margin: 1px;
                    }
                    QPushButton:hover {
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #26C6DA, stop:1 #00BCD4);
                    }
                """)
            else:
                self.auto_scale_btn.setText("Auto Scale: OFF")
                self.auto_scale_btn.setStyleSheet("""
                    QPushButton {
                        padding: 4px 8px;
                        font-size: 9px;
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #757575, stop:1 #616161);
                        border-radius: 4px;
                        margin: 1px;
                    }
                    QPushButton:hover {
                        background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                            stop:0 #9E9E9E, stop:1 #757575);
                    }
                """)
            print(f"Auto-scale: {'enabled' if auto_scale_enabled else 'disabled'}")

    def closeEvent(self, event):
        if self.joystick_thread:
            self.joystick_thread.stop()
        if self.send_timer.isActive():
            self.send_timer.stop()
        if self.obstacle_server:
            self.obstacle_server.stop()
        # Stop position data thread
        if hasattr(self, 'position_data_thread'):
            self.position_data_thread.stop()
        # Stop position viewer process
        if hasattr(self, 'position_viewer_process') and self.position_viewer_process:
            try:
                self.position_viewer_process.terminate()
                self.position_viewer_process.wait(timeout=3)
            except:
                pass
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec()) 
