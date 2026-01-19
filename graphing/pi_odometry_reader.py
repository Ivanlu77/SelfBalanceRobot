#!/usr/bin/env python3
"""
Simplified Raspberry Pi 5 Odometry Data Reader
Receives angle and distance data from ESP32 and forwards to network
Data format: "angle_degrees,distance_meters"
"""

import serial
import socket
import threading
import time
import json

class SimpleOdometryReader:
    def __init__(self, serial_port='/dev/ttyAMA0', baudrate=115200, 
                 network_port=7777, network_host='0.0.0.0'):
        """
        Initialize the simplified odometry reader
        
        Args:
            serial_port: Serial port for ESP32 communication
            baudrate: Serial communication speed
            network_port: Port to send data over network
            network_host: Host to bind network socket
        """
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.network_port = network_port
        self.network_host = network_host
        
        self.serial_connection = None
        self.server_socket = None
        self.client_connections = []
        self.running = False
        
        # Current position data
        self.current_angle = 0.0  # degrees
        self.current_distance = 0.0  # meters
        self.data_lock = threading.Lock()
        
    def connect_serial(self):
        """Connect to ESP32 via serial"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to ESP32: {self.serial_port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"ESP32 connection failed: {e}")
            return False
    
    def setup_network(self):
        """Setup network server for data forwarding"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.network_host, self.network_port))
            self.server_socket.listen(5)
            print(f"Network server started: {self.network_host}:{self.network_port}")
            return True
        except Exception as e:
            print(f"Network server startup failed: {e}")
            return False
    
    def parse_data(self, data_string):
        """Parse received data string"""
        try:
            data_string = data_string.strip()
            if ',' in data_string:
                parts = data_string.split(',')
                if len(parts) >= 4 :
                    angle = float(parts[0])  # degrees
                    distance = float(parts[1])  # meters
                    power = float(parts[2])
                    batteryLevel = float(parts[3])
                    return angle, distance, power, batteryLevel
        except ValueError:
            pass
        return None, None, None, None
    
    def update_position(self, angle, distance):
        """Update current position data"""
        with self.data_lock:
            self.current_angle = angle
            self.current_distance = distance
    
    def get_position_data(self):
        """Get current position data"""
        with self.data_lock:
            return {
                'angle': self.current_angle,
                'distance': self.current_distance,
                'timestamp': time.time()
            }
    
    def broadcast_data(self, data):
        """Broadcast data to all connected clients"""
        if not self.client_connections:
            return
        
        message = json.dumps(data) + '\n'
        disconnected_clients = []
        
        for client in self.client_connections:
            try:
                client.send(message.encode('utf-8'))
            except:
                disconnected_clients.append(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            self.client_connections.remove(client)
            try:
                client.close()
            except:
                pass
    
    def handle_client_connections(self):
        """Handle incoming client connections"""
        while self.running:
            try:
                if self.server_socket:
                    self.server_socket.settimeout(1.0)
                    client_socket, address = self.server_socket.accept()
                    self.client_connections.append(client_socket)
                    print(f"Client connected: {address}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Client connection handling error: {e}")
                break
    
    def read_serial_data(self):
        """Read data from ESP32 serial connection"""
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    if self.serial_connection.in_waiting > 0:
                        raw_data = self.serial_connection.readline().decode('utf-8', errors='ignore')
                        if raw_data:
                            angle, distance, power, batteryLevel = self.parse_data(raw_data)
                            
                            if angle is not None and distance is not None:
                                # Update position
                                self.update_position(angle, distance)
                                
                                # Get position data and broadcast
                                position_data = self.get_position_data()
                                position_data['power'] = power
                                position_data['batteryLevel'] = batteryLevel
                                self.broadcast_data(position_data)
                                
                                print(f"Angle: {angle:6.2f} | Distance: {distance:8.4f}m |Power: {power:.2f}V | Battery: {batteryLevel:.2f}%")
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Serial data reading error: {e}")
                time.sleep(0.1)
    
    def start(self):
        """Start the odometry reader"""
        if not self.connect_serial():
            return False
        
        if not self.setup_network():
            self.serial_connection.close()
            return False
        
        self.running = True
        
        # Start threads
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.network_thread = threading.Thread(target=self.handle_client_connections, daemon=True)
        
        self.serial_thread.start()
        self.network_thread.start()
        
        print("Odometry data receiver started")
        return True
    
    def stop(self):
        """Stop the odometry reader"""
        self.running = False
        
        # Close all client connections
        for client in self.client_connections:
            try:
                client.close()
            except:
                pass
        self.client_connections.clear()
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        # Close serial connection
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        
        print("Odometry data receiver stopped")

def main():
    """Main function"""
    print("Simplified Odometry Data Receiver")
    print("Receives ESP32 angle and distance data and forwards to network")
    print("Press Ctrl+C to stop\n")
    
    reader = SimpleOdometryReader()
    
    try:
        if not reader.start():
            print("Startup failed")
            return
        
        # Keep running
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    finally:
        reader.stop()
        print("Program ended")

if __name__ == "__main__":
    main() 
