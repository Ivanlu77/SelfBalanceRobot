#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Windows Position Viewer
Receives angle and distance data from Raspberry Pi and displays real-time position plot
"""

import tkinter as tk
from tkinter import ttk, messagebox
import socket
import json
import threading
import time
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

class PositionViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Position Viewer")
        self.root.geometry("1000x700")
        
        # Connection settings
        self.pi_host = "192.168.137.13"  # Change to your Pi's IP
        self.pi_port = 7777
        self.socket = None
        self.connected = False
        self.running = False
        
        # Position data
        self.current_angle = 0.0  # degrees (robot heading)
        self.current_distance = 0.0  # meters (movement distance)
        self.cumulative_x = 0.0  # cumulative X position
        self.cumulative_y = 0.0  # cumulative Y position
        self.data_lock = threading.Lock()
        
        # Position history for trail
        self.position_history = []
        self.max_history = 1000  # Increased to keep more path visible
        
        # First data flag
        self.first_data = True
        
        self.setup_ui()
        self.setup_plot()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection Settings", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(conn_frame, text="Pi IP:").grid(row=0, column=0, padx=(0, 5))
        self.ip_entry = ttk.Entry(conn_frame, width=15)
        self.ip_entry.insert(0, self.pi_host)
        self.ip_entry.grid(row=0, column=1, padx=(0, 10))
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, padx=(0, 5))
        self.port_entry = ttk.Entry(conn_frame, width=8)
        self.port_entry.insert(0, str(self.pi_port))
        self.port_entry.grid(row=0, column=3, padx=(0, 10))
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=(10, 0))
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=5, padx=(10, 0))
        
        # Data display frame
        data_frame = ttk.LabelFrame(main_frame, text="Current Data", padding="5")
        data_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N), padx=(0, 10))
        
        ttk.Label(data_frame, text="Heading:").grid(row=0, column=0, sticky=tk.W)
        self.angle_label = ttk.Label(data_frame, text="0.00 deg", font=("Arial", 12, "bold"))
        self.angle_label.grid(row=0, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(data_frame, text="Movement:").grid(row=1, column=0, sticky=tk.W)
        self.distance_label = ttk.Label(data_frame, text="0.0000m", font=("Arial", 12, "bold"))
        self.distance_label.grid(row=1, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(data_frame, text="X Position:").grid(row=2, column=0, sticky=tk.W)
        self.x_label = ttk.Label(data_frame, text="0.0000m", font=("Arial", 12, "bold"))
        self.x_label.grid(row=2, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(data_frame, text="Y Position:").grid(row=3, column=0, sticky=tk.W)
        self.y_label = ttk.Label(data_frame, text="0.0000m", font=("Arial", 12, "bold"))
        self.y_label.grid(row=3, column=1, sticky=tk.W, padx=(10, 0))
        
        # Statistics
        ttk.Label(data_frame, text="Total Distance:").grid(row=4, column=0, sticky=tk.W)
        self.total_dist_label = ttk.Label(data_frame, text="0.0000m", font=("Arial", 10))
        self.total_dist_label.grid(row=4, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(data_frame, text="From Origin:").grid(row=5, column=0, sticky=tk.W)
        self.origin_dist_label = ttk.Label(data_frame, text="0.0000m", font=("Arial", 10))
        self.origin_dist_label.grid(row=5, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(data_frame, text="Path Points:").grid(row=6, column=0, sticky=tk.W)
        self.points_label = ttk.Label(data_frame, text="0", font=("Arial", 10))
        self.points_label.grid(row=6, column=1, sticky=tk.W, padx=(10, 0))
        
        # Control buttons
        control_frame = ttk.Frame(data_frame)
        control_frame.grid(row=7, column=0, columnspan=2, pady=(10, 0))
        
        ttk.Button(control_frame, text="Reset Origin", command=self.reset_origin).pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(control_frame, text="Clear Trail", command=self.clear_trail).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Save Path", command=self.save_path_data).pack(side=tk.LEFT, padx=5)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
    def setup_plot(self):
        """Setup the matplotlib plot"""
        # Create figure and axis
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Setup plot
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Robot Position Map (Cumulative Movement)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Plot elements
        self.origin_point, = self.ax.plot(0, 0, 'ko', markersize=8, label='Origin')
        self.current_point, = self.ax.plot(0, 0, 'ro', markersize=10, label='Current Position')
        self.trail_line, = self.ax.plot([], [], 'b-', alpha=0.6, linewidth=2, label='Trail')
        self.direction_arrow = self.ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                              arrowprops=dict(arrowstyle='->', color='red', lw=2))
        
        self.ax.legend()
        
        # Embed plot in tkinter
        plot_frame = ttk.LabelFrame(self.root.children['!frame'], text="Position Map", padding="5")
        plot_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Add zoom controls
        zoom_frame = ttk.Frame(plot_frame)
        zoom_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        ttk.Button(zoom_frame, text="Zoom In", command=self.zoom_in).pack(side=tk.LEFT, padx=2)
        ttk.Button(zoom_frame, text="Zoom Out", command=self.zoom_out).pack(side=tk.LEFT, padx=2)
        ttk.Button(zoom_frame, text="Fit View", command=self.fit_view).pack(side=tk.LEFT, padx=2)
        ttk.Button(zoom_frame, text="Center on Robot", command=self.center_on_robot).pack(side=tk.LEFT, padx=2)
        
        # Auto-scale checkbox
        self.auto_scale_var = tk.BooleanVar(value=True)  # Enable auto-scale by default
        ttk.Checkbutton(zoom_frame, text="Auto Scale", variable=self.auto_scale_var).pack(side=tk.LEFT, padx=10)
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def zoom_in(self):
        """Zoom in the plot"""
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        x_range = (xlim[1] - xlim[0]) * 0.7  # Zoom in by 30%
        y_range = (ylim[1] - ylim[0]) * 0.7
        
        self.ax.set_xlim(x_center - x_range/2, x_center + x_range/2)
        self.ax.set_ylim(y_center - y_range/2, y_center + y_range/2)
        self.canvas.draw()
    
    def zoom_out(self):
        """Zoom out the plot"""
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        x_range = (xlim[1] - xlim[0]) * 1.4  # Zoom out by 40%
        y_range = (ylim[1] - ylim[0]) * 1.4
        
        self.ax.set_xlim(x_center - x_range/2, x_center + x_range/2)
        self.ax.set_ylim(y_center - y_range/2, y_center + y_range/2)
        self.canvas.draw()
    
    def fit_view(self):
        """Fit view to show all data points"""
        data = self.get_position_data()
        
        if data['history']:
            all_x = [data['x']] + [pos[0] for pos in data['history']] + [0]  # Include origin
            all_y = [data['y']] + [pos[1] for pos in data['history']] + [0]
            
            # Calculate dynamic margin based on data range
            x_range = max(all_x) - min(all_x)
            y_range = max(all_y) - min(all_y)
            margin = max(0.5, max(x_range, y_range) * 0.1)  # Dynamic margin
            
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
            
            # Ensure minimum range for visibility
            min_range = 2.0
            if x_max - x_min < min_range:
                x_center = (x_min + x_max) / 2
                x_min, x_max = x_center - min_range/2, x_center + min_range/2
            
            if y_max - y_min < min_range:
                y_center = (y_min + y_max) / 2
                y_min, y_max = y_center - min_range/2, y_center + min_range/2
            
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
        else:
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
        
        self.canvas.draw()
    
    def center_on_robot(self):
        """Center view on current robot position"""
        data = self.get_position_data()
        
        # Get current view range
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]
        
        # Center on robot position
        self.ax.set_xlim(data['x'] - x_range/2, data['x'] + x_range/2)
        self.ax.set_ylim(data['y'] - y_range/2, data['y'] + y_range/2)
        self.canvas.draw()

    def calculate_movement(self, angle_deg, distance_m):
        """Calculate movement from current position based on heading and distance"""
        # Convert angle to radians
        angle_rad = math.radians(angle_deg)
        
        # Calculate movement delta (assuming 0 degrees is North/Y-axis positive)
        # Use the cumulative angle directly for movement direction
        delta_x = distance_m * math.sin(angle_rad)
        delta_y = distance_m * math.cos(angle_rad)
        
        return delta_x, delta_y
    
    def update_position_data(self, angle, distance):
        """Update position data thread-safely"""
        with self.data_lock:
            self.current_angle = angle  # This is cumulative angle from initial position
            self.current_distance += distance  # Accumulate total distance traveled
            
            # Calculate movement from previous position using current angle and received distance
            if not self.first_data:
                # Use current cumulative angle for movement direction
                # distance parameter is the movement distance from previous point
                delta_x, delta_y = self.calculate_movement(angle, distance)
                
                # Update cumulative position
                self.cumulative_x += delta_x
                self.cumulative_y += delta_y
                
                # Add to history
                self.position_history.append((self.cumulative_x, self.cumulative_y))
                if len(self.position_history) > self.max_history:
                    self.position_history.pop(0)
            else:
                # First data point - set as origin
                self.cumulative_x = 0.0
                self.cumulative_y = 0.0
                self.current_distance = 0.0  # Initialize total distance
                self.position_history.append((0.0, 0.0))
                self.first_data = False
            
            # No need to store previous values since angle is cumulative
            # and distance is always relative to previous point
    
    def get_position_data(self):
        """Get current position data thread-safely"""
        with self.data_lock:
            return {
                'angle': self.current_angle,
                'distance': self.current_distance,
                'x': self.cumulative_x,
                'y': self.cumulative_y,
                'history': self.position_history.copy()
            }
    
    def update_display(self):
        """Update the display with current data"""
        data = self.get_position_data()
        
        # Calculate distance from origin
        origin_distance = math.sqrt(data['x']**2 + data['y']**2)
        
        # Update text labels
        self.angle_label.config(text=f"{data['angle']:.2f} deg")
        self.distance_label.config(text=f"{data['distance']:.4f}m")
        self.x_label.config(text=f"{data['x']:.4f}m")
        self.y_label.config(text=f"{data['y']:.4f}m")
        
        # Update statistics
        self.total_dist_label.config(text=f"{data['distance']:.4f}m")
        self.origin_dist_label.config(text=f"{origin_distance:.4f}m")
        self.points_label.config(text=f"{len(data['history'])}")
        
        # Update plot
        self.current_point.set_data([data['x']], [data['y']])
        
        # Update trail
        if data['history']:
            x_trail = [pos[0] for pos in data['history']]
            y_trail = [pos[1] for pos in data['history']]
            self.trail_line.set_data(x_trail, y_trail)
        
        # Update direction arrow
        if len(data['history']) > 1:  # Only show arrow if robot has moved
            arrow_length = 0.3
            angle_rad = math.radians(data['angle'])
            arrow_end_x = data['x'] + arrow_length * math.sin(angle_rad)
            arrow_end_y = data['y'] + arrow_length * math.cos(angle_rad)
            
            self.direction_arrow.set_position((arrow_end_x, arrow_end_y))
            self.direction_arrow.xy = (data['x'], data['y'])
        
        # Auto-scale plot only if enabled
        if self.auto_scale_var.get():
            all_x = [data['x']] + [pos[0] for pos in data['history']] + [0]  # Include origin
            all_y = [data['y']] + [pos[1] for pos in data['history']] + [0]
            
            if all_x and all_y:
                # Calculate dynamic margin based on data range
                x_range = max(all_x) - min(all_x)
                y_range = max(all_y) - min(all_y)
                margin = max(0.5, max(x_range, y_range) * 0.1)  # Dynamic margin
                
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin
                
                # Ensure minimum range for visibility
                min_range = 2.0
                if x_max - x_min < min_range:
                    x_center = (x_min + x_max) / 2
                    x_min, x_max = x_center - min_range/2, x_center + min_range/2
                
                if y_max - y_min < min_range:
                    y_center = (y_min + y_max) / 2
                    y_min, y_max = y_center - min_range/2, y_center + min_range/2
                
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
        
        self.canvas.draw()
    
    def connect_to_pi(self):
        """Connect to Raspberry Pi"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.pi_host, self.pi_port))
            self.connected = True
            return True
        except Exception as e:
            messagebox.showerror("Connection Error", f"Cannot connect to Pi: {e}")
            return False
    
    def disconnect_from_pi(self):
        """Disconnect from Raspberry Pi"""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
    def receive_data_loop(self):
        """Data receiving loop (runs in separate thread)"""
        buffer = ""
        
        while self.running and self.connected:
            try:
                if self.socket:
                    data = self.socket.recv(1024).decode('utf-8')
                    if not data:
                        break
                    
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            try:
                                json_data = json.loads(line.strip())
                                angle = json_data.get('angle', 0.0)
                                distance = json_data.get('distance', 0.0)
                                
                                self.update_position_data(angle, distance)
                                
                            except json.JSONDecodeError:
                                pass
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.running:
                    print(f"Data receive error: {e}")
                break
        
        self.connected = False
    
    def toggle_connection(self):
        """Toggle connection to Pi"""
        if not self.connected:
            # Get connection settings
            self.pi_host = self.ip_entry.get().strip()
            try:
                self.pi_port = int(self.port_entry.get().strip())
            except ValueError:
                messagebox.showerror("Error", "Port must be a number")
                return
            
            # Connect
            if self.connect_to_pi():
                self.running = True
                self.receive_thread = threading.Thread(target=self.receive_data_loop, daemon=True)
                self.receive_thread.start()
                
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground="green")
                
                # Disable connection settings
                self.ip_entry.config(state='disabled')
                self.port_entry.config(state='disabled')
        else:
            # Disconnect
            self.running = False
            self.disconnect_from_pi()
            
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
            
            # Enable connection settings
            self.ip_entry.config(state='normal')
            self.port_entry.config(state='normal')
    
    def reset_origin(self):
        """Reset the origin point"""
        with self.data_lock:
            self.cumulative_x = 0.0
            self.cumulative_y = 0.0
            self.current_distance = 0.0  # Reset total distance
            self.position_history.clear()
            self.position_history.append((0.0, 0.0))
            self.first_data = True
        
        # Reset plot view
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.canvas.draw()
    
    def clear_trail(self):
        """Clear the position trail"""
        with self.data_lock:
            self.position_history.clear()
            if hasattr(self, 'cumulative_x'):
                self.position_history.append((self.cumulative_x, self.cumulative_y))
        
        self.trail_line.set_data([], [])
        self.canvas.draw()
    
    def save_path_data(self):
        """Save the position history to a file"""
        data = self.get_position_data()
        
        # Create filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"robot_path_{timestamp}.json"
        
        # Prepare detailed data
        save_data = {
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
            'current_position': {
                'x': data['x'],
                'y': data['y'],
                'angle': data['angle'],
                'total_distance': data['distance']
            },
            'path_history': data['history'],
            'statistics': {
                'total_points': len(data['history']),
                'distance_from_origin': math.sqrt(data['x']**2 + data['y']**2)
            }
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            messagebox.showinfo("Path Data Saved", f"Position history saved to {filename}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Cannot save file: {e}")
    
    def update_loop(self):
        """Main update loop for GUI"""
        if self.connected:
            self.update_display()
        
        # Schedule next update
        self.root.after(100, self.update_loop)  # Update every 100ms
    
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        self.disconnect_from_pi()
        self.root.destroy()

def main():
    """Main function"""
    root = tk.Tk()
    app = PositionViewer(root)
    
    # Start update loop
    app.update_loop()
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Start GUI
    root.mainloop()

if __name__ == "__main__":
    main() 