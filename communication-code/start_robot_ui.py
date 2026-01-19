#!/usr/bin/env python3
"""
Robot Control UI Launcher
Simple launcher for the robot control interface
"""

import sys
import os

# Add current directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

def main():
    print("Starting Robot Control UI...")
    
    try:
        from robot_control_ui import QApplication, RobotControlUI
        
        app = QApplication(sys.argv)
        app.setApplicationName("Robot Control")
        app.setApplicationVersion("1.0")
        
        window = RobotControlUI()
        
        print("UI started successfully!")
        print("Connect to your robot and use the Visual Following button to toggle vision mode.")
        
        sys.exit(app.exec())
        
    except ImportError as e:
        print(f"Import error: {e}")
        print("Please install required packages:")
        print("pip install PyQt6 pygame")
        sys.exit(1)
    except Exception as e:
        print(f"Error starting UI: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 