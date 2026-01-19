# TeamAvengers_Robotics
EIE Second Year Project of Imperial College London

## Project Overview

This is an intelligent balancing robot project with integrated visual following and obstacle avoidance capabilities.

## Main Features

- **Balance Control**: Self-balancing robot system
- **Visual Following**: Computer vision-based object following using ping-pong ball detection
- **Obstacle Avoidance**: Smart obstacle detection with directional movement blocking
- **Remote Control**: Joystick control interface with real-time feedback
- **Dual Control Modes**: Manual joystick control and autonomous visual following

## Quick Start

### Launch Robot Control UI

```bash
cd communication-code
python start_robot_ui.py
```

Or use the main UI file directly:
```bash
python robot_control_ui.py
```

### Command Line Control
```bash
cd communication-code
python pc_client.py
```

## How to Use

1. **Connect to Robot**
   - Enter your Raspberry Pi IP address (default: 192.168.137.23)
   - Click "Connect" button

2. **Manual Control**
   - Use joystick for speed (left stick) and steering (right stick)
   - Real-time control data is displayed

3. **Visual Following Mode**
   - Click "Visual Following: OFF" button to enable vision mode
   - The robot will automatically follow ping-pong balls detected by the camera
   - Click again to return to manual control

4. **Obstacle Avoidance**
   - Click "Obstacle Avoidance: OFF" button to enable obstacle detection
   - **Front obstacle**: Blocks forward movement, allows backward and turning
   - **Rear obstacle**: Blocks backward movement, allows forward and turning
   - **Always allows steering** regardless of obstacles

## Testing Features

### Test Obstacle Avoidance
Run the obstacle simulator to test the avoidance feature:
```bash
python obstacle_simulator.py
```

This will simulate different obstacle conditions:
- `CLEAR` - No obstacles
- `FRONT_OBSTACLE` - Obstacle in front
- `REAR_OBSTACLE` - Obstacle behind

## Project Structure

```
TeamAvengers_Robotics/
   communication-code/          # Communication control code
      robot_control_ui.py     # Main graphical interface
      pc_client.py            # Command line client
      start_robot_ui.py       # UI launcher script
      pi_main.py              # Robot server (runs on Raspberry Pi)
      obstacle_simulator.py   # Obstacle testing tool
      pi_server.py            # Alternative server implementation
   opencv-code/                # Computer vision code
   esp32-code/                 # ESP32 microcontroller code
   pingpangDetection/          # Ping pong ball detection system
       pingpangControl.py      # Vision control logic
   balance-robot-resource/     # Balance robot resources
```

## Technical Details

### Communication Protocol
- **Manual Control**: `move::speed,steering` (e.g., `move::5.0,0.2`)
- **Start Following**: `follow` 
- **Stop Following**: `stop_follow`
- **Enable Obstacle Avoidance**: `obstacle_avoidance::on`
- **Disable Obstacle Avoidance**: `obstacle_avoidance::off`
- **Connection Test**: `ping`

### Obstacle Avoidance Logic
- **Front Obstacle Detected**: Blocks positive speed (forward movement)
- **Rear Obstacle Detected**: Blocks negative speed (backward movement) 
- **Steering Always Allowed**: Turning is never blocked by obstacles
- **Status Updates**: Real-time obstacle status display via port 6666

### Control Ranges
- **Speed**: -10.0 to 10.0 (forward/backward)
- **Steering**: -0.2 to 0.2 (left/right)
- **Update Rate**: 10Hz for manual control

## Requirements

```bash
pip install PyQt6 pygame pyserial opencv-python
```

## Hardware Setup

1. **Raspberry Pi**: Running `pi_main.py` server
2. **ESP32**: Connected via serial for motor control
3. **Camera**: For visual following (ping-pong ball detection)
4. **Obstacle Sensors**: For real-time obstacle detection
5. **Joystick**: USB game controller for manual control

## Development & Testing

### UI Development
- The UI automatically connects to obstacle status on port 6666
- Obstacle status updates are displayed in real-time
- Both visual following and obstacle avoidance can be enabled simultaneously

### Testing Workflow
1. Start the robot server: `python pi_main.py`
2. Launch the UI: `python start_robot_ui.py`
3. Test obstacles: `python obstacle_simulator.py`
4. Connect and test all features through the UI

## Team
Team Avengers - Imperial College London EIE
