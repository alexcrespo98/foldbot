# Foldbot

## Overview

Foldbot is a ROS2-based automation project designed to pick up and fold napkins using stepper motors, servos, and a vacuum system, all controlled via an Arduino Uno. The system is modular and leverages ROS2 for flexibility and easy parameter tuning.

---

## Quick Start

### 1. Build ROS2 Workspace

Make sure you are in the root of your ROS2 workspace (e.g., `~/foldbot_ws`):

```bash
colcon build
```

### 2. Source the Workspace

After a successful build, source the install setup file:

```bash
source install/setup.bash
```

### 3. Flash the Arduino

- Use the Arduino IDE or similar tool to upload the `foldbot_arduino` code found in the main `foldbot` folder to your Arduino Uno (with CNC shield and hardware attached).

### 4. Plug in Arduino

- Connect your Arduino Uno to the PC via USB.

### 5. Start the Arduino Serial Bridge

On your PC (the one connected to the Arduino), run:

```bash
python3 foldbot/arduino_serial_bridge.py
```

This script bridges ROS2 messages to the Arduino over serial.

### 6. Start the ROS2 TCP Client

In your ROS2 environment, run the ROS2 client:

```bash
python3 foldbot/ros2_tcp_serial_client.py
```

(This is the main client for ROS2-to-serial communications.)

### 7. Launch Main Controller

On your ROS2 environment (terminal where you sourced the workspace):

```bash
ros2 run foldbot main_controller
```

### 8. Sequence Start

- The system will wait for a handshake between ROS2 and the Arduino.
- Once the handshake completes, the napkin movement and folding sequence will begin automatically.

---

## Notes

- **File Locations:**  
  - Arduino code: `foldbot/foldbot_arduino`
  - Serial bridge script: `foldbot/arduino_serial_bridge.py`
  - ROS2 client: `foldbot/ros2_tcp_serial_client.py`
  - Main controller: `foldbot/foldbot/main_controller.py`
- Ensure all hardware is properly connected before starting the sequence.
- You can tune servo angles, Z height, and other parameters in `foldbot/main_controller.py`.
- The system is modular: other nodes (like servo or vacuum controllers) can be run/tested independently if needed.

---

## Troubleshooting

- If the handshake does not complete, check your Arduino serial connection and ensure the serial bridge script is running.
- To restart the sequence, simply restart the `main_controller` node.

---

**For details on hardware and logic, see `PROJECT_SUMMARY.md`.**
