# Foldbot Final Project Summary

## System Overview

**Foldbot** is a ROS2-controlled automation project designed for napkin manipulation, intended as the first stage of a napkin/silverware folding system. The platform uses a 2020 aluminum extrusion frame with 3D printed holders and mounts.

### Hardware Components

- **Motion**:  
  - **X and Z Axes**: NEMA17 stepper motors, driven by DRV8825 stepper drivers, controlled by an Arduino Uno with a CNC shield.
  - **Servos**: Standard hobby servos (model unspecified), used for folding actions.
  - **Vacuum**: Initial design used a PC fan for suction (not enough force). Switched to a vacuum pump via a 5V relay, which had sufficient suction but could not reliably isolate a single napkin from a stack. Final approach was to pick up a single napkin from a slotted surface.
- **Sensors**:
  - **Limit Switches**: For homing and motion endpoints on X axis.
  - **Napkin Sensor**: Not actually wired in, but the architecture and code support it, allowing easy future integration or toggling its use in ROS2.

### Software Architecture

- **Communication**:  
  - All ROS2 nodes communicate with the Arduino via two topics: `arduino_tx` (commands to Arduino), `arduino_rx` (status/sensor updates).
  - The Arduino acts as a hardware bridge, interpreting string commands to actuate motors/servos/vacuum/solenoid and sending back status or sensor data.
- **Adjustability**:  
  - The servo sequence (angles, timing) and Z axis height (steps/rotations) are adjustable in a single location in `main_controller.py`. This is a key benefit of integrating ROS2: parameters and sequence logic can be easily edited and tuned in software, without reflashing Arduino firmware.
- **Main Logic** (in `main_controller.py`):  
  1. Home X axis (using left limit switch)
  2. Turn vacuum on
  3. Lift Z axis
  4. Move X axis to right endstop
  5. Vacuum off
  6. Return X to left
  7. Lower Z
  8. (Optionally) Wait for napkin sensor ("OFF") if used
  9. Run servo folding sequence (servo angles and sequence configurable)
  10. End of cycle

- **Modularity**:  
  - Each actuator and sensor type has its own ROS2 node (e.g., `servo_controller`, `vacuum_controller`, `solenoid_controller`, `gantry_controller`).
  - All hardware interactions are string-based commands, logic awaits required acknowledgments or sensor changes before proceeding.

### Project Scope and Challenges

- **Original Vision**:  
  - Intended to automate both napkin and silverware folding/placement. The project proved too complex for one phase, so this stage focuses solely on napkin pickup and folding.
- **Vacuum Pickup**:  
  - PC fan was insufficient for suction.
  - Upgraded to a vacuum pump with relay, which worked for lifting napkins but not for reliably picking just one from a stack.
  - Solution: Pick up single napkin from a slotted surface.
- **Sensor Use**:  
  - The napkin sensor is not physically present but is supported in the system and can be toggled on/off in ROS2, showcasing the flexibility of the architecture.

### Testing

The system was tested by running ten consecutive napkin pickup and fold cycles. All ten cycles resulted in successful napkin movement and correct folding, demonstrating reliable operation for its intended task.

### Key Benefits of ROS2 Integration

- Centralized adjustment of motion and actuation parameters (servo sequence, Z height) in Python code (`main_controller.py`).
- Modular node structure for easy expansion, debugging, and hardware swaps.
- Logic and sensor usage can be enabled/disabled or tuned without reflashing Arduino code, simply by editing ROS2 Python scripts.

### Potential Improvements

- Adding limit switches to the Z axis would improve safety and prevent damage; earlier tests revealed that missing Z limit switches could lead to overtravel and mechanical issues.
- Increasing the sensor polling rate in software could provide smoother X axis motion, as more responsive feedback would allow for finer motion control.
- Additional improvements could include wiring in and utilizing the napkin sensor for automated detection, and further optimizing the vacuum pickup method to better separate napkins.

---

This summary covers the sequence, hardware, communication, control logic, project history, testing results, and the advantages of the ROS2-based design.
