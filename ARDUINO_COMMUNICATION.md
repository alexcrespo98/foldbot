# Arduino Communication Architecture

This document describes the refactored Arduino communication architecture for the foldbot project.

## Overview

The hardware controller nodes have been refactored to communicate exclusively with the Arduino through standardized ROS topics `/arduino_tx` and `/arduino_rx`, instead of using direct serial communication or pyfirmata.

## Architecture Changes

### Before Refactor
- Each hardware controller directly communicated with Arduino using pyfirmata
- Controllers created direct Arduino board connections
- No standardized message format
- Topic name mismatches between nodes

### After Refactor  
- All hardware communication goes through `/arduino_tx` (commands) and `/arduino_rx` (responses)
- Standardized JSON message format for all Arduino communication
- No direct Arduino connections in controller nodes
- Fixed topic name mismatches

## Communication Topics

### /arduino_tx (Commands to Arduino)
Controllers publish commands to this topic using JSON format:

```json
{
  "command": "digital_write|servo_set|stepper_move|digital_read|analog_read",
  "pin": <pin_number>,
  "value": <value>,
  "...": "<additional_parameters>"
}
```

### /arduino_rx (Responses from Arduino)
Arduino bridge publishes sensor data and responses to this topic:

```json
{
  "type": "digital_read|analog_read|response",
  "pin": <pin_number>,
  "value": <value>,
  "timestamp": <timestamp>
}
```

## Message Format Details

### Command Types

#### digital_write
```json
{
  "command": "digital_write",
  "pin": 12,
  "value": 1
}
```

#### servo_set
```json
{
  "command": "servo_set", 
  "pin": 11,
  "angle": 90
}
```

#### stepper_move
```json
{
  "command": "stepper_move",
  "pin": 2,
  "steps": 100,
  "delay_us": 1000
}
```

#### digital_read
```json
{
  "command": "digital_read",
  "pin": 9
}
```

#### analog_read
```json
{
  "command": "analog_read",
  "pin": 14
}
```

### Response Types

#### digital_read response
```json
{
  "type": "digital_read",
  "pin": 9,
  "value": 1,
  "timestamp": 1234567890.123
}
```

#### analog_read response
```json
{
  "type": "analog_read", 
  "pin": 14,
  "value": 512,
  "timestamp": 1234567890.123
}
```

## Pin Assignments

| Component | Pin | Type |
|-----------|-----|------|
| X Stepper Step | 2 | Digital Output |
| X Stepper Direction | 5 | Digital Output |  
| Z Stepper Step | 4 | Digital Output |
| Z Stepper Direction | 7 | Digital Output |
| Stepper Enable | 8 | Digital Output |
| Left Limit Switch | 9 | Digital Input |
| Right Limit Switch | 10 | Digital Input |
| Servo | 11 | PWM Output |
| Solenoid | 12 | Digital Output |
| Vacuum Pump | 13 | Digital Output |
| Napkin Sensor | 14 (A0) | Analog Input |

## Refactored Nodes

### Controllers (Publish to /arduino_tx)
- **gantry_controller.py** - Controls X/Z stepper motors
- **servo_controller.py** - Controls folding servo
- **solenoid_controller.py** - Controls solenoid actuator
- **vacuum_controller.py** - Controls vacuum pump

### Sensors (Subscribe to /arduino_rx)
- **left_limit_switch.py** - Publishes left limit switch state
- **right_limit_switch.py** - Publishes right limit switch state  
- **napkin_sensor.py** - Publishes napkin detection state

### Orchestrator (Uses existing ROS topics)
- **main_controller.py** - Orchestrates the folding sequence

## Topic Mappings Fixed

| Node | Old Topic | New Topic | Type |
|------|-----------|-----------|------|
| napkin_sensor | napkin_detected | napkin_detected | Bool |
| main_controller | napkin_present | napkin_detected | Bool |
| left_limit_switch | left_limit_state | left_limit_state | Bool |
| main_controller | left_limit_switch | left_limit_state | Bool |
| right_limit_switch | right_limit_state | right_limit_state | Bool |
| main_controller | right_limit_switch | right_limit_state | Bool |
| servo_controller | servo_angle (Int32) | servo_cmd (String) | String |

## Arduino Bridge Node

An Arduino bridge node (not included in this refactor) should be implemented to:

1. Subscribe to `/arduino_tx` for commands
2. Execute commands on physical Arduino hardware
3. Publish sensor readings and responses to `/arduino_rx`
4. Handle the actual pyfirmata/serial communication

## Benefits

1. **Hardware Abstraction** - Controllers don't need direct Arduino access
2. **Standardized Communication** - Consistent JSON message format
3. **Better Testing** - Easy to mock Arduino responses
4. **Centralized Hardware Access** - Single point of Arduino communication
5. **Topic Consistency** - Fixed mismatched topic names
6. **Maintainability** - Cleaner separation of concerns