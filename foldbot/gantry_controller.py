# gantry_controller.py
# Controls X and Z stepper motors via Arduino communication topics

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# X axis pins
PIN_X_STEP = 2
PIN_X_DIR  = 5

# Z axis pins
PIN_Z_STEP = 4
PIN_Z_DIR  = 7

# Both axes share enable
PIN_ENABLE = 8

class GantryController(Node):
    def __init__(self):
        super().__init__('gantry_controller')
        
        # Arduino communication publisher
        self.arduino_tx_pub = self.create_publisher(String, '/arduino_tx', 10)
        
        # Listen for gantry commands
        self.subscription = self.create_subscription(
            String, 'gantry_cmd', self.listener_callback, 10)
        
        # Enable steppers on startup
        self.enable_steppers(True)

    def enable_steppers(self, enable=True):
        """Enable or disable stepper motors"""
        command = {
            "command": "digital_write",
            "pin": PIN_ENABLE,
            "value": 0 if enable else 1  # LOW to enable stepper
        }
        msg = String()
        msg.data = json.dumps(command)
        self.arduino_tx_pub.publish(msg)
        self.get_logger().info(f'Steppers {"enabled" if enable else "disabled"}')

    def send_stepper_command(self, axis, steps, direction):
        """Send stepper movement command to Arduino"""
        if axis == "x":
            step_pin = PIN_X_STEP
            dir_pin = PIN_X_DIR
            # Direction logic: customize as needed for your wiring!
            dir_value = 1 if direction == "right" else 0
        elif axis == "z":
            step_pin = PIN_Z_STEP
            dir_pin = PIN_Z_DIR
            # Direction logic: customize as needed for your wiring!
            dir_value = 1 if direction == "up" else 0
        else:
            self.get_logger().error(f"Unknown axis: {axis}")
            return

        # Set direction
        dir_command = {
            "command": "digital_write",
            "pin": dir_pin,
            "value": dir_value
        }
        msg = String()
        msg.data = json.dumps(dir_command)
        self.arduino_tx_pub.publish(msg)
        
        # Send step command
        step_command = {
            "command": "stepper_move",
            "pin": step_pin,
            "steps": abs(steps),
            "delay_us": 1000  # 1ms delay between steps
        }
        msg = String()
        msg.data = json.dumps(step_command)
        self.arduino_tx_pub.publish(msg)

    def listener_callback(self, msg):
        # Example: "x:right:100" or "z:up:50"
        try:
            axis, direction, steps = msg.data.split(':')
            steps = int(steps)
            self.send_stepper_command(axis.lower(), steps, direction.lower())
            self.get_logger().info(f'Commanded {axis}-{direction} by {steps} steps')
        except Exception as e:
            self.get_logger().error(f'Invalid gantry command: {msg.data} ({e})')

def main(args=None):
    rclpy.init(args=args)
    node = GantryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
