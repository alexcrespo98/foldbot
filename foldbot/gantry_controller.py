# gantry_controller.py
# Controls X and Z stepper motors using CNC Shield via Firmata.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyfirmata import Arduino, util
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
        self.board = Arduino('/arduino_rx')   # Change if your board is at a different port!
        # X axis setup
        self.x_step_pin = self.board.digital[PIN_X_STEP]
        self.x_dir_pin  = self.board.digital[PIN_X_DIR]
        # Z axis setup
        self.z_step_pin = self.board.digital[PIN_Z_STEP]
        self.z_dir_pin  = self.board.digital[PIN_Z_DIR]
        # Enable pin
        self.enable_pin = self.board.digital[PIN_ENABLE]

        self.x_step_pin.mode = 1  # OUTPUT
        self.x_dir_pin.mode = 1
        self.z_step_pin.mode = 1
        self.z_dir_pin.mode = 1
        self.enable_pin.mode = 1

        self.enable(True)
        self.subscription = self.create_subscription(
            String, 'gantry_cmd', self.listener_callback, 10)

    def enable(self, en=True):
        self.enable_pin.write(0 if en else 1)  # LOW to enable stepper

    def step(self, axis, steps, direction):
        if axis == "x":
            dir_pin = self.x_dir_pin
            step_pin = self.x_step_pin
        elif axis == "z":
            dir_pin = self.z_dir_pin
            step_pin = self.z_step_pin
        else:
            self.get_logger().error(f"Unknown axis: {axis}")
            return

        # Direction logic: customize as needed for your wiring!
        if axis == "x":
            dir_pin.write(1 if direction == "right" else 0)
        elif axis == "z":
            dir_pin.write(1 if direction == "up" else 0)
        for _ in range(abs(steps)):
            step_pin.write(1)
            time.sleep(0.001)
            step_pin.write(0)
            time.sleep(0.001)

    def listener_callback(self, msg):
        # Example: "x:right:100" or "z:up:50"
        try:
            axis, direction, steps = msg.data.split(':')
            steps = int(steps)
            self.step(axis.lower(), steps, direction.lower())
            self.get_logger().info(f'Moved {axis}-{direction} by {steps} steps')
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
