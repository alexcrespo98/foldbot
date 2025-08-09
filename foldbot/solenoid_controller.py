# solenoid_controller.py
# Controls the solenoid actuator

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pyfirmata import Arduino, util

PIN_SOLENOID = 12

class SolenoidController(Node):
    def __init__(self):
        super().__init__('solenoid_controller')
        self.board = Arduino('/dev/ttyACM0')
        self.solenoid_pin = self.board.digital[PIN_SOLENOID]
        self.solenoid_pin.mode = 1  # OUTPUT
        self.sub = self.create_subscription(
            Bool, 'solenoid_cmd', self.solenoid_callback, 10)

    def solenoid_callback(self, msg):
        self.solenoid_pin.write(1 if msg.data else 0)
        self.get_logger().info(f'Solenoid {"ON" if msg.data else "OFF"}')

def main(args=None):
    rclpy.init(args=args)
    node = SolenoidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
