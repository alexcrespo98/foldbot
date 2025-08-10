# solenoid_controller.py
# Controls the solenoid actuator via Arduino communication topics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json

PIN_SOLENOID = 12

class SolenoidController(Node):
    def __init__(self):
        super().__init__('solenoid_controller')
        
        # Arduino communication publisher
        self.arduino_tx_pub = self.create_publisher(String, '/arduino_tx', 10)
        
        # Listen for solenoid commands
        self.sub = self.create_subscription(
            Bool, 'solenoid_cmd', self.solenoid_callback, 10)

    def solenoid_callback(self, msg):
        # Send digital write command to Arduino
        command = {
            "command": "digital_write",
            "pin": PIN_SOLENOID,
            "value": 1 if msg.data else 0
        }
        msg_arduino = String()
        msg_arduino.data = json.dumps(command)
        self.arduino_tx_pub.publish(msg_arduino)
        
        self.get_logger().info(f'Solenoid {"ON" if msg.data else "OFF"}')

def main(args=None):
    rclpy.init(args=args)
    node = SolenoidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
