# left_limit_switch.py
# Publishes state of left (min X) limit switch via Arduino communication topics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json

PIN_LEFT_LIMIT = 9

class LeftLimitSwitch(Node):
    def __init__(self):
        super().__init__('left_limit_switch')
        
        # Subscribe to Arduino responses
        self.arduino_rx_sub = self.create_subscription(
            String, '/arduino_rx', self.arduino_rx_callback, 10)
        
        # Publish limit switch state
        self.pub = self.create_publisher(Bool, 'left_limit_state', 10)
        
        # Request periodic readings from Arduino
        self.arduino_tx_pub = self.create_publisher(String, '/arduino_tx', 10)
        self.timer = self.create_timer(0.05, self.request_reading)

    def request_reading(self):
        # Request digital read from Arduino
        command = {
            "command": "digital_read",
            "pin": PIN_LEFT_LIMIT
        }
        msg = String()
        msg.data = json.dumps(command)
        self.arduino_tx_pub.publish(msg)

    def arduino_rx_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if (data.get("type") == "digital_read" and 
                data.get("pin") == PIN_LEFT_LIMIT):
                
                state = bool(data.get("value", False))
                msg_out = Bool()
                msg_out.data = state
                self.pub.publish(msg_out)
                self.get_logger().debug(f'Left limit: {state}')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().debug(f'Invalid Arduino response: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LeftLimitSwitch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
