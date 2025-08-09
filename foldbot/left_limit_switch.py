# left_limit_switch.py
# Publishes state of left (min X) limit switch

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pyfirmata import Arduino, util

PIN_LEFT_LIMIT = 9

class LeftLimitSwitch(Node):
    def __init__(self):
        super().__init__('left_limit_switch')
        self.board = Arduino('/dev/ttyACM0')
        self.limit_pin = self.board.digital[PIN_LEFT_LIMIT]
        self.limit_pin.mode = 0  # INPUT
        self.pub = self.create_publisher(Bool, 'left_limit_state', 10)
        self.timer = self.create_timer(0.05, self.read_switch)

    def read_switch(self):
        state = bool(self.limit_pin.read())
        msg = Bool()
        msg.data = state
        self.pub.publish(msg)
        self.get_logger().debug(f'Left limit: {state}')

def main(args=None):
    rclpy.init(args=args)
    node = LeftLimitSwitch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
