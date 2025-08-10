# right_limit_switch.py
# Publishes state of right (max X) limit switch

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pyfirmata import Arduino, util

PIN_RIGHT_LIMIT = 10

class RightLimitSwitch(Node):
    def __init__(self):
        super().__init__('right_limit_switch')
        self.board = Arduino('/arduino_rx')
        self.limit_pin = self.board.digital[PIN_RIGHT_LIMIT]
        self.limit_pin.mode = 0  # INPUT
        self.pub = self.create_publisher(Bool, 'right_limit_state', 10)
        self.timer = self.create_timer(0.05, self.read_switch)

    def read_switch(self):
        state = bool(self.limit_pin.read())
        msg = Bool()
        msg.data = state
        self.pub.publish(msg)
        self.get_logger().debug(f'Right limit: {state}')

def main(args=None):
    rclpy.init(args=args)
    node = RightLimitSwitch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
