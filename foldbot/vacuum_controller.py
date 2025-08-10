# vacuum_controller.py
# Controls the vacuum pump

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from pyfirmata import Arduino, util

PIN_VACUUM = 13

class VacuumController(Node):
    def __init__(self):
        super().__init__('vacuum_controller')
        self.board = Arduino('/dev/arduino_rx')
        self.vacuum_pin = self.board.digital[PIN_VACUUM]
        self.vacuum_pin.mode = 1  # OUTPUT
        self.sub = self.create_subscription(
            Bool, 'vacuum_cmd', self.vacuum_callback, 10)

    def vacuum_callback(self, msg):
        self.vacuum_pin.write(1 if msg.data else 0)
        self.get_logger().info(f'Vacuum pump {"ON" if msg.data else "OFF"}')

def main(args=None):
    rclpy.init(args=args)
    node = VacuumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
