# servo_controller.py
# Controls a servo for folding arm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pyfirmata import Arduino, util

PIN_SERVO = 11

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.board = Arduino('/dev/ttyACM0')
        self.servo_pin = self.board.digital[PIN_SERVO]
        self.servo_pin.mode = 4  # SERVO
        self.sub = self.create_subscription(
            Int32, 'servo_angle', self.set_angle_callback, 10)

    def set_angle_callback(self, msg):
        angle = max(0, min(180, msg.data))
        self.servo_pin.write(angle)
        self.get_logger().info(f'Servo set to angle: {angle}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
