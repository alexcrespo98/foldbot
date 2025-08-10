import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.set_servo_position(90)

    def set_servo_position(self, position):
        msg = String()
        msg.data = f"SERVO:{position}"
        self.tx_pub.publish(msg)
        self.get_logger().info(f"Sent: SERVO:{position}")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
