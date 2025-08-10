import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GantryController(Node):
    def __init__(self):
        super().__init__('gantry_controller')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.send_gantry_home()

    def send_gantry_home(self):
        msg = String()
        msg.data = "GANTRY:HOME"
        self.tx_pub.publish(msg)
        self.get_logger().info("Sent: GANTRY:HOME")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = GantryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()