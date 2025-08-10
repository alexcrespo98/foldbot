import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NapkinSensor(Node):
    def __init__(self):
        super().__init__('napkin_sensor')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.query_sensor()

    def query_sensor(self):
        msg = String()
        msg.data = "NAPKIN_SENSOR:QUERY"
        self.tx_pub.publish(msg)
        self.get_logger().info("Sent: NAPKIN_SENSOR:QUERY")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NapkinSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
