import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LeftLimitSwitch(Node):
    def __init__(self):
        super().__init__('left_limit_switch')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.query_switch()

    def query_switch(self):
        msg = String()
        msg.data = "LEFT_LIMIT_SWITCH:QUERY"
        self.tx_pub.publish(msg)
        self.get_logger().info("Sent: LEFT_LIMIT_SWITCH:QUERY")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = LeftLimitSwitch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
