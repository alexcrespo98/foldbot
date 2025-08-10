import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class RightLimitSwitch(Node):
    def __init__(self):
        super().__init__('right_limit_switch')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.switch_pub = self.create_publisher(Bool, '/right_limit_switch', 10)
        self.query_switch()

    def query_switch(self):
        msg = String()
        msg.data = "RIGHT_LIMIT_SWITCH:QUERY"
        self.tx_pub.publish(msg)
        self.get_logger().info("Sent: RIGHT_LIMIT_SWITCH:QUERY")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")
        # Parse for 'right_limit_switch:1' or 'right_limit_switch:0'
        if msg.data.startswith('right_limit_switch:'):
            value = msg.data.split(':')[1].strip()
            bool_msg = Bool()
            bool_msg.data = value == '1'
            self.switch_pub.publish(bool_msg)
            self.get_logger().info(f"Published to /right_limit_switch: {bool_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RightLimitSwitch()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
