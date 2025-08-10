import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VacuumController(Node):
    def __init__(self):
        super().__init__('vacuum_controller')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.set_vacuum(True)

    def set_vacuum(self, on: bool):
        msg = String()
        msg.data = f"VACUUM:{'ON' if on else 'OFF'}"
        self.tx_pub.publish(msg)
        self.get_logger().info(f"Sent: VACUUM:{'ON' if on else 'OFF'}")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = VacuumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
