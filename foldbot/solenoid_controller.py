import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SolenoidController(Node):
    def __init__(self):
        super().__init__('solenoid_controller')
        self.tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.activate_solenoid(True)

    def activate_solenoid(self, activate: bool):
        msg = String()
        msg.data = f"SOLENOID:{'ON' if activate else 'OFF'}"
        self.tx_pub.publish(msg)
        self.get_logger().info(f"Sent: SOLENOID:{'ON' if activate else 'OFF'}")

    def arduino_rx_callback(self, msg):
        self.get_logger().info(f"Arduino says: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SolenoidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
