import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

TCP_IP = 'host.docker.internal'  # This lets Docker container reach Windows host
TCP_PORT = 5005

class ArduinoTCPBridge(Node):
    def __init__(self):
        super().__init__('arduino_tcp_bridge')
        self.publisher_ = self.create_publisher(String, 'arduino_rx', 10)
        self.subscription = self.create_subscription(
            String,
            'arduino_tx',
            self.write_to_socket,
            10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((TCP_IP, TCP_PORT))
        self.get_logger().info("Connected to Arduino TCP bridge")
        threading.Thread(target=self.read_from_socket, daemon=True).start()

    def read_from_socket(self):
        while rclpy.ok():
            data = self.sock.recv(1024)
            if data:
                msg = String()
                msg.data = data.decode('utf-8').rstrip()
                self.publisher_.publish(msg)

    def write_to_socket(self, msg):
        self.sock.sendall((msg.data + '\n').encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoTCPBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()