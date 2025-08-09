import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class NapkinSensor(Node):
    def __init__(self):
        super().__init__('napkin_sensor')
        self.publisher_ = self.create_publisher(Bool, 'napkin_detected', 10)
        self.timer = self.create_timer(0.1, self.check_napkin)

    def check_napkin(self):
        detected = True  # Replace with photoresistor logic
        msg = Bool()
        msg.data = detected
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = NapkinSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

