# servo_controller.py
# Controls a servo for folding arm via Arduino communication topics

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

PIN_SERVO = 11

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Arduino communication publisher
        self.arduino_tx_pub = self.create_publisher(String, '/arduino_tx', 10)
        
        # Listen for servo commands
        self.sub = self.create_subscription(
            String, 'servo_cmd', self.servo_command_callback, 10)

    def servo_command_callback(self, msg):
        # Map string commands to angles
        if msg.data == "fold":
            angle = 90  # Fold position
        elif msg.data == "reset":
            angle = 0   # Reset position
        else:
            self.get_logger().warn(f'Unknown servo command: {msg.data}')
            return
            
        # Send servo command to Arduino
        command = {
            "command": "servo_set",
            "pin": PIN_SERVO,
            "angle": max(0, min(180, angle))
        }
        msg_arduino = String()
        msg_arduino.data = json.dumps(command)
        self.arduino_tx_pub.publish(msg_arduino)
        
        self.get_logger().info(f'Servo command "{msg.data}" -> angle: {angle}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
