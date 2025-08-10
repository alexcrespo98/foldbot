import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json

# Pin for napkin sensor (photoresistor or similar)
PIN_NAPKIN_SENSOR = 14  # A0 analog pin

class NapkinSensor(Node):
    def __init__(self):
        super().__init__('napkin_sensor')
        
        # Subscribe to Arduino responses
        self.arduino_rx_sub = self.create_subscription(
            String, '/arduino_rx', self.arduino_rx_callback, 10)
        
        # Publish napkin detection state (fixed topic name)
        self.publisher_ = self.create_publisher(Bool, 'napkin_detected', 10)
        
        # Request periodic readings from Arduino
        self.arduino_tx_pub = self.create_publisher(String, '/arduino_tx', 10)
        self.timer = self.create_timer(0.1, self.request_reading)

    def request_reading(self):
        # Request analog read from Arduino for napkin sensor
        command = {
            "command": "analog_read",
            "pin": PIN_NAPKIN_SENSOR
        }
        msg = String()
        msg.data = json.dumps(command)
        self.arduino_tx_pub.publish(msg)

    def arduino_rx_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if (data.get("type") == "analog_read" and 
                data.get("pin") == PIN_NAPKIN_SENSOR):
                
                # Convert analog reading to boolean (adjust threshold as needed)
                analog_value = data.get("value", 0)
                threshold = 512  # Middle of 0-1023 range, adjust based on sensor
                detected = analog_value > threshold
                
                msg_out = Bool()
                msg_out.data = detected
                self.publisher_.publish(msg_out)
                self.get_logger().debug(f'Napkin sensor: {analog_value} -> {"detected" if detected else "not detected"}')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().debug(f'Invalid Arduino response: {e}')

def main():
    rclpy.init()
    node = NapkinSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

