import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import time

SERVO_ANGLE = 180        # Main folding servo angle (degrees)
SOLENOID_ANGLE = 9       # "Solenoid" servo angle (degrees, dummy value)
STEPS_PER_ROTATION = 200 # Example value, set to your motor's steps/rev
TWO_ROTATIONS = 2 * STEPS_PER_ROTATION

CNC_LABELS = {
    "left_limit_switch": "X- end stop",
    "right_limit_switch": "X+ end stop",
    "napkin_sensor": "SPIN EN",
}

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.gantry_pub = self.create_publisher(String, 'gantry_cmd', 10)
        self.servo1_pub = self.create_publisher(String, 'servo_cmd', 10)
        self.servo2_pub = self.create_publisher(String, 'solenoid_cmd', 10)
        self.vacuum_pub = self.create_publisher(Bool, 'vacuum_cmd', 10)
        self.arduino_connected = False

        self.napkin_present = None
        self.at_left_limit = None
        self.at_right_limit = None

        self.create_subscription(Bool, 'napkin_present', self.napkin_callback, 10)
        self.create_subscription(Bool, 'left_limit_switch', self.left_limit_callback, 10)
        self.create_subscription(Bool, 'right_limit_switch', self.right_limit_callback, 10)
        self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)

        self.get_logger().info("Waiting for Arduino to connect...")

        # Wait for ARDUINO_READY
        while rclpy.ok() and not self.arduino_connected:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)

        self.get_logger().info("connected to arduino :)")
        time.sleep(1)
        self.get_logger().info("Starting sequence!")
        self.run_sequence()

    def arduino_rx_callback(self, msg):
        if "ARDUINO_READY" in msg.data:
            self.arduino_connected = True

    def napkin_callback(self, msg):
        self.napkin_present = msg.data

    def left_limit_callback(self, msg):
        self.at_left_limit = msg.data

    def right_limit_callback(self, msg):
        self.at_right_limit = msg.data

    def wait_for_limit(self, which, timeout=10):
        label = CNC_LABELS.get(f"{which}_limit_switch", which)
        self.get_logger().info(f"Waiting for {which} limit switch ({label})")
        end_time = time.time() + timeout
        if which == "left":
            self.at_left_limit = None
            while (self.at_left_limit is None or not self.at_left_limit) and rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not self.at_left_limit:
                self.get_logger().error(f"unable to communicate with left limit switch. are you sure it's plugged into {label}?")
                return False
            return True
        elif which == "right":
            self.at_right_limit = None
            while (self.at_right_limit is None or not self.at_right_limit) and rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not self.at_right_limit:
                self.get_logger().error(f"unable to communicate with right limit switch. are you sure it's plugged into {label}?")
                return False
            return True
        else:
            self.get_logger().error(f"Unknown limit switch {which}")
            return False

    def wait_for_napkin(self, timeout=5):
        label = CNC_LABELS["napkin_sensor"]
        self.napkin_present = None
        self.get_logger().info(f"Waiting for napkin sensor ({label})")
        end_time = time.time() + timeout
        while (self.napkin_present is None or not self.napkin_present) and rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.napkin_present:
            self.get_logger().error(f"unable to communicate with napkin sensor. are you sure it's plugged into {label}?")
            return False
        return True

    def run_sequence(self):
        while rclpy.ok():
            self.get_logger().info("=== New folding cycle ===")

            # 1. Home: Move X left until left limit switch
            self.gantry_pub.publish(String(data=f"x:left:{TWO_ROTATIONS}"))
            self.get_logger().info(f"Moving X left for {TWO_ROTATIONS} steps (2 rotations)")
            if not self.wait_for_limit("left"):
                continue
            time.sleep(0.5)

            # 2. Move X right until right limit
            self.gantry_pub.publish(String(data=f"x:right:{TWO_ROTATIONS}"))
            self.get_logger().info(f"Moving X right for {TWO_ROTATIONS} steps (2 rotations)")
            if not self.wait_for_limit("right"):
                continue
            time.sleep(0.5)

            # 3. Activate vacuum (fan)
            self.vacuum_pub.publish(Bool(data=True))
            self.get_logger().info("Fan ON")
            time.sleep(0.5)

            # 4. Wait for napkin presence
            if not self.wait_for_napkin():
                continue
            self.get_logger().info("Napkin detected! Proceeding...")

            # 5. Move main folding servo
            self.servo1_pub.publish(String(data=f"SERVO1:{SERVO_ANGLE}"))
            self.get_logger().info(f"Main servo to {SERVO_ANGLE} degrees")
            time.sleep(1.0)

            # 6. Move 'solenoid' servo
            self.servo2_pub.publish(String(data=f"SERVO2:{SOLENOID_ANGLE}"))
            self.get_logger().info(f'Sending "solenoid" servo to {SOLENOID_ANGLE} degrees')
            time.sleep(1.0)

            # 7. Deactivate vacuum (fan)
            self.vacuum_pub.publish(Bool(data=False))
            self.get_logger().info("Fan OFF")
            time.sleep(0.5)

            self.get_logger().info("Folding cycle complete. Waiting 2s before next cycle.")
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down main controller.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
