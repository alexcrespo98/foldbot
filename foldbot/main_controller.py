import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import time

SERVO_ANGLE = 180        # Main folding servo angle (degrees)
SERVO_HOME = 0
SOLENOID_ANGLE = 90       # "Solenoid" servo angle (degrees, dummy value)
SOLENOID_HOME = 0
Z_DOWN_POS = 100         # Example Z-down position; adjust as needed
Z_UP_POS = 0             # Example Z-up position; adjust as needed

MOVE_STEP_TIME = 0.1     # Time to wait between move commands (seconds)
LIMIT_TIMEOUT = 10       # Limit switch wait timeout (seconds)

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.gantry_pub = self.create_publisher(String, 'gantry_cmd', 10)
        self.servo_pub = self.create_publisher(String, 'servo_cmd', 10)
        self.solenoid_pub = self.create_publisher(String, 'solenoid_cmd', 10)
        self.vacuum_pub = self.create_publisher(Bool, 'vacuum_cmd', 10)
        self.arduino_tx_pub = self.create_publisher(String, 'arduino_tx', 10)

        self.napkin_present = False
        self.at_left_limit = False
        self.at_right_limit = False
        self.z_at_bottom = False
        self.z_at_top = True

        # Subscriptions
        self.create_subscription(Bool, '/napkin_sensor', self.napkin_callback, 10)
        self.create_subscription(Bool, '/left_limit_switch', self.left_limit_callback, 10)
        self.create_subscription(Bool, '/right_limit_switch', self.right_limit_callback, 10)

        # Poll Arduino for sensors every 100ms
        self.create_timer(0.1, self.poll_arduino_sensors)

        self.get_logger().info("Starting main controller sequence!")
        time.sleep(1)
        self.run_cycle()

    def poll_arduino_sensors(self):
        # Query all sensors every 100ms
        self.arduino_tx_pub.publish(String(data="LEFT_LIMIT_SWITCH:QUERY"))
        self.arduino_tx_pub.publish(String(data="RIGHT_LIMIT_SWITCH:QUERY"))
        self.arduino_tx_pub.publish(String(data="NAPKIN_SENSOR:QUERY"))

    def napkin_callback(self, msg):
        self.napkin_present = msg.data

    def left_limit_callback(self, msg):
        self.at_left_limit = msg.data

    def right_limit_callback(self, msg):
        self.at_right_limit = msg.data

    def wait_for_switch(self, attr, timeout=LIMIT_TIMEOUT):
        """Wait for a boolean attribute to become True with timeout."""
        end = time.time() + timeout
        while time.time() < end:
            if getattr(self, attr):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        return getattr(self, attr)

    def move_x_left_until_switch(self):
        self.get_logger().info("Moving X axis left until left limit switch is hit...")
        while not self.at_left_limit and rclpy.ok():
            self.gantry_pub.publish(String(data="MOVE_X_LEFT"))
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(MOVE_STEP_TIME)
        self.gantry_pub.publish(String(data="STOP_X"))
        self.get_logger().info("Left limit switch hit.")

    def move_x_right_until_switch(self):
        self.get_logger().info("Moving X axis right until right limit switch is hit...")
        while not self.at_right_limit and rclpy.ok():
            self.gantry_pub.publish(String(data="MOVE_X_RIGHT"))
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(MOVE_STEP_TIME)
        self.gantry_pub.publish(String(data="STOP_X"))
        self.get_logger().info("Right limit switch hit.")

    def move_z_to(self, pos):
        self.get_logger().info(f"Moving Z axis to position {pos}...")
        self.gantry_pub.publish(String(data=f"MOVE_Z_ABS:{pos}"))
        # For simplicity, wait a fixed time; replace with Z position feedback if available
        time.sleep(2)  # Adjust as needed

    def vacuum_on(self):
        self.vacuum_pub.publish(Bool(data=True))
        self.get_logger().info("Vacuum turned ON")
        time.sleep(0.5)

    def vacuum_off(self):
        self.vacuum_pub.publish(Bool(data=False))
        self.get_logger().info("Vacuum turned OFF")
        time.sleep(0.5)

    def activate_solenoid(self, active=True):
        angle = SOLENOID_ANGLE if active else SOLENOID_HOME
        self.solenoid_pub.publish(String(data=str(angle)))
        self.get_logger().info(f"Solenoid {'activated' if active else 'deactivated'} ({angle})")

    def activate_servo(self, active=True):
        angle = SERVO_ANGLE if active else SERVO_HOME
        self.servo_pub.publish(String(data=str(angle)))
        self.get_logger().info(f"Servo {'activated' if active else 'deactivated'} ({angle})")

    def run_cycle(self):
        while rclpy.ok():
            # 1. Wait for left limit switch, or move X left until found
            self.get_logger().info("Waiting for left limit switch...")
            got_left = self.wait_for_switch("at_left_limit", timeout=LIMIT_TIMEOUT)
            if not got_left:
                self.get_logger().info("Left limit not detected, homing X left...")
                self.move_x_left_until_switch()
            else:
                self.get_logger().info("Left limit switch detected.")

            # 2. Drop Z axis
            self.move_z_to(Z_DOWN_POS)

            # 3. Turn vacuum on
            self.vacuum_on()

            # 4. Raise Z axis
            self.move_z_to(Z_UP_POS)

            # 5. Move to right limit switch
            self.move_x_right_until_switch()

            # 6. Drop Z axis
            self.move_z_to(Z_DOWN_POS)

            # 7. Turn vacuum off
            self.vacuum_off()

            # 8. Raise Z axis
            self.move_z_to(Z_UP_POS)

            # 9. Move back to left limit switch
            self.move_x_left_until_switch()

            # 10. Activate napkin detector
            self.get_logger().info("Checking for napkin...")
            time.sleep(1)
            self.poll_arduino_sensors()
            napkin_found = False
            napkin_wait_start = time.time()
            while time.time() - napkin_wait_start < 2:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.napkin_present:
                    napkin_found = True
                    break

            if not napkin_found:
                self.get_logger().info("Napkin not detected. Repeating cycle.")
                continue

            # 11. Napkin detected! Activate solenoid, then servo, then home both
            self.get_logger().info("Napkin detected! Activating solenoid and servo...")
            self.activate_solenoid(True)
            time.sleep(1)
            self.activate_servo(True)
            time.sleep(1)
            self.activate_servo(False)
            self.activate_solenoid(False)
            self.get_logger().info("Task complete! Waiting before restart...")
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
