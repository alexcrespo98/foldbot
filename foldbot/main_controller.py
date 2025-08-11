import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        self.arduino_tx_pub = self.create_publisher(String, 'arduino_tx', 10)
        self.arduino_rx_sub = self.create_subscription(String, 'arduino_rx', self.arduino_rx_callback, 10)
        self.sensor_states = {
            "LEFT_LIMIT_SWITCH": None,
            "RIGHT_LIMIT_SWITCH": None,
            "NAPKIN_SENSOR": None
        }
        self.last_arduino_msg = ""
        self.waiting_for = None

        # ------ Sequence parameters ------
        self.steps_per_revolution = 200
        self.z_rotations = 7.5
        self.z_steps = int(self.z_rotations * self.steps_per_revolution)
        self.x_step_size =25  # how many steps to move per X axis poll
        self.z_step_size = self.z_steps  # full up/down movement
        self.servo1_angle = 45
        self.servo2_angle = 170
        # ---------------------------------

        # Start polling sensors
        self.sensor_timer = self.create_timer(0.1, self.poll_sensors)

        self.get_logger().info("Waiting for Arduino handshake...")
        self.wait_for_arduino_handshake()
        self.get_logger().info("Arduino connected! Starting sequence in 2s...")
        time.sleep(2)
        self.run_sequence()

    def poll_sensors(self):
        # Query all sensors, 10Hz
        self.arduino_tx_pub.publish(String(data="LEFT_LIMIT_SWITCH:QUERY"))
        self.arduino_tx_pub.publish(String(data="RIGHT_LIMIT_SWITCH:QUERY"))
        self.arduino_tx_pub.publish(String(data="NAPKIN_SENSOR:QUERY"))

    def arduino_rx_callback(self, msg):
        self.last_arduino_msg = msg.data
        # Parse sensor responses
        for key in self.sensor_states.keys():
            if msg.data.startswith(f"{key}:"):
                value = msg.data.split(":")[1].strip()
                self.sensor_states[key] = value
        # Used for waiting for specific replies
        if self.waiting_for and self.waiting_for in msg.data:
            self.waiting_for = None

    def wait_for_arduino_reply(self, expected, timeout=10.0):
        """ Wait for an exact reply from Arduino (actions only, not sensors) """
        self.waiting_for = expected
        start = time.time()
        while rclpy.ok() and self.waiting_for:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start > timeout:
                self.get_logger().error(f"Timeout waiting for Arduino reply: {expected}")
                self.waiting_for = None
                return False
        return True

    def wait_for_sensor_state(self, sensor, desired_state="ON", timeout=10.0):
        """ Wait for sensor to reach a state (e.g. LEFT_LIMIT_SWITCH:ON) """
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.sensor_states.get(sensor)
            if state == desired_state:
                return True
            if time.time() - start > timeout:
                self.get_logger().error(f"Timeout waiting for {sensor} to be {desired_state}")
                return False
            time.sleep(0.05)
        return False

    def wait_for_arduino_handshake(self, timeout=15.0):
        """ Robust handshake: waits for repeated ARDUINO_READY, sends HELLO_ARDUINO, waits for ARDUINO_ACK """
        start = time.time()
        ready_confirmed = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if "ARDUINO_READY" in self.last_arduino_msg:
                self.get_logger().info("Arduino says ready, sending HELLO_ARDUINO")
                self.arduino_tx_pub.publish(String(data="HELLO_ARDUINO"))
                ready_confirmed = True
            if ready_confirmed and "ARDUINO_ACK" in self.last_arduino_msg:
                self.get_logger().info("Received ARDUINO_ACK!")
                return True
            if time.time() - start > timeout:
                self.get_logger().error("Timeout waiting for ARDUINO_READY handshake")
                return False
            time.sleep(0.05)
        return False

    def move_x_until_limit(self, direction, limit_sensor, timeout=30.0):  # Increased timeout for X axis
        """Move X direction (0=left, 1=right) until a limit switch is ON."""
        self.get_logger().info(f"Moving X {'RIGHT' if direction else 'LEFT'} until {limit_sensor} is ON...")
        # Move in small steps, poll sensor after each
        start = time.time()
        while rclpy.ok():
            if self.sensor_states.get(limit_sensor) == "ON":
                self.get_logger().info(f"{limit_sensor} triggered!")
                break
            # Move a small step
            self.arduino_tx_pub.publish(String(data=f"X_MOVE:{direction}:{self.x_step_size}"))
            self.wait_for_arduino_reply("X_MOVE:DONE")
            # Allow polling to catch up
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.time() - start > timeout:
                self.get_logger().error(f"Timeout moving X towards {limit_sensor}")
                break
            time.sleep(0.03)

    def move_z(self, direction, steps):
        """Move Z direction (0=down, 1=up) by steps."""
        self.get_logger().info(f"Moving Z {'UP' if direction else 'DOWN'} {steps} steps...")
        self.arduino_tx_pub.publish(String(data=f"Z_MOVE:{direction}:{steps}"))
        self.wait_for_arduino_reply("Z_MOVE:DONE")

    def run_sequence(self):
        # Step 1: Home X axis left
        self.move_x_until_limit(direction=0, limit_sensor="LEFT_LIMIT_SWITCH")
        time.sleep(0.25)

        # Step 2: Start vacuum (NO napkin sensor check here)
        self.get_logger().info("Turning vacuum ON...")
        self.arduino_tx_pub.publish(String(data="VACUUM:ON"))
        self.wait_for_arduino_reply("VACUUM:ON")
        time.sleep(5.0)

        # Step 3: Move Z up
        self.move_z(direction=1, steps=self.z_step_size)
        time.sleep(0.2)

        # Step 4: Move X right until right endstop
        self.move_x_until_limit(direction=1, limit_sensor="RIGHT_LIMIT_SWITCH")
        time.sleep(0.2)

        # Step 5: Release vacuum
        self.get_logger().info("Turning vacuum OFF...")
        self.arduino_tx_pub.publish(String(data="VACUUM:OFF"))
        self.wait_for_arduino_reply("VACUUM:OFF", timeout=1.0)
        self.get_logger().info("Resting for 5 seconds after vacuum off...")
        time.sleep(5.0)

        # Step 6: Move X left to left endstop
        self.move_x_until_limit(direction=0, limit_sensor="LEFT_LIMIT_SWITCH")
        time.sleep(0.2)

        # Step 7: Move Z down
        self.move_z(direction=0, steps=self.z_step_size)
        time.sleep(0.2)

        # Step 8: Wait for napkin removed (after vacuum is off and X is left)
        self.get_logger().info("Waiting for napkin sensor to be OFF...")
        self.wait_for_sensor_state("NAPKIN_SENSOR", desired_state="OFF", timeout=10.0)

        # Step 9: Servo sequence
        self.get_logger().info("Running servo sequence...")
        self.arduino_tx_pub.publish(String(data=f"SERVO1:{self.servo1_angle}"))
        time.sleep(0.4)
        self.arduino_tx_pub.publish(String(data=f"SERVO2:{self.servo2_angle}"))
        time.sleep(1.0)
        self.arduino_tx_pub.publish(String(data="SERVO1:0"))
        time.sleep(0.4)
        self.arduino_tx_pub.publish(String(data="SERVO2:0"))
        time.sleep(0.4)

        self.get_logger().info("Cycle complete! (not repeating automatically)")

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
