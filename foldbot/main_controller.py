import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import time

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        # Command publishers
        self.gantry_pub = self.create_publisher(String, 'gantry_cmd', 10)
        self.vacuum_pub = self.create_publisher(Bool, 'vacuum_cmd', 10)
        self.solenoid_pub = self.create_publisher(Bool, 'solenoid_cmd', 10)
        self.servo_pub = self.create_publisher(String, 'servo_cmd', 10)

        # State tracking
        self.napkin_present = False
        self.at_left_limit = False
        self.at_right_limit = False

        # Subscribers
        self.create_subscription(Bool, 'napkin_present', self.napkin_callback, 10)
        self.create_subscription(Bool, 'left_limit_switch', self.left_limit_callback, 10)
        self.create_subscription(Bool, 'right_limit_switch', self.right_limit_callback, 10)

        # Allow time for all nodes to connect
        self.get_logger().info("Waiting for other nodes to initialize...")
        time.sleep(3)
        self.get_logger().info("Starting sequence!")

        self.run_sequence()

    def napkin_callback(self, msg):
        self.napkin_present = msg.data

    def left_limit_callback(self, msg):
        self.at_left_limit = msg.data

    def right_limit_callback(self, msg):
        self.at_right_limit = msg.data

    def wait_for_limit(self, which):
        self.get_logger().info(f"Waiting for {which} limit switch")
        timeout = time.time() + 10  # 10s timeout for safety
        if which == "left":
            while not self.at_left_limit and rclpy.ok() and time.time() < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            return self.at_left_limit
        elif which == "right":
            while not self.at_right_limit and rclpy.ok() and time.time() < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            return self.at_right_limit
        else:
            return False

    def wait_for_napkin(self):
        self.get_logger().info("Checking napkin detector...")
        timeout = time.time() + 3
        while not self.napkin_present and rclpy.ok() and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.napkin_present

    def run_sequence(self):
        while rclpy.ok():
            self.get_logger().info("** New folding cycle **")

            # 1. Fan on
            self.vacuum_pub.publish(Bool(data=True))
            self.get_logger().info("Fan ON")
            time.sleep(0.5)

            # 2. Z down
            self.gantry_pub.publish(String(data="z:down:200"))
            self.get_logger().info("Z DOWN")
            time.sleep(1)

            # 3. Z up
            self.gantry_pub.publish(String(data="z:up:200"))
            self.get_logger().info("Z UP")
            time.sleep(1)

            # 4. Move X right until right limit switch
            self.gantry_pub.publish(String(data="x:right:2000"))  # Large number, will stop at limit
            self.get_logger().info("Move X RIGHT")
            self.wait_for_limit("right")
            time.sleep(0.5)

            # 5. Z down
            self.gantry_pub.publish(String(data="z:down:200"))
            self.get_logger().info("Z DOWN")
            time.sleep(1)

            # 6. Fan off
            self.vacuum_pub.publish(Bool(data=False))
            self.get_logger().info("Fan OFF")
            time.sleep(0.5)

            # 7. Z up
            self.gantry_pub.publish(String(data="z:up:200"))
            self.get_logger().info("Z UP")
            time.sleep(1)

            # 8. Move X left until left limit switch
            self.gantry_pub.publish(String(data="x:left:2000"))  # Large number, will stop at limit
            self.get_logger().info("Move X LEFT")
            self.wait_for_limit("left")
            time.sleep(0.5)

            # 9. Confirm napkin presence
            napkin = self.wait_for_napkin()
            if not napkin:
                self.get_logger().info("Napkin NOT detected, restarting sequence.")
                continue

            self.get_logger().info("Napkin detected! Proceeding...")

            # 10. Solenoid activate (extend)
            self.solenoid_pub.publish(Bool(data=True))
            self.get_logger().info("Solenoid ON")
            time.sleep(0.7)

            # 11. Servo move (fold)
            self.servo_pub.publish(String(data="fold"))
            self.get_logger().info("Servo FOLD")
            time.sleep(1.5)

            # 12. Servo back (reset)
            self.servo_pub.publish(String(data="reset"))
            self.get_logger().info("Servo RESET")
            time.sleep(1.5)

            # 13. Solenoid retreat (retract)
            self.solenoid_pub.publish(Bool(data=False))
            self.get_logger().info("Solenoid OFF")
            time.sleep(0.7)

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
