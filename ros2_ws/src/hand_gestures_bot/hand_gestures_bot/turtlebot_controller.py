import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_command_time = time.time()
        self.timeout_duration = 0.5  # Watchdog failsafe
        self.timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info("TurtleBot Controller Initialized (Movement Mode, Watchdog Active)")

    # ──────────────────────────────── MOVEMENT COMMANDS ────────────────────────────────

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.25
        msg.angular.z = 0.0
        self._publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        self._publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5
        self._publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._publish(msg)

    # ─────────────────────────────── INTERNAL HELPERS ────────────────────────────────

    def _publish(self, msg):
        self.last_command_time = time.time()
        self.publisher_.publish(msg)

    def watchdog_check(self):
        """If no commands received for X seconds → auto stop"""
        if (time.time() - self.last_command_time) > self.timeout_duration:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher_.publish(stop_msg)
