import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos)

        # Store last command (continuously re-published)
        self.current_cmd = Twist()

        self.last_command_time = time.time()
        self.timeout_duration = 0.5  # auto-stop after 0.5 sec

        # Publish cmd_vel at 10Hz continuously
        self.publisher_timer = self.create_timer(0.1, self.publish_continuous)

        # Watchdog
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info("TurtleBot Controller Initialized")

    # ─────────────────────────────── MOVEMENT COMMANDS ───────────────────────────────

    def move_forward(self):
        self.current_cmd.linear.x = 0.25
        self.current_cmd.angular.z = 0.0
        self.last_command_time = time.time()

    def turn_left(self):
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.5
        self.last_command_time = time.time()

    def turn_right(self):
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = -0.5
        self.last_command_time = time.time()

    def stop(self):
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.0
        self.last_command_time = time.time()

    # ─────────────────────────────── INTERNAL TIMERS ───────────────────────────────

    def publish_continuous(self):
        """Publish the last command at 10Hz continuously."""
        self.publisher_.publish(self.current_cmd)

    def watchdog_check(self):
        """Stop robot if no new movement command received for too long."""
        if (time.time() - self.last_command_time) > self.timeout_duration:
            self.current_cmd.linear.x = 0.0
            self.current_cmd.angular.z = 0.0
