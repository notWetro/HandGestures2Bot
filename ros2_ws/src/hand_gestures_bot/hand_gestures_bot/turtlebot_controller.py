import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_command_time = time.time()
        self.timeout_duration = 0.5 
        self.timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info("TurtleBot Controller Initialized (Watchdog Active)")

    def move_forward(self):
        """Scenario 1: Forward Movement"""
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self._publish_command(msg)

    def stop(self):
        """Scenario 2: Stop Movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._publish_command(msg)

    def _publish_command(self, msg):
        self.last_command_time = time.time()
        self.publisher_.publish(msg)

    def watchdog_check(self):
        """Scenario 4: Fail-safe Stop"""
        if (time.time() - self.last_command_time) > self.timeout_duration:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher_.publish(stop_msg)
