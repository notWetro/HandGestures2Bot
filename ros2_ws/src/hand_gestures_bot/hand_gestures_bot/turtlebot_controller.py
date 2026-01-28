import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # TurtleBot3 requires RELIABLE QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_in', qos)

        
        self.current_cmd = None

        
        self.declare_parameter("command_timeout_sec", 0.0)
        self.timeout_duration = float(self.get_parameter("command_timeout_sec").value)
        self.last_command_time = time.time()

        
        self.publisher_timer = self.create_timer(0.1, self.publish_continuous)

        
        self.watchdog_timer = None
        if self.timeout_duration and self.timeout_duration > 0.0:
            self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info("TurtleBot Controller Initialized (READY)")


    

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.21        
        msg.angular.z = 0.0
        self.current_cmd = msg
        self.last_command_time = time.time()
        self.get_logger().info("CMD: FORWARD")

    def move_backward(self):
        msg = Twist()
        msg.linear.x = -0.21
        msg.angular.z = 0.0
        self.current_cmd = msg
        self.last_command_time = time.time()
        self.get_logger().info("CMD: BACKWARD")

    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.current_cmd = msg
        self.last_command_time = time.time()
        self.get_logger().info("CMD: LEFT")

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.5
        self.current_cmd = msg
        self.last_command_time = time.time()
        self.get_logger().info("CMD: RIGHT")

    def stop(self):
        msg = Twist()  # all zeros
        self.current_cmd = msg
        self.last_command_time = time.time()
        self.get_logger().info("CMD: STOP")


    

    def publish_continuous(self):
        """
        Publish the last valid Twist at 10Hz.
        IMPORTANT: Do NOT publish until first real command received.
        """
        if self.current_cmd is not None:
            self.publisher_.publish(self.current_cmd)

    def watchdog_check(self):
        """
        If no command received recently → force STOP.
        """
        if not self.timeout_duration or self.timeout_duration <= 0.0:
            return
        if (time.time() - self.last_command_time) > self.timeout_duration:
            self.current_cmd = Twist()
            self.publisher_.publish(self.current_cmd)
