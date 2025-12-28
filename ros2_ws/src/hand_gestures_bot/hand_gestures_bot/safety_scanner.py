#!/usr/bin/env python3
"""
TurtleBot3 Safety Scanner Node (ROS 2 Humble)

This node evaluates LaserScan sensor data from the robot's LiDAR,
logs the minimum distances in three front sectors, and automatically
BLOCKS FORWARD MOTION if any obstacle is detected within the safety distance.

Architecture:
- Subscribes: /scan (sensor_msgs/msg/LaserScan)
- Subscribes: /cmd_vel_in (geometry_msgs/msg/Twist) - incoming commands
- Publishes: /cmd_vel (geometry_msgs/msg/Twist) - filtered commands

Safety Behavior:
- If any sector (Left, Center, Right) detects an obstacle within 25 cm,
  FORWARD motion (positive linear.x) is blocked
- Backward and turning motion is always allowed
- Logging continues every 2 seconds

Sectors (total 45° forward-facing area):
- Left:   +7.5° to +22.5° (15° sector)
- Center: -7.5° to +7.5°  (15° sector)
- Right:  -22.5° to -7.5° (15° sector)

IMPORTANT: Commands should be sent to /cmd_vel_in, NOT /cmd_vel directly!
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def is_valid_range(value: float) -> bool:
    """Check if a LaserScan range value is valid."""
    if value is None:
        return False
    if value <= 0.0:
        return False
    if math.isinf(value):
        return False
    if math.isnan(value):
        return False
    return True


class SafetyScanner(Node):
    """
    ROS 2 node that filters velocity commands based on obstacle proximity.
    
    Subscribes to /cmd_vel_in for incoming commands and publishes filtered
    commands to /cmd_vel. Forward motion is blocked when obstacles are
    detected within the safety distance.
    """

    # -------------------------------------------------------------------------
    # Sector Configuration
    # -------------------------------------------------------------------------
    LEFT_SECTOR_MIN_DEG = 7.5
    LEFT_SECTOR_MAX_DEG = 22.5
    CENTER_SECTOR_MIN_DEG = -7.5
    CENTER_SECTOR_MAX_DEG = 7.5
    RIGHT_SECTOR_MIN_DEG = -22.5
    RIGHT_SECTOR_MAX_DEG = -7.5

    # -------------------------------------------------------------------------
    # Safety Configuration
    # -------------------------------------------------------------------------
    SAFETY_DISTANCE_M = 0.25  # 25 cm in meters

    def __init__(self) -> None:
        """Initialize the SafetyScanner node."""
        super().__init__("safety_scanner")

        # Store the latest LaserScan message
        self.latest_scan: Optional[LaserScan] = None
        
        # Track obstacle state
        self.obstacle_detected: bool = False
        self.last_obstacle_sector: str = ""

        # -------------------------------------------------------------------------
        # QoS Profiles
        # -------------------------------------------------------------------------
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # -------------------------------------------------------------------------
        # Create LaserScan subscriber
        # -------------------------------------------------------------------------
        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            scan_qos,
        )

        # -------------------------------------------------------------------------
        # Create velocity command subscriber (input)
        # -------------------------------------------------------------------------
        self.cmd_vel_in_sub = self.create_subscription(
            Twist,
            "/cmd_vel_in",
            self.cmd_vel_in_callback,
            cmd_qos,
        )

        # -------------------------------------------------------------------------
        # Create velocity publisher (filtered output)
        # -------------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            cmd_qos,
        )

        # -------------------------------------------------------------------------
        # Create timer for periodic logging (every 2 seconds)
        # -------------------------------------------------------------------------
        self.log_timer = self.create_timer(
            timer_period_sec=2.0,
            callback=self.log_timer_callback,
        )

        # -------------------------------------------------------------------------
        # Startup log
        # -------------------------------------------------------------------------
        self.get_logger().info("SafetyScanner node started.")
        self.get_logger().info("Subscribing to: /scan, /cmd_vel_in")
        self.get_logger().info("Publishing to: /cmd_vel")
        self.get_logger().info(f"Safety distance: {self.SAFETY_DISTANCE_M * 100:.0f} cm")
        self.get_logger().info("⚠️ Send commands to /cmd_vel_in (NOT /cmd_vel directly!)")

    def scan_callback(self, msg: LaserScan) -> None:
        """Store the latest scan data and update obstacle state."""
        self.latest_scan = msg
        self._update_obstacle_state()

    def _update_obstacle_state(self) -> None:
        """Check for obstacles and update state."""
        if self.latest_scan is None:
            self.obstacle_detected = False
            return

        left_m = self.get_sector_min_distance(
            self.latest_scan, self.LEFT_SECTOR_MIN_DEG, self.LEFT_SECTOR_MAX_DEG
        )
        center_m = self.get_sector_min_distance(
            self.latest_scan, self.CENTER_SECTOR_MIN_DEG, self.CENTER_SECTOR_MAX_DEG
        )
        right_m = self.get_sector_min_distance(
            self.latest_scan, self.RIGHT_SECTOR_MIN_DEG, self.RIGHT_SECTOR_MAX_DEG
        )

        # Check if any sector has obstacle within safety distance
        obstacle_sectors = []
        if left_m is not None and left_m < self.SAFETY_DISTANCE_M:
            obstacle_sectors.append(f"LEFT ({left_m*100:.1f}cm)")
        if center_m is not None and center_m < self.SAFETY_DISTANCE_M:
            obstacle_sectors.append(f"CENTER ({center_m*100:.1f}cm)")
        if right_m is not None and right_m < self.SAFETY_DISTANCE_M:
            obstacle_sectors.append(f"RIGHT ({right_m*100:.1f}cm)")

        was_detected = self.obstacle_detected
        self.obstacle_detected = len(obstacle_sectors) > 0

        # Log state changes
        if self.obstacle_detected:
            sector_str = ", ".join(obstacle_sectors)
            if not was_detected or sector_str != self.last_obstacle_sector:
                self.get_logger().warn(
                    f"⚠️ OBSTACLE DETECTED in {sector_str}! Blocking forward motion."
                )
                self.last_obstacle_sector = sector_str
        elif was_detected:
            self.get_logger().info("✓ Obstacle cleared. Forward motion allowed.")
            self.last_obstacle_sector = ""

    def cmd_vel_in_callback(self, msg: Twist) -> None:
        """
        Filter incoming velocity commands.
        
        If obstacle detected, blocks forward motion (positive linear.x).
        Backward motion and turning are always allowed.
        """
        output_cmd = Twist()
        output_cmd.linear.y = msg.linear.y
        output_cmd.linear.z = msg.linear.z
        output_cmd.angular.x = msg.angular.x
        output_cmd.angular.y = msg.angular.y
        output_cmd.angular.z = msg.angular.z  # Turning always allowed

        if self.obstacle_detected and msg.linear.x > 0.0:
            # Block forward motion
            output_cmd.linear.x = 0.0
        else:
            # Allow the command
            output_cmd.linear.x = msg.linear.x

        self.cmd_vel_pub.publish(output_cmd)

    def log_timer_callback(self) -> None:
        """
        Logging callback - runs every 2 seconds.
        Logs the minimum distances in each front sector.
        """
        if self.latest_scan is None:
            self.get_logger().warn("No LaserScan data received yet.")
            return

        # Evaluate all three sectors
        left_m = self.get_sector_min_distance(
            self.latest_scan, self.LEFT_SECTOR_MIN_DEG, self.LEFT_SECTOR_MAX_DEG
        )
        center_m = self.get_sector_min_distance(
            self.latest_scan, self.CENTER_SECTOR_MIN_DEG, self.CENTER_SECTOR_MAX_DEG
        )
        right_m = self.get_sector_min_distance(
            self.latest_scan, self.RIGHT_SECTOR_MIN_DEG, self.RIGHT_SECTOR_MAX_DEG
        )

        # Convert to centimeters and format output
        left_cm = self.meters_to_cm_string(left_m)
        center_cm = self.meters_to_cm_string(center_m)
        right_cm = self.meters_to_cm_string(right_m)

        # Add warning indicator if below safety threshold
        def add_warning(val_m: Optional[float], val_str: str) -> str:
            if val_m is not None and val_m < self.SAFETY_DISTANCE_M:
                return f"{val_str} ⚠️"
            return val_str

        # Log the formatted output
        self.get_logger().info(
            f"\nFront distances:\n"
            f"Left:   {add_warning(left_m, left_cm)}\n"
            f"Center: {add_warning(center_m, center_cm)}\n"
            f"Right:  {add_warning(right_m, right_cm)}"
        )

    def get_sector_min_distance(
        self, 
        scan: LaserScan, 
        angle_min_deg: float, 
        angle_max_deg: float
    ) -> Optional[float]:
        """
        Get the minimum valid distance within a specified angular sector.
        """
        if not scan.ranges or scan.angle_increment == 0.0:
            return None

        angle_min_rad = math.radians(angle_min_deg)
        angle_max_rad = math.radians(angle_max_deg)
        
        scan_range = scan.angle_max - scan.angle_min
        num_readings = len(scan.ranges)
        
        def angle_to_index(angle: float) -> int:
            while angle < scan.angle_min:
                angle += scan_range
            while angle >= scan.angle_max:
                angle -= scan_range
            idx = int((angle - scan.angle_min) / scan.angle_increment)
            return max(0, min(num_readings - 1, idx))
        
        start_idx = angle_to_index(angle_min_rad)
        end_idx = angle_to_index(angle_max_rad)
        
        indices_to_scan = []
        if start_idx <= end_idx:
            indices_to_scan = list(range(start_idx, end_idx + 1))
        else:
            indices_to_scan = list(range(start_idx, num_readings)) + list(range(0, end_idx + 1))

        min_distance: Optional[float] = None

        for idx in indices_to_scan:
            range_value = scan.ranges[idx]
            
            if not is_valid_range(range_value):
                continue
            
            if scan.range_min > 0.0 and range_value < scan.range_min:
                continue
            if scan.range_max > 0.0 and range_value > scan.range_max:
                continue

            if min_distance is None or range_value < min_distance:
                min_distance = range_value

        return min_distance

    def meters_to_cm_string(self, distance_m: Optional[float]) -> str:
        """Convert distance to formatted string in centimeters."""
        if distance_m is None:
            return "N/A (no valid data)"
        
        distance_cm = distance_m * 100.0
        return f"{distance_cm:.1f} cm"


def main(args=None) -> None:
    """Main entry point for the SafetyScanner node."""
    rclpy.init(args=args)
    
    node = SafetyScanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
