#!/usr/bin/env python3
"""
TurtleBot3 Safety Scanner Node (ROS 2 Humble)

This node evaluates LaserScan sensor data from the robot's LiDAR,
logs the minimum distances in three front sectors, and automatically
stops the robot if any obstacle is detected within the safety distance.

- Subscribes: /scan (sensor_msgs/msg/LaserScan)
- Publishes: /cmd_vel (geometry_msgs/msg/Twist) - STOP commands when obstacle detected
- Outputs: Periodic log messages with front sector distances

Safety Behavior:
- If any sector (Left, Center, Right) detects an obstacle within 25 cm,
  the robot will be stopped (zero velocity published to /cmd_vel)
- Forward movement is blocked until obstacle clears
- Logging continues every 2 seconds

Sectors (total 45° forward-facing area):
- Left:   +7.5° to +22.5° (15° sector)
- Center: -7.5° to +7.5°  (15° sector)
- Right:  -22.5° to -7.5° (15° sector)
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
    """
    Check if a LaserScan range value is valid.
    
    Invalid values include:
    - inf (infinity)
    - nan (not a number)
    - 0 (zero or negative)
    
    Args:
        value: The range value to check (in meters)
        
    Returns:
        True if the value is valid, False otherwise
    """
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
    ROS 2 node that evaluates LaserScan data, logs front sector distances,
    and automatically stops the robot if obstacles are too close.
    
    The node subscribes to the /scan topic and:
    - Periodically (every 2 seconds) logs the minimum valid distance in sectors
    - Continuously monitors for obstacles within 25 cm
    - Publishes STOP command to /cmd_vel when obstacle detected
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
    SAFETY_CHECK_RATE_HZ = 10.0  # Check for obstacles 10 times per second

    def __init__(self) -> None:
        """Initialize the SafetyScanner node."""
        super().__init__("safety_scanner")

        # Store the latest LaserScan message
        self.latest_scan: Optional[LaserScan] = None
        
        # Track obstacle state to avoid spamming logs
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
        # Create velocity publisher for STOP commands
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
        # Create timer for safety checks (10 Hz)
        # -------------------------------------------------------------------------
        self.safety_timer = self.create_timer(
            timer_period_sec=1.0 / self.SAFETY_CHECK_RATE_HZ,
            callback=self.safety_timer_callback,
        )

        # -------------------------------------------------------------------------
        # Startup log
        # -------------------------------------------------------------------------
        self.get_logger().info("SafetyScanner node started.")
        self.get_logger().info("Subscribed to /scan topic.")
        self.get_logger().info("Publishing STOP commands to /cmd_vel when obstacles detected.")
        self.get_logger().info(f"Safety distance: {self.SAFETY_DISTANCE_M * 100:.0f} cm")
        self.get_logger().info(
            f"Sectors: Left [{self.LEFT_SECTOR_MIN_DEG}° to {self.LEFT_SECTOR_MAX_DEG}°], "
            f"Center [{self.CENTER_SECTOR_MIN_DEG}° to {self.CENTER_SECTOR_MAX_DEG}°], "
            f"Right [{self.RIGHT_SECTOR_MIN_DEG}° to {self.RIGHT_SECTOR_MAX_DEG}°]"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        """Store the latest scan data."""
        self.latest_scan = msg

    def safety_timer_callback(self) -> None:
        """
        Safety check callback - runs at 10 Hz.
        Checks for obstacles and publishes STOP if any are too close.
        """
        if self.latest_scan is None:
            return

        # Get minimum distances for all sectors
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

        if obstacle_sectors:
            # Obstacle detected - publish STOP command
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.linear.y = 0.0
            stop_cmd.linear.z = 0.0
            stop_cmd.angular.x = 0.0
            stop_cmd.angular.y = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)

            # Log only on state change to avoid spam
            sector_str = ", ".join(obstacle_sectors)
            if not self.obstacle_detected or sector_str != self.last_obstacle_sector:
                self.get_logger().warn(
                    f"⚠️ OBSTACLE DETECTED in {sector_str}! Stopping robot."
                )
                self.obstacle_detected = True
                self.last_obstacle_sector = sector_str
        else:
            # Clear obstacle state
            if self.obstacle_detected:
                self.get_logger().info("✓ Obstacle cleared. Robot can move.")
                self.obstacle_detected = False
                self.last_obstacle_sector = ""

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
