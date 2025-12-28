#!/usr/bin/env python3
"""
TurtleBot3 Environment Scanner Node (ROS 2 Humble)

This node evaluates LaserScan sensor data from the robot's LiDAR
and periodically outputs the minimum distances in three front sectors.

- Subscribes: /scan (sensor_msgs/msg/LaserScan)
- Outputs: Periodic log messages with front sector distances

Sectors (total 45° forward-facing area):
- Left:   +7.5° to +22.5° (15° sector)
- Center: -7.5° to +7.5°  (15° sector)
- Right:  -22.5° to -7.5° (15° sector)

This node is purely for sensor processing and logging.
No movement commands, no AI logic, no SLAM.
"""

from __future__ import annotations

import math
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan


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


class EnvironmentScanner(Node):
    """
    ROS 2 node that evaluates LaserScan data and logs front sector distances.
    
    The node subscribes to the /scan topic and periodically (every 2 seconds)
    logs the minimum valid distance in three front-facing sectors:
    - Left sector:   +7.5° to +22.5°
    - Center sector: -7.5° to +7.5°
    - Right sector:  -22.5° to -7.5°
    
    All distances are output in centimeters.
    """

    # -------------------------------------------------------------------------
    # Sector Configuration
    # Total front angle: 45° (from -22.5° to +22.5°)
    # Each sector: 15°
    # -------------------------------------------------------------------------
    # Note: Positive angles are typically to the left, negative to the right
    # in the standard ROS coordinate frame (right-hand rule, z-up)
    
    # Left sector: +7.5° to +22.5° 
    LEFT_SECTOR_MIN_DEG = 7.5
    LEFT_SECTOR_MAX_DEG = 22.5
    
    # Center sector: -7.5° to +7.5°
    CENTER_SECTOR_MIN_DEG = -7.5
    CENTER_SECTOR_MAX_DEG = 7.5
    
    # Right sector: -22.5° to -7.5°
    RIGHT_SECTOR_MIN_DEG = -22.5
    RIGHT_SECTOR_MAX_DEG = -7.5

    def __init__(self) -> None:
        """Initialize the EnvironmentScanner node."""
        super().__init__("environment_scanner")

        # -------------------------------------------------------------------------
        # Store the latest LaserScan message for timer-based evaluation
        # -------------------------------------------------------------------------
        self.latest_scan: Optional[LaserScan] = None

        # -------------------------------------------------------------------------
        # QoS Profile for LaserScan subscription
        # Using BEST_EFFORT reliability (matches typical LiDAR publishers)
        # Small queue size to avoid processing stale data
        # -------------------------------------------------------------------------
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        # Create timer for periodic evaluation (every 2 seconds)
        # -------------------------------------------------------------------------
        self.evaluation_timer = self.create_timer(
            timer_period_sec=2.0,
            callback=self.timer_callback,
        )

        # -------------------------------------------------------------------------
        # Startup log
        # -------------------------------------------------------------------------
        self.get_logger().info("EnvironmentScanner node started.")
        self.get_logger().info("Subscribed to /scan topic.")
        self.get_logger().info("Logging front distances every 2 seconds.")
        self.get_logger().info(
            f"Sectors: Left [{self.LEFT_SECTOR_MIN_DEG}° to {self.LEFT_SECTOR_MAX_DEG}°], "
            f"Center [{self.CENTER_SECTOR_MIN_DEG}° to {self.CENTER_SECTOR_MAX_DEG}°], "
            f"Right [{self.RIGHT_SECTOR_MIN_DEG}° to {self.RIGHT_SECTOR_MAX_DEG}°]"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Callback for LaserScan messages.
        
        This callback only stores the latest scan data.
        Actual evaluation and logging is done in the timer callback
        to avoid processing on every scan message.
        
        Args:
            msg: The incoming LaserScan message
        """
        self.latest_scan = msg

    def timer_callback(self) -> None:
        """
        Timer callback for periodic evaluation and logging.
        
        Called every 2 seconds to evaluate the stored LaserScan data
        and log the minimum distances in each front sector.
        """
        # Check if we have received any scan data yet
        if self.latest_scan is None:
            self.get_logger().warn("No LaserScan data received yet.")
            return

        # Evaluate all three sectors
        left_distance_m = self.get_sector_min_distance(
            self.latest_scan,
            self.LEFT_SECTOR_MIN_DEG,
            self.LEFT_SECTOR_MAX_DEG
        )
        
        center_distance_m = self.get_sector_min_distance(
            self.latest_scan,
            self.CENTER_SECTOR_MIN_DEG,
            self.CENTER_SECTOR_MAX_DEG
        )
        
        right_distance_m = self.get_sector_min_distance(
            self.latest_scan,
            self.RIGHT_SECTOR_MIN_DEG,
            self.RIGHT_SECTOR_MAX_DEG
        )

        # Convert to centimeters and format output
        left_cm = self.meters_to_cm_string(left_distance_m)
        center_cm = self.meters_to_cm_string(center_distance_m)
        right_cm = self.meters_to_cm_string(right_distance_m)

        # Log the formatted output
        self.get_logger().info(
            f"\nFront distances:\n"
            f"Left:   {left_cm}\n"
            f"Center: {center_cm}\n"
            f"Right:  {right_cm}"
        )

    def get_sector_min_distance(
        self, 
        scan: LaserScan, 
        angle_min_deg: float, 
        angle_max_deg: float
    ) -> Optional[float]:
        """
        Get the minimum valid distance within a specified angular sector.
        
        Args:
            scan: The LaserScan message to evaluate
            angle_min_deg: Minimum angle of the sector (degrees)
            angle_max_deg: Maximum angle of the sector (degrees)
            
        Returns:
            Minimum valid distance in meters, or None if no valid readings
        """
        # Check for valid scan data
        if not scan.ranges or scan.angle_increment == 0.0:
            return None

        # Convert angles from degrees to radians
        angle_min_rad = math.radians(angle_min_deg)
        angle_max_rad = math.radians(angle_max_deg)
        
        # Calculate scan parameters
        scan_range = scan.angle_max - scan.angle_min
        num_readings = len(scan.ranges)
        
        # Helper function to get index for an angle
        def angle_to_index(angle: float) -> int:
            # Normalize angle to scan range [angle_min, angle_max)
            while angle < scan.angle_min:
                angle += scan_range
            while angle >= scan.angle_max:
                angle -= scan_range
            idx = int((angle - scan.angle_min) / scan.angle_increment)
            return max(0, min(num_readings - 1, idx))
        
        # Get indices for sector boundaries
        start_idx = angle_to_index(angle_min_rad)
        end_idx = angle_to_index(angle_max_rad)
        
        # Determine which indices to scan
        # If start > end, the sector wraps around (e.g., -7.5° to +7.5° becomes 352.5° to 7.5°)
        indices_to_scan = []
        if start_idx <= end_idx:
            # Simple case: continuous range
            indices_to_scan = list(range(start_idx, end_idx + 1))
        else:
            # Wraparound case: scan from start to end of array, then from 0 to end
            indices_to_scan = list(range(start_idx, num_readings)) + list(range(0, end_idx + 1))

        # Find the minimum valid distance in this sector
        min_distance: Optional[float] = None

        for idx in indices_to_scan:
            range_value = scan.ranges[idx]
            
            # Skip invalid range values
            if not is_valid_range(range_value):
                continue
            
            # Respect the LaserScan's range_min and range_max if specified
            if scan.range_min > 0.0 and range_value < scan.range_min:
                continue
            if scan.range_max > 0.0 and range_value > scan.range_max:
                continue

            # Update minimum if this is the smallest valid reading
            if min_distance is None or range_value < min_distance:
                min_distance = range_value

        return min_distance

    def meters_to_cm_string(self, distance_m: Optional[float]) -> str:
        """
        Convert a distance in meters to a formatted string in centimeters.
        
        Args:
            distance_m: Distance in meters, or None if no valid reading
            
        Returns:
            Formatted string showing distance in cm, or "N/A" if no valid reading
        """
        if distance_m is None:
            return "N/A (no valid data)"
        
        distance_cm = distance_m * 100.0
        return f"{distance_cm:.1f} cm"


def main(args=None) -> None:
    """
    Main entry point for the EnvironmentScanner node.
    
    Initializes ROS 2, creates the node, and spins until shutdown.
    """
    rclpy.init(args=args)
    
    node = EnvironmentScanner()
    
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
