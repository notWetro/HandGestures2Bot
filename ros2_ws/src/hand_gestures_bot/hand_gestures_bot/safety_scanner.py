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
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from turtlebot3_msgs.srv import Sound  # type: ignore
except Exception:  # pragma: no cover
    Sound = None  # type: ignore


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

    # -------------------------------------------------------------------------
    # Beep Configuration (parking-sensor like)
    # -------------------------------------------------------------------------
    BEEP_NEAR_M = 0.25
    BEEP_MID_M = 0.30
    BEEP_FAR_M = 0.40

    BEEP_NEAR_INTERVAL_S = 0.25
    BEEP_MID_INTERVAL_S = 0.6

    def __init__(self) -> None:
        """Initialize the SafetyScanner node."""
        super().__init__("safety_scanner")

        # Store the latest LaserScan message
        self.latest_scan: Optional[LaserScan] = None
        self._last_scan_time_s: Optional[float] = None

        # If scan data stops (real robot LiDAR issues), fail-safe block forward.
        # This is intentionally conservative to prevent driving blind.
        self.declare_parameter("fail_safe_on_scan_timeout", True)
        self.declare_parameter("scan_timeout_sec", 1.0)
        self.declare_parameter("scan_topic", "/scan")
        self.fail_safe_on_scan_timeout = bool(self.get_parameter("fail_safe_on_scan_timeout").value)
        self.scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        
        # Track obstacle state
        self.obstacle_detected: bool = False
        self.last_obstacle_sector: str = ""

        # -------------------------------------------------------------------------
        # Beep state
        # -------------------------------------------------------------------------
        self._last_beep_time_s: float = 0.0
        self._far_beep_done: bool = False

        # -------------------------------------------------------------------------
        # QoS Profiles
        # -------------------------------------------------------------------------
        # TurtleBot3's LiDAR publishes with BEST_EFFORT reliability.
        # Subscriber MUST use BEST_EFFORT to be compatible (RELIABLE subscriber
        # cannot receive from BEST_EFFORT publisher in ROS 2).
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
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
            self.scan_topic,
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
        # Create obstacle status publisher (for WebSocket/app feedback)
        # -------------------------------------------------------------------------
        self.obstacle_status_pub = self.create_publisher(
            String,
            "/obstacle_status",
            cmd_qos,
        )

        # -------------------------------------------------------------------------
        # Optional sound service client (TurtleBot3 buzzer)
        # -------------------------------------------------------------------------
        self.sound_client = None
        if Sound is not None:
            self.sound_client = self.create_client(Sound, "/sound")

        # -------------------------------------------------------------------------
        # Create timer for periodic logging (every 2 seconds)
        # -------------------------------------------------------------------------
        self.log_timer = self.create_timer(
            timer_period_sec=2.0,
            callback=self.log_timer_callback,
        )

        # -------------------------------------------------------------------------
        # Beep timer (10Hz)
        # -------------------------------------------------------------------------
        self.beep_timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.beep_timer_callback,
        )

        # -------------------------------------------------------------------------
        # Startup log
        # -------------------------------------------------------------------------
        self.get_logger().info("SafetyScanner node started.")
        self.get_logger().info(f"Subscribing to: {self.scan_topic}, /cmd_vel_in")
        self.get_logger().info("Publishing to: /cmd_vel, /obstacle_status")
        self.get_logger().info(f"Safety distance: {self.SAFETY_DISTANCE_M * 100:.0f} cm")
        self.get_logger().info("⚠️ Send commands to /cmd_vel_in (NOT /cmd_vel directly!)")
        if self.sound_client is not None:
            self.get_logger().info("Beep enabled: using /sound service")
        else:
            self.get_logger().warn("Beep disabled: turtlebot3_msgs/srv/Sound not available")

    def _call_beep(self) -> None:
        if self.sound_client is None or Sound is None:
            return

        if not self.sound_client.service_is_ready():
            return  # Don't block, just skip if service not ready

        request = Sound.Request()
        # Sound values: OFF=0, ON=1, LOW_BATTERY=2, ERROR=3, BUTTON1=4, BUTTON2=5
        # Use ERROR (3) for a warning-like sound
        request.value = 3
        # Call asynchronously to avoid blocking
        self.sound_client.call_async(request)

    def _get_front_min_distance(self) -> Optional[float]:
        if self.latest_scan is None:
            return None

        left_m = self.get_sector_min_distance(
            self.latest_scan, self.LEFT_SECTOR_MIN_DEG, self.LEFT_SECTOR_MAX_DEG
        )
        center_m = self.get_sector_min_distance(
            self.latest_scan, self.CENTER_SECTOR_MIN_DEG, self.CENTER_SECTOR_MAX_DEG
        )
        right_m = self.get_sector_min_distance(
            self.latest_scan, self.RIGHT_SECTOR_MIN_DEG, self.RIGHT_SECTOR_MAX_DEG
        )

        candidates = [d for d in (left_m, center_m, right_m) if d is not None]
        if not candidates:
            return None
        return min(candidates)

    def beep_timer_callback(self) -> None:
        """Emit beeps depending on closest obstacle distance in front sectors."""
        if self.sound_client is None:
            return

        front_min_m = self._get_front_min_distance()
        if front_min_m is None:
            self._far_beep_done = False
            return

        now_s = time.monotonic()

        # Very close: fast beeps
        if front_min_m <= self.BEEP_NEAR_M:
            if (now_s - self._last_beep_time_s) >= self.BEEP_NEAR_INTERVAL_S:
                self._call_beep()
                self._last_beep_time_s = now_s
            self._far_beep_done = True
            return

        # Close: slower beeps
        if front_min_m <= self.BEEP_MID_M:
            if (now_s - self._last_beep_time_s) >= self.BEEP_MID_INTERVAL_S:
                self._call_beep()
                self._last_beep_time_s = now_s
            self._far_beep_done = True
            return

        # Far: single beep once when entering this band
        if front_min_m <= self.BEEP_FAR_M:
            if not self._far_beep_done:
                self._call_beep()
                self._last_beep_time_s = now_s
                self._far_beep_done = True
            return

        # Clear
        self._far_beep_done = False

    def scan_callback(self, msg: LaserScan) -> None:
        """Store the latest scan data and update obstacle state."""
        self.latest_scan = msg
        self._last_scan_time_s = time.monotonic()
        self._update_obstacle_state()

    def _scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self._last_scan_time_s is None:
            return False
        if self.scan_timeout_sec <= 0.0:
            return True
        return (time.monotonic() - self._last_scan_time_s) <= self.scan_timeout_sec

    def _update_obstacle_state(self) -> None:
        """Check for obstacles and update state, publish status on change."""
        scan_ok = self._scan_is_fresh()

        # If scan is missing/stale, optionally enter fail-safe mode.
        if not scan_ok:
            was_detected = self.obstacle_detected
            self.obstacle_detected = bool(self.fail_safe_on_scan_timeout)

            if self.obstacle_detected and not was_detected:
                import json
                status_msg = String()
                status_msg.data = json.dumps({
                    "type": "obstacle_status",
                    "blocked": True,
                    "sectors": ["no_scan"],
                    "distances": {},
                    "safety_distance_cm": self.SAFETY_DISTANCE_M * 100,
                    "scan_ok": False,
                    "reason": "no_scan_data",
                })
                self.obstacle_status_pub.publish(status_msg)
                self.get_logger().error(
                    f"No fresh {self.scan_topic} data. Fail-safe: blocking forward motion. "
                    "Check LiDAR topic + bringup laser publisher."
                )
            elif (not self.obstacle_detected) and was_detected:
                import json
                status_msg = String()
                status_msg.data = json.dumps({
                    "type": "obstacle_status",
                    "blocked": False,
                    "sectors": [],
                    "distances": {},
                    "safety_distance_cm": self.SAFETY_DISTANCE_M * 100,
                    "scan_ok": False,
                    "reason": "no_scan_data_fail_safe_disabled",
                })
                self.obstacle_status_pub.publish(status_msg)
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
        distances = {}
        
        if left_m is not None:
            distances["left"] = round(left_m * 100, 1)
            if left_m < self.SAFETY_DISTANCE_M:
                obstacle_sectors.append("left")
        
        if center_m is not None:
            distances["center"] = round(center_m * 100, 1)
            if center_m < self.SAFETY_DISTANCE_M:
                obstacle_sectors.append("center")
        
        if right_m is not None:
            distances["right"] = round(right_m * 100, 1)
            if right_m < self.SAFETY_DISTANCE_M:
                obstacle_sectors.append("right")

        was_detected = self.obstacle_detected
        self.obstacle_detected = len(obstacle_sectors) > 0

        # Publish status on state change
        if self.obstacle_detected != was_detected:
            import json
            status_msg = String()
            status_msg.data = json.dumps({
                "type": "obstacle_status",
                "blocked": self.obstacle_detected,
                "sectors": obstacle_sectors,
                "distances": distances,
                "safety_distance_cm": self.SAFETY_DISTANCE_M * 100
            })
            self.obstacle_status_pub.publish(status_msg)

        # Log state changes
        if self.obstacle_detected:
            sector_str = ", ".join([f"{s.upper()} ({distances.get(s, 0):.1f}cm)" for s in obstacle_sectors])
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
        # Ensure fail-safe is applied even if scan never arrives (real robot misconfig).
        self._update_obstacle_state()

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
        # Keep obstacle state updated even if scan never arrives.
        self._update_obstacle_state()

        scan_ok = self._scan_is_fresh()
        if not scan_ok:
            self.get_logger().warn("No fresh LaserScan data received yet.")

            # Still publish status so the app can show why obstacle system is inactive.
            import json
            status_msg = String()
            status_msg.data = json.dumps({
                "type": "obstacle_status",
                "blocked": bool(self.fail_safe_on_scan_timeout),
                "sectors": ["no_scan"] if self.fail_safe_on_scan_timeout else [],
                "distances": {},
                "safety_distance_cm": self.SAFETY_DISTANCE_M * 100,
                "scan_ok": False,
                "reason": "no_scan_data",
            })
            self.obstacle_status_pub.publish(status_msg)
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

        # Build distances dict for status message
        distances = {}
        obstacle_sectors = []
        
        if left_m is not None:
            distances["left"] = round(left_m * 100, 1)
            if left_m < self.SAFETY_DISTANCE_M:
                obstacle_sectors.append("left")
        if center_m is not None:
            distances["center"] = round(center_m * 100, 1)
            if center_m < self.SAFETY_DISTANCE_M:
                obstacle_sectors.append("center")
        if right_m is not None:
            distances["right"] = round(right_m * 100, 1)
            if right_m < self.SAFETY_DISTANCE_M:
                obstacle_sectors.append("right")

        # Publish status EVERY timer tick (every 2 seconds)
        import json
        status_msg = String()
        status_msg.data = json.dumps({
            "type": "obstacle_status",
            "blocked": self.obstacle_detected,
            "sectors": obstacle_sectors,
            "distances": distances,
            "safety_distance_cm": self.SAFETY_DISTANCE_M * 100,
            "scan_ok": True
        })
        self.obstacle_status_pub.publish(status_msg)

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
