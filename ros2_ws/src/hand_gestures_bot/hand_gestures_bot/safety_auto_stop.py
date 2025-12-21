#!/usr/bin/env python3
"""
TurtleBot3 Safety Auto-Stop / Parking-Sensor-Style Front Scanner (ROS 2 Humble)

- Subscribes:  LaserScan (default: /scan)
- Subscribes:  incoming Twist commands (default: /cmd_vel_in)
- Publishes:   filtered Twist commands (default: /cmd_vel)

Behavior:
- Computes the minimum valid obstacle distance in a configurable front cone (±front_angle_deg/2).
- If an obstacle is closer than safe_distance, it overrides output with STOP.
- Uses a stop-hold time to prevent flickering when scan ranges fluctuate.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def _is_valid_range(r: float) -> bool:
    """Valid ranges are finite, > 0."""
    return (r is not None) and (r > 0.0) and math.isfinite(r)


class SafetyAutoStop(Node):
    """
    Filters incoming velocity commands based on LiDAR front-cone proximity.
    Publishes STOP if an obstacle is too close; otherwise forwards commands.
    """

    def __init__(self) -> None:
        super().__init__("safety_auto_stop")

        # -------------------------
        # Parameters (ROS 2 params)
        # -------------------------
        self.declare_parameter("safe_distance", 0.25)        # meters
        self.declare_parameter("front_angle_deg", 50.0)      # total cone width (±25° around forward)
        self.declare_parameter("cmd_in_topic", "/cmd_vel_in")
        self.declare_parameter("cmd_out_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("stop_hold_s", 0.25)          # seconds (short hold to prevent flicker)
        self.declare_parameter("debug_log_min_distance", False)

        self.safe_distance: float = float(self.get_parameter("safe_distance").value)
        self.front_angle_deg: float = float(self.get_parameter("front_angle_deg").value)
        self.cmd_in_topic: str = str(self.get_parameter("cmd_in_topic").value)
        self.cmd_out_topic: str = str(self.get_parameter("cmd_out_topic").value)
        self.scan_topic: str = str(self.get_parameter("scan_topic").value)
        self.stop_hold_s: float = float(self.get_parameter("stop_hold_s").value)
        self.debug_log_min_distance: bool = bool(self.get_parameter("debug_log_min_distance").value)

        # Sanity checks / clamping
        if self.safe_distance <= 0.0:
            self.get_logger().warn("safe_distance <= 0.0; clamping to 0.05")
            self.safe_distance = 0.05
        if self.front_angle_deg <= 0.0:
            self.get_logger().warn("front_angle_deg <= 0.0; clamping to 10.0")
            self.front_angle_deg = 10.0
        if self.stop_hold_s < 0.0:
            self.get_logger().warn("stop_hold_s < 0.0; clamping to 0.0")
            self.stop_hold_s = 0.0

        # -------------------------
        # QoS
        # -------------------------
        # LaserScan in Gazebo is typically BEST_EFFORT to avoid latency/backpressure.
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # cmd_vel filtering: reliable is fine (control commands).
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # -------------------------
        # State
        # -------------------------
        self.min_front_distance_m: Optional[float] = None
        self.last_cmd_in: Twist = Twist()
        self.last_obstacle_time_ns: Optional[int] = None

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            scan_qos,
        )

        self.cmd_in_sub = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self._on_cmd_in,
            cmd_qos,
        )

        self.cmd_out_pub = self.create_publisher(
            Twist,
            self.cmd_out_topic,
            cmd_qos,
        )

        # Timer to publish filtered cmd at a steady rate.
        # This ensures STOP can be held even if cmd_in messages pause.
        self.publish_rate_hz = 20.0
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_filtered_cmd)

        # Startup log
        self.get_logger().info("SafetyAutoStop node started.")
        self.get_logger().info(
            f"Parameters: safe_distance={self.safe_distance:.3f} m, "
            f"front_angle_deg={self.front_angle_deg:.1f} deg, "
            f"stop_hold_s={self.stop_hold_s:.2f} s, "
            f"scan_topic='{self.scan_topic}', "
            f"cmd_in_topic='{self.cmd_in_topic}', "
            f"cmd_out_topic='{self.cmd_out_topic}', "
            f"debug_log_min_distance={self.debug_log_min_distance}"
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_cmd_in(self, msg: Twist) -> None:
        """Store the latest incoming velocity command to be filtered."""
        self.last_cmd_in = msg

    def _on_scan(self, msg: LaserScan) -> None:
        """Compute the minimum valid obstacle distance in the configured front cone."""
        min_dist = self._compute_min_front_distance(msg)
        self.min_front_distance_m = min_dist

        now_ns = self.get_clock().now().nanoseconds
        if min_dist is not None and min_dist < self.safe_distance:
            # Obstacle too close → trigger/refresh stop hold
            self.last_obstacle_time_ns = now_ns

        if self.debug_log_min_distance and min_dist is not None:
            self.get_logger().info(f"Min front distance: {min_dist:.3f} m")

    # -------------------------
    # Core logic
    # -------------------------
    def _compute_min_front_distance(self, scan: LaserScan) -> Optional[float]:
        """
        Compute minimum valid range within the front cone.

        Assumptions:
        - Forward direction is angle=0 in the LaserScan frame (typical for TB3).
        - LaserScan angles go from angle_min to angle_max with angle_increment.
        """
        if not scan.ranges or scan.angle_increment == 0.0:
            return None

        # Front cone is centered at 0 rad, half-angle is front_angle_deg/2.
        half_angle_rad = math.radians(self.front_angle_deg * 0.5)

        # Compute index range that corresponds to [-half_angle_rad, +half_angle_rad]
        # Clamp to scan bounds.
        start_angle = -half_angle_rad
        end_angle = +half_angle_rad

        # Convert angles to indices
        start_idx = int(math.floor((start_angle - scan.angle_min) / scan.angle_increment))
        end_idx = int(math.ceil((end_angle - scan.angle_min) / scan.angle_increment))

        start_idx = max(0, start_idx)
        end_idx = min(len(scan.ranges) - 1, end_idx)

        if start_idx > end_idx:
            return None

        min_valid = None
        # Iterate only the relevant cone slice
        for i in range(start_idx, end_idx + 1):
            r = scan.ranges[i]
            if not _is_valid_range(r):
                continue
            # Also respect scan.range_min/range_max if provided
            if scan.range_min > 0.0 and r < scan.range_min:
                continue
            if scan.range_max > 0.0 and r > scan.range_max:
                continue

            if min_valid is None or r < min_valid:
                min_valid = r

        return min_valid

    def _should_stop(self) -> Tuple[bool, Optional[float]]:
        """
        Decide whether to stop based on current min distance and stop-hold timer.

        Returns:
            (stop_active, min_front_distance_m)
        """
        now_ns = self.get_clock().now().nanoseconds

        # If we have a recent obstacle trigger, hold STOP for stop_hold_s
        if self.last_obstacle_time_ns is not None and self.stop_hold_s > 0.0:
            hold_ns = int(self.stop_hold_s * 1e9)
            if now_ns - self.last_obstacle_time_ns <= hold_ns:
                return True, self.min_front_distance_m

        # Otherwise, stop only if we currently see an obstacle too close
        if self.min_front_distance_m is not None and self.min_front_distance_m < self.safe_distance:
            # refresh last_obstacle_time_ns so it becomes "sticky" even without hold
            self.last_obstacle_time_ns = now_ns
            return True, self.min_front_distance_m

        return False, self.min_front_distance_m

    def _publish_filtered_cmd(self) -> None:
        """Publish either the forwarded command or STOP depending on safety."""
        stop, min_dist = self._should_stop()

        if stop:
            # Override with STOP
            out = Twist()
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.x = 0.0
            out.angular.y = 0.0
            out.angular.z = 0.0

            self.cmd_out_pub.publish(out)

            # Optional debug: log transitions (avoid spamming every tick)
            # We'll log only when obstacle is near and the incoming cmd is non-zero.
            if (abs(self.last_cmd_in.linear.x) > 1e-3) or (abs(self.last_cmd_in.angular.z) > 1e-3):
                if min_dist is not None:
                    self.get_logger().warn(
                        f"AUTO-STOP: obstacle at {min_dist:.3f} m < {self.safe_distance:.3f} m. Publishing STOP."
                    )
                else:
                    self.get_logger().warn("AUTO-STOP: obstacle detected. Publishing STOP.")
            return

        # Forward incoming command unchanged
        self.cmd_out_pub.publish(self.last_cmd_in)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyAutoStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
