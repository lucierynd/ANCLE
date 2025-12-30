#!/usr/bin/env python3
"""
AUV Depth and Attitude Monitor Node for ROS 2 Humble

This node monitors: 
1. Depth changes from altimeter data: warns if depth changes exceed a threshold
2. Tilt from IMU data: warns if the AUV is not level (roll/pitch != 0)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Altimeter
from std_msgs.msg import String

import math


class AUVMonitorNode(Node):
    """Node to monitor AUV depth and attitude for safety warnings"""

    def __init__(self):
        super().__init__('auv_monitor_node')

        # Declare parameters
        self.declare_parameter('depth_change_threshold', 2.0)  # meters
        self.declare_parameter('tilt_tolerance', 0.05)  # radians (~2.86 degrees)
        self.declare_parameter('warning_rate_limit', 1.0)  # seconds between warnings

        # Get parameters
        self.depth_threshold = self.get_parameter('depth_change_threshold').value
        self.tilt_tolerance = self.get_parameter('tilt_tolerance').value
        self.warning_rate_limit = self.get_parameter('warning_rate_limit').value

        self.get_logger().info(f'Depth change threshold: {self.depth_threshold} m')
        self.get_logger().info(f'Tilt tolerance: {self.tilt_tolerance} rad ({math.degrees(self.tilt_tolerance):.2f} deg)')

        # State variables
        self.previous_depth = None
        self.last_depth_warning_time = None
        self.last_tilt_warning_time = None

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.altimeter_sub = self.create_subscription(
            Altimeter,
            '/altimeter/data',
            self.altimeter_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Publisher for warnings
        self.warning_pub = self.create_publisher(
            String,
            '/auv/warnings',
            10
        )

        self.get_logger().info('AUV Monitor Node initialized and running')

    def publish_warning(self, message: str):
        """Publish a warning message."""
        msg = String()
        msg.data = message
        self.warning_pub.publish(msg)
        self.get_logger().warn(message)

    def altimeter_callback(self, msg: Altimeter):
        """
        Callback for altimeter data.
        Monitors depth changes and warns if they exceed the threshold.
        """
        current_depth = msg.vertical_position
        # self.get_logger().info(f'[DEBUG] Altimeter reading: {current_depth:.2f} m')

        # initial loop 
        if self.previous_depth is None:
            self.previous_depth = current_depth
            return
        
        depth_change = abs(current_depth - self.previous_depth)
        # self.get_logger().info(f'[DEBUG] Depth change: {depth_change:.2f} m')
    
        if depth_change > self.depth_threshold:

            direction = "increased" if current_depth > self.previous_depth else "decreased"
            self.publish_warning(
                f'DEPTH WARNING:  Depth {direction} by {depth_change:.2f} m '
                f'(threshold: {self.depth_threshold} m).'
                f'Previous:  {self.previous_depth:.2f} m, Current: {current_depth:.2f} m'
            )
            self.previous_depth = current_depth

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU data.
        Monitors tilt (roll and pitch) and warns if not level.
        """
        # Extract quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        # self.get_logger().info(f'[DEBUG] IMU reading - Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°')

        # Check if tilted beyond tolerance
        if abs(roll) > self.tilt_tolerance or abs(pitch) > self.tilt_tolerance:
            # Rate limit warnings
            current_time = self.get_clock().now()
            if (self.last_tilt_warning_time is None or
                (current_time - self.last_tilt_warning_time).nanoseconds / 1e9 > self.warning_rate_limit):

                self.publish_warning(
                    f'TILT WARNING: AUV is not level!  '
                    f'Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}° '
                    f'(tolerance: {math.degrees(self.tilt_tolerance):.2f}°)'
                )
                self.last_tilt_warning_time = current_time

    @staticmethod
    def quaternion_to_euler(x:  float, y: float, z: float, w: float) -> tuple: 
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Args:
            x, y, z, w: Quaternion components

        Returns: 
            tuple: (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)

    node = AUVMonitorNode()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AUV Monitor Node')
    finally: 
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()