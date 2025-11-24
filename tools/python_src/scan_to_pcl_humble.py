import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg


class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')

        self.lp = lg.LaserProjection()

        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/lidar/points',
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("LaserScan → PointCloud2 converter running.")

    def scan_callback(self, msg: LaserScan):
        # Convert LaserScan to PointCloud2
        pc2_msg = self.lp.projectLaser(msg)
        self.pc_pub.publish(pc2_msg)

        # Iterate through points (generator)
        point_generator = pc2.read_points(pc2_msg, skip_nans=True)

        sum_z = 0.0
        num = 0

        for point in point_generator:
            sum_z += point[2]
            num += 1

        if num > 0:
            avg_z = sum_z / num
            self.get_logger().info(f"Avg Z = {avg_z:.4f}")

        # Convert to list
        point_list = list(pc2.read_points(pc2_msg, skip_nans=True))

        if len(point_list) > 0:
            mid_index = len(point_list) // 2
            mid_point = point_list[mid_index]
            self.get_logger().info(f"Mid point X = {mid_point[0]:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
