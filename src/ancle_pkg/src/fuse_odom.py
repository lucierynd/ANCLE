#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class FuseOdom(Node):
    def __init__(self):
        super().__init__('fuse_odom')

        # Parameters
        self.declare_parameter('rf2o_topic', '/odom_rf2o_projected')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('altimeter_topic', '/altimeter_odom')
        self.declare_parameter('odom_topic', '/fused_odom')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('position_covariance', 0.05)
        self.declare_parameter('velocity_covariance', 0.1)
        self.declare_parameter('publish_tf', False)

        rf2o_topic = self.get_parameter('rf2o_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        altimeter_topic = self.get_parameter('altimeter_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.pos_cov = self.get_parameter('position_covariance').value
        self.vel_cov = self.get_parameter('velocity_covariance').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Latest sensor data
        self.latest_rf2o = None
        self.latest_imu = None
        self.latest_altimeter = None

        # Subscribers 
        self.rf2o_sub = self.create_subscription(
            Odometry,
            rf2o_topic,
            self.rf2o_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        self.altimeter_sub = self.create_subscription(
            Odometry,
            altimeter_topic,
            self.altimeter_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(Odometry, odom_topic, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Fusing {rf2o_topic} + {imu_topic} + {altimeter_topic} -> {odom_topic}'
        )
        if self.publish_tf:
            self.get_logger().info(
                f'Broadcasting TF: {self.output_frame} -> {self.child_frame}'
            )

    # Store latest data, then attempt to publish

    def rf2o_callback(self, msg: Odometry):
        self.latest_rf2o = msg
        self.publish_fused()

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self.publish_fused()

    def altimeter_callback(self, msg: Altimeter):
        self.latest_altimeter = msg
        self.publish_fused()

    # Fusion

    def publish_fused(self):
        # Wait until we have at least one message from every source
        if self.latest_rf2o is None or self.latest_imu is None or self.latest_altimeter is None:
            return

        stamp = self.get_clock().now().to_msg()

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.child_frame

        # Position
        # x, y from rf2o
        out.pose.pose.position.x = self.latest_rf2o.pose.pose.position.x
        out.pose.pose.position.y = self.latest_rf2o.pose.pose.position.y
        # z from the altimeter
        out.pose.pose.position.z = self.latest_altimeter.pose.pose.position.z

        # Orientation
        # Full orientation (roll, pitch, yaw) from the IMU quaternion
        out.pose.pose.orientation = self.latest_imu.orientation

        # Linear velocity
        # x, y from rf2o twist, z from altimeter vertical_velocity
        out.twist.twist.linear.x = self.latest_rf2o.twist.twist.linear.x
        out.twist.twist.linear.y = self.latest_rf2o.twist.twist.linear.y
        out.twist.twist.linear.z = self.latest_altimeter.twist.twist.linear.z

        # Angular velocity
        # From IMU gyroscope
        out.twist.twist.angular = self.latest_imu.angular_velocity

        # Covariance: same for everything except x and y (twist and pose) -> depends on rf2o covariance
        rf2o_pose_cov = self.latest_rf2o.pose.covariance
        rf2o_twist_cov = self.latest_rf2o.twist.covariance
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        for i in range(6):
            pose_cov[i * 7] = self.pos_cov
            twist_cov[i * 7] = self.vel_cov
        pose_cov[0]  = rf2o_pose_cov[0]    # x from rf2o
        pose_cov[7]  = rf2o_pose_cov[7]    # y from rf2o
        twist_cov[0]  = rf2o_twist_cov[0]  # vx from rf2o
        twist_cov[7]  = rf2o_twist_cov[7]  # vy from rf2o
        out.pose.covariance = pose_cov
        out.twist.covariance = twist_cov

        self.publisher.publish(out)

        # TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.output_frame
            t.child_frame_id = self.child_frame

            t.transform.translation.x = out.pose.pose.position.x
            t.transform.translation.y = out.pose.pose.position.y
            t.transform.translation.z = out.pose.pose.position.z
            t.transform.rotation = out.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FuseOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()