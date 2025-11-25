#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from smbus import SMBus
import struct
import math


class IMUPublisher(object):
    def __init__(self):
        # --- Parameters ---
        self.I2C_BUS = rospy.get_param('~i2c_bus', 7)
        self.ADDRESS = rospy.get_param('~address', 0x6B)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.rate_hz = rospy.get_param('~rate', 800.0)  # 800 Hz default

        # --- I2C setup ---
        self.bus = SMBus(self.I2C_BUS)

        who_am_i = self.bus.read_byte_data(self.ADDRESS, 0x0F)
        rospy.loginfo("WHO_AM_I: 0x%02X" % who_am_i)
        if who_am_i != 0x6B:
            rospy.logwarn("Unexpected device ID — check wiring or address")

        # Configure accelerometer: 208 Hz, ±2 g
        self.bus.write_byte_data(self.ADDRESS, 0x10, 0b01110000)
        # Configure gyroscope: 208 Hz, 2000 dps
        self.bus.write_byte_data(self.ADDRESS, 0x11, 0b01111100)

        # --- Conversion factors ---
        self.ACC_SENS = 0.061 / 1000.0   # g/LSB
        self.GYRO_SENS = 70 / 1000.0     # dps/LSB

        # --- ROS Publisher ---
        self.pub = rospy.Publisher('imu/data', Imu, queue_size=20)
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("IMU publisher started at %.1f Hz" % self.rate_hz)

    def read_xyz(self, base_addr):
        """Read 3-axis data starting at base_addr."""
        data = self.bus.read_i2c_block_data(self.ADDRESS, base_addr, 6)
        x, y, z = struct.unpack('<hhh', bytes(bytearray(data)))
        return x, y, z

    def spin(self):
        """Main loop."""
        while not rospy.is_shutdown():
            try:
                # Read raw gyro and accel
                gx, gy, gz = self.read_xyz(0x22)
                ax, ay, az = self.read_xyz(0x28)

                # Convert to physical units
                ax_g = ax * self.ACC_SENS * 9.805
                ay_g = ay * self.ACC_SENS * 9.805
                az_g = az * self.ACC_SENS * 9.805

                gx_rps = math.radians(gx * self.GYRO_SENS)
                gy_rps = math.radians(gy * self.GYRO_SENS)
                gz_rps = math.radians(gz * self.GYRO_SENS)

                # Fill message
                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id

                msg.linear_acceleration.x = ax_g
                msg.linear_acceleration.y = ay_g
                msg.linear_acceleration.z = az_g

                msg.angular_velocity.x = gx_rps
                msg.angular_velocity.y = gy_rps
                msg.angular_velocity.z = gz_rps

                # Publish
                self.pub.publish(msg)

                self.rate.sleep()

            except IOError as e:
                rospy.logerr_throttle(5.0, "I2C read error: %s" % e)
                self.rate.sleep()
            except Exception as e:
                rospy.logerr_throttle(5.0, "Unexpected error: %s" % e)
                self.rate.sleep()

        self.bus.close()


if __name__ == '__main__':
    rospy.init_node('imu_publisher', anonymous=True)
    try:
        node = IMUPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass

