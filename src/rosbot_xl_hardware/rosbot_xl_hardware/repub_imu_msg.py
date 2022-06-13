#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


class ImuMsgRepublisher(Node):
    def __init__(self):
        super().__init__("repub_imu_msg")
        self.imu_sub_ = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.imu_cb,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

        self.pub_ = self.create_publisher(
            Imu,
            "/imu/data",
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

        self.get_logger().info("IMU msg republisher has been started")

    def imu_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.orientation_covariance[0] = 0.0001
        msg.orientation_covariance[4] = 0.0001
        msg.orientation_covariance[8] = 0.0001

        msg.angular_velocity_covariance[0] = 0.0001
        msg.angular_velocity_covariance[4] = 0.0001
        msg.angular_velocity_covariance[8] = 0.0001

        msg.linear_acceleration_covariance[0] = 0.0001
        msg.linear_acceleration_covariance[4] = 0.0001
        msg.linear_acceleration_covariance[8] = 0.0001

        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuMsgRepublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    try:
        main()
    except rclpy.ROSInterruptException:
        rclpy.shutdown()
