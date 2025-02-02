# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import random
from threading import Event

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class BringupTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0
    __test__ = False

    def __init__(self, name="test_node", namespace=None):
        super().__init__(
            name,
            namespace=namespace,
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
        )
        self.odom_tf_event = Event()
        self.scan_filter_event = Event()

        self.imu_pub = self.create_publisher(Imu, "/_imu/data_raw", 10)
        self.joint_pub = self.create_publisher(JointState, "/_motors_response", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.scan_filtered_sub = self.create_subscription(
            LaserScan, "scan_filtered", self.filtered_scan_callback, 10
        )

        self.timer = None

    def lookup_transform_odom(self):
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            self.odom_tf_event.set()
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform odom to base_link: {ex}",
                skip_first=True,
                throttle_duration_sec=3.0,
            )

    def start_publishing_fake_hardware(self):
        self.timer = self.create_timer(
            1.0 / self.ROSBOT_HARDWARE_PUBLISHERS_RATE,
            self.timer_callback,
        )

    def filtered_scan_callback(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            self.scan_filter_event.set()

    def timer_callback(self):
        self.publish_fake_hardware_messages()
        self.lookup_transform_odom()
        self.publish_scan()

    def publish_fake_hardware_messages(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            "fl_wheel_joint",
            "fr_wheel_joint",
            "rl_wheel_joint",
            "rr_wheel_joint",
        ]
        joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]
        joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]

        self.imu_pub.publish(imu_msg)
        self.joint_pub.publish(joint_state_msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 0.05
        msg.time_increment = 0.1
        msg.scan_time = 0.1
        msg.range_min = 0.0
        msg.range_max = 10.0

        # fill ranges from 0.0m to 1.0m
        msg.ranges = [random.random() for _ in range(int(msg.angle_max / msg.angle_increment))]
        self.scan_pub.publish(msg)


def ekf_and_scan_test(node: BringupTestNode, robot_name="ROSbot"):
    assert node.odom_tf_event.wait(20.0), (
        f"{robot_name}: Expected odom to base_link tf but it was not received. Check"
        " robot_localization!"
    )

    assert node.scan_filter_event.wait(
        20.0
    ), f"{robot_name}: Expected filtered scan but it is not filtered properly. Check laser_filter!"
