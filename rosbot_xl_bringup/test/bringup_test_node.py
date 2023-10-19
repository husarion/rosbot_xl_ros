# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Husarion
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

import rclpy

from threading import Event
from threading import Thread

from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class BringupTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0

    __test__ = False

    def __init__(self, name="test_node"):
        super().__init__(name)
        self.odom_tf_event = Event()
        self.scan_filter_event = Event()

    def create_test_subscribers_and_publishers(self):
        self.imu_publisher = self.create_publisher(Imu, "_imu/data_raw", 10)

        self.joint_states_publisher = self.create_publisher(
            JointState, "_motors_response", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_publisher = self.create_publisher(LaserScan, "scan", 10)
        self.filtered_scan_subscriber = self.create_subscription(
            LaserScan, "scan_filtered", self.filtered_scan_callback, 10
        )

        self.timer = None

    def lookup_transform_odom(self):
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            self.odom_tf_event.set()
        except TransformException as ex:
            self.get_logger().error(f"Could not transform odom to base_link: {ex}")

    def start_node_thread(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def start_publishing_fake_hardware(self):
        self.timer = self.create_timer(
            1.0 / self.ROSBOT_HARDWARE_PUBLISHERS_RATE,
            self.timer_callback,
        )

    def filtered_scan_callback(self, msg):
        if msg.ranges[0] == 5.0 and msg.ranges[-1] == 5.0:
            for i in range(len(msg.ranges) - 2):
                # when is not .nan
                if msg.ranges[i + 1] == msg.ranges[i + 1]:
                    return

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

        self.imu_publisher.publish(imu_msg)
        self.joint_states_publisher.publish(joint_state_msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "base_link"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * 3.14159265
        msg.angle_increment = 0.05
        msg.time_increment = 0.1
        msg.scan_time = 0.1
        msg.range_min = 0.0
        msg.range_max = 10.0

        # All points inside filtered box
        msg.ranges = [0.1] * 126
        msg.ranges[0] = 5.0
        msg.ranges[-1] = 5.0
        self.scan_publisher.publish(msg)
