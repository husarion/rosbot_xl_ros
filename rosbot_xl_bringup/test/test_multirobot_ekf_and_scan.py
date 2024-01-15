# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Intel Corporation. All Rights Reserved.
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

import launch_pytest
import pytest
import rclpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from test_utils import BringupTestNode


robot_names = ["rosbot1", "rosbot2", "rosbot3", "rosbot4"]


@launch_pytest.fixture
def generate_test_description():
    rosbot_xl_bringup = get_package_share_directory("rosbot_xl_bringup")
    actions = []
    for i in range(len(robot_names)):
        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    rosbot_xl_bringup,
                    "launch",
                    "bringup.launch.py",
                ])
            ),
            launch_arguments={
                "use_sim": "False",
                "mecanum": "False",
                "namespace": robot_names[i],
            }.items(),
        )

        actions.append(bringup_launch)

    return LaunchDescription(actions)


@pytest.mark.launch(fixture=generate_test_description)
def test_namespaced_bringup_startup_success():
    for robot_name in robot_names:
        rclpy.init()
        try:
            node = BringupTestNode("test_bringup", namespace=robot_name)
            node.create_test_subscribers_and_publishers()
            node.start_publishing_fake_hardware()

            node.start_node_thread()
            msgs_received_flag = node.odom_tf_event.wait(timeout=10.0)
            assert (
                msgs_received_flag
            ), "Expected odom to base_link tf but it was not received. Check robot_localization!"

        finally:
            rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_namespaced_bringup_scan_filter():
    for robot_name in robot_names:
        rclpy.init()
        try:
            node = BringupTestNode("test_bringup", namespace=robot_name)
            node.create_test_subscribers_and_publishers()
            node.start_publishing_fake_hardware()

            node.start_node_thread()
            msgs_received_flag = node.scan_filter_event.wait(timeout=10.0)
            assert (
                msgs_received_flag
            ), "Expected filtered scan but it is not filtered properly. Check laser_filter!"

        finally:
            rclpy.shutdown()
