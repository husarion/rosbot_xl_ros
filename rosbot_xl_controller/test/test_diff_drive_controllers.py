# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Intel Corporation. All Rights Reserved.
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

from threading import Thread

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from test_utils import ControllersTestNode, controller_test


@launch_pytest.fixture
def generate_test_description():
    rosbot_xl_controller = FindPackageShare("rosbot_xl_controller")
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_xl_controller,
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": "False",
            "mecanum": "False",
        }.items(),
    )

    return LaunchDescription([bringup_launch])


@pytest.mark.launch(fixture=generate_test_description)
def test_controllers_startup_fail():
    rclpy.init()
    try:
        node = ControllersTestNode("test_controllers_startup_fail")

        Thread(target=lambda node: rclpy.spin(node), args=(node,)).start()
        msgs_received_flag = node.joint_state_msg_event.wait(10.0)
        assert not msgs_received_flag, (
            "Received JointStates message that should not have appeared. Check whether other"
            " robots are connected to your network.!"
        )
        msgs_received_flag = node.odom_msg_event.wait(10.0)
        assert not msgs_received_flag, (
            "Received Odom message that should not have appeared. Check whether other robots are"
            " connected to your network.!"
        )
        msgs_received_flag = node.imu_msg_event.wait(10.0)
        assert not msgs_received_flag, (
            "Received Imu message that should not have appeared. Check whether other robots are"
            " connected to your network.!"
        )
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_controllers_startup():
    rclpy.init()
    try:
        node = ControllersTestNode("test_controllers_startup")
        node.start_publishing_fake_hardware()
        Thread(target=lambda node: rclpy.spin(node), args=(node,)).start()
        controller_test(node)
    finally:
        rclpy.shutdown()
