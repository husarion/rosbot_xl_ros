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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from test_utils import ControllersTestNode, controller_test

robot_names = ["rosbot_xl1", "rosbot_xl2", "rosbot_xl3"]


@launch_pytest.fixture
def generate_test_description():
    rosbot_controller = FindPackageShare("rosbot_xl_controller")
    actions = []
    for i in range(len(robot_names)):
        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        rosbot_controller,
                        "launch",
                        "controller.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_sim": "False",
                "mecanum": "True",
                "namespace": robot_names[i],
            }.items(),
        )

        # When there is no delay the controllers doesn't spawn correctly
        delayed_controller_launch = TimerAction(period=i * 5.0, actions=[controller_launch])
        actions.append(delayed_controller_launch)

    return LaunchDescription(actions)


@pytest.mark.launch(fixture=generate_test_description)
def test_multirobot_controllers_startup_success():
    for robot_name in robot_names:
        rclpy.init()
        try:
            node = ControllersTestNode(f"test_{robot_name}_controllers", namespace=robot_name)
            node.start_publishing_fake_hardware()
            Thread(target=lambda node: rclpy.spin(node), args=(node,)).start()
            controller_test(node, robot_name)
        finally:
            rclpy.shutdown()
