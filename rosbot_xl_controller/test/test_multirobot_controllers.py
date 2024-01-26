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
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from test_utils import ControllersTestNode, controller_test

robot_names = ["rosbot_xl1", "rosbot_xl2", "rosbot_xl3"]


@launch_pytest.fixture
def generate_test_description():
    proc_env = os.environ.copy()
    proc_env["ROS_LOCALHOST_ONLY"] = "1"
    proc_env["ROS_DOMAIN_ID"] = random.randint(0, 255)

    rosbot_controller = get_package_share_directory("rosbot_xl_controller")
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
        delayed_controller_launch = TimerAction(period=i * 2.0, actions=[controller_launch])
        actions.append(delayed_controller_launch)

    return LaunchDescription(actions)


@pytest.mark.launch(fixture=generate_test_description)
def test_multirobot_controllers_startup_success():
    for robot_name in robot_names:
        rclpy.init()
        try:
            node = ControllersTestNode(f"test_{robot_name}_controllers", namespace=robot_name)
            node.create_test_subscribers_and_publishers()
            node.start_publishing_fake_hardware()
            node.start_node_thread()
            controller_test(node, robot_name)
        finally:
            rclpy.shutdown()
