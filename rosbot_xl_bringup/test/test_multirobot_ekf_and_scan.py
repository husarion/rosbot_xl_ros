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
from rclpy.executors import SingleThreadedExecutor
from test_utils import BringupTestNode, ekf_and_scan_test

robot_names = ["rosbot1", "rosbot2", "rosbot3"]


@launch_pytest.fixture
def generate_test_description():
    rosbot_xl_bringup = FindPackageShare("rosbot_xl_bringup")
    actions = []
    for i in range(len(robot_names)):
        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        rosbot_xl_bringup,
                        "launch",
                        "bringup.launch.py",
                    ]
                )
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
    rclpy.init()
    try:
        nodes = {}
        executor = SingleThreadedExecutor()

        for node_namespace in robot_names:
            node = BringupTestNode("test_bringup", namespace=node_namespace)
            node.start_publishing_fake_hardware()
            nodes[node_namespace] = node
            executor.add_node(node)

        Thread(target=lambda executor: executor.spin(), args=(executor,)).start()

        for node_namespace in robot_names:
            node = nodes[node_namespace]
            ekf_and_scan_test(node, node_namespace)
            executor.remove_node(node)
            node.destroy_node()

    finally:
        executor.shutdown()
        rclpy.shutdown()
