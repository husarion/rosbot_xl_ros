# Copyright 2024 Husarion
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

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_microros_agent_node(context, *args, **kwargs):
    env_setup_actions = []

    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
    if ros_domain_id:
        env_setup_actions.append(
            SetEnvironmentVariable(name="XRCE_DOMAIN_ID_OVERRIDE", value=ros_domain_id)
        )

    port = LaunchConfiguration("port").perform(context)

    localhost_only_fastrtps_profiles_file = LaunchConfiguration(
        "localhost_only_fastrtps_profiles_file"
    ).perform(context)

    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        env_setup_actions.extend(
            [
                LogInfo(
                    msg=[
                        "ROS_LOCALHOST_ONLY set to 1. Using FASTRTPS_DEFAULT_PROFILES_FILE=",
                        localhost_only_fastrtps_profiles_file,
                        ".",
                    ]
                ),
                SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
                SetEnvironmentVariable(
                    name="FASTRTPS_DEFAULT_PROFILES_FILE",
                    value=localhost_only_fastrtps_profiles_file,
                ),
            ]
        )

    microros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["udp4", "--port", port],
        output="screen",
    )

    return env_setup_actions + [microros_agent_node]


def generate_launch_description():
    declare_port_arg = DeclareLaunchArgument(
        "port",
        default_value="8888",
        description="UDP4 port for micro-ROS agent",
    )

    # Locate the rosbot_bringup package
    package_dir = FindPackageShare("rosbot_xl_bringup").find("rosbot_xl_bringup")

    # Construct the path to the XML file within the package
    fastrtps_profiles_file = PathJoinSubstitution(
        [package_dir, "config", "microros_localhost_only.xml"]
    )

    declare_localhost_only_fastrtps_profiles_file_arg = DeclareLaunchArgument(
        "localhost_only_fastrtps_profiles_file",
        default_value=fastrtps_profiles_file,
        description=(
            "Path to the Fast RTPS default profiles file for Micro-ROS agent for localhost only"
            " setup"
        ),
    )

    return LaunchDescription(
        [
            declare_port_arg,
            declare_localhost_only_fastrtps_profiles_file_arg,
            OpaqueFunction(function=generate_microros_agent_node),
        ]
    )
