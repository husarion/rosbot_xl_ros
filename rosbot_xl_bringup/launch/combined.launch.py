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
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
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

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
    )

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="None",
        description="Add camera model to the robot URDF",
        choices=[
            "None",
            "intel_realsense_d435",
            "orbbec_astra",
            "stereolabs_zed",
            "stereolabs_zedm",
            "stereolabs_zed2",
            "stereolabs_zed2i",
            "stereolabs_zedx",
            "stereolabs_zedxm",
        ],
    )

    lidar_model = LaunchConfiguration("lidar_model")
    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="slamtec_rplidar_s1",
        description="Add LiDAR model to the robot URDF",
        choices=[
            "None",
            "slamtec_rplidar_a2",
            "slamtec_rplidar_a3",
            "slamtec_rplidar_s1",
            "slamtec_rplidar_s2",
            "slamtec_rplidar_s3",
            "velodyne_puck",
        ],
    )

    include_camera_mount = LaunchConfiguration("include_camera_mount")
    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
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

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/bringup.launch.py"]),
        launch_arguments={
            "mecanum": mecanum,
            "lidar_model": lidar_model,
            "camera_model": camera_model,
            "include_camera_mount": include_camera_mount,
            "combined_launch_deprecated": LaunchConfiguration(
                "combined_launch_deprecated", default=True
            ),
        }.items(),
    )

    print(
        "\033[93m[WARN] [launch]: combined.launch.py will be is deprecated and will be removed please use bringup.launch.py instead.\033[0m"
    )
    return LaunchDescription(
        [
            declare_port_arg,
            declare_localhost_only_fastrtps_profiles_file_arg,
            declare_mecanum_arg,
            declare_camera_model_arg,
            declare_lidar_model_arg,
            declare_include_camera_mount_arg,
            OpaqueFunction(function=generate_microros_agent_node),
            bringup_launch,
        ]
    )
