#!/usr/bin/env python3

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    components_config = LaunchConfiguration("components_config")
    mecanum = LaunchConfiguration("mecanum")
    robot_model = LaunchConfiguration("robot_model")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher", default="True")
    use_sim = LaunchConfiguration("use_sim")
    
    declare_components_config_arg = DeclareLaunchArgument(
        "components_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rosbot_xl_description"), "config", "components.yaml"]
        ),
        description=(
            "Specify file which contains components. These components will be included in URDF."
            "Available options can be found in manuals: https://husarion.com/manuals"
        ),
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
        choices=["True", "False"],
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
        choices=["True", "False"],
    )

    controller_config = PythonExpression(
        [
            "'mecanum_drive_controller.yaml' if ",
            mecanum,
            " else 'diff_drive_controller.yaml'",
        ]
    )

    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_xl_controller"),
            "config",
            controller_config,
        ]
    )

    urdf_file = PythonExpression(["'", robot_model, ".urdf.xacro'"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rosbot_xl_description"), "urdf", urdf_file]),
            " components_config:=",
            components_config,
            " controller_config:=",
            controller_config,
            " mecanum:=",
            mecanum,
            " use_sim:=",
            use_sim,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        emulate_tty=True,
        condition=IfCondition(use_joint_state_publisher),
    )

    return LaunchDescription(
        [
            declare_components_config_arg,
            declare_mecanum_arg,
            declare_robot_model_arg,
            declare_use_sim_arg,
            SetParameter(name="use_sim_time", value=use_sim),
            robot_state_pub_node,
            joint_state_publisher_node,
        ]
    )
