#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    controller_manager_name = "controller_manager"
    if namespace != "":
        controller_manager_name = namespace + "/" + controller_manager_name

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
            "--namespace",
            namespace,
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        name=LaunchConfiguration(
            "robot_controller_spawner_name",
            default=[namespace, "_robot_controller_spawner"],
        ),
        executable="spawner",
        arguments=[
            "rosbot_base_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
            "--namespace",
            namespace,
        ],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        name=LaunchConfiguration(
            "imu_spawner_name", default=[namespace, "_imu_broadcaster_spawner"]
        ),
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
            "--namespace",
            namespace,
        ],
    )

    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )
    return [
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
    ]


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and tfs",
    )

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    controller_config_name = PythonExpression(
        [
            "'mecanum_drive_controller.yaml' if ",
            mecanum,
            " else 'diff_drive_controller.yaml'",
        ]
    )

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_xl_controller"),
            "config",
            controller_config_name,
        ]
    )

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_xl_description"),
                    "launch",
                    "load_urdf.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_joint_state_publisher": "False",
            "use_sim": "True",
        }.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_config_path,
        ],
        remappings=[
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("rosbot_base_controller/cmd_vel", "cmd_vel"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        condition=UnlessCondition(use_sim),
        namespace=namespace,
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_use_sim_arg,
            SetParameter(name="use_sim_time", value=use_sim),
            load_urdf,
            control_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
