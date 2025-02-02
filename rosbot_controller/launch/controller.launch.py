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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    robot_model = LaunchConfiguration("robot_model")
    use_sim = LaunchConfiguration("use_sim", default="False")

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    config_file = PythonExpression(
        ["'mecanum_drive_controller.yaml' if ", mecanum, " else 'diff_drive_controller.yaml'"]
    )

    controllers_config_file = PathJoinSubstitution(
        [FindPackageShare("rosbot_controller"), "config", robot_model, config_file]
    )

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "launch",
                    "load_urdf.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_model": robot_model,
            "use_joint_state_publisher": "False",
            "use_sim": use_sim,
        }.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_config_file],
        remappings=[
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("rosbot_base_controller/cmd_vel", "cmd_vel"),
        ],
        condition=UnlessCondition(use_sim),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
    )

    robot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_base_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
    )

    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
        ],
    )

    controllers = [joint_state_broadcaster, robot_controller, imu_broadcaster]

    # spawners expect ros2_control_node to be running
    delayed_spawner_nodes = TimerAction(
        period=3.0,
        actions=controllers,
    )

    def check_if_log_is_fatal(event):
        red_color = "\033[91m"
        reset_color = "\033[0m"
        if "fatal" in event.text.decode().lower() or "failed" in event.text.decode().lower():
            print(f"{red_color}Fatal error: {event.text}. Emitting shutdown...{reset_color}")
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    controllers_monitor = [
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawner,
                on_stderr=check_if_log_is_fatal,
            )
        )
        for spawner in controllers
    ]

    controllers_monitor = GroupAction(controllers_monitor)

    return LaunchDescription(
        [
            declare_mecanum_arg,
            load_urdf,
            control_node,
            delayed_spawner_nodes,
            controllers_monitor,
        ]
    )
