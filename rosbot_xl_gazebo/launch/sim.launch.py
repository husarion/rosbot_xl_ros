#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbot_xl_description = get_package_share_directory("rosbot_xl_description")
    xacro_file = PathJoinSubstitution(
        [rosbot_xl_description, "urdf", "rosbot_xl.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                "xacro --verbosity 0 ",
                xacro_file,
                " use_sim:=true",
                " use_gpu:=false",
                " simulation_engine:=ignition",
            ]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rosbot_xl",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_xl_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of imu_broadcaster after robot_controller
    # when spawning without delay ros2_control_node sometimes crashed
    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )

    return LaunchDescription(
        [
            gz_sim,
            robot_state_publisher_node,
            gz_spawn_entity,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_imu_broadcaster_spawner_after_robot_controller_spawner,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
