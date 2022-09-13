from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch.conditions import IfCondition

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_cmd = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_hardware"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={"mecanum": mecanum}.items(),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [get_package_share_directory("rosbot_xl_bringup"), "config", "ekf.yaml"]
            )
        ],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_bringup"),
                    "config",
                    "laser_filter.yaml",
                ]
            )
        ],
    )

    actions = [
        declare_mecanum_cmd,
        controller_launch,
        robot_localization_node,
        laser_filter_node,
    ]

    return LaunchDescription(actions)
