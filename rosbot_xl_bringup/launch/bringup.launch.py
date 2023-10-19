#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    lidar_model = LaunchConfiguration("lidar_model")
    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="slamtec_rplidar_s1",
        description="Lidar model added to the URDF",
    )

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="None",
        description="Camera model added to the URDF",
    )

    include_camera_mount = LaunchConfiguration("include_camera_mount")
    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="ignition-gazebo",
        description="Which simulation engine will be used",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "lidar_model": lidar_model,
            "camera_model": camera_model,
            "include_camera_mount": include_camera_mount,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
        }.items(),
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
        declare_mecanum_arg,
        declare_lidar_model_arg,
        declare_camera_model_arg,
        declare_include_camera_mount_arg,
        declare_use_sim_arg,
        declare_simulation_engine_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        controller_launch,
        robot_localization_node,
        laser_filter_node,
    ]

    return LaunchDescription(actions)
