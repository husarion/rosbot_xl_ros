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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    namespace_tf_prefix = PythonExpression([
        "''", " if '", namespace, "' == '' ", "else ", "'", namespace, "_'"
    ])
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and tfs",
    )

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller (otherwise diff drive controller is used)"
        ),
    )

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="None",
        description="Add camera model to the robot URDF",
        choices=[
            "None",
            "intel_realsense_d435",
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
            "ouster_os1_32",
            "slamtec_rplidar_a2",
            "slamtec_rplidar_a3",
            "slamtec_rplidar_s1",
            "velodyne_puck",
        ],
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

    rosbot_xl_controller = get_package_share_directory("rosbot_xl_controller")
    rosbot_xl_bringup = get_package_share_directory("rosbot_xl_bringup")

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                rosbot_xl_controller,
                "launch",
                "controller.launch.py",
            ])
        ),
        launch_arguments={
            "mecanum": mecanum,
            "lidar_model": lidar_model,
            "camera_model": camera_model,
            "include_camera_mount": include_camera_mount,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
            "namespace": namespace,
        }.items(),
    )

    ekf_config = PathJoinSubstitution([rosbot_xl_bringup, "config", "ekf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {
                "map_frame": LaunchConfiguration(
                    "ekf_map_frame", default=[namespace_tf_prefix, "map"]
                )
            },
            {
                "odom_frame": LaunchConfiguration(
                    "ekf_odom_frame", default=[namespace_tf_prefix, "odom"]
                )
            },
            {
                "base_link_frame": LaunchConfiguration(
                    "ekf_base_link_frame", default=[namespace_tf_prefix, "base_link"]
                )
            },
            {
                "world_frame": LaunchConfiguration(
                    "ekf_world_frame", default=[namespace_tf_prefix, "odom"]
                )
            },
        ],
        namespace=namespace,
    )

    laser_filter_config = PathJoinSubstitution([
        rosbot_xl_bringup,
        "config",
        "laser_filter.yaml",
    ])

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            laser_filter_config,
            {
                "filter1.params.box_frame": LaunchConfiguration(
                    "laser_filter_box_frame", default=[namespace_tf_prefix, "base_link"]
                )
            },
        ],
        namespace=namespace,
    )

    return LaunchDescription([
        declare_namespace_arg,
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
    ])
