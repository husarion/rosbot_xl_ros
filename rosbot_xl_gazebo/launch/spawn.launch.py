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

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def launch_gz_bridge(context: LaunchContext, *args, **kwargs):
    lidar_model = context.perform_substitution(LaunchConfiguration("lidar_model"))
    camera_model = context.perform_substitution(LaunchConfiguration("camera_model"))
    namespace = context.perform_substitution(LaunchConfiguration("namespace"))
    actions = []

    pointcloud_rpy = [
        "1.57",
        "-1.57",
        "0",
    ]

    namespace_ext = "" if namespace == "" else "/" + namespace
    robot_name = "rosbot_xl"
    if namespace != "":
        robot_name = namespace

    depth_camera_child_tf = (
        robot_name + "/base_link" + namespace_ext + "/camera_" + camera_model + "_depth"
    )
    depth_camera_parent_tf = "camera_depth_optical_frame"

    if lidar_model.startswith("slamtec_rplidar"):
        lidar_model = "slamtec_rplidar"

    if lidar_model != "None":
        gz_lidar_remappings_file = PathJoinSubstitution(
            [
                FindPackageShare("rosbot_xl_gazebo"),
                "config",
                LaunchConfiguration(
                    "gz_lidar_remappings_file",
                    default=["gz_", lidar_model, "_remappings.yaml"],
                ),
            ]
        )

        namespaced_gz_lidar_remappings_file = ReplaceString(
            source_file=gz_lidar_remappings_file,
            replacements={"<robot_namespace>": namespace_ext},
        )

        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_lidar_bridge",
                parameters=[{"config_file": namespaced_gz_lidar_remappings_file}],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                output="screen",
                namespace=namespace,
                condition=LaunchConfigurationNotEquals(lidar_model, "None"),
            )
        )

    if camera_model != "None":
        zed_model = None
        if camera_model.startswith("stereolabs_zed"):
            zed_model = camera_model.replace("stereolabs_", "")
            camera_model = "stereolabs_zed"
            depth_camera_child_tf = "rosbot_xl/base_link/camera_" + camera_model + "_depth"
            depth_camera_parent_tf = "camera_center_optical_frame"
            pointcloud_rpy = ["0", "0", "0"]

        gz_camera_remappings_file = PathJoinSubstitution(
            [
                FindPackageShare("rosbot_xl_gazebo"),
                "config",
                LaunchConfiguration(
                    "gz_camera_remappings_file",
                    default=["gz_", camera_model, "_remappings.yaml"],
                ),
            ]
        )

        namespaced_gz_camera_remappings_file = ReplaceString(
            source_file=gz_camera_remappings_file,
            replacements={"<robot_namespace>": namespace_ext},
        )

        if zed_model is not None:
            namespaced_gz_camera_remappings_file = ReplaceString(
                source_file=namespaced_gz_camera_remappings_file,
                replacements={"<zed>": zed_model},
            )

        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_camera_bridge",
                parameters=[{"config_file": namespaced_gz_camera_remappings_file}],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                output="screen",
                namespace=namespace,
                condition=LaunchConfigurationNotEquals(camera_model, "None"),
            )
        )

        # The frame of the point cloud from ignition gazebo 6 isn't provided by <frame_id>.
        # See https://github.com/gazebosim/gz-sensors/issues/239
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="point_cloud_tf",
                output="log",
                arguments=[
                    "0",
                    "0",
                    "0",
                ]
                + pointcloud_rpy
                + [
                    depth_camera_parent_tf,
                    depth_camera_child_tf,
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
                namespace=namespace,
            )
        )

    return actions


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

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="intel_realsense_d435",
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
        default_value="slamtec_rplidar_s3",
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

    robot_name = PythonExpression(
        ["'rosbot_xl'", " if '", namespace, "' == '' ", "else ", "'", namespace, "'"]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x", default="0.00"),
            "-y",
            LaunchConfiguration("y", default="0.00"),
            "-z",
            LaunchConfiguration("z", default="0.00"),
            "-R",
            LaunchConfiguration("roll", default="0.00"),
            "-P",
            LaunchConfiguration("pitch", default="0.00"),
            "-Y",
            LaunchConfiguration("yaw", default="0.00"),
        ],
        output="screen",
        namespace=namespace,
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace=namespace,
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_xl_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "use_sim": "True",
            "simulation_engine": "ignition-gazebo",
            "camera_model": camera_model,
            "lidar_model": lidar_model,
            "include_camera_mount": include_camera_mount,
            "namespace": namespace,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_camera_model_arg,
            declare_lidar_model_arg,
            declare_include_camera_mount_arg,
            # Sets use_sim_time for all nodes started below
            # (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            ign_clock_bridge,
            gz_spawn_entity,
            bringup_launch,
            OpaqueFunction(function=launch_gz_bridge),
        ]
    )
