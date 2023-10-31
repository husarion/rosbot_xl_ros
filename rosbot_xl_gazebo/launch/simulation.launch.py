#!/usr/bin/env python3

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory

def launch_gz_bridge(context: LaunchContext, *args, **kwargs):
    camera_model = context.perform_substitution(LaunchConfiguration('camera_model'))
    lidar_model = context.perform_substitution(LaunchConfiguration('lidar_model'))

    gz_args = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]
    gz_remapping = []

    # Add camera topic
    if camera_model.startswith("intel_realsense"):
        gz_args.append("/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo")
        gz_args.append("/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image")
        gz_args.append("/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo")
        gz_args.append("/camera/depth@sensor_msgs/msg/Image[ignition.msgs.Image")
        gz_args.append("/camera/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked")

        gz_remapping.append(("/camera/camera_info", "/camera/depth/camera_info"))
        gz_remapping.append(("/camera/depth", "/camera/depth/image_raw"))

    elif camera_model.startswith("stereolabs_zed"):
        zed = camera_model[len("stereolabs_"):]

        gz_args.append(f"/{zed}/zed_node/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo")
        gz_args.append(f"/{zed}/zed_node/rgb/image_rect_color@sensor_msgs/msg/Image[ignition.msgs.Image")
        gz_args.append(f"/{zed}/zed_node/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo")
        gz_args.append(f"/{zed}/zed_node/depth@sensor_msgs/msg/Image[ignition.msgs.Image")
        gz_args.append(f"/{zed}/zed_node/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked")

        gz_remapping.append((f"{zed}/zed_node/camera_info", f"/{zed}/zed_node/depth/camera_info"))
        gz_remapping.append((f"{zed}/zed_node/depth", f"/{zed}/zed_node/depth/depth_registered"))
        gz_remapping.append((f"{zed}/zed_node/depth/points", f"/{zed}/zed_node/point_cloud/cloud_registered"))
    else:
        pass

    # Add lidar topic
    if lidar_model.startswith("slamtec_rplidar"):
        gz_args.append("/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan")

    elif lidar_model.startswith("velodyne"):
        gz_args.append("/velodyne_points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked")

        gz_remapping.append(("/velodyne_points/points", "/velodyne_points"))
    else: # TODO: Check ouster
        pass

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=gz_args,
        remappings=gz_remapping,
        output="screen",
    )

    return [gz_bridge_node]

def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="intel_realsense_d435",
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

    map_package = get_package_share_directory("husarion_office_gz")
    world_file = PathJoinSubstitution([map_package, "worlds", "husarion_world.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file], description="SDF world file"
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
        launch_arguments={"gz_args": world_cfg}.items(),
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
            "-x",
            "0",
            "-y",
            "2.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "lidar_model": lidar_model,
            "camera_model": camera_model,
            "include_camera_mount": include_camera_mount,
            "use_sim": "True",
            "simulation_engine": "ignition-gazebo",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_mecanum_arg,
            declare_lidar_model_arg,
            declare_camera_model_arg,
            declare_include_camera_mount_arg,
            declare_world_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            OpaqueFunction(function=launch_gz_bridge),
            gz_spawn_entity,
            bringup_launch,
        ]
    )
