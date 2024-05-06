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
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    mecanum = LaunchConfiguration("mecanum").perform(context)
    camera_model = context.perform_substitution(LaunchConfiguration("camera_model"))
    lidar_model = context.perform_substitution(LaunchConfiguration("lidar_model"))
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    x = LaunchConfiguration("x", default="0.0").perform(context)
    y = LaunchConfiguration("y", default="2.0").perform(context)
    z = LaunchConfiguration("z", default="0.0").perform(context)
    roll = LaunchConfiguration("roll", default="0.0").perform(context)
    pitch = LaunchConfiguration("pitch", default="0.0").perform(context)
    yaw = LaunchConfiguration("yaw", default="0.0").perform(context)

    gz_args = f"--headless-rendering -s -v 4 -r {world}" if eval(headless) else f"-r {world}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "True",
        }.items(),
    )

    logger = LogInfo(msg=["Robots:", LaunchConfiguration("robots")])
    robots_list = ParseMultiRobotPose("robots").value()
    if len(robots_list) == 0:
        robots_list = {
            namespace: {
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        }

    spawn_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]

        spawn_log = LogInfo(
            msg=[f"Launching namespace={robot_name} with init_pose= {str(init_pose)}"]
        )

        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbot_xl_gazebo"),
                        "launch",
                        "spawn.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "mecanum": mecanum,
                "use_sim": "True",
                "lidar_model": lidar_model,
                "camera_model": camera_model,
                "simulation_engine": "ignition-gazebo",
                "namespace": TextSubstitution(text=robot_name),
                "x": TextSubstitution(text=str(init_pose["x"])),
                "y": TextSubstitution(text=str(init_pose["y"])),
                "z": TextSubstitution(text=str(init_pose["z"])),
                "roll": TextSubstitution(text=str(init_pose["roll"])),
                "pitch": TextSubstitution(text=str(init_pose["pitch"])),
                "yaw": TextSubstitution(text=str(init_pose["yaw"])),
            }.items(),
        )

        group = GroupAction(
            actions=[
                spawn_log,
                spawn_robot,
            ],
        )
        spawn_group.append(group)

    return [logger, gz_sim, *spawn_group]


def generate_launch_description():
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all topics and tfs",
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
    )

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

    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
    )

    world_package = FindPackageShare("husarion_gz_worlds")
    world_file = PathJoinSubstitution([world_package, "worlds", "husarion_world.sdf"])
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=world_file, description="Path to SDF world file"
    )

    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo Ignition in the headless mode",
    )

    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value=[],
        description=(
            "List of robots that will be spawn in the simulation e. g. robots:='robot1={x: 0.0, y:"
            " -1.0}; robot2={x: 1.0, y: -1.0}; robot3={x: 2.0, y: -1.0}'"
        ),
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_lidar_model_arg,
            declare_camera_model_arg,
            declare_include_camera_mount_arg,
            declare_world_arg,
            declare_headless_arg,
            declare_robots_arg,
            # Sets use_sim_time for all nodes started below
            # (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=launch_setup),
        ]
    )
