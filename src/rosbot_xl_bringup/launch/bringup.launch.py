from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("rosbot_xl_hardware")
            + "/launch/diff_drive.launch.py"
        )
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("rosbot_xl_ekf"), "config", "ekf.yaml"
            )
        ],
    )

    nodes = [
        diff_drive_launch,
        robot_localization_node,
    ]

    return LaunchDescription(nodes)
