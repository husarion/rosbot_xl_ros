#!/usr/bin/env python3

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default=False)
  rosbot_xl_description = get_package_share_directory('rosbot_xl_description')
  xacro_file = os.path.join(rosbot_xl_description, 'models', 'rosbot_xl', 'rosbot_xl.urdf.xacro')

  robot_controllers = PathJoinSubstitution(
      [
        rosbot_xl_description,
        "config",
        "controllers.yaml",
      ]
    )

  
  robot_description = {
    'robot_description' : Command([
      'xacro --verbosity 0 ', xacro_file,
      ' use_sim:=false',
    ])
  }


  return LaunchDescription([

    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[
        {'use_sim_time': use_sim_time},
        robot_description
      ]
    ),

    Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    ),

    # Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # ),

    # Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    # ),
  ])

if __name__ == '__main__':
  generate_launch_description()